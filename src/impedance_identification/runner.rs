//! Safe register-sequenced impedance-identification runner.
//!
//! This module provides the shared firmware logic used by the active impedance
//! diagnostic dispatch. It centralizes the register sequencing and safety
//! gates, while hardware validation remains a separate workflow.

use foci_tmc4671::registers::{
    ABN_DECODER_COUNT, ADC_IWY_IUX, CONFIG_BIQUAD_F_ENABLE, CONFIG_BIQUAD_T_ENABLE,
    DSADC_MDEC_B_MDEC_A, MODE_RAMP_MODE_MOTION, MOTOR_TYPE_N_POLE_PAIRS, MotionMode, MotorType,
    MotorTypeNPolePairs, OPENLOOP_ACCELERATION, OPENLOOP_MODE, OPENLOOP_PHI,
    OPENLOOP_VELOCITY_ACTUAL, OPENLOOP_VELOCITY_TARGET, PHI_E_EXT, PHI_E_SELECTION,
    PID_TORQUE_FLUX_ACTUAL, PIDOUT_UQ_UD_LIMITS, PWM_BBM_H_BBM_L, PWM_MAXCNT, PWM_SV_CHOP,
    STATUS_FLAGS, SubRegister, UQ_UD_EXT, UqUdExt,
};

use super::fit_impedance;
use super::profile::ImpedanceProfile;
use super::types::{
    IMPEDANCE_PROFILE_POINTS, ImpedanceAngleObservation, ImpedanceBaselineObservation,
    ImpedanceError, ImpedanceFilterState, ImpedanceFitSummary, ImpedancePoint,
    ImpedancePowerStageReadback, ImpedanceReadbackMismatch, ImpedanceSample, ImpedanceWarningFlags,
};

const PHI_E_EXTERNAL: u32 = 1;
const OPENLOOP_PHI_DIRECTION_POSITIVE: u32 = 0;
const MIN_SAMPLE_INTERVAL_US: u16 = 1;
const TURN_COUNTS: f64 = 65_536.0;
const Q10_SCALE: i16 = 1024;
const CARRIER_REF_Q10: [(i16, i16); 20] = [
    (1024, 0),
    (974, 316),
    (828, 602),
    (602, 828),
    (316, 974),
    (0, 1024),
    (-316, 974),
    (-602, 828),
    (-828, 602),
    (-974, 316),
    (-1024, 0),
    (-974, -316),
    (-828, -602),
    (-602, -828),
    (-316, -974),
    (0, -1024),
    (316, -974),
    (602, -828),
    (828, -602),
    (974, -316),
];

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct CarrierAxisQ10 {
    ud: i16,
    uq: i16,
}

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
    /// Encoder counts per revolution, used to wrap encoder motion deltas.
    fn encoder_counts_per_rev(&self) -> u32;
    /// Record bounded held-angle observability before a later fit-gate failure.
    fn record_angle_observation(
        &mut self,
        _point_index: usize,
        _observation: ImpedanceAngleObservation,
    ) {
    }
    /// Record the exact register mismatch behind `ProfileReadbackMismatch`.
    fn record_profile_readback_mismatch(&mut self, _addr: u8, _expected: u32, _actual: u32) {}
}

/// Complete held-angle rotating-carrier runner output.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct ImpedanceRunResult {
    /// Profile used for the run.
    pub profile: ImpedanceProfile,
    /// Per-angle lock-in observations.
    pub angle_observations: [ImpedanceAngleObservation; IMPEDANCE_PROFILE_POINTS],
    /// Aggregate held-angle fit summary.
    pub summary: ImpedanceFitSummary,
    /// Aggregate warning flags from sampling, status, and fit.
    pub warning_flags: ImpedanceWarningFlags,
    /// OR of every sampled `STATUS_FLAGS` value.
    pub status_flags_or: u32,
    /// Power-stage readbacks verified before the first nonzero voltage command.
    pub power_stage_readback: ImpedancePowerStageReadback,
    /// Last profile-readback mismatch observed before failure, if any.
    pub readback_mismatch: ImpedanceReadbackMismatch,
    /// Torque/flux filter state around carrier sampling.
    pub filter_state: ImpedanceFilterState,
}

/// Run held-angle rotating-carrier impedance identification over an already armed motor.
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

    let profile = ImpedanceProfile::standard_held_angle_rotating_carrier();
    validate_profile(&profile)?;
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

    let snapshot = snapshot_torque_flux_filters(backend).await?;
    let body_result = run_energized_body(backend, profile, snapshot).await;
    let restore_result = restore_safe_state(backend, saved_voltage_limit).await;
    let filter_restore_result = restore_torque_flux_filters(backend, snapshot).await;

    match (body_result, restore_result, filter_restore_result) {
        (Ok(result), Ok(()), Ok(())) => Ok(result),
        (_, Err(()), _) | (_, _, Err(())) => Err(ImpedanceError::RestoreFailed),
        (Err(error), Ok(()), Ok(())) => Err(error),
    }
}

fn validate_profile(profile: &ImpedanceProfile) -> Result<(), ImpedanceError> {
    if profile.sample_count as usize != super::types::IMPEDANCE_SAMPLE_COUNT
        || profile.sample_count > profile.max_sample_count
        || profile.sample_interval_us < MIN_SAMPLE_INTERVAL_US
        || profile.carrier_settle_us >= profile.sample_interval_us
        || profile.samples_per_cycle == 0
        || profile.carrier_frequency_millihz() == 0
        || usize::from(profile.samples_per_cycle) != CARRIER_REF_Q10.len()
        || profile.cycles_per_angle == 0
        || profile.discard_cycles >= profile.cycles_per_angle
        || profile.sample_count
            != profile
                .samples_per_cycle
                .saturating_mul(profile.cycles_per_angle)
        || profile.hold_ud_abs == 0
        || profile.hold_settle_ms == 0
        || profile.max_expected_current_count == 0
        || profile.encoder_move_fail_counts == 0
        || profile.encoder_move_warn_counts > profile.encoder_move_fail_counts
        || profile.encoder_angle_invalid_counts > profile.encoder_move_fail_counts
        || profile.max_abs_ud() == 0
        || profile.max_abs_ud() > i16::MAX as u16
        || profile.max_window_exposure() > profile.max_window_exposure_budget
        || profile.max_total_exposure() > profile.max_total_exposure_budget
    {
        return Err(ImpedanceError::UnsupportedProfile);
    }

    for point in profile.angle_points {
        if point.ud_abs == 0
            || point.ud_abs > i16::MAX as u16
            || profile.hold_ud_abs.saturating_add(point.ud_abs) > i16::MAX as u16
            || profile.hold_ud_abs < point.ud_abs
        {
            return Err(ImpedanceError::UnsupportedProfile);
        }
    }

    Ok(())
}

async fn run_energized_body<B: ImpedanceBackend>(
    backend: &mut B,
    profile: ImpedanceProfile,
    filter_snapshot: ImpedanceFilterSnapshot,
) -> Result<ImpedanceRunResult, ImpedanceError> {
    let power_stage_readback = prepare_zero_voltage_setup(backend, &profile).await?;
    disable_torque_flux_filters(backend).await?;
    let filter_state = ImpedanceFilterState {
        torque_enable_before: filter_snapshot.torque_enable,
        flux_enable_before: filter_snapshot.flux_enable,
        torque_enable_during: read_sub(backend, CONFIG_BIQUAD_T_ENABLE).await? as u8,
        flux_enable_during: read_sub(backend, CONFIG_BIQUAD_F_ENABLE).await? as u8,
    };
    write(backend, STATUS_FLAGS.addr(), 0).await?;

    let mut warning_flags = ImpedanceWarningFlags::NONE;
    let mut status_flags_or = 0u32;
    let mut angle_observations = [ImpedanceAngleObservation::EMPTY; IMPEDANCE_PROFILE_POINTS];

    for (index, point) in profile.angle_points.iter().copied().enumerate() {
        let observation = capture_fixed_angle_window(
            backend,
            &profile,
            index,
            point,
            &mut warning_flags,
            &mut status_flags_or,
        )
        .await?;
        warning_flags.insert(observation.warning_flags);
        angle_observations[index] = observation;
        backend.record_angle_observation(index, observation);
    }

    let mut summary = fit_impedance(&profile, &angle_observations)?;
    summary.warning_flags.insert(warning_flags);
    warning_flags = summary.warning_flags;

    Ok(ImpedanceRunResult {
        profile,
        angle_observations,
        summary,
        warning_flags,
        status_flags_or,
        power_stage_readback,
        readback_mismatch: ImpedanceReadbackMismatch::EMPTY,
        filter_state,
    })
}

async fn prepare_zero_voltage_setup<B: ImpedanceBackend>(
    backend: &mut B,
    profile: &ImpedanceProfile,
) -> Result<ImpedancePowerStageReadback, ImpedanceError> {
    write(
        backend,
        PIDOUT_UQ_UD_LIMITS.addr(),
        pidout_limit_word(profile.max_abs_ud()),
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
        pidout_limit_word(profile.max_abs_ud()),
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
    if let Some(mismatch) = power_stage_mismatch(profile, power_stage_readback) {
        backend.record_profile_readback_mismatch(mismatch.addr, mismatch.expected, mismatch.actual);
        return Err(ImpedanceError::ProfileReadbackMismatch);
    }

    Ok(power_stage_readback)
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

fn power_stage_mismatch(
    profile: &ImpedanceProfile,
    readback: ImpedancePowerStageReadback,
) -> Option<ImpedanceReadbackMismatch> {
    let checks = [
        (
            PWM_SV_CHOP.addr(),
            profile.pwm_sv_chop,
            readback.pwm_sv_chop_readback,
        ),
        (
            PWM_MAXCNT.addr(),
            u32::from(profile.pwm_maxcnt),
            readback.pwm_maxcnt_readback,
        ),
        (
            PWM_BBM_H_BBM_L.addr(),
            expected_bbm_readback(profile),
            readback.bbm_readback,
        ),
        (
            DSADC_MDEC_B_MDEC_A.addr(),
            expected_dsadc_readback(profile),
            readback.dsadc_mdec_readback,
        ),
    ];
    for (addr, expected, actual) in checks {
        if actual != expected {
            return Some(ImpedanceReadbackMismatch {
                addr,
                expected,
                actual,
            });
        }
    }
    None
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
struct ImpedanceFilterSnapshot {
    torque_enable: u8,
    flux_enable: u8,
}

async fn snapshot_torque_flux_filters<B: ImpedanceBackend>(
    backend: &mut B,
) -> Result<ImpedanceFilterSnapshot, ImpedanceError> {
    Ok(ImpedanceFilterSnapshot {
        torque_enable: read_sub(backend, CONFIG_BIQUAD_T_ENABLE).await? as u8,
        flux_enable: read_sub(backend, CONFIG_BIQUAD_F_ENABLE).await? as u8,
    })
}

async fn disable_torque_flux_filters<B: ImpedanceBackend>(
    backend: &mut B,
) -> Result<(), ImpedanceError> {
    write_sub(backend, CONFIG_BIQUAD_T_ENABLE, 0).await?;
    write_sub(backend, CONFIG_BIQUAD_F_ENABLE, 0).await
}

async fn restore_torque_flux_filters<B: ImpedanceBackend>(
    backend: &mut B,
    snapshot: ImpedanceFilterSnapshot,
) -> Result<(), ()> {
    let mut failed = false;
    if write_sub_restore(
        backend,
        CONFIG_BIQUAD_T_ENABLE,
        u32::from(snapshot.torque_enable),
    )
    .await
    .is_err()
    {
        failed = true;
    }
    if write_sub_restore(
        backend,
        CONFIG_BIQUAD_F_ENABLE,
        u32::from(snapshot.flux_enable),
    )
    .await
    .is_err()
    {
        failed = true;
    }

    if failed { Err(()) } else { Ok(()) }
}

async fn capture_fixed_angle_window<B: ImpedanceBackend>(
    backend: &mut B,
    profile: &ImpedanceProfile,
    point_index: usize,
    point: ImpedancePoint,
    warning_flags: &mut ImpedanceWarningFlags,
    status_flags_or: &mut u32,
) -> Result<ImpedanceAngleObservation, ImpedanceError> {
    let baseline = sample_zero_voltage_baseline(backend).await?;

    write(backend, UQ_UD_EXT.addr(), 0).await?;
    write(
        backend,
        MODE_RAMP_MODE_MOTION.addr(),
        MotionMode::Stopped as u32,
    )
    .await?;
    write(backend, PHI_E_SELECTION.addr(), PHI_E_EXTERNAL).await?;
    write(backend, PHI_E_EXT.addr(), 0).await?;
    verify_readback(backend, PHI_E_SELECTION.addr(), PHI_E_EXTERNAL).await?;
    verify_readback(backend, PHI_E_EXT.addr(), 0).await?;
    write(
        backend,
        MODE_RAMP_MODE_MOTION.addr(),
        MotionMode::UqUdExt as u32,
    )
    .await?;
    write(
        backend,
        UQ_UD_EXT.addr(),
        pack_uq_ud(0, profile.hold_ud_abs as i16),
    )
    .await?;
    backend
        .delay_us(u32::from(profile.hold_settle_ms).saturating_mul(1_000))
        .await;
    check_current_limit(
        profile,
        decode_current_pair(
            backend.timestamp_us(),
            read(backend, ADC_IWY_IUX.addr()).await?,
        ),
    )?;
    let mut point_warning_flags = ImpedanceWarningFlags::NONE;
    let _ = sample_status(backend, profile, &mut point_warning_flags, status_flags_or).await?;
    warning_flags.insert(point_warning_flags);

    let encoder_before = read(backend, ABN_DECODER_COUNT.addr()).await?;
    let encoder_counts_per_rev = backend.encoder_counts_per_rev();
    let mut encoder_min_delta = 0i32;
    let mut encoder_max_delta = 0i32;

    let mut accumulator =
        ImpedanceAngleAccumulator::new(point.angle_electrical_counts, point.ud_abs);
    let carrier_axis = carrier_axis_q10(point.angle_electrical_counts);
    let sample_count = profile.sample_count as usize;
    let samples_per_cycle = profile.samples_per_cycle as usize;
    let discarded_samples = usize::from(profile.discard_cycles) * samples_per_cycle;
    for index in 0..sample_count {
        let (carrier_uq, carrier_ud) =
            carrier_vector_for_sample(index, samples_per_cycle, profile, point, carrier_axis);
        write(
            backend,
            UQ_UD_EXT.addr(),
            pack_uq_ud(carrier_uq, carrier_ud),
        )
        .await?;
        backend.delay_us(u32::from(profile.carrier_settle_us)).await;
        let t_us = backend.timestamp_us();
        let transformed = decode_transformed_current_sample(
            t_us,
            read(backend, PID_TORQUE_FLUX_ACTUAL.addr()).await?,
        );
        let raw = decode_current_pair(t_us, read(backend, ADC_IWY_IUX.addr()).await?);
        check_current_limit(profile, raw)?;
        if index >= discarded_samples {
            accumulator.accumulate(index - discarded_samples, transformed, raw);
        }

        let encoder = read(backend, ABN_DECODER_COUNT.addr()).await?;
        update_encoder_delta_bounds(
            encoder_before,
            encoder,
            encoder_counts_per_rev,
            &mut encoder_min_delta,
            &mut encoder_max_delta,
        );
        let peak = encoder_peak_from_delta_span(encoder_min_delta, encoder_max_delta);
        if peak > profile.encoder_move_fail_counts {
            let _ = write(backend, UQ_UD_EXT.addr(), 0).await;
            let observation = accumulator.finish(
                profile,
                baseline,
                EncoderWindow {
                    before: encoder_before,
                    after: encoder,
                    min: encoder_count_from_delta(
                        encoder_before,
                        encoder_min_delta,
                        encoder_counts_per_rev,
                    ),
                    max: encoder_count_from_delta(
                        encoder_before,
                        encoder_max_delta,
                        encoder_counts_per_rev,
                    ),
                    counts_per_rev: encoder_counts_per_rev,
                },
            );
            warning_flags.insert(observation.warning_flags);
            backend.record_angle_observation(point_index, observation);
            return Err(ImpedanceError::EncoderMoved);
        }
        if peak > profile.encoder_move_warn_counts {
            warning_flags.insert(ImpedanceWarningFlags::ENCODER_MOVED);
        }

        let mut point_warning_flags = ImpedanceWarningFlags::NONE;
        let _ = sample_status(backend, profile, &mut point_warning_flags, status_flags_or).await?;
        warning_flags.insert(point_warning_flags);
        accumulator.insert_warning_flags(point_warning_flags);

        if index + 1 < sample_count {
            backend
                .delay_us(u32::from(
                    profile.sample_interval_us - profile.carrier_settle_us,
                ))
                .await;
        }
    }

    write(backend, UQ_UD_EXT.addr(), 0).await?;
    let encoder_after = read(backend, ABN_DECODER_COUNT.addr()).await?;
    update_encoder_delta_bounds(
        encoder_before,
        encoder_after,
        encoder_counts_per_rev,
        &mut encoder_min_delta,
        &mut encoder_max_delta,
    );
    let observation = accumulator.finish(
        profile,
        baseline,
        EncoderWindow {
            before: encoder_before,
            after: encoder_after,
            min: encoder_count_from_delta(
                encoder_before,
                encoder_min_delta,
                encoder_counts_per_rev,
            ),
            max: encoder_count_from_delta(
                encoder_before,
                encoder_max_delta,
                encoder_counts_per_rev,
            ),
            counts_per_rev: encoder_counts_per_rev,
        },
    );
    warning_flags.insert(observation.warning_flags);
    Ok(observation)
}

fn carrier_vector_for_sample(
    index: usize,
    samples_per_cycle: usize,
    profile: &ImpedanceProfile,
    point: ImpedancePoint,
    axis: CarrierAxisQ10,
) -> (i16, i16) {
    let half_cycle = samples_per_cycle / 2;
    let cycle_index = index % samples_per_cycle;
    let sign = if cycle_index < half_cycle { 1 } else { -1 };
    let carrier_ud = scale_q10(point.ud_abs, signed_axis_component(axis.ud, sign));
    let carrier_uq = scale_q10(point.ud_abs, signed_axis_component(axis.uq, sign));
    (
        carrier_uq,
        (profile.hold_ud_abs as i16).saturating_add(carrier_ud),
    )
}

fn carrier_axis_q10(angle_electrical_counts: u16) -> CarrierAxisQ10 {
    let theta = core::f64::consts::TAU * f64::from(angle_electrical_counts) / TURN_COUNTS;
    CarrierAxisQ10 {
        ud: round_unit_q10(libm::cos(theta)),
        uq: round_unit_q10(libm::sin(theta)),
    }
}

fn round_unit_q10(value: f64) -> i16 {
    let rounded = libm::round(value * f64::from(Q10_SCALE)) as i32;
    if rounded > i32::from(Q10_SCALE) {
        Q10_SCALE
    } else if rounded < -i32::from(Q10_SCALE) {
        -Q10_SCALE
    } else {
        rounded as i16
    }
}

fn signed_axis_component(component: i16, sign: i16) -> i16 {
    if sign < 0 { -component } else { component }
}

fn scale_q10(amplitude: u16, component_q10: i16) -> i16 {
    let numerator = i32::from(amplitude).saturating_mul(i32::from(component_q10));
    let rounded = if numerator >= 0 {
        numerator.saturating_add(i32::from(Q10_SCALE) / 2) / i32::from(Q10_SCALE)
    } else {
        numerator.saturating_sub(i32::from(Q10_SCALE) / 2) / i32::from(Q10_SCALE)
    };
    rounded.clamp(i32::from(i16::MIN), i32::from(i16::MAX)) as i16
}

fn project_transformed_current(sample: ImpedanceSample, axis: CarrierAxisQ10) -> i16 {
    let d = i32::from(sample.i_ch0).saturating_mul(i32::from(axis.ud));
    let q = i32::from(sample.i_ch1).saturating_mul(i32::from(axis.uq));
    round_q10_i32(d.saturating_add(q))
}

fn round_q10_i32(value: i32) -> i16 {
    let rounded = if value >= 0 {
        value.saturating_add(i32::from(Q10_SCALE) / 2) / i32::from(Q10_SCALE)
    } else {
        value.saturating_sub(i32::from(Q10_SCALE) / 2) / i32::from(Q10_SCALE)
    };
    rounded.clamp(i32::from(i16::MIN), i32::from(i16::MAX)) as i16
}

async fn sample_status<B: ImpedanceBackend>(
    backend: &mut B,
    profile: &ImpedanceProfile,
    warning_flags: &mut ImpedanceWarningFlags,
    status_flags_or: &mut u32,
) -> Result<u32, ImpedanceError> {
    let status_flags = read(backend, STATUS_FLAGS.addr()).await?;
    *status_flags_or |= status_flags;
    if status_flags & profile.status_flags_warn_mask != 0 {
        warning_flags.insert(ImpedanceWarningFlags::STATUS_FLAGS);
    }
    if status_flags & profile.status_flags_fail_mask != 0 {
        return Err(ImpedanceError::UnsafeStatusFlags);
    }
    Ok(status_flags)
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
        backend.record_profile_readback_mismatch(addr, expected, actual);
        Err(ImpedanceError::ProfileReadbackMismatch)
    }
}

async fn read<B: ImpedanceBackend>(backend: &mut B, addr: u8) -> Result<u32, ImpedanceError> {
    backend
        .read_register(addr)
        .await
        .map_err(|_| ImpedanceError::BackendIo)
}

async fn read_sub<B: ImpedanceBackend>(
    backend: &mut B,
    sub: SubRegister,
) -> Result<u32, ImpedanceError> {
    write(backend, sub.addr.addr(), u32::from(sub.sub_addr)).await?;
    read(backend, sub.data.addr()).await
}

async fn write_sub<B: ImpedanceBackend>(
    backend: &mut B,
    sub: SubRegister,
    value: u32,
) -> Result<(), ImpedanceError> {
    write(backend, sub.addr.addr(), u32::from(sub.sub_addr)).await?;
    write(backend, sub.data.addr(), value).await
}

async fn write_sub_restore<B: ImpedanceBackend>(
    backend: &mut B,
    sub: SubRegister,
    value: u32,
) -> Result<(), ()> {
    if backend
        .write_register(sub.addr.addr(), u32::from(sub.sub_addr))
        .await
        .is_err()
    {
        return Err(());
    }
    backend
        .write_register(sub.data.addr(), value)
        .await
        .map_err(|_| ())
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

fn split_signed_halves(packed: u32) -> (i16, i16) {
    ((packed >> 16) as u16 as i16, packed as u16 as i16)
}

fn pack_uq_ud(uq: i16, ud: i16) -> u32 {
    UqUdExt::new().with_ud_ext(ud).with_uq_ext(uq).into()
}

fn pidout_limit_word(limit: u16) -> u32 {
    u32::from(limit)
}

fn pidout_ud_limit(value: u32) -> u16 {
    (value & 0xffff) as u16
}

fn decode_adc_sample(t_us: u32, openloop_phi: u16, packed: u32) -> ImpedanceSample {
    ImpedanceSample {
        t_us,
        openloop_phi,
        i_ch0: (packed & 0xffff) as u16 as i16,
        i_ch1: (packed >> 16) as u16 as i16,
    }
}

async fn sample_zero_voltage_baseline<B: ImpedanceBackend>(
    backend: &mut B,
) -> Result<ImpedanceBaselineObservation, ImpedanceError> {
    let (q_count, d_count) =
        split_signed_halves(read(backend, PID_TORQUE_FLUX_ACTUAL.addr()).await?);
    let (adc_ch1_count, adc_ch0_count) =
        split_signed_halves(read(backend, ADC_IWY_IUX.addr()).await?);

    Ok(ImpedanceBaselineObservation {
        d_count,
        q_count,
        adc_ch0_count,
        adc_ch1_count,
    })
}

fn decode_current_pair(t_us: u32, packed: u32) -> ImpedanceSample {
    decode_adc_sample(t_us, 0, packed)
}

fn decode_transformed_current_sample(t_us: u32, packed: u32) -> ImpedanceSample {
    decode_adc_sample(t_us, 0, packed)
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

fn update_encoder_delta_bounds(
    before: u32,
    current: u32,
    counts_per_rev: u32,
    min_delta: &mut i32,
    max_delta: &mut i32,
) {
    let delta = signed_encoder_delta(before, current, counts_per_rev);
    *min_delta = (*min_delta).min(delta);
    *max_delta = (*max_delta).max(delta);
}

fn signed_encoder_delta(before: u32, current: u32, counts_per_rev: u32) -> i32 {
    let raw = (current as i32).wrapping_sub(before as i32);
    let counts_per_rev = counts_per_rev as i32;
    if counts_per_rev <= 0 {
        return raw;
    }

    let half = counts_per_rev / 2;
    if raw > half {
        raw - counts_per_rev
    } else if raw < -half {
        raw + counts_per_rev
    } else {
        raw
    }
}

fn encoder_peak_counts(before: u32, min: u32, max: u32, counts_per_rev: u32) -> u16 {
    let min_delta = signed_encoder_delta(before, min, counts_per_rev);
    let max_delta = signed_encoder_delta(before, max, counts_per_rev);
    encoder_peak_from_delta_span(min_delta, max_delta)
}

fn encoder_count_from_delta(before: u32, delta: i32, counts_per_rev: u32) -> u32 {
    if counts_per_rev == 0 {
        return before.wrapping_add(delta as u32);
    }

    let counts = i64::from(counts_per_rev);
    let raw = i64::from(before) + i64::from(delta);
    raw.rem_euclid(counts) as u32
}

fn encoder_peak_from_delta_span(min_delta: i32, max_delta: i32) -> u16 {
    i64::from(max_delta)
        .saturating_sub(i64::from(min_delta))
        .unsigned_abs()
        .min(u64::from(u16::MAX)) as u16
}

/// Streaming lock-in accumulator for one held-angle rotating-carrier window.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct ImpedanceAngleAccumulator {
    angle_electrical_counts: u16,
    ud_abs: u16,
    probe_axis: CarrierAxisQ10,
    sample_count: u32,
    d_sin_sum: i64,
    d_cos_sum: i64,
    raw_sin_sum: i64,
    raw_cos_sum: i64,
    warning_flags: ImpedanceWarningFlags,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct EncoderWindow {
    pub(super) before: u32,
    pub(super) after: u32,
    pub(super) min: u32,
    pub(super) max: u32,
    pub(super) counts_per_rev: u32,
}

impl ImpedanceAngleAccumulator {
    /// Create a streaming accumulator for one carrier probe angle.
    pub fn new(angle_electrical_counts: u16, ud_abs: u16) -> Self {
        Self {
            angle_electrical_counts,
            ud_abs,
            probe_axis: carrier_axis_q10(angle_electrical_counts),
            sample_count: 0,
            d_sin_sum: 0,
            d_cos_sum: 0,
            raw_sin_sum: 0,
            raw_cos_sum: 0,
            warning_flags: ImpedanceWarningFlags::NONE,
        }
    }

    /// Add one transformed/raw current sample pair to the lock-in sums.
    pub fn accumulate(
        &mut self,
        sample_index: usize,
        transformed: ImpedanceSample,
        raw: ImpedanceSample,
    ) {
        let (cos_q10, sin_q10) = CARRIER_REF_Q10[sample_index % CARRIER_REF_Q10.len()];
        let projected = project_transformed_current(transformed, self.probe_axis);
        self.d_sin_sum += i64::from(projected) * i64::from(sin_q10);
        self.d_cos_sum += i64::from(projected) * i64::from(cos_q10);
        self.raw_sin_sum += i64::from(raw.i_ch0) * i64::from(sin_q10);
        self.raw_cos_sum += i64::from(raw.i_ch0) * i64::from(cos_q10);
        self.sample_count += 1;
    }

    fn insert_warning_flags(&mut self, warning_flags: ImpedanceWarningFlags) {
        self.warning_flags.insert(warning_flags);
    }

    /// Finish the observation with baseline and encoder evidence.
    pub fn finish(
        self,
        profile: &ImpedanceProfile,
        baseline: ImpedanceBaselineObservation,
        encoder: EncoderWindow,
    ) -> ImpedanceAngleObservation {
        let encoder_peak_counts = encoder_peak_counts(
            encoder.before,
            encoder.min,
            encoder.max,
            encoder.counts_per_rev,
        );
        let mut warning_flags = self.warning_flags;
        if encoder_peak_counts > profile.encoder_angle_invalid_counts {
            warning_flags.insert(ImpedanceWarningFlags::ANGLE_LOCAL_MOTION);
        }
        if encoder_peak_counts > profile.encoder_move_warn_counts {
            warning_flags.insert(ImpedanceWarningFlags::ENCODER_MOVED);
        }

        ImpedanceAngleObservation {
            angle_electrical_counts: self.angle_electrical_counts,
            ud_abs: self.ud_abs,
            inductive_response_count_per_ud_milli: self.normalized_response(self.d_cos_sum),
            in_phase_response_count_per_ud_milli: self.normalized_response(self.d_sin_sum),
            secondary_adc_response_count_per_ud_milli: self.normalized_response(self.raw_cos_sum),
            baseline_d_count: baseline.d_count,
            baseline_q_count: baseline.q_count,
            baseline_adc_ch0_count: baseline.adc_ch0_count,
            baseline_adc_ch1_count: baseline.adc_ch1_count,
            residual_permille: 0,
            coverage_permille: 1000,
            encoder_before: encoder.before,
            encoder_after: encoder.after,
            encoder_min: encoder.min,
            encoder_max: encoder.max,
            encoder_peak_counts,
            valid: encoder_peak_counts <= profile.encoder_angle_invalid_counts,
            warning_flags,
        }
    }

    fn normalized_response(&self, sum: i64) -> u32 {
        if self.sample_count == 0 || self.ud_abs == 0 {
            return 0;
        }

        let numerator = i128::from(sum.unsigned_abs())
            .saturating_mul(2)
            .saturating_mul(1_000);
        let denominator = i128::from(self.sample_count)
            .saturating_mul(1024)
            .saturating_mul(i128::from(self.ud_abs));
        if denominator == 0 {
            return 0;
        }
        let value = (numerator + denominator / 2) / denominator;
        value.min(i128::from(u32::MAX)) as u32
    }
}
