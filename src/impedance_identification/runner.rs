//! Safe register-sequenced impedance-identification runner.
//!
//! This module provides the shared firmware logic used by the active impedance
//! diagnostic dispatch. It centralizes the register sequencing and safety
//! gates, while hardware validation remains a separate workflow.

use foci_tmc4671::registers::{
    ABN_DECODER_COUNT, ADC_IWY_IUX, CONFIG_BIQUAD_F_ENABLE, CONFIG_BIQUAD_T_ENABLE,
    DSADC_MDEC_B_MDEC_A, INTERIM_FOC_IQ_ID, INTERIM_FOC_IWY_IUX, INTERIM_FOC_UQ_UD,
    INTERIM_FOC_UQ_UD_LIMITED, MODE_RAMP_MODE_MOTION, MOTOR_TYPE_N_POLE_PAIRS, MotionMode,
    MotorType, MotorTypeNPolePairs, OPENLOOP_ACCELERATION, OPENLOOP_MODE, OPENLOOP_PHI,
    OPENLOOP_VELOCITY_ACTUAL, OPENLOOP_VELOCITY_TARGET, PHI_E_EXT, PHI_E_SELECTION,
    PID_TORQUE_FLUX_ACTUAL, PIDOUT_UQ_UD_LIMITS, PWM_BBM_H_BBM_L, PWM_MAXCNT, PWM_SV_CHOP,
    STATUS_FLAGS, SubRegister, UQ_UD_EXT, UqUdExt,
};

use super::fit::{ImpedanceFitPoint, ImpedanceFitSummary, fit_impedance_point_with_evidence};
use super::fit_pulsating_impedance;
use super::profile::{ImpedanceProfile, frequency_millihz_to_openloop_velocity_rpm};
use super::pulsating_profile::PulsatingImpedanceProfile;
use super::types::{
    IMPEDANCE_PROFILE_POINTS, IMPEDANCE_SAMPLE_COUNT, ImpedanceBaselineObservation, ImpedanceError,
    ImpedancePointObservation, ImpedancePowerStageReadback, ImpedanceReadbackMismatch,
    ImpedanceSample, ImpedanceWarningFlags, PULSATING_IMPEDANCE_PROFILE_POINTS,
    PulsatingAngleObservation, PulsatingBaselineObservation, PulsatingFilterState,
    PulsatingFitSummary, PulsatingImpedancePoint,
};
use crate::impedance_identification::fit::evaluate_impedance_result;

const PHI_E_EXTERNAL: u32 = 1;
const PHI_E_OPEN_LOOP: u32 = 2;
const OPENLOOP_PHI_DIRECTION_POSITIVE: u32 = 0;
const MIN_SAMPLE_INTERVAL_US: u16 = 1_000;
const MIN_PULSATING_SAMPLE_INTERVAL_US: u16 = 1;
const MILLIS_PER_SECOND: u64 = 1_000;
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

const EMPTY_FIT: ImpedanceFitPoint = ImpedanceFitPoint {
    axis_max_count_per_ud_milli: 0,
    axis_min_count_per_ud_milli: 0,
    axis_angle_electrical_counts: 0,
    residual_permille: 0,
    coverage_permille: 0,
    scalar_2f_mismatch_permille: 0,
    magnitude_mean_count_per_ud_milli: 0,
    magnitude_2f_count_per_ud_milli: 0,
    magnitude_2f_residual_permille: 0,
    magnitude_2f_coverage_permille: 0,
    warning_flags: ImpedanceWarningFlags::NONE,
};

struct ImpedanceCapture {
    raw_samples: [ImpedanceSample; IMPEDANCE_SAMPLE_COUNT],
    transformed_samples: [ImpedanceSample; IMPEDANCE_SAMPLE_COUNT],
    observation: ImpedancePointObservation,
}

struct DcBaselineCapture {
    status_flags_or: u32,
    observation: ImpedanceBaselineObservation,
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
    /// Board-specific impedance-identification profile.
    fn profile(&self) -> ImpedanceProfile;
    /// Record bounded point observability before a fit-gate failure is returned.
    fn record_point_observation(
        &mut self,
        _point_index: usize,
        _observation: ImpedancePointObservation,
    ) {
    }
    /// Record bounded DC baseline observability before later AC/fit failures.
    fn record_baseline_observation(
        &mut self,
        _point_index: usize,
        _observation: ImpedanceBaselineObservation,
    ) {
    }
    /// Record bounded fit evidence before a fit-gate failure is returned.
    fn record_fit_attempt(&mut self, _point_index: usize, _fit: ImpedanceFitPoint) {}
    /// Record the exact register mismatch behind `ProfileReadbackMismatch`.
    fn record_profile_readback_mismatch(&mut self, _addr: u8, _expected: u32, _actual: u32) {}
}

/// Complete runner output returned through the active diagnostic surface.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct ImpedanceRunResult {
    /// Profile used for the run.
    pub profile: ImpedanceProfile,
    /// Per-injection fitted impedance points.
    pub point_fits: [ImpedanceFitPoint; IMPEDANCE_PROFILE_POINTS],
    /// Per-injection chip observability sampled while voltage was active.
    pub point_observations: [ImpedancePointObservation; IMPEDANCE_PROFILE_POINTS],
    /// Per-injection DC baseline current observations sampled before AC capture.
    pub point_baselines: [ImpedanceBaselineObservation; IMPEDANCE_PROFILE_POINTS],
    /// Aggregate impedance summary.
    pub summary: ImpedanceFitSummary,
    /// Aggregate warning flags from fit, status, and encoder gates.
    pub warning_flags: ImpedanceWarningFlags,
    /// OR of every sampled `STATUS_FLAGS` value.
    pub status_flags_or: u32,
    /// Power-stage readbacks verified before the first nonzero voltage command.
    pub power_stage_readback: ImpedancePowerStageReadback,
    /// Last profile-readback mismatch observed before failure, if any.
    pub readback_mismatch: ImpedanceReadbackMismatch,
}

/// Complete fixed-angle pulsating runner output returned by v2.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct PulsatingImpedanceRunResult {
    /// Profile used for the run.
    pub profile: PulsatingImpedanceProfile,
    /// Per-angle lock-in observations.
    pub angle_observations: [PulsatingAngleObservation; PULSATING_IMPEDANCE_PROFILE_POINTS],
    /// Aggregate fixed-angle fit summary.
    pub summary: PulsatingFitSummary,
    /// Aggregate warning flags from sampling, status, and fit.
    pub warning_flags: ImpedanceWarningFlags,
    /// OR of every sampled `STATUS_FLAGS` value.
    pub status_flags_or: u32,
    /// Power-stage readbacks verified before the first nonzero voltage command.
    pub power_stage_readback: ImpedancePowerStageReadback,
    /// Last profile-readback mismatch observed before failure, if any.
    pub readback_mismatch: ImpedanceReadbackMismatch,
    /// Torque/flux filter state around carrier sampling.
    pub filter_state: PulsatingFilterState,
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
    let motor_config = MotorTypeNPolePairs::from(motor_type_raw);
    let motor_type = motor_config.motor_type();
    let n_pole_pairs = motor_config.n_pole_pairs();
    if motor_type != MotorType::TwoPhase as u8 || n_pole_pairs == 0 {
        return Err(ImpedanceError::UnsupportedMotorType);
    }

    validate_profile(&profile, n_pole_pairs)?;

    let body_result = run_energized_body(backend, profile, n_pole_pairs).await;
    let restore_result = restore_safe_state(backend, saved_voltage_limit).await;

    match (body_result, restore_result) {
        (Ok(result), Ok(())) => Ok(result),
        (_, Err(())) => Err(ImpedanceError::RestoreFailed),
        (Err(error), Ok(())) => Err(error),
    }
}

/// Run v2 fixed-angle pulsating impedance identification over an already armed motor.
pub async fn run_pulsating_impedance_identification<B: ImpedanceBackend>(
    backend: &mut B,
) -> Result<PulsatingImpedanceRunResult, ImpedanceError> {
    if !backend.is_armed_for_open_loop_voltage() {
        return Err(ImpedanceError::NotArmed);
    }

    let saved_voltage_limit = match backend.read_register(PIDOUT_UQ_UD_LIMITS.addr()).await {
        Ok(value) => value,
        Err(_) => return Err(ImpedanceError::BackendIo),
    };

    let profile = PulsatingImpedanceProfile::standard_openffboard();
    validate_pulsating_profile(&profile)?;
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
    let body_result = run_pulsating_energized_body(backend, profile, snapshot).await;
    let restore_result = restore_safe_state(backend, saved_voltage_limit).await;
    let filter_restore_result = restore_torque_flux_filters(backend, snapshot).await;

    match (body_result, restore_result, filter_restore_result) {
        (Ok(result), Ok(()), Ok(())) => Ok(result),
        (_, Err(()), _) | (_, _, Err(())) => Err(ImpedanceError::RestoreFailed),
        (Err(error), Ok(()), Ok(())) => Err(error),
    }
}

fn validate_profile(profile: &ImpedanceProfile, n_pole_pairs: u16) -> Result<(), ImpedanceError> {
    if profile.sample_count as usize != IMPEDANCE_SAMPLE_COUNT
        || profile.sample_count > profile.max_sample_count
        || profile.sample_interval_us < MIN_SAMPLE_INTERVAL_US
        || !profile
            .sample_interval_us
            .is_multiple_of(MIN_SAMPLE_INTERVAL_US)
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
        let openloop_velocity_rpm = frequency_millihz_to_openloop_velocity_rpm(
            u64::from(point.frequency_millihz),
            n_pole_pairs,
        );
        // Reject if the open-loop ramp cannot reach the injection velocity within
        // the settle window: velocity_rpm / accel_rpm_per_s seconds must be no
        // longer than settle_ms, i.e. velocity_rpm * 1000 <= settle_ms * accel.
        let ramp_demand =
            u64::from(openloop_velocity_rpm.max(0) as u32).saturating_mul(MILLIS_PER_SECOND);
        let ramp_capacity = u64::from(point.settle_ms)
            .saturating_mul(u64::from(point.openloop_acceleration_rpm_per_s));
        if point.ud_abs == 0
            || point.ud_abs > i16::MAX as u16
            || point.frequency_millihz == 0
            || openloop_velocity_rpm <= 0
            || point.openloop_acceleration_rpm_per_s == 0
            || ramp_demand > ramp_capacity
        {
            return Err(ImpedanceError::UnsupportedProfile);
        }
    }

    Ok(())
}

fn validate_pulsating_profile(profile: &PulsatingImpedanceProfile) -> Result<(), ImpedanceError> {
    if profile.sample_count as usize != super::types::PULSATING_IMPEDANCE_SAMPLE_COUNT
        || profile.sample_count > profile.max_sample_count
        || profile.sample_interval_us < MIN_PULSATING_SAMPLE_INTERVAL_US
        || profile.samples_per_cycle == 0
        || usize::from(profile.samples_per_cycle) != CARRIER_REF_Q10.len()
        || profile.cycles_per_angle == 0
        || profile.sample_count
            != profile
                .samples_per_cycle
                .saturating_mul(profile.cycles_per_angle)
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
        if point.ud_abs == 0 || point.ud_abs > i16::MAX as u16 {
            return Err(ImpedanceError::UnsupportedProfile);
        }
    }

    Ok(())
}

async fn run_energized_body<B: ImpedanceBackend>(
    backend: &mut B,
    profile: ImpedanceProfile,
    n_pole_pairs: u16,
) -> Result<ImpedanceRunResult, ImpedanceError> {
    let power_stage_readback = prepare_zero_voltage_setup(backend, &profile).await?;

    let mut warning_flags = ImpedanceWarningFlags::NONE;
    let mut status_flags_or = 0u32;
    let mut point_fits = [EMPTY_FIT; IMPEDANCE_PROFILE_POINTS];
    let mut point_observations = [ImpedancePointObservation::EMPTY; IMPEDANCE_PROFILE_POINTS];
    let mut point_baselines = [ImpedanceBaselineObservation::EMPTY; IMPEDANCE_PROFILE_POINTS];
    let mut fit_quality_error = None;

    for (index, point) in profile.injection_points.iter().copied().enumerate() {
        let dc_baseline = run_dc_baseline_check(
            backend,
            &profile,
            point,
            &mut warning_flags,
            &mut status_flags_or,
        )
        .await?;
        backend.record_baseline_observation(index, dc_baseline.observation);
        point_baselines[index] = dc_baseline.observation;

        let capture = capture_ac_window(
            backend,
            &profile,
            point,
            n_pole_pairs,
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
        let mut observation = capture.observation;
        observation.status_flags_or |= dc_baseline.status_flags_or;
        backend.record_point_observation(index, observation);
        point_observations[index] = observation;

        let fit = match fit_impedance_point_with_evidence(
            &profile,
            point,
            &capture.raw_samples,
            &capture.transformed_samples,
        ) {
            Ok(fit) => fit,
            Err(failure) => {
                if let Some(fit) = failure.fit {
                    backend.record_fit_attempt(index, fit);
                    point_fits[index] = fit;
                    warning_flags.insert(fit.warning_flags);
                }
                if is_deferred_fit_quality_error(failure.error) {
                    if fit_quality_error.is_none() {
                        fit_quality_error = Some(failure.error);
                    }
                    continue;
                }
                return Err(failure.error);
            }
        };
        backend.record_fit_attempt(index, fit);
        point_fits[index] = fit;
        warning_flags.insert(point_fits[index].warning_flags);
    }

    if let Some(error) = fit_quality_error {
        return Err(error);
    }

    let mut summary = evaluate_impedance_result(&profile, &point_fits)?;
    summary.warning_flags.insert(warning_flags);
    warning_flags = summary.warning_flags;

    Ok(ImpedanceRunResult {
        profile,
        point_fits,
        point_observations,
        point_baselines,
        summary,
        warning_flags,
        status_flags_or,
        power_stage_readback,
        readback_mismatch: ImpedanceReadbackMismatch::EMPTY,
    })
}

async fn run_pulsating_energized_body<B: ImpedanceBackend>(
    backend: &mut B,
    profile: PulsatingImpedanceProfile,
    filter_snapshot: PulsatingFilterSnapshot,
) -> Result<PulsatingImpedanceRunResult, ImpedanceError> {
    let power_stage_readback = prepare_pulsating_zero_voltage_setup(backend, &profile).await?;
    disable_torque_flux_filters(backend).await?;
    let filter_state = PulsatingFilterState {
        torque_enable_before: filter_snapshot.torque_enable,
        flux_enable_before: filter_snapshot.flux_enable,
        torque_enable_during: read_sub(backend, CONFIG_BIQUAD_T_ENABLE).await? as u8,
        flux_enable_during: read_sub(backend, CONFIG_BIQUAD_F_ENABLE).await? as u8,
    };

    let mut warning_flags = ImpedanceWarningFlags::NONE;
    let mut status_flags_or = 0u32;
    let mut angle_observations =
        [PulsatingAngleObservation::EMPTY; PULSATING_IMPEDANCE_PROFILE_POINTS];

    for (index, point) in profile.angle_points.iter().copied().enumerate() {
        let observation = capture_fixed_angle_window(
            backend,
            &profile,
            point,
            &mut warning_flags,
            &mut status_flags_or,
        )
        .await?;
        warning_flags.insert(observation.warning_flags);
        angle_observations[index] = observation;
    }

    let mut summary = fit_pulsating_impedance(&profile, &angle_observations)?;
    summary.warning_flags.insert(warning_flags);
    warning_flags = summary.warning_flags;

    Ok(PulsatingImpedanceRunResult {
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

fn is_deferred_fit_quality_error(error: ImpedanceError) -> bool {
    matches!(
        error,
        ImpedanceError::LowSignal
            | ImpedanceError::PoorCoverage
            | ImpedanceError::Scalar2fMismatch
            | ImpedanceError::HighResidual
    )
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

async fn prepare_pulsating_zero_voltage_setup<B: ImpedanceBackend>(
    backend: &mut B,
    profile: &PulsatingImpedanceProfile,
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
    if let Some(mismatch) = pulsating_power_stage_mismatch(profile, power_stage_readback) {
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

fn expected_pulsating_bbm_readback(profile: &PulsatingImpedanceProfile) -> u32 {
    (u32::from(profile.bbm_h) << 8) | u32::from(profile.bbm_l)
}

fn expected_pulsating_dsadc_readback(profile: &PulsatingImpedanceProfile) -> u32 {
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

fn pulsating_power_stage_mismatch(
    profile: &PulsatingImpedanceProfile,
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
            expected_pulsating_bbm_readback(profile),
            readback.bbm_readback,
        ),
        (
            DSADC_MDEC_B_MDEC_A.addr(),
            expected_pulsating_dsadc_readback(profile),
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
struct PulsatingFilterSnapshot {
    torque_enable: u8,
    flux_enable: u8,
}

async fn snapshot_torque_flux_filters<B: ImpedanceBackend>(
    backend: &mut B,
) -> Result<PulsatingFilterSnapshot, ImpedanceError> {
    Ok(PulsatingFilterSnapshot {
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
    snapshot: PulsatingFilterSnapshot,
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

async fn run_dc_baseline_check<B: ImpedanceBackend>(
    backend: &mut B,
    profile: &ImpedanceProfile,
    point: super::types::ImpedanceInjectionPoint,
    warning_flags: &mut ImpedanceWarningFlags,
    status_flags_or: &mut u32,
) -> Result<DcBaselineCapture, ImpedanceError> {
    let mut point_status_flags_or = 0u32;
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
    point_status_flags_or |=
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
    let sample = decode_adc_sample(0, 0, packed);
    check_current_limit(profile, sample)?;
    let (dc_foc_iwy_sample, dc_foc_iux_sample) =
        split_signed_halves(read_sub(backend, INTERIM_FOC_IWY_IUX).await?);
    let (dc_foc_iq_sample, dc_foc_id_sample) =
        split_signed_halves(read_sub(backend, INTERIM_FOC_IQ_ID).await?);
    point_status_flags_or |=
        sample_status(backend, profile, warning_flags, status_flags_or).await?;

    let encoder_after = read(backend, ABN_DECODER_COUNT.addr()).await?;
    check_encoder_delta(profile, encoder_before, encoder_after, warning_flags)?;
    Ok(DcBaselineCapture {
        status_flags_or: point_status_flags_or,
        observation: ImpedanceBaselineObservation {
            dc_adc_iux_sample: sample.i_ch0,
            dc_adc_iwy_sample: sample.i_ch1,
            dc_foc_iux_sample,
            dc_foc_iwy_sample,
            dc_foc_iq_sample,
            dc_foc_id_sample,
        },
    })
}

async fn capture_ac_window<B: ImpedanceBackend>(
    backend: &mut B,
    profile: &ImpedanceProfile,
    point: super::types::ImpedanceInjectionPoint,
    n_pole_pairs: u16,
    warning_flags: &mut ImpedanceWarningFlags,
    status_flags_or: &mut u32,
) -> Result<ImpedanceCapture, ImpedanceError> {
    let encoder_before = read(backend, ABN_DECODER_COUNT.addr()).await?;
    let openloop_velocity_rpm = frequency_millihz_to_openloop_velocity_rpm(
        u64::from(point.frequency_millihz),
        n_pole_pairs,
    ) as u32;

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
        openloop_velocity_rpm,
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
        openloop_velocity_rpm,
    )
    .await?;
    verify_readback(
        backend,
        OPENLOOP_MODE.addr(),
        OPENLOOP_PHI_DIRECTION_POSITIVE,
    )
    .await?;
    let mut point_status_flags_or =
        sample_status(backend, profile, warning_flags, status_flags_or).await?;
    write(backend, UQ_UD_EXT.addr(), pack_ud(point.ud_abs as i16)).await?;
    write(
        backend,
        MODE_RAMP_MODE_MOTION.addr(),
        MotionMode::UqUdExt as u32,
    )
    .await?;

    backend.delay_ms(u64::from(point.settle_ms)).await;

    let mut observation = sample_point_observation(backend, profile).await?;
    point_status_flags_or |=
        sample_status(backend, profile, warning_flags, status_flags_or).await?;

    let start_us = backend.timestamp_us();
    let mut raw_samples = [ImpedanceSample {
        t_us: 0,
        openloop_phi: 0,
        i_ch0: 0,
        i_ch1: 0,
    }; IMPEDANCE_SAMPLE_COUNT];
    let mut transformed_samples = [ImpedanceSample {
        t_us: 0,
        openloop_phi: 0,
        i_ch0: 0,
        i_ch1: 0,
    }; IMPEDANCE_SAMPLE_COUNT];
    let sample_count = profile.sample_count as usize;
    for index in 0..sample_count {
        let openloop_phi = read(backend, OPENLOOP_PHI.addr()).await? as u16;
        let raw_packed = read(backend, ADC_IWY_IUX.addr()).await?;
        let transformed_packed = read(backend, PID_TORQUE_FLUX_ACTUAL.addr()).await?;
        let t_us = backend.timestamp_us().wrapping_sub(start_us);
        let raw_sample = decode_adc_sample(t_us, openloop_phi, raw_packed);
        check_current_limit(profile, raw_sample)?;
        raw_samples[index] = raw_sample;
        transformed_samples[index] = decode_adc_sample(t_us, openloop_phi, transformed_packed);
        point_status_flags_or |=
            sample_status(backend, profile, warning_flags, status_flags_or).await?;
        if index + 1 < sample_count {
            backend
                .delay_us(u32::from(profile.sample_interval_us))
                .await;
        }
    }

    let encoder_after = read(backend, ABN_DECODER_COUNT.addr()).await?;
    check_encoder_delta(profile, encoder_before, encoder_after, warning_flags)?;
    observation.status_flags_or = point_status_flags_or;

    Ok(ImpedanceCapture {
        raw_samples,
        transformed_samples,
        observation,
    })
}

async fn capture_fixed_angle_window<B: ImpedanceBackend>(
    backend: &mut B,
    profile: &PulsatingImpedanceProfile,
    point: PulsatingImpedancePoint,
    warning_flags: &mut ImpedanceWarningFlags,
    status_flags_or: &mut u32,
) -> Result<PulsatingAngleObservation, ImpedanceError> {
    let encoder_before = read(backend, ABN_DECODER_COUNT.addr()).await?;
    let mut encoder_min_delta = 0i32;
    let mut encoder_max_delta = 0i32;
    let baseline = sample_zero_voltage_baseline(backend).await?;

    write(backend, UQ_UD_EXT.addr(), 0).await?;
    write(
        backend,
        MODE_RAMP_MODE_MOTION.addr(),
        MotionMode::Stopped as u32,
    )
    .await?;
    write(backend, PHI_E_SELECTION.addr(), PHI_E_EXTERNAL).await?;
    write(
        backend,
        PHI_E_EXT.addr(),
        u32::from(point.angle_electrical_counts),
    )
    .await?;
    verify_readback(backend, PHI_E_SELECTION.addr(), PHI_E_EXTERNAL).await?;
    verify_readback(
        backend,
        PHI_E_EXT.addr(),
        u32::from(point.angle_electrical_counts),
    )
    .await?;
    write(
        backend,
        MODE_RAMP_MODE_MOTION.addr(),
        MotionMode::UqUdExt as u32,
    )
    .await?;

    let mut accumulator =
        PulsatingAngleAccumulator::new(point.angle_electrical_counts, point.ud_abs);
    let sample_count = profile.sample_count as usize;
    let samples_per_cycle = profile.samples_per_cycle as usize;
    for index in 0..sample_count {
        let carrier_ud = carrier_ud_for_sample(index, samples_per_cycle, point.ud_abs);
        write(backend, UQ_UD_EXT.addr(), pack_ud(carrier_ud)).await?;
        let t_us = backend.timestamp_us();
        let transformed = decode_transformed_current_sample(
            t_us,
            read(backend, PID_TORQUE_FLUX_ACTUAL.addr()).await?,
        );
        let raw = decode_current_pair(t_us, read(backend, ADC_IWY_IUX.addr()).await?);
        check_pulsating_current_limit(profile, raw)?;
        accumulator.accumulate(transformed, raw);

        let encoder = read(backend, ABN_DECODER_COUNT.addr()).await?;
        update_encoder_delta_bounds(
            encoder_before,
            encoder,
            &mut encoder_min_delta,
            &mut encoder_max_delta,
        );
        let peak = encoder_peak_from_delta_span(encoder_min_delta, encoder_max_delta);
        if peak > profile.encoder_move_fail_counts {
            let _ = write(backend, UQ_UD_EXT.addr(), 0).await;
            return Err(ImpedanceError::EncoderMoved);
        }
        if peak > profile.encoder_move_warn_counts {
            warning_flags.insert(ImpedanceWarningFlags::ENCODER_MOVED);
        }

        let mut point_warning_flags = ImpedanceWarningFlags::NONE;
        let _ =
            sample_pulsating_status(backend, profile, &mut point_warning_flags, status_flags_or)
                .await?;
        warning_flags.insert(point_warning_flags);
        accumulator.insert_warning_flags(point_warning_flags);

        if index + 1 < sample_count {
            backend
                .delay_us(u32::from(profile.sample_interval_us))
                .await;
        }
    }

    write(backend, UQ_UD_EXT.addr(), 0).await?;
    let encoder_after = read(backend, ABN_DECODER_COUNT.addr()).await?;
    update_encoder_delta_bounds(
        encoder_before,
        encoder_after,
        &mut encoder_min_delta,
        &mut encoder_max_delta,
    );
    let observation = accumulator.finish(
        profile,
        baseline,
        encoder_before,
        encoder_after,
        encoder_before.wrapping_add(encoder_min_delta as u32),
        encoder_before.wrapping_add(encoder_max_delta as u32),
    );
    warning_flags.insert(observation.warning_flags);
    Ok(observation)
}

fn carrier_ud_for_sample(index: usize, samples_per_cycle: usize, ud_abs: u16) -> i16 {
    let half_cycle = samples_per_cycle / 2;
    let cycle_index = index % samples_per_cycle;
    if cycle_index < half_cycle {
        ud_abs as i16
    } else {
        -(ud_abs as i16)
    }
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

async fn sample_pulsating_status<B: ImpedanceBackend>(
    backend: &mut B,
    profile: &PulsatingImpedanceProfile,
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

async fn sample_point_observation<B: ImpedanceBackend>(
    backend: &mut B,
    profile: &ImpedanceProfile,
) -> Result<ImpedancePointObservation, ImpedanceError> {
    let openloop_phi_sample = read(backend, OPENLOOP_PHI.addr()).await? as u16;
    let openloop_velocity_actual = read(backend, OPENLOOP_VELOCITY_ACTUAL.addr()).await? as i32;
    let adc_sample = decode_adc_sample(0, 0, read(backend, ADC_IWY_IUX.addr()).await?);
    check_current_limit(profile, adc_sample)?;
    let (foc_iwy_sample, foc_iux_sample) =
        split_signed_halves(read_sub(backend, INTERIM_FOC_IWY_IUX).await?);
    let (foc_uq_sample, foc_ud_sample) =
        split_signed_halves(read_sub(backend, INTERIM_FOC_UQ_UD).await?);
    let (foc_uq_limited_sample, foc_ud_limited_sample) =
        split_signed_halves(read_sub(backend, INTERIM_FOC_UQ_UD_LIMITED).await?);
    let (foc_iq_sample, foc_id_sample) =
        split_signed_halves(read_sub(backend, INTERIM_FOC_IQ_ID).await?);

    Ok(ImpedancePointObservation {
        status_flags_or: 0,
        openloop_phi_sample,
        openloop_velocity_actual,
        adc_iux_sample: adc_sample.i_ch0,
        adc_iwy_sample: adc_sample.i_ch1,
        foc_iux_sample,
        foc_iwy_sample,
        foc_uq_sample,
        foc_ud_sample,
        foc_uq_limited_sample,
        foc_ud_limited_sample,
        foc_iq_sample,
        foc_id_sample,
    })
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

fn pack_ud(ud: i16) -> u32 {
    UqUdExt::new().with_ud_ext(ud).with_uq_ext(0).into()
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
) -> Result<PulsatingBaselineObservation, ImpedanceError> {
    let (q_count, d_count) =
        split_signed_halves(read(backend, PID_TORQUE_FLUX_ACTUAL.addr()).await?);
    let (adc_ch1_count, adc_ch0_count) =
        split_signed_halves(read(backend, ADC_IWY_IUX.addr()).await?);

    Ok(PulsatingBaselineObservation {
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

fn check_pulsating_current_limit(
    profile: &PulsatingImpedanceProfile,
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
    min_delta: &mut i32,
    max_delta: &mut i32,
) {
    let delta = signed_encoder_delta(before, current);
    *min_delta = (*min_delta).min(delta);
    *max_delta = (*max_delta).max(delta);
}

fn signed_encoder_delta(before: u32, current: u32) -> i32 {
    current.wrapping_sub(before) as i32
}

fn encoder_peak_counts(before: u32, min: u32, max: u32) -> u16 {
    let min_delta = signed_encoder_delta(before, min);
    let max_delta = signed_encoder_delta(before, max);
    encoder_peak_from_delta_span(min_delta, max_delta)
}

fn encoder_peak_from_delta_span(min_delta: i32, max_delta: i32) -> u16 {
    i64::from(max_delta)
        .saturating_sub(i64::from(min_delta))
        .unsigned_abs()
        .min(u64::from(u16::MAX)) as u16
}

/// Streaming lock-in accumulator for one fixed-angle pulsating carrier window.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct PulsatingAngleAccumulator {
    angle_electrical_counts: u16,
    ud_abs: u16,
    sample_count: u32,
    d_sin_sum: i64,
    d_cos_sum: i64,
    raw_sin_sum: i64,
    raw_cos_sum: i64,
    warning_flags: ImpedanceWarningFlags,
}

impl PulsatingAngleAccumulator {
    /// Create a streaming accumulator for one fixed electrical angle.
    pub fn new(angle_electrical_counts: u16, ud_abs: u16) -> Self {
        Self {
            angle_electrical_counts,
            ud_abs,
            sample_count: 0,
            d_sin_sum: 0,
            d_cos_sum: 0,
            raw_sin_sum: 0,
            raw_cos_sum: 0,
            warning_flags: ImpedanceWarningFlags::NONE,
        }
    }

    /// Add one transformed/raw current sample pair to the lock-in sums.
    pub fn accumulate(&mut self, transformed: ImpedanceSample, raw: ImpedanceSample) {
        let (cos_q10, sin_q10) =
            CARRIER_REF_Q10[self.sample_count as usize % CARRIER_REF_Q10.len()];
        self.d_sin_sum += i64::from(transformed.i_ch0) * i64::from(sin_q10);
        self.d_cos_sum += i64::from(transformed.i_ch0) * i64::from(cos_q10);
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
        profile: &PulsatingImpedanceProfile,
        baseline: PulsatingBaselineObservation,
        encoder_before: u32,
        encoder_after: u32,
        encoder_min: u32,
        encoder_max: u32,
    ) -> PulsatingAngleObservation {
        let encoder_peak_counts = encoder_peak_counts(encoder_before, encoder_min, encoder_max);
        let mut warning_flags = self.warning_flags;
        if encoder_peak_counts > profile.encoder_angle_invalid_counts {
            warning_flags.insert(ImpedanceWarningFlags::ANGLE_LOCAL_MOTION);
        }
        if encoder_peak_counts > profile.encoder_move_warn_counts {
            warning_flags.insert(ImpedanceWarningFlags::ENCODER_MOVED);
        }

        PulsatingAngleObservation {
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
            encoder_before,
            encoder_after,
            encoder_min,
            encoder_max,
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
