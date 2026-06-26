use super::profile::{ImpedanceProfile, frequency_millihz_to_openloop_velocity_rpm};
use super::runner::{
    ImpedanceBackend, ImpedanceBackendError, PulsatingAngleAccumulator,
    run_impedance_identification, run_pulsating_impedance_identification,
};
use super::types::{
    IMPEDANCE_PROFILE_POINTS, IMPEDANCE_SAMPLE_COUNT, ImpedanceError, ImpedanceInjectionPoint,
    ImpedanceSample, ImpedanceWarningFlags, PulsatingAngleObservation,
    PulsatingBaselineObservation,
};
use super::{
    ImpedanceFitPoint, PULSATING_IMPEDANCE_PROFILE_POINTS, PULSATING_IMPEDANCE_SAMPLE_COUNT,
    PulsatingImpedancePoint, PulsatingImpedanceProfile, evaluate_impedance_result,
    fit_impedance_point, fit_impedance_point_with_transformed_current, fit_pulsating_impedance,
};
use foci_tmc4671::registers::{
    ABN_DECODER_COUNT, ADC_IWY_IUX, CONFIG_BIQUAD_F_ENABLE, CONFIG_BIQUAD_T_ENABLE,
    DSADC_MDEC_B_MDEC_A, INTERIM_FOC_IQ_ID, INTERIM_FOC_IWY_IUX, INTERIM_FOC_UQ_UD,
    INTERIM_FOC_UQ_UD_LIMITED, MODE_RAMP_MODE_MOTION, MOTOR_TYPE_N_POLE_PAIRS, MotorType,
    MotorTypeNPolePairs, OPENLOOP_ACCELERATION, OPENLOOP_MODE, OPENLOOP_PHI,
    OPENLOOP_VELOCITY_ACTUAL, OPENLOOP_VELOCITY_TARGET, PHI_E_EXT, PHI_E_SELECTION,
    PID_TORQUE_FLUX_ACTUAL, PIDOUT_UQ_UD_LIMITS, PWM_BBM_H_BBM_L, PWM_MAXCNT, PWM_SV_CHOP,
    STATUS_FLAGS, SubRegister, UQ_UD_EXT,
};
use pollster::block_on;

const TURN: f64 = core::f64::consts::PI * 2.0;

fn openloop_phi_from_time(frequency_millihz: u32, t_us: u32) -> u16 {
    let counts = u128::from(frequency_millihz)
        .saturating_mul(u128::from(t_us))
        .saturating_mul(65_536)
        / 1_000_000_000u128;
    (counts & 0xffff) as u16
}

fn test_point(frequency_millihz: u32, ud_abs: u16) -> ImpedanceInjectionPoint {
    ImpedanceInjectionPoint {
        ud_abs,
        frequency_millihz,
        openloop_acceleration_rpm_per_s: 3_000,
        settle_ms: 250,
    }
}

fn synthetic_pulsating_observations(
    profile: &PulsatingImpedanceProfile,
    mean: u32,
    saliency: u32,
    axis_angle: u16,
    residual: u16,
) -> [PulsatingAngleObservation; PULSATING_IMPEDANCE_PROFILE_POINTS] {
    core::array::from_fn(|index| {
        let angle = profile.angle_points[index].angle_electrical_counts;
        let theta = core::f64::consts::TAU * f64::from(angle.wrapping_sub(axis_angle)) / 65_536.0;
        let response = f64::from(mean) + f64::from(saliency) * libm::cos(2.0 * theta);
        PulsatingAngleObservation {
            angle_electrical_counts: angle,
            ud_abs: profile.angle_points[index].ud_abs,
            inductive_response_count_per_ud_milli: response.round() as u32,
            in_phase_response_count_per_ud_milli: mean / 2,
            secondary_adc_response_count_per_ud_milli: response.round() as u32,
            baseline_d_count: 0,
            baseline_q_count: 0,
            baseline_adc_ch0_count: 0,
            baseline_adc_ch1_count: 0,
            residual_permille: residual,
            coverage_permille: 1000,
            encoder_before: 0,
            encoder_after: 0,
            encoder_min: 0,
            encoder_max: 0,
            encoder_peak_counts: 0,
            valid: true,
            warning_flags: ImpedanceWarningFlags::NONE,
        }
    })
}

fn response_sample(
    point: ImpedanceInjectionPoint,
    t_us: u32,
    axis_max_count_per_ud_milli: f64,
    axis_min_count_per_ud_milli: f64,
    angle_rad: f64,
    scalar_2f_distortion_count: f64,
) -> ImpedanceSample {
    response_sample_with_offsets(
        point,
        t_us,
        axis_max_count_per_ud_milli,
        axis_min_count_per_ud_milli,
        angle_rad,
        scalar_2f_distortion_count,
        17.0,
        -23.0,
    )
}

fn response_sample_with_offsets(
    point: ImpedanceInjectionPoint,
    t_us: u32,
    axis_max_count_per_ud_milli: f64,
    axis_min_count_per_ud_milli: f64,
    angle_rad: f64,
    scalar_2f_distortion_count: f64,
    offset_ch0: f64,
    offset_ch1: f64,
) -> ImpedanceSample {
    let phase = TURN * f64::from(point.frequency_millihz) * f64::from(t_us) / 1_000_000_000.0;
    let major = axis_max_count_per_ud_milli * f64::from(point.ud_abs) / 1_000.0;
    let minor = axis_min_count_per_ud_milli * f64::from(point.ud_abs) / 1_000.0;
    let major_signal = major * libm::cos(phase);
    let minor_signal = minor * libm::sin(phase);
    let cos_angle = libm::cos(angle_rad);
    let sin_angle = libm::sin(angle_rad);
    let distortion = scalar_2f_distortion_count * libm::cos(phase * 2.0);
    let i_ch0 = offset_ch0 + major_signal * cos_angle - minor_signal * sin_angle + distortion;
    let i_ch1 = offset_ch1 + major_signal * sin_angle + minor_signal * cos_angle;

    ImpedanceSample {
        t_us,
        openloop_phi: openloop_phi_from_time(point.frequency_millihz, t_us),
        i_ch0: i_ch0.round() as i16,
        i_ch1: i_ch1.round() as i16,
    }
}

fn response_sample_at_phi(
    point: ImpedanceInjectionPoint,
    t_us: u32,
    openloop_phi: u16,
    axis_max_count_per_ud_milli: f64,
    axis_min_count_per_ud_milli: f64,
    angle_rad: f64,
) -> ImpedanceSample {
    let phase = TURN * f64::from(openloop_phi) / 65_536.0;
    let major = axis_max_count_per_ud_milli * f64::from(point.ud_abs) / 1_000.0;
    let minor = axis_min_count_per_ud_milli * f64::from(point.ud_abs) / 1_000.0;
    let major_signal = major * libm::cos(phase);
    let minor_signal = minor * libm::sin(phase);
    let cos_angle = libm::cos(angle_rad);
    let sin_angle = libm::sin(angle_rad);

    ImpedanceSample {
        t_us,
        openloop_phi,
        i_ch0: (17.0 + major_signal * cos_angle - minor_signal * sin_angle).round() as i16,
        i_ch1: (-23.0 + major_signal * sin_angle + minor_signal * cos_angle).round() as i16,
    }
}

fn single_period_samples(
    point: ImpedanceInjectionPoint,
    axis_max_count_per_ud_milli: f64,
    axis_min_count_per_ud_milli: f64,
    angle_rad: f64,
    scalar_2f_distortion_count: f64,
) -> [ImpedanceSample; IMPEDANCE_SAMPLE_COUNT] {
    single_period_samples_with_offsets(
        point,
        axis_max_count_per_ud_milli,
        axis_min_count_per_ud_milli,
        angle_rad,
        scalar_2f_distortion_count,
        17.0,
        -23.0,
    )
}

fn single_period_samples_with_offsets(
    point: ImpedanceInjectionPoint,
    axis_max_count_per_ud_milli: f64,
    axis_min_count_per_ud_milli: f64,
    angle_rad: f64,
    scalar_2f_distortion_count: f64,
    offset_ch0: f64,
    offset_ch1: f64,
) -> [ImpedanceSample; IMPEDANCE_SAMPLE_COUNT] {
    let period_us = 1_000_000_000u64 / u64::from(point.frequency_millihz);
    core::array::from_fn(|index| {
        let t_us = ((index as u64) * period_us / IMPEDANCE_SAMPLE_COUNT as u64) as u32;
        response_sample_with_offsets(
            point,
            t_us,
            axis_max_count_per_ud_milli,
            axis_min_count_per_ud_milli,
            angle_rad,
            scalar_2f_distortion_count,
            offset_ch0,
            offset_ch1,
        )
    })
}

fn profile_cadence_samples(
    profile: &ImpedanceProfile,
    point: ImpedanceInjectionPoint,
    axis_max_count_per_ud_milli: f64,
    axis_min_count_per_ud_milli: f64,
    angle_rad: f64,
) -> [ImpedanceSample; IMPEDANCE_SAMPLE_COUNT] {
    core::array::from_fn(|index| {
        let t_us = index as u32 * u32::from(profile.sample_interval_us);
        response_sample(
            point,
            t_us,
            axis_max_count_per_ud_milli,
            axis_min_count_per_ud_milli,
            angle_rad,
            0.0,
        )
    })
}

fn transformed_magnitude_samples(
    profile: &ImpedanceProfile,
    point: ImpedanceInjectionPoint,
    mean_count_per_ud_milli: f64,
    ripple_2f_count_per_ud_milli: f64,
) -> [ImpedanceSample; IMPEDANCE_SAMPLE_COUNT] {
    core::array::from_fn(|index| {
        let t_us = index as u32 * u32::from(profile.sample_interval_us);
        let phase_2f =
            2.0 * TURN * f64::from(point.frequency_millihz) * f64::from(t_us) / 1_000_000_000.0;
        let mean = mean_count_per_ud_milli * f64::from(point.ud_abs) / 1_000.0;
        let ripple = ripple_2f_count_per_ud_milli * f64::from(point.ud_abs) / 1_000.0;
        let magnitude = mean + ripple * libm::cos(phase_2f);
        ImpedanceSample {
            t_us,
            openloop_phi: openloop_phi_from_time(point.frequency_millihz, t_us),
            i_ch0: magnitude.round() as i16,
            i_ch1: 0,
        }
    })
}

fn unwrap_fit(result: Result<ImpedanceFitPoint, ImpedanceError>) -> ImpedanceFitPoint {
    match result {
        Ok(fit) => fit,
        Err(error) => panic!("fit failed: {:?}", error),
    }
}

fn aggregate_fit(
    axis_max_count_per_ud_milli: u32,
    axis_min_count_per_ud_milli: u32,
    warning_flags: ImpedanceWarningFlags,
) -> ImpedanceFitPoint {
    ImpedanceFitPoint {
        axis_max_count_per_ud_milli,
        axis_min_count_per_ud_milli,
        axis_angle_electrical_counts: 0,
        residual_permille: 0,
        coverage_permille: 1000,
        scalar_2f_mismatch_permille: 0,
        magnitude_mean_count_per_ud_milli: 0,
        magnitude_2f_count_per_ud_milli: 0,
        magnitude_2f_residual_permille: 0,
        magnitude_2f_coverage_permille: 0,
        warning_flags,
    }
}

fn aggregate_fits(
    first_axis_max: u32,
    first_axis_min: u32,
    second_axis_max: u32,
    second_axis_min: u32,
) -> [ImpedanceFitPoint; IMPEDANCE_PROFILE_POINTS] {
    [
        aggregate_fit(first_axis_max, first_axis_min, ImpedanceWarningFlags::NONE),
        aggregate_fit(
            second_axis_max,
            second_axis_min,
            ImpedanceWarningFlags::NONE,
        ),
    ]
}

#[test]
fn standard_profile_v2_uses_fixed_angle_pulsating_shape() {
    let profile = PulsatingImpedanceProfile::standard_openffboard();

    assert_eq!(profile.profile_version, 2);
    assert_eq!(profile.carrier_frequency_millihz, 100_000);
    assert_eq!(profile.samples_per_cycle, 20);
    assert_eq!(profile.cycles_per_angle, 8);
    assert_eq!(profile.sample_interval_us, 500);
    assert_eq!(
        profile.sample_count as usize,
        PULSATING_IMPEDANCE_SAMPLE_COUNT
    );
    assert_eq!(PULSATING_IMPEDANCE_SAMPLE_COUNT, 160);
    assert_eq!(PULSATING_IMPEDANCE_PROFILE_POINTS, 9);
    assert_eq!(
        profile
            .angle_points
            .map(|point| point.angle_electrical_counts),
        [0, 8192, 16384, 24576, 32768, 40960, 49152, 57344, 0]
    );
    assert!(profile.angle_points.iter().all(|point| point.ud_abs == 640));
    assert_eq!(profile.max_abs_ud(), 640);
}

#[test]
fn pulsating_fit_recovers_two_theta_saliency() {
    let profile = PulsatingImpedanceProfile::standard_openffboard();
    let observations = synthetic_pulsating_observations(&profile, 80, 24, 0, 0);

    let summary = fit_pulsating_impedance(&profile, &observations).unwrap();

    assert!(summary.mean_response_count_per_ud_milli.abs_diff(80) <= 2);
    assert!(summary.axis_max_count_per_ud_milli.abs_diff(104) <= 3);
    assert!(summary.axis_min_count_per_ud_milli.abs_diff(56) <= 3);
    assert!(summary.saliency_determinable);
    assert!(summary.axis_angle_electrical_counts.abs_diff(0) <= 256);
}

#[test]
fn pulsating_fit_warns_when_axis_is_undetermined() {
    let profile = PulsatingImpedanceProfile::standard_openffboard();
    let observations = synthetic_pulsating_observations(&profile, 80, 2, 0, 0);

    let summary = fit_pulsating_impedance(&profile, &observations).unwrap();

    assert!(!summary.saliency_determinable);
    assert!(
        summary
            .warning_flags
            .contains(ImpedanceWarningFlags::AXIS_UNDETERMINED)
    );
}

#[test]
fn pulsating_fit_rejects_clustered_angle_coverage() {
    let profile = PulsatingImpedanceProfile::standard_openffboard();
    let mut observations = synthetic_pulsating_observations(&profile, 80, 24, 0, 0);
    observations[2].valid = false;
    observations[6].valid = false;

    let err = fit_pulsating_impedance(&profile, &observations).unwrap_err();

    assert_eq!(err, ImpedanceError::PoorCoverage);
}

#[test]
fn pulsating_fit_rejects_determinable_axis_deviation() {
    let profile = PulsatingImpedanceProfile::standard_openffboard();
    let observations = synthetic_pulsating_observations(&profile, 80, 24, 8192, 0);

    let err = fit_pulsating_impedance(&profile, &observations).unwrap_err();

    assert_eq!(err, ImpedanceError::AxisDeviation);
}

#[test]
fn pulsating_fit_rejects_repeatability_failure() {
    let profile = PulsatingImpedanceProfile::standard_openffboard();
    let mut observations = synthetic_pulsating_observations(&profile, 80, 24, 0, 0);
    observations[PULSATING_IMPEDANCE_PROFILE_POINTS - 1].inductive_response_count_per_ud_milli = 60;

    let err = fit_pulsating_impedance(&profile, &observations).unwrap_err();

    assert_eq!(err, ImpedanceError::RepeatabilityMismatch);
}

#[test]
fn pulsating_fit_warns_on_repeatability_warning_band() {
    let profile = PulsatingImpedanceProfile::standard_openffboard();
    let mut observations = synthetic_pulsating_observations(&profile, 80, 24, 0, 0);
    observations[PULSATING_IMPEDANCE_PROFILE_POINTS - 1].inductive_response_count_per_ud_milli = 92;

    let summary = fit_pulsating_impedance(&profile, &observations).unwrap();

    assert!(
        summary
            .warning_flags
            .contains(ImpedanceWarningFlags::REPEATABILITY)
    );
}

#[test]
fn pulsating_fit_rejects_high_residual_failure() {
    let profile = PulsatingImpedanceProfile::standard_openffboard();
    let observations = synthetic_pulsating_observations(&profile, 80, 24, 0, 800);

    let err = fit_pulsating_impedance(&profile, &observations).unwrap_err();

    assert_eq!(err, ImpedanceError::HighResidual);
}

#[test]
fn pulsating_fit_warns_on_high_residual_warning_band() {
    let profile = PulsatingImpedanceProfile::standard_openffboard();
    let observations = synthetic_pulsating_observations(&profile, 80, 24, 0, 300);

    let summary = fit_pulsating_impedance(&profile, &observations).unwrap();

    assert!(
        summary
            .warning_flags
            .contains(ImpedanceWarningFlags::HIGH_RESIDUAL)
    );
}

#[test]
fn standard_profile_v2_enforces_carrier_exposure_budget() {
    let profile = PulsatingImpedanceProfile::standard_openffboard();

    assert_eq!(profile.nominal_carrier_window_ms(), 80);
    assert_eq!(profile.energized_window_ms(), 160);
    assert_eq!(profile.max_window_exposure(), 640 * 640 * 160);
    assert_eq!(profile.max_total_exposure(), 640 * 640 * 9 * 160);
    assert!(profile.max_window_exposure() <= profile.max_window_exposure_budget);
    assert!(profile.max_total_exposure() <= profile.max_total_exposure_budget);
}

#[test]
fn profile_v2_rejects_ud_960_without_budget_review() {
    let mut profile = PulsatingImpedanceProfile::standard_openffboard();
    for point in &mut profile.angle_points {
        point.ud_abs = 960;
    }

    assert!(profile.max_total_exposure() > profile.max_total_exposure_budget);
}

#[test]
fn profile_v2_zero_carrier_frequency_saturates_nominal_window() {
    let mut profile = PulsatingImpedanceProfile::standard_openffboard();
    profile.carrier_frequency_millihz = 0;

    assert_eq!(profile.nominal_carrier_window_ms(), u64::MAX);
}

#[test]
fn profile_v2_point_type_is_public_at_module_boundary() {
    let point = PulsatingImpedancePoint {
        angle_electrical_counts: 8192,
        ud_abs: 640,
    };

    assert_eq!(point.angle_electrical_counts, 8192);
    assert_eq!(point.ud_abs, 640);
}

#[test]
fn legacy_rotating_profile_still_compiles_for_regression_tests() {
    let profile = ImpedanceProfile::standard_openffboard();

    assert_eq!(profile.profile_version, 1);
    assert_eq!(IMPEDANCE_SAMPLE_COUNT, 128);
    assert_eq!(IMPEDANCE_PROFILE_POINTS, 2);
    assert_eq!(profile.injection_points[0].frequency_millihz, 25_000);
}

#[test]
fn standard_profile_is_bounded_and_physical_projection_is_not_validated_by_default() {
    let profile = ImpedanceProfile::standard_openffboard();

    assert_eq!(profile.profile_version, 1);
    assert_eq!(profile.sample_interval_us, 1000);
    assert!(profile.sample_count >= 64);
    assert!(profile.sample_count <= profile.max_sample_count);
    assert_eq!(profile.injection_points.len(), 2);
    assert!(profile.max_abs_ud() > 0);
    assert!(profile.max_window_exposure() <= profile.max_window_exposure_budget);
    assert!(profile.max_total_exposure() <= profile.max_total_exposure_budget);
    assert!(!profile.scale_metadata_validated);
    assert_eq!(
        profile.injection_points[0].frequency_millihz,
        profile.injection_points[1].frequency_millihz,
        "standard repeatability must compare amplitudes at one frequency"
    );
    assert_ne!(
        profile.injection_points[0].ud_abs, profile.injection_points[1].ud_abs,
        "standard repeatability should compare distinct amplitudes"
    );
    for point in profile.injection_points {
        let openloop_velocity_rpm =
            frequency_millihz_to_openloop_velocity_rpm(u64::from(point.frequency_millihz), 50);
        assert_eq!(openloop_velocity_rpm, 30);
        assert!(point.openloop_acceleration_rpm_per_s > 0);
        assert!(
            u64::from(point.openloop_acceleration_rpm_per_s)
                .saturating_mul(u64::from(point.settle_ms))
                >= (openloop_velocity_rpm as u64).saturating_mul(1000),
            "open-loop acceleration should reach target before capture starts"
        );
    }
}

#[test]
fn standard_profile_uses_explicit_status_masks() {
    let profile = ImpedanceProfile::standard_openffboard();

    assert_ne!(profile.status_flags_fail_mask, 0);
    assert_ne!(profile.status_flags_fail_mask, u32::MAX);
    assert_ne!(profile.status_flags_warn_mask, u32::MAX);
}

#[test]
fn standard_profile_sample_count_controls_capture_size() {
    let profile = ImpedanceProfile::standard_openffboard();

    assert_eq!(profile.sample_count, 128);
    assert_eq!(profile.sample_count as usize, IMPEDANCE_SAMPLE_COUNT);
}

#[test]
fn standard_profile_uses_observation_capture_exposure_budget() {
    let profile = ImpedanceProfile::standard_openffboard();

    assert_eq!(profile.max_window_exposure_budget, 640_000_000);
    assert_eq!(profile.max_total_exposure_budget, 925_000_000);
}

#[test]
fn exposure_budget_counts_dc_ac_settle_capture_and_sample_overhead() {
    let mut profile = ImpedanceProfile::standard_openffboard();
    profile.sample_count = 4;
    profile.max_sample_count = 4;
    profile.sample_interval_us = 130;
    profile.injection_points[0].ud_abs = 10;
    profile.injection_points[0].settle_ms = 3;
    profile.injection_points[1].ud_abs = 20;
    profile.injection_points[1].settle_ms = 5;

    // ceil(4 * (130 us + standard conservative overhead) / 1000) = 3 ms.
    // Each point contributes a DC settle window plus an AC settle, point
    // observation, and capture window. The per-window gate must count the full
    // exposure of one injection point.
    assert_eq!(profile.max_window_exposure(), 20 * 20 * (5 + 5 + 2 + 3));
    assert_eq!(
        profile.max_total_exposure(),
        10 * 10 * (3 + 3 + 2 + 3) + 20 * 20 * (5 + 5 + 2 + 3)
    );
}

#[test]
fn warning_flags_contains_requires_all_bits() {
    let flags = ImpedanceWarningFlags::HIGH_RESIDUAL;

    assert!(flags.contains(ImpedanceWarningFlags::HIGH_RESIDUAL));
    assert!(!flags.contains(ImpedanceWarningFlags(
        ImpedanceWarningFlags::HIGH_RESIDUAL.0 | ImpedanceWarningFlags::HIGH_NOISE.0,
    ),));
}

#[test]
fn warning_flags_intersects_reports_any_overlap() {
    let flags = ImpedanceWarningFlags(
        ImpedanceWarningFlags::HIGH_RESIDUAL.0 | ImpedanceWarningFlags::ENCODER_MOVED.0,
    );

    assert!(flags.intersects(ImpedanceWarningFlags::HIGH_RESIDUAL));
    assert!(!flags.intersects(ImpedanceWarningFlags::SCALAR_2F_MISMATCH));
}

#[test]
fn warning_flags_status_bit_is_distinct_and_nonzero() {
    assert_ne!(ImpedanceWarningFlags::STATUS_FLAGS.0, 0);
    assert_ne!(
        ImpedanceWarningFlags::STATUS_FLAGS.0,
        ImpedanceWarningFlags::ENCODER_MOVED.0
    );
    assert_ne!(
        ImpedanceWarningFlags::STATUS_FLAGS.0,
        ImpedanceWarningFlags::HIGH_NOISE.0
    );
}

#[test]
fn high_frequency_openloop_velocity_clamps_to_i32_max() {
    assert_eq!(
        frequency_millihz_to_openloop_velocity_rpm(u64::MAX, 1),
        i32::MAX
    );
}

#[test]
fn vector_lockin_recovers_non_circular_response() {
    let profile = ImpedanceProfile::standard_openffboard();
    let point = test_point(37_000, 800);
    let samples = single_period_samples(point, 900.0, 300.0, 0.62, 0.0);

    let fit = unwrap_fit(fit_impedance_point(&profile, point, &samples));

    assert!(fit.axis_max_count_per_ud_milli.abs_diff(900) <= 3);
    assert!(fit.axis_min_count_per_ud_milli.abs_diff(300) <= 3);
    assert!(fit.residual_permille <= 5);
    assert!(fit.coverage_permille >= 990);
}

#[test]
fn vector_lockin_rejects_low_signal() {
    let profile = ImpedanceProfile::standard_openffboard();
    let point = test_point(37_000, 800);
    let samples = single_period_samples(point, 20.0, 10.0, 0.0, 0.0);

    assert_eq!(
        fit_impedance_point(&profile, point, &samples),
        Err(ImpedanceError::LowSignal)
    );
}

#[test]
fn vector_lockin_rejects_poor_phase_coverage() {
    let profile = ImpedanceProfile::standard_openffboard();
    let point = test_point(37_000, 800);
    let sample = response_sample(point, 0, 900.0, 300.0, 0.0, 0.0);
    let samples = [sample; IMPEDANCE_SAMPLE_COUNT];

    assert_eq!(
        fit_impedance_point(&profile, point, &samples),
        Err(ImpedanceError::PoorCoverage)
    );
}

#[test]
fn vector_lockin_recovers_response_at_profile_cadence() {
    let mut profile = ImpedanceProfile::standard_openffboard();
    profile.sample_interval_us = 250;
    let point = test_point(250_000, 640);
    let samples = profile_cadence_samples(&profile, point, 700.0, 250.0, 0.31);

    let fit = unwrap_fit(fit_impedance_point(&profile, point, &samples));

    assert!(fit.axis_max_count_per_ud_milli.abs_diff(700) <= 3);
    assert!(fit.axis_min_count_per_ud_milli.abs_diff(250) <= 3);
    assert!(fit.coverage_permille >= 990);
}

#[test]
fn vector_lockin_uses_sampled_openloop_phi_when_timestamp_reference_drifts() {
    let profile = ImpedanceProfile::standard_openffboard();
    let point = profile.injection_points[0];
    let samples = core::array::from_fn(|index| {
        let t_us = index as u32 * u32::from(profile.sample_interval_us);
        let openloop_phi = ((index as u32 * 2_750) & 0xffff) as u16;
        response_sample_at_phi(point, t_us, openloop_phi, 700.0, 250.0, 0.31)
    });

    let fit = unwrap_fit(fit_impedance_point(&profile, point, &samples));

    assert!(fit.axis_max_count_per_ud_milli.abs_diff(700) <= 3);
    assert!(fit.axis_min_count_per_ud_milli.abs_diff(250) <= 3);
    assert!(fit.residual_permille <= 10);
    assert!(fit.coverage_permille >= 990);
}

#[test]
fn magnitude_2f_fit_recovers_transformed_current_ripple() {
    let mut profile = ImpedanceProfile::standard_openffboard();
    profile.min_signal_count = 1;
    let point = profile.injection_points[0];
    let raw_samples = profile_cadence_samples(&profile, point, 700.0, 250.0, 0.31);
    let transformed_samples = transformed_magnitude_samples(&profile, point, 840.0, 125.0);

    let fit = unwrap_fit(fit_impedance_point_with_transformed_current(
        &profile,
        point,
        &raw_samples,
        &transformed_samples,
    ));

    assert!(fit.magnitude_mean_count_per_ud_milli.abs_diff(840) <= 3);
    assert!(fit.magnitude_2f_count_per_ud_milli.abs_diff(125) <= 3);
    assert!(fit.magnitude_2f_residual_permille <= 10);
    assert!(fit.magnitude_2f_coverage_permille >= 990);
}

#[test]
fn scalar_2f_mismatch_clean_ellipse_stays_below_warn_threshold() {
    let profile = ImpedanceProfile::standard_openffboard();
    let point = test_point(37_000, 800);
    let samples = single_period_samples(point, 900.0, 300.0, 0.62, 0.0);

    let fit = unwrap_fit(fit_impedance_point(&profile, point, &samples));

    assert!(fit.scalar_2f_mismatch_permille < profile.scalar_2f_mismatch_warn_permille);
    assert!(
        !fit.warning_flags
            .contains(ImpedanceWarningFlags::SCALAR_2F_MISMATCH)
    );
}

#[test]
fn scalar_2f_mismatch_clean_ellipse_with_large_dc_offsets_stays_below_warn_threshold() {
    let profile = ImpedanceProfile::standard_openffboard();
    let point = test_point(37_000, 800);
    let samples =
        single_period_samples_with_offsets(point, 900.0, 300.0, 0.62, 0.0, 5_000.0, -4_000.0);

    let fit = unwrap_fit(fit_impedance_point(&profile, point, &samples));

    assert!(fit.scalar_2f_mismatch_permille < profile.scalar_2f_mismatch_warn_permille);
    assert!(
        !fit.warning_flags
            .contains(ImpedanceWarningFlags::SCALAR_2F_MISMATCH)
    );
}

#[test]
fn scalar_2f_mismatch_clean_circular_ellipse_with_large_dc_offsets_stays_below_warn_threshold() {
    let profile = ImpedanceProfile::standard_openffboard();
    let point = test_point(37_000, 800);
    let samples =
        single_period_samples_with_offsets(point, 600.0, 600.0, 0.62, 0.0, 5_000.0, -4_000.0);

    let fit = unwrap_fit(fit_impedance_point(&profile, point, &samples));

    assert!(fit.scalar_2f_mismatch_permille < profile.scalar_2f_mismatch_warn_permille);
    assert!(
        !fit.warning_flags
            .contains(ImpedanceWarningFlags::SCALAR_2F_MISMATCH)
    );
}

#[test]
fn scalar_2f_mismatch_clean_near_circular_ellipse_with_large_dc_offsets_stays_below_warn_threshold()
{
    let profile = ImpedanceProfile::standard_openffboard();
    let point = test_point(37_000, 800);
    let samples =
        single_period_samples_with_offsets(point, 608.0, 600.0, 0.62, 0.0, 5_000.0, -4_000.0);

    let fit = unwrap_fit(fit_impedance_point(&profile, point, &samples));

    assert!(fit.scalar_2f_mismatch_permille < profile.scalar_2f_mismatch_warn_permille);
    assert!(
        !fit.warning_flags
            .contains(ImpedanceWarningFlags::SCALAR_2F_MISMATCH)
    );
}

#[test]
fn scalar_2f_mismatch_distorted_high_saliency_still_warns_and_fails() {
    let mut warning_profile = ImpedanceProfile::standard_openffboard();
    warning_profile.residual_fail_permille = 1000;
    warning_profile.scalar_2f_mismatch_warn_permille = 20;
    warning_profile.scalar_2f_mismatch_fail_permille = 900;
    let point = test_point(37_000, 800);
    let samples = single_period_samples(point, 900.0, 300.0, 0.62, 250.0);

    let fit = unwrap_fit(fit_impedance_point(&warning_profile, point, &samples));
    assert!(
        fit.warning_flags
            .contains(ImpedanceWarningFlags::SCALAR_2F_MISMATCH)
    );

    let mut failing_profile = warning_profile;
    failing_profile.scalar_2f_mismatch_fail_permille = 20;
    assert_eq!(
        fit_impedance_point(&failing_profile, point, &samples),
        Err(ImpedanceError::Scalar2fMismatch)
    );
}

#[test]
fn scalar_2f_mismatch_sets_warning() {
    let mut profile = ImpedanceProfile::standard_openffboard();
    profile.residual_fail_permille = 1000;
    profile.scalar_2f_mismatch_warn_permille = 20;
    profile.scalar_2f_mismatch_fail_permille = 900;
    let point = test_point(37_000, 800);
    let samples = single_period_samples(point, 900.0, 300.0, 0.62, 250.0);

    let fit = unwrap_fit(fit_impedance_point(&profile, point, &samples));

    assert!(fit.scalar_2f_mismatch_permille >= profile.scalar_2f_mismatch_warn_permille);
    assert!(
        fit.warning_flags
            .contains(ImpedanceWarningFlags::SCALAR_2F_MISMATCH)
    );
}

#[test]
fn scalar_2f_mismatch_can_fail() {
    let mut profile = ImpedanceProfile::standard_openffboard();
    profile.residual_fail_permille = 1000;
    profile.scalar_2f_mismatch_fail_permille = 20;
    let point = test_point(37_000, 800);
    let samples = single_period_samples(point, 900.0, 300.0, 0.62, 250.0);

    assert_eq!(
        fit_impedance_point(&profile, point, &samples),
        Err(ImpedanceError::Scalar2fMismatch)
    );
}

#[test]
fn vector_lockin_gate_order_reports_poor_coverage_before_low_signal() {
    let profile = ImpedanceProfile::standard_openffboard();
    let point = test_point(37_000, 800);
    let samples = [ImpedanceSample {
        t_us: 0,
        openloop_phi: 0,
        i_ch0: 0,
        i_ch1: 0,
    }; IMPEDANCE_SAMPLE_COUNT];

    assert_eq!(
        fit_impedance_point(&profile, point, &samples),
        Err(ImpedanceError::PoorCoverage)
    );
}

#[test]
fn vector_lockin_gate_order_reports_low_signal_before_high_residual() {
    let mut profile = ImpedanceProfile::standard_openffboard();
    profile.residual_fail_permille = 1;
    let point = test_point(37_000, 800);
    let samples = single_period_samples(point, 20.0, 10.0, 0.0, 400.0);

    assert_eq!(
        fit_impedance_point(&profile, point, &samples),
        Err(ImpedanceError::LowSignal)
    );
}

#[test]
fn vector_lockin_gate_order_reports_high_residual_before_scalar_2f_mismatch() {
    let mut profile = ImpedanceProfile::standard_openffboard();
    profile.residual_fail_permille = 1;
    profile.scalar_2f_mismatch_fail_permille = 1;
    let point = test_point(37_000, 800);
    let samples = single_period_samples(point, 900.0, 300.0, 0.62, 250.0);

    assert_eq!(
        fit_impedance_point(&profile, point, &samples),
        Err(ImpedanceError::HighResidual)
    );
}

#[test]
fn aggregate_repeatability_accepts_two_similar_fit_points() {
    let mut profile = ImpedanceProfile::standard_openffboard();
    profile.scale_metadata_validated = true;
    let fits = aggregate_fits(900, 300, 910, 295);

    let summary = evaluate_impedance_result(&profile, &fits).expect("summary should pass");

    assert_eq!(summary.selected_axis_max_count_per_ud_milli, 905);
    assert_eq!(summary.selected_axis_min_count_per_ud_milli, 298);
    assert!(summary.repeatability_permille < profile.repeatability_warn_permille);
    assert_eq!(summary.warning_flags, ImpedanceWarningFlags::NONE);
}

#[test]
fn aggregate_repeatability_warning_sets_repeatability_flag() {
    let mut profile = ImpedanceProfile::standard_openffboard();
    profile.scale_metadata_validated = true;
    profile.repeatability_warn_permille = 20;
    profile.repeatability_fail_permille = 200;
    let fits = aggregate_fits(900, 300, 950, 300);

    let summary = evaluate_impedance_result(&profile, &fits).expect("summary should warn");

    assert!(summary.repeatability_permille > profile.repeatability_warn_permille);
    assert!(
        summary
            .warning_flags
            .contains(ImpedanceWarningFlags::REPEATABILITY)
    );
}

#[test]
fn aggregate_repeatability_failure_returns_repeatability_mismatch() {
    let mut profile = ImpedanceProfile::standard_openffboard();
    profile.repeatability_fail_permille = 100;
    let fits = aggregate_fits(900, 300, 1200, 300);

    assert_eq!(
        evaluate_impedance_result(&profile, &fits),
        Err(ImpedanceError::RepeatabilityMismatch)
    );
}

#[test]
fn aggregate_repeatability_failure_detects_minor_axis_mismatch() {
    let mut profile = ImpedanceProfile::standard_openffboard();
    profile.repeatability_fail_permille = 200;
    let fits = aggregate_fits(10_000, 100, 10_000, 200);

    assert_eq!(
        evaluate_impedance_result(&profile, &fits),
        Err(ImpedanceError::RepeatabilityMismatch)
    );
}

#[test]
fn aggregate_repeatability_ors_point_warning_flags() {
    let mut profile = ImpedanceProfile::standard_openffboard();
    profile.scale_metadata_validated = true;
    let fits = [
        aggregate_fit(900, 300, ImpedanceWarningFlags::HIGH_RESIDUAL),
        aggregate_fit(910, 295, ImpedanceWarningFlags::SCALAR_2F_MISMATCH),
    ];

    let summary = evaluate_impedance_result(&profile, &fits).expect("summary should pass");

    assert!(
        summary
            .warning_flags
            .contains(ImpedanceWarningFlags::HIGH_RESIDUAL)
    );
    assert!(
        summary
            .warning_flags
            .contains(ImpedanceWarningFlags::SCALAR_2F_MISMATCH)
    );
}

#[test]
fn aggregate_repeatability_sets_physical_projection_unavailable_without_scale_metadata() {
    let profile = ImpedanceProfile::standard_openffboard();
    let fits = aggregate_fits(900, 300, 910, 295);

    let summary = evaluate_impedance_result(&profile, &fits).expect("summary should pass");

    assert!(
        summary
            .warning_flags
            .contains(ImpedanceWarningFlags::PHYSICAL_PROJECTION_UNAVAILABLE)
    );
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct RecordedWrite {
    addr: u8,
    value: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct RecordedRead {
    addr: u8,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum RecordedOp {
    Read { addr: u8 },
    Write { addr: u8, value: u32 },
}

const AC_SAMPLE_TIMESTAMP_CAPACITY: usize = (IMPEDANCE_SAMPLE_COUNT + 1) * IMPEDANCE_PROFILE_POINTS;

struct FakeImpedanceBackend {
    writes: heapless::Vec<RecordedWrite, 2048>,
    reads: heapless::Vec<RecordedRead, 1024>,
    ops: heapless::Vec<RecordedOp, 4096>,
    profile: ImpedanceProfile,
    armed_for_open_loop_voltage: bool,
    motor_type: u8,
    n_pole_pairs: u16,
    saved_voltage_limit: u32,
    pidout_limit: u32,
    uq_ud_ext: u32,
    mode_ramp_mode_motion: u32,
    openloop_mode: u32,
    openloop_acceleration: u32,
    openloop_velocity_target: u32,
    openloop_velocity_actual: u32,
    openloop_phi: u32,
    phi_e_selection: u32,
    phi_e_ext: u32,
    pwm_sv_chop: u32,
    pwm_maxcnt_readback: u32,
    bbm_readback: u32,
    dsadc_mdec_readback: u32,
    interim_sub_addr: u8,
    config_sub_addr: u8,
    torque_filter_enable: u32,
    flux_filter_enable: u32,
    timestamp_us: u32,
    encoder_count: u32,
    scripted_encoder_counts: heapless::Vec<u32, 64>,
    scripted_encoder_index: usize,
    v2_angle_starts: usize,
    pulsating_encoder_peak_counts: Option<u32>,
    baseline_current_count: i16,
    status_flags: u32,
    readback_mismatch_addr: Option<u8>,
    dc_external_phi_e_selection_readback_mismatch: bool,
    dc_stopped_mode_readback_mismatch: bool,
    ac_stopped_mode_readback_mismatch: bool,
    ac_zero_voltage_readback_mismatch: bool,
    ac_openloop_acceleration_readback_mismatch: bool,
    ac_openloop_velocity_target_readback_mismatch: bool,
    stale_dynamic_openloop_state_when_stopped: bool,
    ac_openloop_mode_readback_mismatch: bool,
    ac_openloop_phi_advances_after_setup: bool,
    open_loop_phi_e_selection_readback_mismatch: bool,
    ac_over_current_sample: Option<usize>,
    fail_ac_adc_read_at: Option<usize>,
    low_signal_ud_abs: Option<u16>,
    restore_fail_addr: Option<u8>,
    restore_failure_armed: bool,
    readback_mismatch: Option<(u8, u32, u32)>,
    ac_adc_reads: usize,
    ac_transformed_reads: usize,
    ac_sample_timestamps: heapless::Vec<u32, AC_SAMPLE_TIMESTAMP_CAPACITY>,
    fit_attempts: [Option<ImpedanceFitPoint>; IMPEDANCE_PROFILE_POINTS],
    pulsating_observations: [Option<PulsatingAngleObservation>; PULSATING_IMPEDANCE_PROFILE_POINTS],
    encoder_delta_per_energized_delay: u32,
}

impl FakeImpedanceBackend {
    fn standard() -> Self {
        let profile = ImpedanceProfile::standard_openffboard();
        let saved_voltage_limit = 16_000;
        Self {
            writes: heapless::Vec::new(),
            reads: heapless::Vec::new(),
            ops: heapless::Vec::new(),
            profile,
            armed_for_open_loop_voltage: true,
            motor_type: MotorType::TwoPhase as u8,
            n_pole_pairs: 50,
            saved_voltage_limit,
            pidout_limit: saved_voltage_limit,
            uq_ud_ext: 0,
            mode_ramp_mode_motion: 0,
            openloop_mode: 0,
            openloop_acceleration: 0,
            openloop_velocity_target: 0,
            openloop_velocity_actual: 0,
            openloop_phi: 0,
            phi_e_selection: 1,
            phi_e_ext: 0,
            pwm_sv_chop: 0,
            pwm_maxcnt_readback: 3999,
            bbm_readback: 0x0909,
            dsadc_mdec_readback: 0x014B_014B,
            interim_sub_addr: 0,
            config_sub_addr: 0,
            torque_filter_enable: 0,
            flux_filter_enable: 0,
            timestamp_us: 0,
            encoder_count: 0,
            scripted_encoder_counts: heapless::Vec::new(),
            scripted_encoder_index: 0,
            v2_angle_starts: 0,
            pulsating_encoder_peak_counts: None,
            baseline_current_count: 0,
            status_flags: 0,
            readback_mismatch_addr: None,
            dc_external_phi_e_selection_readback_mismatch: false,
            dc_stopped_mode_readback_mismatch: false,
            ac_stopped_mode_readback_mismatch: false,
            ac_zero_voltage_readback_mismatch: false,
            ac_openloop_acceleration_readback_mismatch: false,
            ac_openloop_velocity_target_readback_mismatch: false,
            stale_dynamic_openloop_state_when_stopped: false,
            ac_openloop_mode_readback_mismatch: false,
            ac_openloop_phi_advances_after_setup: false,
            open_loop_phi_e_selection_readback_mismatch: false,
            ac_over_current_sample: None,
            fail_ac_adc_read_at: None,
            low_signal_ud_abs: None,
            restore_fail_addr: None,
            restore_failure_armed: false,
            readback_mismatch: None,
            ac_adc_reads: 0,
            ac_transformed_reads: 0,
            ac_sample_timestamps: heapless::Vec::new(),
            fit_attempts: [None; IMPEDANCE_PROFILE_POINTS],
            pulsating_observations: [None; PULSATING_IMPEDANCE_PROFILE_POINTS],
            encoder_delta_per_energized_delay: 0,
        }
    }

    fn wrote_nothing(&self) -> bool {
        self.writes.is_empty()
    }

    fn wrote_safe_state(&self) -> bool {
        let expected = [
            (UQ_UD_EXT.addr(), 0),
            (OPENLOOP_VELOCITY_TARGET.addr(), 0),
            (OPENLOOP_VELOCITY_ACTUAL.addr(), 0),
            (OPENLOOP_ACCELERATION.addr(), 0),
            (OPENLOOP_PHI.addr(), 0),
            (OPENLOOP_MODE.addr(), 0),
            (MODE_RAMP_MODE_MOTION.addr(), 0),
            (PHI_E_SELECTION.addr(), 0),
            (PWM_SV_CHOP.addr(), 0),
            (PIDOUT_UQ_UD_LIMITS.addr(), self.saved_voltage_limit),
        ];
        if self.writes.len() < expected.len() {
            return false;
        }
        let offset = self.writes.len() - expected.len();
        expected.iter().enumerate().all(|(index, (addr, value))| {
            self.writes[offset + index].addr == *addr && self.writes[offset + index].value == *value
        })
    }

    fn energized(&self) -> bool {
        self.mode_ramp_mode_motion == foci_tmc4671::registers::MotionMode::UqUdExt as u32
            && self.uq_ud_ext != 0
    }

    fn last_ud(&self) -> i16 {
        (self.uq_ud_ext & 0xffff) as u16 as i16
    }

    fn packed_current(ch0: i16, ch1: i16) -> u32 {
        ((ch1 as u16 as u32) << 16) | ch0 as u16 as u32
    }

    fn packed_halves(high: i16, low: i16) -> u32 {
        ((high as u16 as u32) << 16) | low as u16 as u32
    }

    fn ac_openloop_phi(&self) -> u16 {
        let frequency_millihz = self
            .openloop_velocity_target
            .saturating_mul(1000)
            .saturating_mul(u32::from(self.n_pole_pairs))
            / 60;
        openloop_phi_from_time(frequency_millihz, self.timestamp_us)
    }

    fn ac_current(&self) -> u32 {
        let phase = TURN * f64::from(self.ac_openloop_phi()) / 65_536.0;
        if self.low_signal_ud_abs == Some(self.last_ud().unsigned_abs()) {
            return Self::packed_current(0, 0);
        }
        let ud = f64::from(self.last_ud());
        let ch0 = (ud * libm::cos(phase)).round() as i16;
        let ch1 = ((ud / 2.0) * libm::sin(phase)).round() as i16;
        Self::packed_current(ch0, ch1)
    }

    fn readback_value(&self, addr: u8) -> u32 {
        let nonzero_uq_written = self.nonzero_uq_write_count() != 0;
        if Some(addr) == self.readback_mismatch_addr {
            return 0xffff_ffff;
        }
        if addr == PIDOUT_UQ_UD_LIMITS.addr() {
            return self.pidout_limit;
        }
        if addr == UQ_UD_EXT.addr() {
            if self.ac_zero_voltage_readback_mismatch && nonzero_uq_written && self.uq_ud_ext == 0 {
                return 0xffff_ffff;
            }
            return self.uq_ud_ext;
        }
        if addr == MODE_RAMP_MODE_MOTION.addr() {
            if self.dc_stopped_mode_readback_mismatch
                && !nonzero_uq_written
                && self.pwm_sv_chop == 7
            {
                return 0xffff_ffff;
            }
            if self.ac_stopped_mode_readback_mismatch
                && nonzero_uq_written
                && self.mode_ramp_mode_motion == 0
            {
                return 0xffff_ffff;
            }
            return self.mode_ramp_mode_motion;
        }
        if addr == OPENLOOP_MODE.addr() {
            if self.ac_openloop_mode_readback_mismatch
                && nonzero_uq_written
                && self.openloop_mode == 0
            {
                return 0xffff_ffff;
            }
            return self.openloop_mode;
        }
        if addr == OPENLOOP_ACCELERATION.addr() {
            if self.ac_openloop_acceleration_readback_mismatch
                && nonzero_uq_written
                && self.openloop_acceleration != 0
            {
                return 0xffff_ffff;
            }
            return self.openloop_acceleration;
        }
        if addr == OPENLOOP_VELOCITY_TARGET.addr() {
            if self.ac_openloop_velocity_target_readback_mismatch
                && nonzero_uq_written
                && self.openloop_velocity_target != 0
            {
                return 0xffff_ffff;
            }
            return self.openloop_velocity_target;
        }
        if addr == OPENLOOP_VELOCITY_ACTUAL.addr() {
            if self.openloop_velocity_target == 0 && self.stale_dynamic_openloop_state_when_stopped
            {
                return 1_499;
            }
            if self.phi_e_selection == 2 && self.openloop_velocity_target != 0 {
                return self.openloop_velocity_target;
            }
            return self.openloop_velocity_actual;
        }
        if addr == OPENLOOP_PHI.addr() {
            if self.openloop_velocity_target == 0 && self.stale_dynamic_openloop_state_when_stopped
            {
                return 0x61c8;
            }
            if self.phi_e_selection == 2 && self.openloop_velocity_target != 0 {
                if self.ac_openloop_phi_advances_after_setup {
                    return self.ac_openloop_phi().max(1).into();
                }
                return self.ac_openloop_phi().into();
            }
            return self.openloop_phi;
        }
        if addr == PHI_E_SELECTION.addr() {
            if self.dc_external_phi_e_selection_readback_mismatch
                && !nonzero_uq_written
                && self.pwm_sv_chop == 7
                && self.phi_e_selection == 1
            {
                return 0xffff_ffff;
            }
            if self.open_loop_phi_e_selection_readback_mismatch && self.phi_e_selection == 2 {
                return 0xffff_ffff;
            }
            return self.phi_e_selection;
        }
        if addr == PHI_E_EXT.addr() {
            return self.phi_e_ext;
        }
        if addr == PWM_SV_CHOP.addr() {
            return self.pwm_sv_chop;
        }
        if addr == PWM_MAXCNT.addr() {
            return self.pwm_maxcnt_readback;
        }
        if addr == PWM_BBM_H_BBM_L.addr() {
            return self.bbm_readback;
        }
        if addr == DSADC_MDEC_B_MDEC_A.addr() {
            return self.dsadc_mdec_readback;
        }
        0
    }

    fn nonzero_uq_write_count(&self) -> usize {
        self.writes
            .iter()
            .filter(|write| write.addr == UQ_UD_EXT.addr() && write.value != 0)
            .count()
    }

    fn ready_for_success() -> Self {
        Self::standard()
    }

    fn with_encoder_script(mut self, counts: &[u32]) -> Self {
        for count in counts {
            self.scripted_encoder_counts
                .push(*count)
                .expect("encoder script overflow");
        }
        self
    }

    fn seed_subregister(&mut self, sub: SubRegister, value: u32) {
        if sub.sub_addr == CONFIG_BIQUAD_T_ENABLE.sub_addr {
            self.torque_filter_enable = value;
        } else if sub.sub_addr == CONFIG_BIQUAD_F_ENABLE.sub_addr {
            self.flux_filter_enable = value;
        } else {
            panic!("unsupported seeded subregister {sub:?}");
        }
    }

    fn read_index(&self, addr: u8) -> usize {
        self.ops
            .iter()
            .position(|op| matches!(op, RecordedOp::Read { addr: read_addr } if *read_addr == addr))
            .expect("read address missing")
    }

    fn first_nonzero_uq_write_index(&self) -> usize {
        self.ops
            .iter()
            .position(|op| {
                matches!(
                    op,
                    RecordedOp::Write { addr, value }
                        if *addr == UQ_UD_EXT.addr() && *value != 0
                )
            })
            .expect("nonzero UQ_UD_EXT write missing")
    }

    fn write_index(&self, addr: u8, value: u32) -> usize {
        self.writes
            .iter()
            .position(|write| write.addr == addr && write.value == value)
            .expect("write missing")
    }

    fn sub_write_indices(&self, sub: SubRegister, value: u32) -> heapless::Vec<usize, 16> {
        let mut indices = heapless::Vec::new();
        for (index, pair) in self.writes.windows(2).enumerate() {
            if pair[0].addr == sub.addr.addr()
                && pair[0].value == u32::from(sub.sub_addr)
                && pair[1].addr == sub.data.addr()
                && pair[1].value == value
            {
                indices.push(index).expect("sub write index overflow");
            }
        }
        indices
    }
}

impl ImpedanceBackend for FakeImpedanceBackend {
    async fn read_register(&mut self, addr: u8) -> Result<u32, ImpedanceBackendError> {
        if addr == PWM_MAXCNT.addr()
            || addr == PWM_BBM_H_BBM_L.addr()
            || addr == DSADC_MDEC_B_MDEC_A.addr()
            || addr == PWM_SV_CHOP.addr()
        {
            self.reads
                .push(RecordedRead { addr })
                .expect("read log overflow");
            self.ops
                .push(RecordedOp::Read { addr })
                .expect("op log overflow");
        }
        if addr == MOTOR_TYPE_N_POLE_PAIRS.addr() {
            return Ok(MotorTypeNPolePairs::new()
                .with_motor_type(self.motor_type)
                .with_n_pole_pairs(self.n_pole_pairs)
                .into());
        }
        if addr == ABN_DECODER_COUNT.addr() {
            if !self.scripted_encoder_counts.is_empty() {
                let index = self
                    .scripted_encoder_index
                    .min(self.scripted_encoder_counts.len() - 1);
                let count = self.scripted_encoder_counts[index];
                self.scripted_encoder_index += 1;
                return Ok(count);
            }
            if let Some(peak) = self.pulsating_encoder_peak_counts {
                if self.phi_e_selection == 1 && self.energized() {
                    return Ok(self.encoder_count.wrapping_add(peak));
                }
            }
            return Ok(self.encoder_count);
        }
        if addr == STATUS_FLAGS.addr() {
            return Ok(self.status_flags);
        }
        if addr == ADC_IWY_IUX.addr() {
            if self.phi_e_selection == 2 && self.energized() {
                let next_sample = self.ac_adc_reads + 1;
                if self.fail_ac_adc_read_at == Some(next_sample) {
                    return Err(ImpedanceBackendError::Spi);
                }
                self.ac_adc_reads = next_sample;
                self.ac_sample_timestamps
                    .push(self.timestamp_us)
                    .expect("AC sample timestamp log overflow");
                if self.ac_over_current_sample == Some(next_sample) {
                    let over = self.profile.max_expected_current_count as i16 + 1;
                    self.restore_failure_armed = true;
                    return Ok(Self::packed_current(over, 0));
                }
                return Ok(self.ac_current());
            }
            if i32::from(self.baseline_current_count).unsigned_abs()
                > u32::from(self.profile.max_expected_current_count)
            {
                self.restore_failure_armed = true;
            }
            return Ok(Self::packed_current(self.baseline_current_count, 0));
        }
        if addr == PID_TORQUE_FLUX_ACTUAL.addr() {
            if self.phi_e_selection == 1 && self.energized() {
                self.ac_transformed_reads += 1;
                return Ok(self.ac_current());
            }
            if self.phi_e_selection == 2 && self.energized() {
                self.ac_transformed_reads += 1;
                return Ok(self.ac_current());
            }
            return Ok(Self::packed_current(0, self.baseline_current_count));
        }
        if addr == INTERIM_FOC_UQ_UD.data.addr() {
            if self.interim_sub_addr == INTERIM_FOC_IWY_IUX.sub_addr {
                return Ok(Self::packed_halves(
                    self.last_ud().abs() / 3,
                    self.last_ud().abs() / 2,
                ));
            }
            if self.interim_sub_addr == INTERIM_FOC_UQ_UD.sub_addr {
                return Ok(Self::packed_halves(0, self.last_ud()));
            }
            if self.interim_sub_addr == INTERIM_FOC_UQ_UD_LIMITED.sub_addr {
                return Ok(Self::packed_halves(0, self.last_ud()));
            }
            if self.interim_sub_addr == INTERIM_FOC_IQ_ID.sub_addr {
                return Ok(Self::packed_halves(
                    self.last_ud().abs() / 4,
                    self.last_ud().abs() / 8,
                ));
            }
        }
        if addr == CONFIG_BIQUAD_T_ENABLE.data.addr() {
            if self.config_sub_addr == CONFIG_BIQUAD_T_ENABLE.sub_addr {
                return Ok(self.torque_filter_enable);
            }
            if self.config_sub_addr == CONFIG_BIQUAD_F_ENABLE.sub_addr {
                return Ok(self.flux_filter_enable);
            }
        }
        Ok(self.readback_value(addr))
    }

    async fn write_register(&mut self, addr: u8, value: u32) -> Result<(), ImpedanceBackendError> {
        self.writes
            .push(RecordedWrite { addr, value })
            .expect("write log overflow");
        self.ops
            .push(RecordedOp::Write { addr, value })
            .expect("op log overflow");
        if addr == PIDOUT_UQ_UD_LIMITS.addr() {
            self.pidout_limit = value;
        } else if addr == UQ_UD_EXT.addr() {
            self.uq_ud_ext = value;
        } else if addr == MODE_RAMP_MODE_MOTION.addr() {
            self.mode_ramp_mode_motion = value;
        } else if addr == OPENLOOP_MODE.addr() {
            self.openloop_mode = value;
        } else if addr == OPENLOOP_ACCELERATION.addr() {
            self.openloop_acceleration = value;
        } else if addr == OPENLOOP_VELOCITY_TARGET.addr() {
            self.openloop_velocity_target = value;
        } else if addr == OPENLOOP_VELOCITY_ACTUAL.addr() {
            self.openloop_velocity_actual = value;
        } else if addr == OPENLOOP_PHI.addr() {
            self.openloop_phi = value;
        } else if addr == PHI_E_SELECTION.addr() {
            self.phi_e_selection = value;
        } else if addr == PHI_E_EXT.addr() {
            self.phi_e_ext = value;
        } else if addr == PWM_SV_CHOP.addr() {
            self.pwm_sv_chop = value;
        } else if addr == INTERIM_FOC_UQ_UD.addr.addr() {
            self.interim_sub_addr = value as u8;
        } else if addr == CONFIG_BIQUAD_T_ENABLE.addr.addr() {
            self.config_sub_addr = value as u8;
        } else if addr == CONFIG_BIQUAD_T_ENABLE.data.addr() {
            if self.config_sub_addr == CONFIG_BIQUAD_T_ENABLE.sub_addr {
                self.torque_filter_enable = value;
            } else if self.config_sub_addr == CONFIG_BIQUAD_F_ENABLE.sub_addr {
                self.flux_filter_enable = value;
            }
        }
        if addr == MODE_RAMP_MODE_MOTION.addr()
            && value == foci_tmc4671::registers::MotionMode::UqUdExt as u32
            && self.phi_e_selection == 1
        {
            self.v2_angle_starts += 1;
        }
        if self.restore_failure_armed && self.restore_fail_addr == Some(addr) {
            return Err(ImpedanceBackendError::Spi);
        }
        Ok(())
    }

    async fn delay_us(&mut self, us: u32) {
        self.timestamp_us = self.timestamp_us.wrapping_add(us);
    }

    async fn delay_ms(&mut self, ms: u64) {
        self.timestamp_us = self
            .timestamp_us
            .wrapping_add((ms as u32).saturating_mul(1000));
        if self.energized() {
            self.encoder_count = self
                .encoder_count
                .wrapping_add(self.encoder_delta_per_energized_delay);
        }
    }

    fn timestamp_us(&self) -> u32 {
        self.timestamp_us
    }

    fn is_armed_for_open_loop_voltage(&self) -> bool {
        self.armed_for_open_loop_voltage
    }

    fn profile(&self) -> ImpedanceProfile {
        self.profile
    }

    fn record_fit_attempt(&mut self, point_index: usize, fit: ImpedanceFitPoint) {
        self.fit_attempts[point_index] = Some(fit);
    }

    fn record_pulsating_angle_observation(
        &mut self,
        point_index: usize,
        observation: PulsatingAngleObservation,
    ) {
        self.pulsating_observations[point_index] = Some(observation);
    }

    fn record_profile_readback_mismatch(&mut self, addr: u8, expected: u32, actual: u32) {
        self.readback_mismatch = Some((addr, expected, actual));
    }
}

fn assert_write_order(writes: &[RecordedWrite], first_addr: u8, second_addr: u8) {
    let first = writes
        .iter()
        .position(|write| write.addr == first_addr)
        .expect("first address never written");
    let second = writes
        .iter()
        .position(|write| write.addr == second_addr)
        .expect("second address never written");
    assert!(
        first < second,
        "expected {first_addr:#x} before {second_addr:#x}"
    );
}

fn assert_write_before_nonzero_uq(writes: &[RecordedWrite], first_addr: u8) {
    let first = writes
        .iter()
        .position(|write| write.addr == first_addr)
        .expect("first address never written");
    let uq = writes
        .iter()
        .position(|write| write.addr == UQ_UD_EXT.addr() && write.value != 0)
        .expect("non-zero UQ_UD_EXT never written");
    assert!(
        first < uq,
        "expected {first_addr:#x} before non-zero UQ_UD_EXT"
    );
}

fn assert_no_nonzero_uq_writes(writes: &[RecordedWrite]) {
    assert!(
        !writes
            .iter()
            .any(|write| write.addr == UQ_UD_EXT.addr() && write.value != 0),
        "expected no non-zero UQ_UD_EXT writes, got {writes:?}"
    );
}

fn assert_no_nonzero_uq_after_open_loop_selection(writes: &[RecordedWrite]) {
    let open_loop_selection = writes
        .iter()
        .position(|write| write.addr == PHI_E_SELECTION.addr() && write.value == 2)
        .expect("open-loop PHI_E_SELECTION write missing");
    assert!(
        !writes[open_loop_selection + 1..]
            .iter()
            .any(|write| write.addr == UQ_UD_EXT.addr() && write.value != 0),
        "expected no AC non-zero UQ_UD_EXT after open-loop selection, got {writes:?}"
    );
}

fn assert_restore_wrote_phi_e_selection_zero(writes: &[RecordedWrite]) {
    assert!(
        writes
            .iter()
            .rev()
            .any(|write| write.addr == PHI_E_SELECTION.addr() && write.value == 0),
        "expected restore PHI_E_SELECTION=0 write, got {writes:?}"
    );
}

type MockImpedanceBackend = FakeImpedanceBackend;

#[test]
fn runner_v2_never_writes_openloop_velocity_target() {
    let mut backend = MockImpedanceBackend::ready_for_success();

    let result = block_on(run_pulsating_impedance_identification(&mut backend)).unwrap();

    assert_eq!(result.profile.profile_version, 2);
    assert!(
        !backend
            .writes
            .iter()
            .any(|write| write.addr == OPENLOOP_VELOCITY_TARGET.addr() && write.value != 0),
        "v2 must not command the open-loop velocity generator: {:?}",
        backend.writes
    );
    assert!(
        backend
            .writes
            .iter()
            .any(|write| write.addr == PHI_E_SELECTION.addr() && write.value == 1),
        "v2 must select external electrical angle: {:?}",
        backend.writes
    );
}

#[test]
fn runner_v2_snapshots_disables_and_restores_torque_flux_filters() {
    let mut backend = MockImpedanceBackend::ready_for_success();
    backend.seed_subregister(CONFIG_BIQUAD_T_ENABLE, 1);
    backend.seed_subregister(CONFIG_BIQUAD_F_ENABLE, 1);

    let result = block_on(run_pulsating_impedance_identification(&mut backend)).unwrap();

    assert_eq!(result.filter_state.torque_enable_before, 1);
    assert_eq!(result.filter_state.flux_enable_before, 1);
    assert_eq!(result.filter_state.torque_enable_during, 0);
    assert_eq!(result.filter_state.flux_enable_during, 0);
    let torque_disable = backend.sub_write_indices(CONFIG_BIQUAD_T_ENABLE, 0);
    let flux_disable = backend.sub_write_indices(CONFIG_BIQUAD_F_ENABLE, 0);
    let torque_restore = backend.sub_write_indices(CONFIG_BIQUAD_T_ENABLE, 1);
    let flux_restore = backend.sub_write_indices(CONFIG_BIQUAD_F_ENABLE, 1);
    assert!(!torque_disable.is_empty(), "torque filter was not disabled");
    assert!(!flux_disable.is_empty(), "flux filter was not disabled");
    assert!(!torque_restore.is_empty(), "torque filter was not restored");
    assert!(!flux_restore.is_empty(), "flux filter was not restored");
    assert!(torque_disable[0] < torque_restore[0]);
    assert!(flux_disable[0] < flux_restore[0]);
}

#[test]
fn runner_v2_verifies_power_stage_before_first_carrier_write() {
    let mut backend = MockImpedanceBackend::ready_for_success();

    let result = block_on(run_pulsating_impedance_identification(&mut backend));

    assert!(result.is_ok(), "expected ok, got {result:?}");
    let first_carrier = backend.first_nonzero_uq_write_index();
    let pwm_write = backend.write_index(PWM_SV_CHOP.addr(), 7);
    assert!(pwm_write < first_carrier);
    for addr in [
        PWM_MAXCNT.addr(),
        PWM_BBM_H_BBM_L.addr(),
        DSADC_MDEC_B_MDEC_A.addr(),
        PWM_SV_CHOP.addr(),
    ] {
        assert!(
            backend.read_index(addr) < first_carrier,
            "read {addr:#x} did not happen before first carrier write"
        );
    }
}

#[test]
fn runner_v2_setup_readback_mismatches_block_first_carrier_write() {
    for (addr, expected) in [
        (OPENLOOP_VELOCITY_TARGET.addr(), 0),
        (OPENLOOP_VELOCITY_ACTUAL.addr(), 0),
        (OPENLOOP_ACCELERATION.addr(), 0),
        (OPENLOOP_PHI.addr(), 0),
        (OPENLOOP_MODE.addr(), 0),
        (MODE_RAMP_MODE_MOTION.addr(), 0),
        (PHI_E_SELECTION.addr(), 1),
        (PHI_E_EXT.addr(), 0),
    ] {
        let mut backend = MockImpedanceBackend::ready_for_success();
        backend.readback_mismatch_addr = Some(addr);

        let err = block_on(run_pulsating_impedance_identification(&mut backend)).unwrap_err();

        assert_eq!(err, ImpedanceError::ProfileReadbackMismatch);
        assert_eq!(
            backend.readback_mismatch,
            Some((addr, expected, 0xffff_ffff))
        );
        assert_no_nonzero_uq_writes(&backend.writes);
    }
}

#[test]
fn runner_v2_uses_encoder_peak_excursion_not_before_after_delta() {
    let mut backend =
        MockImpedanceBackend::ready_for_success().with_encoder_script(&[1000, 1036, 1000]);

    let result = block_on(run_pulsating_impedance_identification(&mut backend)).unwrap();

    assert!(
        result
            .warning_flags
            .contains(ImpedanceWarningFlags::ANGLE_LOCAL_MOTION)
    );
    assert_eq!(result.angle_observations[0].encoder_peak_counts, 36);
}

#[test]
fn runner_v2_tracks_encoder_peak_across_wraparound() {
    let mut backend =
        MockImpedanceBackend::ready_for_success().with_encoder_script(&[3, u32::MAX - 4, 83]);

    let result = block_on(run_pulsating_impedance_identification(&mut backend)).unwrap();

    assert_eq!(result.angle_observations[0].encoder_peak_counts, 88);
    assert_eq!(result.angle_observations[0].encoder_min, u32::MAX - 4);
    assert_eq!(result.angle_observations[0].encoder_max, 83);
    assert!(
        result
            .warning_flags
            .contains(ImpedanceWarningFlags::ENCODER_MOVED)
    );
    assert!(
        backend.v2_angle_starts >= 2,
        "runner did not continue after wraparound warning"
    );
}

#[test]
fn runner_v2_warns_but_continues_for_encoder_warn_band() {
    let mut backend =
        MockImpedanceBackend::ready_for_success().with_encoder_script(&[1000, 1080, 1000]);

    let result = block_on(run_pulsating_impedance_identification(&mut backend)).unwrap();

    assert!(
        result
            .warning_flags
            .contains(ImpedanceWarningFlags::ENCODER_MOVED)
    );
    assert!(
        backend.v2_angle_starts >= 2,
        "runner did not continue into the second angle"
    );
}

#[test]
fn runner_v2_hard_aborts_when_encoder_peak_exceeds_fail_threshold() {
    let mut backend =
        MockImpedanceBackend::ready_for_success().with_encoder_script(&[1000, 1136, 1136]);

    let err = block_on(run_pulsating_impedance_identification(&mut backend)).unwrap_err();

    assert_eq!(err, ImpedanceError::EncoderMoved);
    assert!(
        backend
            .writes
            .iter()
            .any(|write| write.addr == UQ_UD_EXT.addr() && write.value == 0),
        "runner did not command zero voltage after encoder peak"
    );
    assert_eq!(backend.v2_angle_starts, 1);
}

#[test]
fn runner_v2_fit_quality_failure_records_angle_observations() {
    let profile = PulsatingImpedanceProfile::standard_openffboard();
    let mut backend = MockImpedanceBackend::ready_for_success();
    backend.pulsating_encoder_peak_counts =
        Some(u32::from(profile.encoder_angle_invalid_counts) + 1);

    let err = block_on(run_pulsating_impedance_identification(&mut backend)).unwrap_err();

    assert_eq!(err, ImpedanceError::PoorCoverage);
    assert_eq!(backend.v2_angle_starts, PULSATING_IMPEDANCE_PROFILE_POINTS);
    for (index, observation) in backend.pulsating_observations.iter().enumerate() {
        let observation = observation.expect("captured angle observation");
        assert_eq!(
            observation.angle_electrical_counts,
            profile.angle_points[index].angle_electrical_counts
        );
        assert_eq!(
            observation.encoder_peak_counts,
            profile.encoder_angle_invalid_counts + 1
        );
        assert!(!observation.valid);
    }
}

#[test]
fn runner_v2_accumulator_puts_phase_lag_in_inductive_channel() {
    let profile = PulsatingImpedanceProfile::standard_openffboard();
    let point = profile.angle_points[0];
    let mut accumulator =
        PulsatingAngleAccumulator::new(point.angle_electrical_counts, point.ud_abs);

    for index in 0..profile.samples_per_cycle {
        let phase = TURN * f64::from(index) / f64::from(profile.samples_per_cycle);
        let resistive = (80.0 * libm::sin(phase)).round() as i16;
        let inductive = (32.0 * libm::cos(phase)).round() as i16;
        let sample = ImpedanceSample {
            t_us: u32::from(index) * u32::from(profile.sample_interval_us),
            openloop_phi: 0,
            i_ch0: resistive + inductive,
            i_ch1: 0,
        };
        accumulator.accumulate(sample, sample);
    }

    let observation = accumulator.finish(
        &profile,
        PulsatingBaselineObservation {
            d_count: 0,
            q_count: 0,
            adc_ch0_count: 0,
            adc_ch1_count: 0,
        },
        0,
        0,
        0,
        0,
    );

    assert!(observation.in_phase_response_count_per_ud_milli > 100);
    assert!(observation.inductive_response_count_per_ud_milli > 40);
    assert!(
        observation.in_phase_response_count_per_ud_milli
            > observation.inductive_response_count_per_ud_milli
    );
}

#[test]
fn runner_unarmed_externally_armed_mode_writes_nothing() {
    let mut backend = FakeImpedanceBackend::standard();
    backend.armed_for_open_loop_voltage = false;

    let err = block_on(run_impedance_identification(&mut backend)).unwrap_err();

    assert_eq!(err, ImpedanceError::NotArmed);
    assert!(backend.wrote_nothing());
}

#[test]
fn runner_low_saved_pidout_limit_writes_nothing() {
    let mut backend = FakeImpedanceBackend::standard();
    backend.saved_voltage_limit = u32::from(backend.profile.max_abs_ud()) - 1;
    backend.pidout_limit = backend.saved_voltage_limit;

    let err = block_on(run_impedance_identification(&mut backend)).unwrap_err();

    assert_eq!(err, ImpedanceError::InsufficientVoltageAuthority);
    assert!(backend.wrote_nothing());
}

#[test]
fn runner_saved_pidout_with_only_uq_authority_writes_nothing() {
    let mut backend = FakeImpedanceBackend::standard();
    backend.saved_voltage_limit = 0x0001_0000;
    backend.pidout_limit = backend.saved_voltage_limit;

    let err = block_on(run_impedance_identification(&mut backend)).unwrap_err();

    assert_eq!(err, ImpedanceError::InsufficientVoltageAuthority);
    assert!(backend.wrote_nothing());
}

#[test]
fn runner_valid_saved_pidout_limit_passes_authority_gate() {
    let mut backend = FakeImpedanceBackend::standard();
    backend.saved_voltage_limit = u32::from(backend.profile.max_abs_ud());
    backend.pidout_limit = backend.saved_voltage_limit;

    let result = block_on(run_impedance_identification(&mut backend));

    assert!(result.is_ok(), "expected ok, got {result:?}");
}

#[test]
fn runner_unsupported_motor_type_writes_nothing() {
    let mut backend = FakeImpedanceBackend::standard();
    backend.motor_type = MotorType::ThreePhase as u8;

    let err = block_on(run_impedance_identification(&mut backend)).unwrap_err();

    assert_eq!(err, ImpedanceError::UnsupportedMotorType);
    assert!(backend.wrote_nothing());
}

#[test]
fn runner_unsupported_profile_exposure_writes_nothing() {
    let mut backend = FakeImpedanceBackend::standard();
    backend.profile.max_total_exposure_budget = backend.profile.max_total_exposure() - 1;

    let err = block_on(run_impedance_identification(&mut backend)).unwrap_err();

    assert_eq!(err, ImpedanceError::UnsupportedProfile);
    assert!(backend.wrote_nothing());
}

#[test]
fn runner_unsupported_profile_sample_count_writes_nothing() {
    let mut backend = FakeImpedanceBackend::standard();
    backend.profile.sample_count = IMPEDANCE_SAMPLE_COUNT as u16 - 1;

    let err = block_on(run_impedance_identification(&mut backend)).unwrap_err();

    assert_eq!(err, ImpedanceError::UnsupportedProfile);
    assert!(backend.wrote_nothing());
    assert!(!backend.wrote_safe_state());
}

#[test]
fn runner_unsupported_profile_sub_millisecond_sample_interval_writes_nothing() {
    let mut backend = FakeImpedanceBackend::standard();
    backend.profile.sample_interval_us = 999;
    backend.profile.max_window_exposure_budget = u64::MAX;
    backend.profile.max_total_exposure_budget = u64::MAX;

    let err = block_on(run_impedance_identification(&mut backend)).unwrap_err();

    assert_eq!(err, ImpedanceError::UnsupportedProfile);
    assert!(backend.wrote_nothing());
    assert!(!backend.wrote_safe_state());
}

#[test]
fn runner_unsupported_profile_non_millisecond_sample_interval_writes_nothing() {
    let mut backend = FakeImpedanceBackend::standard();
    backend.profile.sample_interval_us = 1500;
    backend.profile.max_window_exposure_budget = u64::MAX;
    backend.profile.max_total_exposure_budget = u64::MAX;

    let err = block_on(run_impedance_identification(&mut backend)).unwrap_err();

    assert_eq!(err, ImpedanceError::UnsupportedProfile);
    assert!(backend.wrote_nothing());
    assert!(!backend.wrote_safe_state());
}

#[test]
fn runner_unsupported_profile_ud_range_writes_nothing() {
    let mut backend = FakeImpedanceBackend::standard();
    backend.profile.injection_points[0].ud_abs = i16::MAX as u16 + 1;
    backend.saved_voltage_limit = u32::from(backend.profile.injection_points[0].ud_abs);
    backend.pidout_limit = backend.saved_voltage_limit;

    let err = block_on(run_impedance_identification(&mut backend)).unwrap_err();

    assert_eq!(err, ImpedanceError::UnsupportedProfile);
    assert!(backend.wrote_nothing());
    assert!(!backend.wrote_safe_state());
}

#[test]
fn runner_unsupported_profile_openloop_velocity_underflow_writes_nothing() {
    let mut backend = FakeImpedanceBackend::standard();
    backend.profile.injection_points[0].frequency_millihz = 1;

    let err = block_on(run_impedance_identification(&mut backend)).unwrap_err();

    assert_eq!(err, ImpedanceError::UnsupportedProfile);
    assert!(backend.wrote_nothing());
    assert!(!backend.wrote_safe_state());
}

#[test]
fn runner_unsupported_profile_acceleration_cannot_reach_velocity_writes_nothing() {
    let mut backend = FakeImpedanceBackend::standard();
    backend.profile.injection_points[0].openloop_acceleration_rpm_per_s = 1;

    let err = block_on(run_impedance_identification(&mut backend)).unwrap_err();

    assert_eq!(err, ImpedanceError::UnsupportedProfile);
    assert!(backend.wrote_nothing());
    assert!(!backend.wrote_safe_state());
}

#[test]
fn runner_voltage_limit_write_happens_before_chopper_and_open_loop_voltage() {
    let mut backend = FakeImpedanceBackend::standard();

    let result = block_on(run_impedance_identification(&mut backend));

    assert!(result.is_ok(), "expected ok, got {result:?}");
    assert!(backend.writes.iter().any(|write| {
        write.addr == PIDOUT_UQ_UD_LIMITS.addr()
            && write.value == u32::from(backend.profile.max_abs_ud())
    }));
    assert_write_order(
        &backend.writes,
        PIDOUT_UQ_UD_LIMITS.addr(),
        PWM_SV_CHOP.addr(),
    );
    assert_write_before_nonzero_uq(&backend.writes, PIDOUT_UQ_UD_LIMITS.addr());
}

#[test]
fn runner_restore_includes_phi_e_selection_zero_after_success() {
    let mut backend = FakeImpedanceBackend::standard();

    let result = block_on(run_impedance_identification(&mut backend));

    assert!(result.is_ok(), "expected ok, got {result:?}");
    assert!(backend.wrote_safe_state());
    assert_restore_wrote_phi_e_selection_zero(&backend.writes);
}

#[test]
fn runner_openloop_mode_uses_positive_direction_bit_value() {
    let mut backend = FakeImpedanceBackend::standard();

    let result = block_on(run_impedance_identification(&mut backend));

    assert!(result.is_ok(), "expected ok, got {result:?}");
    assert!(
        backend
            .writes
            .iter()
            .any(|write| write.addr == OPENLOOP_MODE.addr()),
        "expected OPENLOOP_MODE writes, got {:?}",
        backend.writes
    );
    assert!(
        backend
            .writes
            .iter()
            .filter(|write| write.addr == OPENLOOP_MODE.addr())
            .all(|write| write.value == 0),
        "expected audited positive direction OPENLOOP_MODE=0 writes, got {:?}",
        backend.writes
    );
}

#[test]
fn runner_derives_openloop_velocity_from_electrical_frequency_and_pole_pairs() {
    let mut backend = FakeImpedanceBackend::standard();

    let result = block_on(run_impedance_identification(&mut backend));

    assert!(result.is_ok(), "expected ok, got {result:?}");
    assert!(
        backend
            .writes
            .iter()
            .any(|write| write.addr == OPENLOOP_VELOCITY_TARGET.addr() && write.value == 30),
        "expected 25 Hz electrical on a 50-pole-pair motor to write 30 mechanical rpm, got {:?}",
        backend.writes
    );
    assert!(
        !backend
            .writes
            .iter()
            .any(|write| write.addr == OPENLOOP_VELOCITY_TARGET.addr() && write.value == 1_500),
        "expected runner not to write electrical rpm directly, got {:?}",
        backend.writes
    );
}

#[test]
fn runner_does_not_gate_zero_setup_on_dynamic_openloop_state_readbacks() {
    let mut backend = FakeImpedanceBackend::standard();
    backend.stale_dynamic_openloop_state_when_stopped = true;

    let result = block_on(run_impedance_identification(&mut backend));

    assert!(result.is_ok(), "expected ok, got {result:?}");
    assert_eq!(backend.readback_mismatch, None);
    assert!(backend.wrote_safe_state());
}

#[test]
fn runner_profile_critical_readback_mismatch_returns_profile_readback_mismatch() {
    let mut backend = FakeImpedanceBackend::standard();
    backend.readback_mismatch_addr = Some(OPENLOOP_MODE.addr());

    let err = block_on(run_impedance_identification(&mut backend)).unwrap_err();

    assert_eq!(err, ImpedanceError::ProfileReadbackMismatch);
    assert_eq!(
        backend.readback_mismatch,
        Some((OPENLOOP_MODE.addr(), 0, 0xffff_ffff))
    );
    assert!(backend.wrote_safe_state());
}

#[test]
fn runner_power_stage_readback_mismatch_blocks_nonzero_uq() {
    let mut backend = FakeImpedanceBackend::standard();
    backend.pwm_maxcnt_readback = 2047;

    let err = block_on(run_impedance_identification(&mut backend)).unwrap_err();

    assert_eq!(err, ImpedanceError::ProfileReadbackMismatch);
    assert!(backend.wrote_safe_state());
    assert_no_nonzero_uq_writes(&backend.writes);
}

#[test]
fn runner_phi_e_ext_readback_mismatch_returns_profile_readback_mismatch() {
    let mut backend = FakeImpedanceBackend::standard();
    backend.readback_mismatch_addr = Some(PHI_E_EXT.addr());

    let err = block_on(run_impedance_identification(&mut backend)).unwrap_err();

    assert_eq!(err, ImpedanceError::ProfileReadbackMismatch);
    assert!(backend.wrote_safe_state());
}

#[test]
fn runner_dc_phi_e_selection_readback_mismatch_blocks_nonzero_uq() {
    let mut backend = FakeImpedanceBackend::standard();
    backend.dc_external_phi_e_selection_readback_mismatch = true;

    let err = block_on(run_impedance_identification(&mut backend)).unwrap_err();

    assert_eq!(err, ImpedanceError::ProfileReadbackMismatch);
    assert!(backend.wrote_safe_state());
    assert_no_nonzero_uq_writes(&backend.writes);
}

#[test]
fn runner_dc_stopped_mode_readback_mismatch_blocks_nonzero_uq() {
    let mut backend = FakeImpedanceBackend::standard();
    backend.dc_stopped_mode_readback_mismatch = true;

    let err = block_on(run_impedance_identification(&mut backend)).unwrap_err();

    assert_eq!(err, ImpedanceError::ProfileReadbackMismatch);
    assert!(backend.wrote_safe_state());
    assert_no_nonzero_uq_writes(&backend.writes);
}

#[test]
fn runner_open_loop_phi_e_selection_readback_mismatch_blocks_ac_voltage() {
    let mut backend = FakeImpedanceBackend::standard();
    backend.open_loop_phi_e_selection_readback_mismatch = true;

    let err = block_on(run_impedance_identification(&mut backend)).unwrap_err();

    assert_eq!(err, ImpedanceError::ProfileReadbackMismatch);
    assert!(backend.wrote_safe_state());
    assert_no_nonzero_uq_after_open_loop_selection(&backend.writes);
}

#[test]
fn runner_ac_stopped_mode_readback_mismatch_blocks_ac_nonzero_uq() {
    let mut backend = FakeImpedanceBackend::standard();
    backend.ac_stopped_mode_readback_mismatch = true;

    let err = block_on(run_impedance_identification(&mut backend)).unwrap_err();

    assert_eq!(err, ImpedanceError::ProfileReadbackMismatch);
    assert!(backend.wrote_safe_state());
    assert_eq!(backend.nonzero_uq_write_count(), 1);
    assert_no_nonzero_uq_after_open_loop_selection(&backend.writes);
}

#[test]
fn runner_ac_zero_voltage_readback_mismatch_blocks_ac_nonzero_uq() {
    let mut backend = FakeImpedanceBackend::standard();
    backend.ac_zero_voltage_readback_mismatch = true;

    let err = block_on(run_impedance_identification(&mut backend)).unwrap_err();

    assert_eq!(err, ImpedanceError::ProfileReadbackMismatch);
    assert!(backend.wrote_safe_state());
    assert_eq!(backend.nonzero_uq_write_count(), 1);
    assert_no_nonzero_uq_after_open_loop_selection(&backend.writes);
}

#[test]
fn runner_ac_openloop_acceleration_readback_mismatch_blocks_ac_nonzero_uq() {
    let mut backend = FakeImpedanceBackend::standard();
    backend.ac_openloop_acceleration_readback_mismatch = true;

    let err = block_on(run_impedance_identification(&mut backend)).unwrap_err();

    assert_eq!(err, ImpedanceError::ProfileReadbackMismatch);
    assert!(backend.wrote_safe_state());
    assert_eq!(backend.nonzero_uq_write_count(), 1);
    assert_no_nonzero_uq_after_open_loop_selection(&backend.writes);
}

#[test]
fn runner_ac_openloop_velocity_target_readback_mismatch_blocks_ac_nonzero_uq() {
    let mut backend = FakeImpedanceBackend::standard();
    backend.ac_openloop_velocity_target_readback_mismatch = true;

    let err = block_on(run_impedance_identification(&mut backend)).unwrap_err();

    assert_eq!(err, ImpedanceError::ProfileReadbackMismatch);
    assert!(backend.wrote_safe_state());
    assert_eq!(backend.nonzero_uq_write_count(), 1);
    assert_no_nonzero_uq_after_open_loop_selection(&backend.writes);
}

#[test]
fn runner_ac_openloop_mode_readback_mismatch_blocks_ac_nonzero_uq() {
    let mut backend = FakeImpedanceBackend::standard();
    backend.ac_openloop_mode_readback_mismatch = true;

    let err = block_on(run_impedance_identification(&mut backend)).unwrap_err();

    assert_eq!(err, ImpedanceError::ProfileReadbackMismatch);
    assert!(backend.wrote_safe_state());
    assert_eq!(backend.nonzero_uq_write_count(), 1);
    assert_no_nonzero_uq_after_open_loop_selection(&backend.writes);
}

#[test]
fn runner_ac_openloop_phi_drift_does_not_fail_profile_readback() {
    let mut backend = FakeImpedanceBackend::standard();
    backend.ac_openloop_phi_advances_after_setup = true;

    let result = block_on(run_impedance_identification(&mut backend));

    assert!(result.is_ok(), "expected ok, got {result:?}");
}

#[test]
fn runner_dc_baseline_over_current_returns_current_limit_exceeded_and_restores() {
    let mut backend = FakeImpedanceBackend::standard();
    backend.baseline_current_count = backend.profile.max_expected_current_count as i16 + 1;

    let err = block_on(run_impedance_identification(&mut backend)).unwrap_err();

    assert_eq!(err, ImpedanceError::CurrentLimitExceeded);
    assert!(backend.wrote_safe_state());
    assert_restore_wrote_phi_e_selection_zero(&backend.writes);
}

#[test]
fn runner_restore_write_failure_takes_precedence_over_body_failure() {
    let mut backend = FakeImpedanceBackend::standard();
    backend.baseline_current_count = backend.profile.max_expected_current_count as i16 + 1;
    backend.restore_fail_addr = Some(OPENLOOP_MODE.addr());

    let err = block_on(run_impedance_identification(&mut backend)).unwrap_err();

    assert_eq!(err, ImpedanceError::RestoreFailed);
    assert!(backend.wrote_safe_state());
}

#[test]
fn runner_sampled_ac_over_current_returns_current_limit_exceeded_and_restores() {
    let mut backend = FakeImpedanceBackend::standard();
    backend.ac_over_current_sample = Some(1);

    let err = block_on(run_impedance_identification(&mut backend)).unwrap_err();

    assert_eq!(err, ImpedanceError::CurrentLimitExceeded);
    assert!(backend.wrote_safe_state());
}

#[test]
fn runner_sampled_fail_status_flags_return_unsafe_status_flags_and_restore() {
    let mut backend = FakeImpedanceBackend::standard();
    backend.status_flags = backend.profile.status_flags_fail_mask;

    let err = block_on(run_impedance_identification(&mut backend)).unwrap_err();

    assert_eq!(err, ImpedanceError::UnsafeStatusFlags);
    assert!(backend.wrote_safe_state());
}

#[test]
fn runner_fit_quality_failure_samples_all_points_before_returning_error() {
    let mut backend = FakeImpedanceBackend::standard();
    backend.low_signal_ud_abs = Some(backend.profile.injection_points[0].ud_abs);

    let err = block_on(run_impedance_identification(&mut backend)).unwrap_err();

    assert_eq!(err, ImpedanceError::LowSignal);
    let expected_raw_samples =
        (backend.profile.sample_count as usize + 1) * IMPEDANCE_PROFILE_POINTS;
    let expected_transformed_samples =
        backend.profile.sample_count as usize * IMPEDANCE_PROFILE_POINTS;
    assert_eq!(backend.ac_adc_reads, expected_raw_samples);
    assert_eq!(backend.ac_transformed_reads, expected_transformed_samples);
    assert!(backend.fit_attempts[0].is_some());
    assert!(backend.fit_attempts[1].is_some());
    assert_eq!(
        backend.fit_attempts[0].unwrap().axis_max_count_per_ud_milli,
        0
    );
    assert!(
        backend.fit_attempts[1].unwrap().axis_max_count_per_ud_milli
            > u32::from(backend.profile.min_signal_count)
    );
    assert!(backend.wrote_safe_state());
}

#[test]
fn runner_standard_profile_treats_latched_hold_status_as_warning_only() {
    let mut backend = FakeImpedanceBackend::standard();
    let observed_hold_status = 0x7000_4480;
    backend.status_flags = observed_hold_status;

    let result = block_on(run_impedance_identification(&mut backend)).unwrap();

    assert_eq!(result.status_flags_or, observed_hold_status);
    assert!(
        result
            .warning_flags
            .contains(ImpedanceWarningFlags::STATUS_FLAGS)
    );
    assert!(backend.wrote_safe_state());
}

#[test]
fn runner_standard_profile_adc_clipped_status_remains_unsafe() {
    let mut backend = FakeImpedanceBackend::standard();
    backend.status_flags = 1 << 26;

    let err = block_on(run_impedance_identification(&mut backend)).unwrap_err();

    assert_eq!(err, ImpedanceError::UnsafeStatusFlags);
    assert!(backend.wrote_safe_state());
}

#[test]
fn runner_initial_fail_status_blocks_nonzero_voltage_injection() {
    let mut backend = FakeImpedanceBackend::standard();
    backend.status_flags = backend.profile.status_flags_fail_mask;

    let err = block_on(run_impedance_identification(&mut backend)).unwrap_err();

    assert_eq!(err, ImpedanceError::UnsafeStatusFlags);
    assert!(backend.wrote_safe_state());
    assert_no_nonzero_uq_writes(&backend.writes);
}

#[test]
fn runner_sampled_warn_status_flags_record_warning_without_failing() {
    let mut backend = FakeImpedanceBackend::standard();
    backend.profile.status_flags_fail_mask = 0;
    backend.profile.status_flags_warn_mask = 0x20;
    backend.status_flags = 0x20;

    let result = block_on(run_impedance_identification(&mut backend)).unwrap();

    assert_eq!(result.status_flags_or, 0x20);
    assert!(
        result
            .warning_flags
            .contains(ImpedanceWarningFlags::STATUS_FLAGS)
    );
}

#[test]
fn runner_encoder_delta_over_fail_limit_returns_encoder_moved_and_restores() {
    let mut backend = FakeImpedanceBackend::standard();
    backend.encoder_delta_per_energized_delay =
        u32::from(backend.profile.encoder_move_fail_counts) + 1;

    let err = block_on(run_impedance_identification(&mut backend)).unwrap_err();

    assert_eq!(err, ImpedanceError::EncoderMoved);
    assert!(backend.wrote_safe_state());
}

#[test]
fn runner_sample_read_failure_returns_backend_io_and_restores() {
    let mut backend = FakeImpedanceBackend::standard();
    backend.fail_ac_adc_read_at = Some(1);

    let err = block_on(run_impedance_identification(&mut backend)).unwrap_err();

    assert_eq!(err, ImpedanceError::BackendIo);
    assert!(backend.wrote_safe_state());
}

#[test]
fn runner_success_reports_nonzero_status_flags_or_when_observed_status_is_safe() {
    let mut backend = FakeImpedanceBackend::standard();
    backend.profile.status_flags_fail_mask = 0;
    backend.profile.status_flags_warn_mask = 0;
    backend.status_flags = 0x20;

    let result = block_on(run_impedance_identification(&mut backend)).unwrap();

    assert_eq!(result.status_flags_or, 0x20);
    assert!(
        !result
            .warning_flags
            .contains(ImpedanceWarningFlags::STATUS_FLAGS)
    );
}

#[test]
fn runner_success_uses_profile_sample_count_and_sample_interval_us() {
    let mut backend = FakeImpedanceBackend::standard();

    let result = block_on(run_impedance_identification(&mut backend));

    assert!(result.is_ok(), "expected ok, got {result:?}");
    assert_eq!(backend.profile.sample_interval_us, 1000);
    let capture_samples = backend.profile.sample_count as usize;
    let raw_samples_per_point = capture_samples + 1;
    let expected_raw_samples = raw_samples_per_point * IMPEDANCE_PROFILE_POINTS;
    let expected_transformed_samples = capture_samples * IMPEDANCE_PROFILE_POINTS;
    assert_eq!(backend.ac_adc_reads, expected_raw_samples);
    assert_eq!(backend.ac_transformed_reads, expected_transformed_samples);
    assert_eq!(backend.ac_sample_timestamps.len(), expected_raw_samples);
    for point_index in 0..IMPEDANCE_PROFILE_POINTS {
        let base = point_index * raw_samples_per_point;
        for sample_index in 2..raw_samples_per_point {
            let index = base + sample_index;
            let delta =
                backend.ac_sample_timestamps[index] - backend.ac_sample_timestamps[index - 1];
            assert_eq!(delta, u32::from(backend.profile.sample_interval_us));
        }
    }
}

#[test]
fn runner_success_records_point_observations() {
    let mut backend = FakeImpedanceBackend::standard();
    backend.ac_openloop_phi_advances_after_setup = true;
    backend.baseline_current_count = 123;

    let result = block_on(run_impedance_identification(&mut backend)).unwrap();

    assert_eq!(result.point_observations[0].status_flags_or, 0);
    assert_eq!(result.point_observations[0].foc_uq_sample, 0);
    assert_eq!(
        result.point_observations[0].foc_ud_sample,
        backend.profile.injection_points[0].ud_abs as i16
    );
    assert_eq!(result.point_observations[0].foc_uq_limited_sample, 0);
    assert_eq!(
        result.point_observations[0].foc_ud_limited_sample,
        backend.profile.injection_points[0].ud_abs as i16
    );
    assert!(result.point_observations[0].foc_iq_sample > 0);
    assert!(result.point_observations[0].foc_id_sample > 0);
    assert_eq!(result.point_observations[0].openloop_phi_sample, 32_768);
    assert_eq!(result.point_observations[0].openloop_velocity_actual, 30);
    assert_ne!(result.point_observations[0].adc_iux_sample, 0);
    assert_ne!(result.point_observations[0].foc_iux_sample, 0);
    assert_ne!(result.point_baselines[0].dc_adc_iux_sample, 0);
    assert_ne!(result.point_baselines[0].dc_foc_iux_sample, 0);
    assert_eq!(
        result.point_observations[1].foc_ud_sample,
        backend.profile.injection_points[1].ud_abs as i16
    );
}
