use super::profile::{frequency_millihz_to_openloop_velocity_rpm, ImpedanceProfile};
use super::types::{
    ImpedanceError, ImpedanceInjectionPoint, ImpedanceSample, ImpedanceWarningFlags,
    IMPEDANCE_PROFILE_POINTS, IMPEDANCE_SAMPLE_COUNT,
};
use super::{evaluate_impedance_result, fit_impedance_point, ImpedanceFitPoint};

const TURN: f64 = core::f64::consts::PI * 2.0;

fn test_point(frequency_millihz: u32, ud_abs: u16) -> ImpedanceInjectionPoint {
    ImpedanceInjectionPoint {
        ud_abs,
        frequency_millihz,
        openloop_velocity_rpm: frequency_millihz_to_openloop_velocity_rpm(u64::from(
            frequency_millihz,
        )),
        openloop_acceleration_rpm_per_s: 3_000,
        settle_ms: 250,
    }
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
        i_ch0: i_ch0.round() as i16,
        i_ch1: i_ch1.round() as i16,
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
fn standard_profile_is_bounded_and_physical_projection_is_not_validated_by_default() {
    let profile = ImpedanceProfile::standard_openffboard();

    assert_eq!(profile.profile_version, 1);
    assert!(profile.sample_count >= 64);
    assert!(profile.sample_count <= profile.max_sample_count);
    assert_eq!(profile.injection_points.len(), 2);
    assert!(profile.max_abs_ud() > 0);
    assert!(profile.max_window_exposure() <= profile.max_window_exposure_budget);
    assert!(profile.max_total_exposure() <= profile.max_total_exposure_budget);
    assert!(!profile.scale_metadata_validated);
    for point in profile.injection_points {
        assert!(point.openloop_velocity_rpm > 0);
        assert_eq!(
            point.openloop_velocity_rpm,
            ((u64::from(point.frequency_millihz) * 60) / 1000) as i32
        );
        assert!(point.openloop_acceleration_rpm_per_s > 0);
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

    assert_eq!(profile.sample_count as usize, IMPEDANCE_SAMPLE_COUNT);
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
        frequency_millihz_to_openloop_velocity_rpm(u64::MAX),
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
fn scalar_2f_mismatch_clean_ellipse_stays_below_warn_threshold() {
    let profile = ImpedanceProfile::standard_openffboard();
    let point = test_point(37_000, 800);
    let samples = single_period_samples(point, 900.0, 300.0, 0.62, 0.0);

    let fit = unwrap_fit(fit_impedance_point(&profile, point, &samples));

    assert!(fit.scalar_2f_mismatch_permille < profile.scalar_2f_mismatch_warn_permille);
    assert!(!fit
        .warning_flags
        .contains(ImpedanceWarningFlags::SCALAR_2F_MISMATCH));
}

#[test]
fn scalar_2f_mismatch_clean_ellipse_with_large_dc_offsets_stays_below_warn_threshold() {
    let profile = ImpedanceProfile::standard_openffboard();
    let point = test_point(37_000, 800);
    let samples =
        single_period_samples_with_offsets(point, 900.0, 300.0, 0.62, 0.0, 5_000.0, -4_000.0);

    let fit = unwrap_fit(fit_impedance_point(&profile, point, &samples));

    assert!(fit.scalar_2f_mismatch_permille < profile.scalar_2f_mismatch_warn_permille);
    assert!(!fit
        .warning_flags
        .contains(ImpedanceWarningFlags::SCALAR_2F_MISMATCH));
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
    assert!(fit
        .warning_flags
        .contains(ImpedanceWarningFlags::SCALAR_2F_MISMATCH));
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
    assert!(summary
        .warning_flags
        .contains(ImpedanceWarningFlags::REPEATABILITY));
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

    assert!(summary
        .warning_flags
        .contains(ImpedanceWarningFlags::HIGH_RESIDUAL));
    assert!(summary
        .warning_flags
        .contains(ImpedanceWarningFlags::SCALAR_2F_MISMATCH));
}

#[test]
fn aggregate_repeatability_sets_physical_projection_unavailable_without_scale_metadata() {
    let profile = ImpedanceProfile::standard_openffboard();
    let fits = aggregate_fits(900, 300, 910, 295);

    let summary = evaluate_impedance_result(&profile, &fits).expect("summary should pass");

    assert!(summary
        .warning_flags
        .contains(ImpedanceWarningFlags::PHYSICAL_PROJECTION_UNAVAILABLE));
}
