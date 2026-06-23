use super::profile::{frequency_millihz_to_openloop_velocity_rpm, ImpedanceProfile};
use super::types::{ImpedanceWarningFlags, IMPEDANCE_SAMPLE_COUNT};

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
    let flags =
        ImpedanceWarningFlags::HIGH_RESIDUAL;

    assert!(flags.contains(ImpedanceWarningFlags::HIGH_RESIDUAL));
    assert!(!flags.contains(
        ImpedanceWarningFlags(
            ImpedanceWarningFlags::HIGH_RESIDUAL.0 | ImpedanceWarningFlags::HIGH_NOISE.0,
        ),
    ));
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
