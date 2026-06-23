use super::profile::ImpedanceProfile;
use super::types::IMPEDANCE_SAMPLE_COUNT;

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
