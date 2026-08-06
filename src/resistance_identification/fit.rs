//! Deterministic fixed-point fit and quality gates for resistance identification.

use super::profile::ResistanceProfile;
use super::types::{
    ElectricalAxis, ResistanceAxisResult, ResistanceCurrentEvidence, ResistanceError,
    ResistancePoint, ResistanceRunResult, ResistanceWarningFlags, SignedAnchorEvidence,
};

/// Fits a single electrical axis to a count-space line using ordinary least
/// squares restricted to the high-current linear region, then applies the
/// residual, thermal-drift, and signed-anchor quality gates.
///
/// # Errors
///
/// Returns [`ResistanceError::InsufficientLinearPoints`] when fewer than
/// `profile.min_selected_points` samples clear the linear-current threshold,
/// [`ResistanceError::HighResidual`] when the fit residual exceeds the
/// profile's fail threshold, and [`ResistanceError::ThermalDrift`] when the
/// signed-anchor drift exceeds the profile's fail threshold.
pub fn fit_resistance_axis(
    axis: ElectricalAxis,
    profile: &ResistanceProfile,
    points: &[ResistancePoint; 8],
    signed_anchor: Option<SignedAnchorEvidence>,
) -> Result<ResistanceAxisResult, ResistanceError> {
    let mut selected_mask = 0u16;
    let mut excluded_mask = 0u16;
    let mut n = 0i64;
    let mut sum_x = 0i64;
    let mut sum_y = 0i64;
    let mut sum_xx = 0i64;
    let mut sum_xy = 0i64;

    for (idx, point) in points.iter().enumerate() {
        if point.current_delta_count.unsigned_abs()
            >= u32::from(profile.linear_current_threshold_count)
        {
            selected_mask |= 1 << idx;
            n += 1;
            let x = i64::from(point.current_delta_count);
            let y = i64::from(point.voltage_delta_count);
            sum_x += x;
            sum_y += y;
            sum_xx += x * x;
            sum_xy += x * y;
        } else {
            excluded_mask |= 1 << idx;
        }
    }

    if n < i64::from(profile.min_selected_points) {
        return Err(ResistanceError::InsufficientLinearPoints);
    }

    let denom = n * sum_xx - sum_x * sum_x;
    if denom == 0 {
        return Err(ResistanceError::InsufficientLinearPoints);
    }

    let slope_num = n * sum_xy - sum_x * sum_y;
    if slope_num <= 0 {
        return Err(ResistanceError::NonPositiveSlope);
    }
    let r_count_slope_milli = ((slope_num * 1000) / denom) as i32;
    let intercept_count = ((sum_y * denom - slope_num * sum_x) / (n * denom)) as i32;

    let mut residual_sq = 0i64;
    let mut selected_abs_y = 0i64;
    for point in points.iter() {
        if point.current_delta_count.unsigned_abs()
            < u32::from(profile.linear_current_threshold_count)
        {
            continue;
        }
        let x = i64::from(point.current_delta_count);
        let y = i64::from(point.voltage_delta_count);
        let predicted_num = slope_num * x + intercept_count as i64 * denom;
        let residual = y * denom - predicted_num;
        let scaled = residual / denom;
        residual_sq += scaled * scaled;
        selected_abs_y += y.abs();
    }
    let mean_abs_y = (selected_abs_y / n).max(1);
    let rmse_count = int_sqrt((residual_sq / n) as u64) as i64;
    let rmse_permille = ((rmse_count * 1000 / mean_abs_y) as u16).min(1000);

    let mut warning_flags = ResistanceWarningFlags::NONE;
    if rmse_permille >= profile.high_residual_fail_permille {
        return Err(ResistanceError::HighResidual);
    }
    if rmse_permille >= profile.high_residual_warn_permille {
        warning_flags.insert(ResistanceWarningFlags::HIGH_RESIDUAL);
    }

    if let Some(anchor) = signed_anchor {
        if anchor.signed_count_slope_milli <= 0 {
            return Err(ResistanceError::NonPositiveSlope);
        }
        if anchor.drift_permille >= profile.thermal_drift_fail_permille {
            return Err(ResistanceError::ThermalDrift);
        }
        if anchor.drift_permille >= profile.thermal_drift_warn_permille {
            warning_flags.insert(ResistanceWarningFlags::THERMAL_DRIFT);
        }
        if anchor.asymmetry_permille >= profile.signed_asymmetry_warn_permille {
            warning_flags.insert(ResistanceWarningFlags::SIGNED_CURRENT_ASYMMETRY);
        }
    }

    Ok(ResistanceAxisResult {
        axis,
        r_count_slope_milli,
        slope_num,
        slope_den: denom,
        intercept_count,
        rmse_permille,
        selected_mask,
        excluded_point_mask: excluded_mask,
        selected_count: n as u8,
        signed_anchor,
        warning_flags,
    })
}

/// Combines two per-axis fits into a two-axis run result, applying the
/// electrical-axis mismatch gate and aggregating warning flags.
///
/// # Errors
///
/// Returns [`ResistanceError::ElectricalAxisMismatch`] when the two axes'
/// slopes disagree by more than `profile.electrical_axis_mismatch_permille`.
pub fn evaluate_resistance_result(
    profile: &ResistanceProfile,
    axis0: ResistanceAxisResult,
    axis1: ResistanceAxisResult,
    status_flags_or: u32,
) -> Result<ResistanceRunResult, ResistanceError> {
    let diff = axis0
        .r_count_slope_milli
        .abs_diff(axis1.r_count_slope_milli);
    let avg =
        ((axis0.r_count_slope_milli.unsigned_abs() + axis1.r_count_slope_milli.unsigned_abs()) / 2)
            .max(1);
    let mismatch_permille = (diff * 1000 / avg) as u16;
    if mismatch_permille >= profile.electrical_axis_mismatch_permille {
        return Err(ResistanceError::ElectricalAxisMismatch);
    }

    let mut warning_flags = ResistanceWarningFlags(axis0.warning_flags.0 | axis1.warning_flags.0);
    if !profile.scale_metadata_validated {
        warning_flags.insert(ResistanceWarningFlags::SCALE_METADATA_UNVALIDATED);
    }
    if status_flags_or & profile.status_flags_warn_mask != 0 {
        warning_flags.insert(ResistanceWarningFlags::STATUS_FLAGS);
    }

    Ok(ResistanceRunResult {
        selected_r_count_slope_milli: (axis0.r_count_slope_milli + axis1.r_count_slope_milli) / 2,
        status_flags_or,
        axis0,
        axis1,
        warning_flags,
        current: ResistanceCurrentEvidence::new(profile.max_expected_current_count),
    })
}

/// Integer square root via Newton's method, used to keep RMSE computation
/// in fixed-point count space without floating-point support.
fn int_sqrt(value: u64) -> u64 {
    if value <= 1 {
        return value;
    }
    let mut x = value;
    let mut y = x.div_ceil(2);
    while y < x {
        x = y;
        y = (x + value / x) / 2;
    }
    x
}

#[cfg(test)]
mod tests {
    use super::super::profile::{ResistanceProfile, STANDARD_UD_POINTS};
    use super::super::types::{
        ElectricalAxis, ResistanceAxisResult, ResistanceError, ResistancePoint,
        ResistanceWarningFlags, SignedAnchorEvidence,
    };
    use super::{evaluate_resistance_result, fit_resistance_axis};

    fn synthetic_axis_points(
        currents: [i32; 8],
        slope_milli: i32,
        intercept: i32,
    ) -> [ResistancePoint; 8] {
        let ud = STANDARD_UD_POINTS;
        core::array::from_fn(|idx| {
            let current = currents[idx];
            ResistancePoint {
                requested_ud: ud[idx],
                voltage_delta_count: current * slope_milli / 1000 + intercept,
                current_delta_count: current,
                selected: false,
            }
        })
    }

    fn axis_result_for_test(axis: ElectricalAxis, slope_milli: i32) -> ResistanceAxisResult {
        ResistanceAxisResult {
            axis,
            r_count_slope_milli: slope_milli,
            slope_num: i64::from(slope_milli),
            slope_den: 1000,
            intercept_count: 0,
            rmse_permille: 0,
            selected_mask: 0xff,
            excluded_point_mask: 0,
            selected_count: 8,
            signed_anchor: None,
            warning_flags: ResistanceWarningFlags::NONE,
        }
    }

    #[test]
    fn fit_selects_high_current_region_and_reports_mask() {
        let profile = ResistanceProfile::standard();
        let points = synthetic_axis_points([450, 575, 710, 850, 990, 1130, 1275, 1420], 1050, 20);

        let result = fit_resistance_axis(ElectricalAxis::PHASE0, &profile, &points, None).unwrap();

        assert_eq!(result.selected_count, 5);
        assert_eq!(result.excluded_point_mask & 0b0000_0111, 0b0000_0111);
        assert!((1030..=1070).contains(&result.r_count_slope_milli));
    }

    #[test]
    fn fit_rejects_negative_slope() {
        let profile = ResistanceProfile::standard();
        let points = synthetic_axis_points([450, 575, 710, 850, 990, 1130, 1275, 1420], -1050, 20);

        let err = fit_resistance_axis(ElectricalAxis::PHASE0, &profile, &points, None).unwrap_err();

        assert_eq!(err, ResistanceError::NonPositiveSlope);
    }

    #[test]
    fn fit_rejects_zero_slope() {
        let profile = ResistanceProfile::standard();
        let points = synthetic_axis_points([450, 575, 710, 850, 990, 1130, 1275, 1420], 0, 20);

        let err = fit_resistance_axis(ElectricalAxis::PHASE0, &profile, &points, None).unwrap_err();

        assert_eq!(err, ResistanceError::NonPositiveSlope);
    }

    #[test]
    fn fit_rejects_nonpositive_signed_anchor_slope() {
        let profile = ResistanceProfile::standard();
        let points = synthetic_axis_points([800, 900, 1000, 1100, 1200, 1300, 1400, 1500], 1050, 0);
        // Fitted slope is healthy and positive; only the independent
        // signed-anchor cross-check slope is sign-inverted. Drift/asymmetry
        // are well under their warn thresholds so this isolates the sign
        // check from the thermal-drift gate.
        let anchor = SignedAnchorEvidence {
            positive_current_count: 900,
            negative_current_count: -880,
            signed_count_slope_milli: -1070,
            asymmetry_permille: 10,
            repeated_positive_current_count: 900,
            drift_permille: 10,
        };

        let err = fit_resistance_axis(ElectricalAxis::PHASE0, &profile, &points, Some(anchor))
            .unwrap_err();

        assert_eq!(err, ResistanceError::NonPositiveSlope);
    }

    #[test]
    fn two_similarly_negative_axes_are_rejected_at_the_first_fit_not_the_mismatch_gate() {
        // Regression case from the issue: two negative axes that agree with
        // each other in magnitude would sail through evaluate_resistance_result's
        // abs_diff mismatch gate undetected. The fit-layer rejection must catch
        // axis0 before a second fit or the mismatch gate is ever reached.
        let profile = ResistanceProfile::standard();
        let axis0_points =
            synthetic_axis_points([450, 575, 710, 850, 990, 1130, 1275, 1420], -1000, 20);
        let axis1_points =
            synthetic_axis_points([450, 575, 710, 850, 990, 1130, 1275, 1420], -1050, 20);

        let axis0_err =
            fit_resistance_axis(ElectricalAxis::PHASE0, &profile, &axis0_points, None).unwrap_err();
        let axis1_err = fit_resistance_axis(ElectricalAxis::PHASE90, &profile, &axis1_points, None)
            .unwrap_err();

        assert_eq!(axis0_err, ResistanceError::NonPositiveSlope);
        assert_eq!(axis1_err, ResistanceError::NonPositiveSlope);
    }

    #[test]
    fn fit_fails_when_too_few_linear_points() {
        let profile = ResistanceProfile::standard();
        let points = synthetic_axis_points([100, 120, 140, 160, 180, 200, 220, 240], 1050, 0);

        let err = fit_resistance_axis(ElectricalAxis::PHASE0, &profile, &points, None).unwrap_err();

        assert_eq!(err, ResistanceError::InsufficientLinearPoints);
    }

    #[test]
    fn repeated_anchor_sets_thermal_drift_warning() {
        let profile = ResistanceProfile::standard();
        let points = synthetic_axis_points([800, 900, 1000, 1100, 1200, 1300, 1400, 1500], 1050, 0);
        let anchor = SignedAnchorEvidence {
            positive_current_count: 900,
            negative_current_count: -880,
            signed_count_slope_milli: 1070,
            asymmetry_permille: 20,
            repeated_positive_current_count: 960,
            drift_permille: 66,
        };

        let err = fit_resistance_axis(ElectricalAxis::PHASE0, &profile, &points, Some(anchor))
            .unwrap_err();

        assert_eq!(err, ResistanceError::ThermalDrift);
    }

    #[test]
    fn signed_anchor_asymmetry_is_warning_not_default_failure() {
        let profile = ResistanceProfile::standard();
        let points = synthetic_axis_points([800, 900, 1000, 1100, 1200, 1300, 1400, 1500], 1050, 0);
        let anchor = SignedAnchorEvidence {
            positive_current_count: 900,
            negative_current_count: -650,
            signed_count_slope_milli: 1200,
            asymmetry_permille: 320,
            repeated_positive_current_count: 910,
            drift_permille: 11,
        };

        let result =
            fit_resistance_axis(ElectricalAxis::PHASE0, &profile, &points, Some(anchor)).unwrap();

        assert!(
            result
                .warning_flags
                .contains(ResistanceWarningFlags::SIGNED_CURRENT_ASYMMETRY)
        );
    }

    #[test]
    fn electrical_axis_mismatch_is_hard_failure() {
        let profile = ResistanceProfile::standard();
        let a = axis_result_for_test(ElectricalAxis::PHASE0, 1000);
        let b = axis_result_for_test(ElectricalAxis::PHASE90, 1300);

        let err = evaluate_resistance_result(&profile, a, b, 0).unwrap_err();

        assert_eq!(err, ResistanceError::ElectricalAxisMismatch);
    }

    #[test]
    fn status_flags_mask_sets_run_warning() {
        let profile = ResistanceProfile::standard();
        let a = axis_result_for_test(ElectricalAxis::PHASE0, 1000);
        let b = axis_result_for_test(ElectricalAxis::PHASE90, 1000);

        let result =
            evaluate_resistance_result(&profile, a, b, profile.status_flags_warn_mask).unwrap();

        assert!(
            result
                .warning_flags
                .contains(ResistanceWarningFlags::STATUS_FLAGS)
        );
    }
}
