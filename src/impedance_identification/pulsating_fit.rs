//! Pure fixed-angle pulsating impedance fit.

use super::PulsatingImpedanceProfile;
use super::types::{
    ImpedanceError, ImpedanceWarningFlags, PULSATING_IMPEDANCE_PROFILE_POINTS,
    PulsatingAngleObservation, PulsatingFitSummary,
};

const FULL_TWO_THETA_BIN_MASK: u8 = 0b1111;
const TURN_COUNTS: f64 = 65_536.0;
const HALF_TURN_COUNTS: u16 = 32_768;

#[derive(Debug, Clone, Copy)]
struct FitBasis {
    cos_two_theta: f64,
    sin_two_theta: f64,
}

/// Fit a fixed-angle pulsating impedance summary from reduced lock-in observations.
pub fn fit_pulsating_impedance(
    profile: &PulsatingImpedanceProfile,
    observations: &[PulsatingAngleObservation; PULSATING_IMPEDANCE_PROFILE_POINTS],
) -> Result<PulsatingFitSummary, ImpedanceError> {
    if two_theta_bin_mask(observations) != FULL_TWO_THETA_BIN_MASK {
        return Err(ImpedanceError::PoorCoverage);
    }

    let valid_count = observations
        .iter()
        .filter(|observation| observation.valid)
        .count();
    if valid_count < 4 {
        return Err(ImpedanceError::PoorCoverage);
    }

    let angle_coverage_permille = coverage_permille(observations);
    if angle_coverage_permille < profile.angle_coverage_fail_permille {
        return Err(ImpedanceError::PoorCoverage);
    }

    let [mean, cos_coeff, sin_coeff] = fit_response(observations)?;
    let saliency = libm::sqrt(cos_coeff * cos_coeff + sin_coeff * sin_coeff);
    let axis_angle_electrical_counts = angle_to_u16(0.5 * libm::atan2(sin_coeff, cos_coeff));
    let axis_deviation_counts = half_turn_deviation_counts(axis_angle_electrical_counts);
    let saliency_rounded = round_to_u32(saliency);
    let saliency_determinable =
        saliency_rounded >= u32::from(profile.saliency_axis_determinability_count_per_ud_milli);

    if saliency_determinable && axis_deviation_counts > profile.axis_deviation_fail_counts {
        return Err(ImpedanceError::AxisDeviation);
    }

    let mut warning_flags = ImpedanceWarningFlags::NONE;
    for observation in observations {
        warning_flags.insert(observation.warning_flags);
    }
    if !saliency_determinable {
        warning_flags.insert(ImpedanceWarningFlags::AXIS_UNDETERMINED);
    }

    let residual_permille = max_residual_permille(observations);
    if residual_permille > profile.residual_fail_permille {
        return Err(ImpedanceError::HighResidual);
    }
    if residual_permille > profile.residual_warn_permille {
        warning_flags.insert(ImpedanceWarningFlags::HIGH_RESIDUAL);
    }

    let repeatability_permille = duplicate_zero_repeatability(observations);
    if repeatability_permille > profile.repeatability_fail_permille {
        return Err(ImpedanceError::RepeatabilityMismatch);
    }
    if repeatability_permille > profile.repeatability_warn_permille {
        warning_flags.insert(ImpedanceWarningFlags::REPEATABILITY);
    }

    let axis_max = mean + saliency;
    let axis_min = mean - saliency;

    Ok(PulsatingFitSummary {
        mean_response_count_per_ud_milli: round_to_u32(mean),
        axis_max_count_per_ud_milli: round_to_u32(axis_max),
        axis_min_count_per_ud_milli: round_to_u32(axis_min),
        saliency_permille: permille(saliency, mean),
        axis_angle_electrical_counts,
        axis_deviation_counts,
        saliency_determinable,
        angle_coverage_permille,
        repeatability_permille,
        warning_flags,
    })
}

fn fit_basis(angle_electrical_counts: u16) -> FitBasis {
    let theta = core::f64::consts::TAU * f64::from(angle_electrical_counts) / TURN_COUNTS;
    let two_theta = 2.0 * theta;

    FitBasis {
        cos_two_theta: libm::cos(two_theta),
        sin_two_theta: libm::sin(two_theta),
    }
}

fn two_theta_bin_mask(
    observations: &[PulsatingAngleObservation; PULSATING_IMPEDANCE_PROFILE_POINTS],
) -> u8 {
    let mut mask = 0u8;
    for observation in observations {
        if observation.valid {
            let two_theta_counts =
                u32::from(observation.angle_electrical_counts).wrapping_mul(2) & 0xffff;
            let bin = (two_theta_counts >> 14) as u8;
            mask |= 1 << bin;
        }
    }
    mask
}

fn fit_response(
    observations: &[PulsatingAngleObservation; PULSATING_IMPEDANCE_PROFILE_POINTS],
) -> Result<[f64; 3], ImpedanceError> {
    let mut normal = [[0.0; 3]; 3];
    let mut rhs = [0.0; 3];

    for observation in observations {
        if !observation.valid {
            continue;
        }

        let basis = fit_basis(observation.angle_electrical_counts);
        let row = [1.0, basis.cos_two_theta, basis.sin_two_theta];
        let response = f64::from(observation.inductive_response_count_per_ud_milli);

        for row_index in 0..3 {
            rhs[row_index] += row[row_index] * response;
            for col_index in 0..3 {
                normal[row_index][col_index] += row[row_index] * row[col_index];
            }
        }
    }

    solve_3x3(normal, rhs).ok_or(ImpedanceError::PoorCoverage)
}

fn solve_3x3(mut matrix: [[f64; 3]; 3], mut rhs: [f64; 3]) -> Option<[f64; 3]> {
    for pivot in 0..3 {
        let mut best = pivot;
        let mut best_abs = libm::fabs(matrix[pivot][pivot]);
        for (row, matrix_row) in matrix.iter().enumerate().skip(pivot + 1) {
            let candidate_abs = libm::fabs(matrix_row[pivot]);
            if candidate_abs > best_abs {
                best = row;
                best_abs = candidate_abs;
            }
        }

        if best_abs < 1.0e-9 {
            return None;
        }

        if best != pivot {
            matrix.swap(pivot, best);
            rhs.swap(pivot, best);
        }

        let pivot_value = matrix[pivot][pivot];
        for value in matrix[pivot].iter_mut().skip(pivot) {
            *value /= pivot_value;
        }
        rhs[pivot] /= pivot_value;

        let pivot_row = matrix[pivot];
        let pivot_rhs = rhs[pivot];
        for (row, matrix_row) in matrix.iter_mut().enumerate() {
            if row == pivot {
                continue;
            }

            let factor = matrix_row[pivot];
            for (value, pivot_value) in matrix_row.iter_mut().zip(pivot_row).skip(pivot) {
                *value -= factor * pivot_value;
            }
            rhs[row] -= factor * pivot_rhs;
        }
    }

    Some(rhs)
}

fn coverage_permille(
    observations: &[PulsatingAngleObservation; PULSATING_IMPEDANCE_PROFILE_POINTS],
) -> u16 {
    let mut count = 0u32;
    let mut sum_cos = 0.0;
    let mut sum_sin = 0.0;

    for observation in observations {
        if observation.valid {
            let basis = fit_basis(observation.angle_electrical_counts);
            count += 1;
            sum_cos += basis.cos_two_theta;
            sum_sin += basis.sin_two_theta;
        }
    }

    if count < 4 {
        return 0;
    }

    let mean_cos = sum_cos / f64::from(count);
    let mean_sin = sum_sin / f64::from(count);
    let mut centered_cos_cos = 0.0;
    let mut centered_sin_sin = 0.0;
    let mut centered_cos_sin = 0.0;

    for observation in observations {
        if observation.valid {
            let basis = fit_basis(observation.angle_electrical_counts);
            let cos_centered = basis.cos_two_theta - mean_cos;
            let sin_centered = basis.sin_two_theta - mean_sin;
            centered_cos_cos += cos_centered * cos_centered;
            centered_sin_sin += sin_centered * sin_centered;
            centered_cos_sin += cos_centered * sin_centered;
        }
    }

    let determinant = centered_cos_cos * centered_sin_sin - centered_cos_sin * centered_cos_sin;
    let ideal_determinant = f64::from(count) * f64::from(count) * 0.25;
    permille(determinant, ideal_determinant)
}

fn angle_to_u16(radians: f64) -> u16 {
    let turns = radians / core::f64::consts::TAU;
    let counts = libm::round(turns * TURN_COUNTS) as i32;
    counts.rem_euclid(65_536) as u16
}

fn half_turn_deviation_counts(angle_electrical_counts: u16) -> u16 {
    let folded = angle_electrical_counts % HALF_TURN_COUNTS;
    folded.min(HALF_TURN_COUNTS - folded)
}

fn duplicate_zero_repeatability(
    observations: &[PulsatingAngleObservation; PULSATING_IMPEDANCE_PROFILE_POINTS],
) -> u16 {
    let mut first_response: Option<u32> = None;
    let mut first_angle = 0u16;

    for observation in observations {
        if !observation.valid {
            continue;
        }

        if let Some(response) = first_response {
            if observation.angle_electrical_counts == first_angle {
                let current = observation.inductive_response_count_per_ud_milli;
                let diff = response.abs_diff(current);
                let mean = (u64::from(response) + u64::from(current)) / 2;
                return permille(diff as f64, mean as f64);
            }
        } else {
            first_response = Some(observation.inductive_response_count_per_ud_milli);
            first_angle = observation.angle_electrical_counts;
        }
    }

    0
}

fn max_residual_permille(
    observations: &[PulsatingAngleObservation; PULSATING_IMPEDANCE_PROFILE_POINTS],
) -> u16 {
    observations
        .iter()
        .filter(|observation| observation.valid)
        .map(|observation| observation.residual_permille)
        .max()
        .unwrap_or(0)
}

fn permille(numerator: f64, denominator: f64) -> u16 {
    if denominator <= 0.0 {
        return 0;
    }

    let value = round_to_u32(numerator * 1_000.0 / denominator);
    value.min(u32::from(u16::MAX)) as u16
}

fn round_to_u32(value: f64) -> u32 {
    if value <= 0.0 {
        return 0;
    }

    let rounded = libm::round(value);
    if rounded >= f64::from(u32::MAX) {
        u32::MAX
    } else {
        rounded as u32
    }
}
