//! Count-space impedance response fitting.

use super::profile::ImpedanceProfile;
use super::types::{
    IMPEDANCE_PROFILE_POINTS, IMPEDANCE_SAMPLE_COUNT, ImpedanceError, ImpedanceInjectionPoint,
    ImpedanceSample, ImpedanceWarningFlags,
};

const TURN_RADIANS: f64 = core::f64::consts::PI * 2.0;
const COVERAGE_FAIL_PERMILLE: u16 = 100;
/// Relative scalar/vector `2f` mismatch is not meaningful below this saliency
/// because count-space quantization can dominate both fitted saliency values.
const SCALAR_2F_SALIENCY_FLOOR: f64 = 0.02;
const EPSILON: f64 = 1.0e-9;

/// Fitted count-space response for one impedance injection point.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct ImpedanceFitPoint {
    /// Major response axis normalized to counts per 1000 UD counts.
    pub axis_max_count_per_ud_milli: u32,
    /// Minor response axis normalized to counts per 1000 UD counts.
    pub axis_min_count_per_ud_milli: u32,
    /// Major-axis stationary-frame electrical angle in 16-bit turn counts.
    pub axis_angle_electrical_counts: u16,
    /// Fitted-model residual as permille of fitted signal RMS.
    pub residual_permille: u16,
    /// Phase coverage determinant score, where 1000 is ideal circular coverage.
    pub coverage_permille: u16,
    /// Scalar `2f` saliency mismatch as permille.
    pub scalar_2f_mismatch_permille: u16,
    /// Mean transformed-current magnitude normalized to counts per 1000 UD counts.
    pub magnitude_mean_count_per_ud_milli: u32,
    /// `2f` transformed-current magnitude ripple normalized to counts per 1000 UD counts.
    pub magnitude_2f_count_per_ud_milli: u32,
    /// Transformed-current magnitude `2f` fit residual in permille.
    pub magnitude_2f_residual_permille: u16,
    /// Transformed-current magnitude `2f` coverage score.
    pub magnitude_2f_coverage_permille: u16,
    /// Non-fatal quality warnings accumulated after hard gates pass.
    pub warning_flags: ImpedanceWarningFlags,
}

/// Aggregate impedance result across all fitted injection points.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct ImpedanceFitSummary {
    /// Selected major response axis normalized to counts per 1000 UD counts.
    pub selected_axis_max_count_per_ud_milli: u32,
    /// Selected minor response axis normalized to counts per 1000 UD counts.
    pub selected_axis_min_count_per_ud_milli: u32,
    /// Worse cross-point spread of independently normalized major and minor axes.
    pub repeatability_permille: u16,
    /// Aggregate non-fatal quality warnings.
    pub warning_flags: ImpedanceWarningFlags,
}

/// Fit-gate failure with any bounded fit evidence computed before rejection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct ImpedanceFitFailure {
    /// Stable failure reason reported by the diagnostic.
    pub error: ImpedanceError,
    /// Fit evidence available when the failure occurred after basis coverage.
    pub fit: Option<ImpedanceFitPoint>,
}

#[derive(Clone, Copy)]
struct HarmonicBasis {
    mean_cos: f64,
    mean_sin: f64,
    cos_cos: f64,
    sin_sin: f64,
    cos_sin: f64,
    determinant: f64,
    coverage_permille: u16,
}

#[derive(Clone, Copy)]
struct ChannelFit {
    offset: f64,
    cos_coeff: f64,
    sin_coeff: f64,
}

#[derive(Clone, Copy)]
struct MagnitudeFit {
    mean_count: f64,
    ripple_count: f64,
    residual_permille: u16,
    coverage_permille: u16,
}

/// Fit one impedance injection capture into a count-space response ellipse.
pub fn fit_impedance_point(
    profile: &ImpedanceProfile,
    point: ImpedanceInjectionPoint,
    samples: &[ImpedanceSample; IMPEDANCE_SAMPLE_COUNT],
) -> Result<ImpedanceFitPoint, ImpedanceError> {
    fit_impedance_point_with_transformed_current(profile, point, samples, samples)
}

pub fn fit_impedance_point_with_transformed_current(
    profile: &ImpedanceProfile,
    point: ImpedanceInjectionPoint,
    raw_samples: &[ImpedanceSample; IMPEDANCE_SAMPLE_COUNT],
    transformed_samples: &[ImpedanceSample; IMPEDANCE_SAMPLE_COUNT],
) -> Result<ImpedanceFitPoint, ImpedanceError> {
    fit_impedance_point_with_evidence(profile, point, raw_samples, transformed_samples)
        .map_err(|failure| failure.error)
}

pub(crate) fn fit_impedance_point_with_evidence(
    profile: &ImpedanceProfile,
    point: ImpedanceInjectionPoint,
    raw_samples: &[ImpedanceSample; IMPEDANCE_SAMPLE_COUNT],
    transformed_samples: &[ImpedanceSample; IMPEDANCE_SAMPLE_COUNT],
) -> Result<ImpedanceFitPoint, ImpedanceFitFailure> {
    let fundamental = harmonic_basis(1.0, raw_samples);
    if fundamental.coverage_permille < COVERAGE_FAIL_PERMILLE {
        return Err(ImpedanceFitFailure {
            error: ImpedanceError::PoorCoverage,
            fit: None,
        });
    }

    let ch0 = fit_channel(1.0, raw_samples, fundamental, Channel::Ch0);
    let ch1 = fit_channel(1.0, raw_samples, fundamental, Channel::Ch1);
    let axes = response_axes(ch0, ch1);
    let axis_max_count = axes.axis_max_count;
    let axis_min_count = axes.axis_min_count;

    let residual_permille = residual_permille(raw_samples, ch0, ch1);

    let scalar_2f_mismatch_permille = scalar_2f_mismatch_permille(raw_samples, axes, ch0, ch1);
    let magnitude_2f = fit_magnitude_2f(transformed_samples);

    let mut warning_flags = ImpedanceWarningFlags::NONE;
    if residual_permille > profile.residual_warn_permille {
        warning_flags.insert(ImpedanceWarningFlags::HIGH_RESIDUAL);
    }
    if scalar_2f_mismatch_permille > profile.scalar_2f_mismatch_warn_permille {
        warning_flags.insert(ImpedanceWarningFlags::SCALAR_2F_MISMATCH);
    }

    let fit = ImpedanceFitPoint {
        axis_max_count_per_ud_milli: count_per_ud_milli(axis_max_count, point.ud_abs),
        axis_min_count_per_ud_milli: count_per_ud_milli(axis_min_count, point.ud_abs),
        axis_angle_electrical_counts: angle_to_u16_turn(axes.axis_angle_radians),
        residual_permille,
        coverage_permille: fundamental.coverage_permille,
        scalar_2f_mismatch_permille,
        magnitude_mean_count_per_ud_milli: count_per_ud_milli(
            magnitude_2f.mean_count,
            point.ud_abs,
        ),
        magnitude_2f_count_per_ud_milli: count_per_ud_milli(
            magnitude_2f.ripple_count,
            point.ud_abs,
        ),
        magnitude_2f_residual_permille: magnitude_2f.residual_permille,
        magnitude_2f_coverage_permille: magnitude_2f.coverage_permille,
        warning_flags,
    };

    if axis_max_count < f64::from(profile.min_signal_count) {
        return Err(ImpedanceFitFailure {
            error: ImpedanceError::LowSignal,
            fit: Some(fit),
        });
    }
    if residual_permille > profile.residual_fail_permille {
        return Err(ImpedanceFitFailure {
            error: ImpedanceError::HighResidual,
            fit: Some(fit),
        });
    }
    if scalar_2f_mismatch_permille > profile.scalar_2f_mismatch_fail_permille {
        return Err(ImpedanceFitFailure {
            error: ImpedanceError::Scalar2fMismatch,
            fit: Some(fit),
        });
    }

    Ok(fit)
}

/// Evaluate fitted impedance points into one aggregate diagnostic summary.
pub fn evaluate_impedance_result(
    profile: &ImpedanceProfile,
    fits: &[ImpedanceFitPoint; IMPEDANCE_PROFILE_POINTS],
) -> Result<ImpedanceFitSummary, ImpedanceError> {
    let selected_axis_max_count_per_ud_milli =
        average_axis_count_per_ud_milli(fits, AxisSelector::Max);
    let selected_axis_min_count_per_ud_milli =
        average_axis_count_per_ud_milli(fits, AxisSelector::Min);
    let repeatability_permille = repeatability_permille(
        fits,
        selected_axis_max_count_per_ud_milli,
        selected_axis_min_count_per_ud_milli,
    );

    if repeatability_permille > profile.repeatability_fail_permille {
        return Err(ImpedanceError::RepeatabilityMismatch);
    }

    let mut warning_flags = ImpedanceWarningFlags::NONE;
    for fit in fits {
        warning_flags.insert(fit.warning_flags);
    }
    if repeatability_permille > profile.repeatability_warn_permille {
        warning_flags.insert(ImpedanceWarningFlags::REPEATABILITY);
    }
    if !profile.scale_metadata_validated {
        warning_flags.insert(ImpedanceWarningFlags::PHYSICAL_PROJECTION_UNAVAILABLE);
    }

    Ok(ImpedanceFitSummary {
        selected_axis_max_count_per_ud_milli,
        selected_axis_min_count_per_ud_milli,
        repeatability_permille,
        warning_flags,
    })
}

#[derive(Clone, Copy)]
enum Channel {
    Ch0,
    Ch1,
}

#[derive(Clone, Copy)]
struct ResponseAxes {
    axis_max_count: f64,
    axis_min_count: f64,
    axis_angle_radians: f64,
}

#[derive(Clone, Copy)]
enum AxisSelector {
    Max,
    Min,
}

fn phase_radians(openloop_phi: u16, harmonic: f64) -> f64 {
    TURN_RADIANS * harmonic * f64::from(openloop_phi) / 65_536.0
}

fn harmonic_basis(
    harmonic: f64,
    samples: &[ImpedanceSample; IMPEDANCE_SAMPLE_COUNT],
) -> HarmonicBasis {
    let sample_count = IMPEDANCE_SAMPLE_COUNT as f64;
    let mut sum_cos = 0.0;
    let mut sum_sin = 0.0;

    for sample in samples {
        let phase = phase_radians(sample.openloop_phi, harmonic);
        sum_cos += libm::cos(phase);
        sum_sin += libm::sin(phase);
    }

    let mean_cos = sum_cos / sample_count;
    let mean_sin = sum_sin / sample_count;
    let mut cos_cos = 0.0;
    let mut sin_sin = 0.0;
    let mut cos_sin = 0.0;

    for sample in samples {
        let phase = phase_radians(sample.openloop_phi, harmonic);
        let centered_cos = libm::cos(phase) - mean_cos;
        let centered_sin = libm::sin(phase) - mean_sin;
        cos_cos += centered_cos * centered_cos;
        sin_sin += centered_sin * centered_sin;
        cos_sin += centered_cos * centered_sin;
    }

    let determinant = cos_cos * sin_sin - cos_sin * cos_sin;
    let trace = cos_cos + sin_sin;
    let coverage = if determinant <= EPSILON || trace <= EPSILON {
        0.0
    } else {
        (4.0 * determinant / (trace * trace)).clamp(0.0, 1.0)
    };

    HarmonicBasis {
        mean_cos,
        mean_sin,
        cos_cos,
        sin_sin,
        cos_sin,
        determinant,
        coverage_permille: permille(coverage),
    }
}

fn fit_channel(
    harmonic: f64,
    samples: &[ImpedanceSample; IMPEDANCE_SAMPLE_COUNT],
    basis: HarmonicBasis,
    channel: Channel,
) -> ChannelFit {
    let sample_count = IMPEDANCE_SAMPLE_COUNT as f64;
    let mut sum_current = 0.0;

    for sample in samples {
        sum_current += channel_value(*sample, channel);
    }

    let mean_current = sum_current / sample_count;
    let mut current_cos = 0.0;
    let mut current_sin = 0.0;

    for sample in samples {
        let phase = phase_radians(sample.openloop_phi, harmonic);
        let centered_current = channel_value(*sample, channel) - mean_current;
        let centered_cos = libm::cos(phase) - basis.mean_cos;
        let centered_sin = libm::sin(phase) - basis.mean_sin;
        current_cos += centered_current * centered_cos;
        current_sin += centered_current * centered_sin;
    }

    let cos_coeff = (current_cos * basis.sin_sin - current_sin * basis.cos_sin) / basis.determinant;
    let sin_coeff = (current_sin * basis.cos_cos - current_cos * basis.cos_sin) / basis.determinant;
    let offset = mean_current - cos_coeff * basis.mean_cos - sin_coeff * basis.mean_sin;

    ChannelFit {
        offset,
        cos_coeff,
        sin_coeff,
    }
}

fn channel_value(sample: ImpedanceSample, channel: Channel) -> f64 {
    match channel {
        Channel::Ch0 => f64::from(sample.i_ch0),
        Channel::Ch1 => f64::from(sample.i_ch1),
    }
}

fn response_axes(ch0: ChannelFit, ch1: ChannelFit) -> ResponseAxes {
    let covariance_xx = ch0.cos_coeff * ch0.cos_coeff + ch0.sin_coeff * ch0.sin_coeff;
    let covariance_xy = ch0.cos_coeff * ch1.cos_coeff + ch0.sin_coeff * ch1.sin_coeff;
    let covariance_yy = ch1.cos_coeff * ch1.cos_coeff + ch1.sin_coeff * ch1.sin_coeff;
    let trace = covariance_xx + covariance_yy;
    let delta = covariance_xx - covariance_yy;
    let root = libm::sqrt(delta * delta + 4.0 * covariance_xy * covariance_xy);
    let axis_max_sq = ((trace + root) * 0.5).max(0.0);
    let axis_min_sq = ((trace - root) * 0.5).max(0.0);
    let axis_angle_radians = 0.5 * libm::atan2(2.0 * covariance_xy, delta);

    ResponseAxes {
        axis_max_count: libm::sqrt(axis_max_sq),
        axis_min_count: libm::sqrt(axis_min_sq),
        axis_angle_radians,
    }
}

fn residual_permille(
    samples: &[ImpedanceSample; IMPEDANCE_SAMPLE_COUNT],
    ch0: ChannelFit,
    ch1: ChannelFit,
) -> u16 {
    let mut residual_energy = 0.0;
    let mut signal_energy = 0.0;

    for sample in samples {
        let phase = phase_radians(sample.openloop_phi, 1.0);
        let cos_phase = libm::cos(phase);
        let sin_phase = libm::sin(phase);
        let model_ch0 = ch0.offset + ch0.cos_coeff * cos_phase + ch0.sin_coeff * sin_phase;
        let model_ch1 = ch1.offset + ch1.cos_coeff * cos_phase + ch1.sin_coeff * sin_phase;
        let signal_ch0 = model_ch0 - ch0.offset;
        let signal_ch1 = model_ch1 - ch1.offset;
        let residual_ch0 = f64::from(sample.i_ch0) - model_ch0;
        let residual_ch1 = f64::from(sample.i_ch1) - model_ch1;

        residual_energy += residual_ch0 * residual_ch0 + residual_ch1 * residual_ch1;
        signal_energy += signal_ch0 * signal_ch0 + signal_ch1 * signal_ch1;
    }

    if residual_energy <= EPSILON {
        0
    } else {
        permille(libm::sqrt(residual_energy / signal_energy.max(EPSILON)))
    }
}

fn scalar_2f_mismatch_permille(
    samples: &[ImpedanceSample; IMPEDANCE_SAMPLE_COUNT],
    axes: ResponseAxes,
    ch0: ChannelFit,
    ch1: ChannelFit,
) -> u16 {
    let axis_max_sq = axes.axis_max_count * axes.axis_max_count;
    let axis_min_sq = axes.axis_min_count * axes.axis_min_count;
    let vector_saliency = (axis_max_sq - axis_min_sq) / (axis_max_sq + axis_min_sq).max(EPSILON);
    let mean_mag_sq = mean_ac_magnitude_squared(samples, ch0.offset, ch1.offset);
    let scalar_amplitude = scalar_2f_amplitude(samples, mean_mag_sq, ch0.offset, ch1.offset);
    let scalar_saliency = scalar_amplitude / mean_mag_sq.max(EPSILON);
    let saliency = vector_saliency.max(scalar_saliency);
    if saliency < SCALAR_2F_SALIENCY_FLOOR {
        return 0;
    }
    let mismatch = libm::fabs(vector_saliency - scalar_saliency) / saliency.max(EPSILON);

    permille(mismatch)
}

fn mean_ac_magnitude_squared(
    samples: &[ImpedanceSample; IMPEDANCE_SAMPLE_COUNT],
    offset_ch0: f64,
    offset_ch1: f64,
) -> f64 {
    let mut sum = 0.0;
    for sample in samples {
        let ch0 = f64::from(sample.i_ch0) - offset_ch0;
        let ch1 = f64::from(sample.i_ch1) - offset_ch1;
        sum += ch0 * ch0 + ch1 * ch1;
    }
    sum / IMPEDANCE_SAMPLE_COUNT as f64
}

fn scalar_2f_amplitude(
    samples: &[ImpedanceSample; IMPEDANCE_SAMPLE_COUNT],
    mean_mag_sq: f64,
    offset_ch0: f64,
    offset_ch1: f64,
) -> f64 {
    let basis = harmonic_basis(2.0, samples);
    if basis.determinant <= EPSILON {
        return 0.0;
    }

    let mut mag_cos = 0.0;
    let mut mag_sin = 0.0;
    for sample in samples {
        let phase = phase_radians(sample.openloop_phi, 2.0);
        let ch0 = f64::from(sample.i_ch0) - offset_ch0;
        let ch1 = f64::from(sample.i_ch1) - offset_ch1;
        let centered_mag_sq = ch0 * ch0 + ch1 * ch1 - mean_mag_sq;
        let centered_cos = libm::cos(phase) - basis.mean_cos;
        let centered_sin = libm::sin(phase) - basis.mean_sin;
        mag_cos += centered_mag_sq * centered_cos;
        mag_sin += centered_mag_sq * centered_sin;
    }

    let cos_coeff = (mag_cos * basis.sin_sin - mag_sin * basis.cos_sin) / basis.determinant;
    let sin_coeff = (mag_sin * basis.cos_cos - mag_cos * basis.cos_sin) / basis.determinant;

    libm::sqrt(cos_coeff * cos_coeff + sin_coeff * sin_coeff)
}

fn fit_magnitude_2f(samples: &[ImpedanceSample; IMPEDANCE_SAMPLE_COUNT]) -> MagnitudeFit {
    let basis = harmonic_basis(2.0, samples);
    if basis.determinant <= EPSILON {
        return MagnitudeFit {
            mean_count: 0.0,
            ripple_count: 0.0,
            residual_permille: 0,
            coverage_permille: basis.coverage_permille,
        };
    }

    let sample_count = IMPEDANCE_SAMPLE_COUNT as f64;
    let mut sum_magnitude = 0.0;
    for sample in samples {
        sum_magnitude += sample_magnitude(*sample);
    }
    let mean_magnitude = sum_magnitude / sample_count;

    let mut magnitude_cos = 0.0;
    let mut magnitude_sin = 0.0;
    for sample in samples {
        let phase = phase_radians(sample.openloop_phi, 2.0);
        let centered_magnitude = sample_magnitude(*sample) - mean_magnitude;
        let centered_cos = libm::cos(phase) - basis.mean_cos;
        let centered_sin = libm::sin(phase) - basis.mean_sin;
        magnitude_cos += centered_magnitude * centered_cos;
        magnitude_sin += centered_magnitude * centered_sin;
    }

    let cos_coeff =
        (magnitude_cos * basis.sin_sin - magnitude_sin * basis.cos_sin) / basis.determinant;
    let sin_coeff =
        (magnitude_sin * basis.cos_cos - magnitude_cos * basis.cos_sin) / basis.determinant;
    let offset = mean_magnitude - cos_coeff * basis.mean_cos - sin_coeff * basis.mean_sin;

    let mut residual_energy = 0.0;
    let mut signal_energy = 0.0;
    for sample in samples {
        let phase = phase_radians(sample.openloop_phi, 2.0);
        let model = offset + cos_coeff * libm::cos(phase) + sin_coeff * libm::sin(phase);
        let signal = model - offset;
        let residual = sample_magnitude(*sample) - model;
        residual_energy += residual * residual;
        signal_energy += signal * signal;
    }

    MagnitudeFit {
        mean_count: offset,
        ripple_count: libm::sqrt(cos_coeff * cos_coeff + sin_coeff * sin_coeff),
        residual_permille: if residual_energy <= EPSILON {
            0
        } else {
            permille(libm::sqrt(residual_energy / signal_energy.max(EPSILON)))
        },
        coverage_permille: basis.coverage_permille,
    }
}

fn sample_magnitude(sample: ImpedanceSample) -> f64 {
    let ch0 = f64::from(sample.i_ch0);
    let ch1 = f64::from(sample.i_ch1);
    libm::sqrt(ch0 * ch0 + ch1 * ch1)
}

fn count_per_ud_milli(axis_count: f64, ud_abs: u16) -> u32 {
    if ud_abs == 0 {
        return 0;
    }
    round_to_u32(axis_count * 1_000.0 / f64::from(ud_abs))
}

fn angle_to_u16_turn(mut radians: f64) -> u16 {
    while radians < 0.0 {
        radians += TURN_RADIANS;
    }
    while radians >= TURN_RADIANS {
        radians -= TURN_RADIANS;
    }

    round_to_u32(radians * 65_536.0 / TURN_RADIANS) as u16
}

fn permille(value: f64) -> u16 {
    round_to_u32(value * 1_000.0).min(u32::from(u16::MAX)) as u16
}

fn round_to_u32(value: f64) -> u32 {
    if value <= 0.0 {
        0
    } else if value >= f64::from(u32::MAX) {
        u32::MAX
    } else {
        libm::floor(value + 0.5) as u32
    }
}

fn average_axis_count_per_ud_milli(
    fits: &[ImpedanceFitPoint; IMPEDANCE_PROFILE_POINTS],
    selector: AxisSelector,
) -> u32 {
    let mut sum = 0u64;
    for fit in fits {
        sum += u64::from(axis_count_per_ud_milli(*fit, selector));
    }

    ((sum + (IMPEDANCE_PROFILE_POINTS as u64 / 2)) / IMPEDANCE_PROFILE_POINTS as u64) as u32
}

fn repeatability_permille(
    fits: &[ImpedanceFitPoint; IMPEDANCE_PROFILE_POINTS],
    selected_axis_max_count_per_ud_milli: u32,
    selected_axis_min_count_per_ud_milli: u32,
) -> u16 {
    axis_spread_permille(
        fits,
        AxisSelector::Max,
        selected_axis_max_count_per_ud_milli,
    )
    .max(axis_spread_permille(
        fits,
        AxisSelector::Min,
        selected_axis_min_count_per_ud_milli,
    ))
}

fn axis_spread_permille(
    fits: &[ImpedanceFitPoint; IMPEDANCE_PROFILE_POINTS],
    selector: AxisSelector,
    selected_axis_count_per_ud_milli: u32,
) -> u16 {
    let spread = axis_spread_count_per_ud_milli(fits, selector);
    let denominator = selected_axis_count_per_ud_milli.max(1);

    permille(f64::from(spread) / f64::from(denominator))
}

fn axis_spread_count_per_ud_milli(
    fits: &[ImpedanceFitPoint; IMPEDANCE_PROFILE_POINTS],
    selector: AxisSelector,
) -> u32 {
    let mut min = u32::MAX;
    let mut max = 0u32;
    for fit in fits {
        let axis = axis_count_per_ud_milli(*fit, selector);
        if axis < min {
            min = axis;
        }
        if axis > max {
            max = axis;
        }
    }

    max - min
}

fn axis_count_per_ud_milli(fit: ImpedanceFitPoint, selector: AxisSelector) -> u32 {
    match selector {
        AxisSelector::Max => fit.axis_max_count_per_ud_milli,
        AxisSelector::Min => fit.axis_min_count_per_ud_milli,
    }
}
