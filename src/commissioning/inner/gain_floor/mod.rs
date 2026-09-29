pub(crate) mod capture;
pub(crate) mod gains;
pub(crate) mod quadratic_fit;

use crate::autotune::{ClosedLoopActivationState, CommissionError, CommissionPhase};
use crate::commissioning::backend::CommissioningBackend;
use crate::commissioning::inner::InnerPhase;
use crate::commissioning::types::StepResult;
use crate::tmc_register_values::MOTION_TORQUE_ADVANCED_PI;
use foci_tmc4671::pid::{PidIGain, PidPGain};

const INTER_STEP_SETTLE_US: u64 = 100_000;

const PLANT_GAIN_STEP_DEVIATION_FAIL_PERMILLE: i64 = 250;

#[derive(Debug, Clone, Copy, Default, defmt::Format)]
pub(crate) struct GainFloorPoint {
    pub(crate) displacement_units: i32,
    pub(crate) current_impulse: i64,
}

#[derive(Debug, Clone, Copy, defmt::Format)]
pub(crate) enum GainFloorState {
    Seat,
    Step {
        index: usize,
        points: [GainFloorPoint; 3],
    },
}

pub(crate) async fn step_gain_floor_measurement<B>(
    phase: &mut InnerPhase,
    backend: &mut B,
) -> StepResult
where
    B: CommissioningBackend,
{
    let (state, i_targets, resistance_current_ceiling_count) = match phase {
        InnerPhase::GainFloorMeasurement {
            state,
            i_targets,
            resistance_current_ceiling_count,
            ..
        } => (*state, *i_targets, *resistance_current_ceiling_count),
        _ => return StepResult::ReadyNow,
    };

    let points = match state {
        GainFloorState::Seat => {
            let InnerPhase::GainFloorMeasurement { capture, .. } = phase else {
                return StepResult::ReadyNow;
            };
            if let Err(e) = run_torque_step(
                backend,
                capture,
                i_targets[0],
                resistance_current_ceiling_count,
            )
            .await
            {
                return super::fail_inner(phase, e);
            }
            if let InnerPhase::GainFloorMeasurement { state, .. } = phase {
                *state = GainFloorState::Step {
                    index: 0,
                    points: [GainFloorPoint::default(); 3],
                };
            }
            return StepResult::At(backend.elapsed_us() + INTER_STEP_SETTLE_US);
        }
        GainFloorState::Step { index, mut points } => {
            let InnerPhase::GainFloorMeasurement { capture, .. } = phase else {
                return StepResult::ReadyNow;
            };
            let point = match capture_and_measure(
                backend,
                capture,
                i_targets[index],
                resistance_current_ceiling_count,
            )
            .await
            {
                Ok(v) => v,
                Err(e) => return super::fail_inner(phase, e),
            };
            points[index] = point;
            if index + 1 < i_targets.len() {
                if let InnerPhase::GainFloorMeasurement { state, .. } = phase {
                    *state = GainFloorState::Step {
                        index: index + 1,
                        points,
                    };
                }
                return StepResult::At(backend.elapsed_us() + INTER_STEP_SETTLE_US);
            }
            points
        }
    };

    let plant_gain = match fit_plant_gain(&points) {
        Ok(v) => v,
        Err(e) => return super::fail_inner(phase, e),
    };
    let accel_per_count: f32 = match gains::accel_raw_per_count_to_accel_per_count(
        plant_gain,
        crate::motion_scale::POSITION_UNITS_PER_REV as f64,
    ) {
        Some(v) => v,
        None => {
            return super::fail_inner(phase, CommissionError::GainFloorInsufficientLinearPoints);
        }
    };

    let velocity_p =
        match gains::compute_velocity_p(accel_per_count, gains::OUTER_FLOOR_REFERENCE_BANDWIDTH_HZ)
        {
            Ok(v) => v,
            Err(e) => return super::fail_inner(phase, e),
        };
    let velocity_i = match gains::compute_velocity_i(velocity_p) {
        Ok(v) => v,
        Err(e) => return super::fail_inner(phase, e),
    };
    let position_p = match gains::compute_position_p(gains::OUTER_FLOOR_REFERENCE_BANDWIDTH_HZ) {
        Ok(v) => v,
        Err(e) => return super::fail_inner(phase, e),
    };
    defmt::info!(
        "gain-floor: accel_per_count_milli={} velocity_p={} velocity_i={} position_p={}",
        (accel_per_count * 1000.0) as i32,
        velocity_p.raw(),
        velocity_i.raw(),
        position_p.raw()
    );

    let vel_limit = backend.operational_velocity_limit();
    let default_current_filter_hz = backend.tmc_config().default_current_filter_hz();
    let torque_filter_hz = backend
        .configured_torque_filter_hz()
        .unwrap_or(default_current_filter_hz);
    let flux_filter_hz = backend
        .configured_flux_filter_hz()
        .unwrap_or(default_current_filter_hz);
    let inner_result = match phase {
        InnerPhase::GainFloorMeasurement { inner_result, .. } => inner_result.take(),
        _ => None,
    };
    let Some(inner_result) = inner_result else {
        return super::fail_inner(phase, CommissionError::Spi);
    };

    let flux_p = PidPGain::new(inner_result.flux_p).unwrap_or(PidPGain::ZERO);
    let flux_i = PidIGain::new(inner_result.flux_i).unwrap_or(PidIGain::ZERO);
    let torque_p = PidPGain::new(inner_result.torque_p).unwrap_or(PidPGain::ZERO);
    let torque_i = PidIGain::new(inner_result.torque_i).unwrap_or(PidIGain::ZERO);

    *phase = InnerPhase::ClosedLoopActivation {
        state: ClosedLoopActivationState::WriteGains,
        flux_p,
        flux_i,
        torque_p,
        torque_i,
        velocity_p,
        velocity_i,
        position_p,
        position_i: PidIGain::ZERO,
        velocity_limit: vel_limit,
        velocity_filter_hz: 0,
        torque_filter_hz,
        position_filter_hz: 0,
        flux_filter_hz,
        inner_result: Some(inner_result),
        attempt: 0,
    };
    StepResult::PhaseChanged(CommissionPhase::ClosedLoopActivation)
}

fn fit_plant_gain(points: &[GainFloorPoint; 3]) -> Result<f64, CommissionError> {
    let [d0, d1, d2] = points.map(|p| p.displacement_units);
    let [i0, i1, i2] = points.map(|p| p.current_impulse);
    if d0 <= 0 || d1 <= 0 || d2 <= d0 || i0 <= 0 || i0 >= i1 || i1 >= i2 {
        defmt::warn!(
            "gain-floor: motion did not grow from the lowest to the highest step: units {} {} {}, impulse {} {} {}",
            d0,
            d1,
            d2,
            i0,
            i1,
            i2
        );
        return Err(CommissionError::GainFloorNonPositiveSlope);
    }
    let pooled = f64::from(d0 + d1 + d2) / (i0 + i1 + i2) as f64;
    for (index, point) in points.iter().enumerate() {
        let ratio = f64::from(point.displacement_units) / point.current_impulse as f64;
        let deviation_permille = ((ratio / pooled - 1.0) * 1000.0).abs();
        if deviation_permille > PLANT_GAIN_STEP_DEVIATION_FAIL_PERMILLE as f64 {
            defmt::warn!(
                "gain-floor: step {} off the pooled plant gain by {} permille",
                index,
                deviation_permille as i32
            );
            return Err(CommissionError::GainFloorInsufficientLinearPoints);
        }
    }
    Ok(pooled)
}

async fn capture_and_measure<B>(
    backend: &mut B,
    capture: &mut capture::CaptureBuffer,
    i_target: i16,
    resistance_current_ceiling_count: u16,
) -> Result<GainFloorPoint, CommissionError>
where
    B: CommissioningBackend,
{
    run_torque_step(backend, capture, i_target, resistance_current_ceiling_count).await?;
    measure_step(capture, i_target)
}

async fn run_torque_step<B>(
    backend: &mut B,
    capture: &mut capture::CaptureBuffer,
    i_target: i16,
    resistance_current_ceiling_count: u16,
) -> Result<(), CommissionError>
where
    B: CommissioningBackend,
{
    use foci_tmc4671::registers::{
        MODE_RAMP_MODE_MOTION, PID_ERROR_PID_FLUX_ERROR_SUM, PID_ERROR_PID_TORQUE_ERROR_SUM,
        PID_TORQUE_FLUX_TARGET, PIDOUT_UQ_UD_LIMITS, PidTorqueFluxTarget,
    };

    let zero_target = PidTorqueFluxTarget::new()
        .with_pid_flux_target(0)
        .with_pid_torque_target(0);
    backend
        .write_register(PID_TORQUE_FLUX_TARGET, zero_target.into())
        .await
        .map_err(|_| CommissionError::Spi)?;

    crate::tmc_control::interlock::clear_status_flags_via_backend(backend).await?;

    crate::commissioning::helpers::write_sub_register_checked(
        backend,
        PID_ERROR_PID_TORQUE_ERROR_SUM,
        0,
    )
    .await
    .map_err(|_| CommissionError::Spi)?;
    crate::commissioning::helpers::write_sub_register_checked(
        backend,
        PID_ERROR_PID_FLUX_ERROR_SUM,
        0,
    )
    .await
    .map_err(|_| CommissionError::Spi)?;

    backend
        .write_register(MODE_RAMP_MODE_MOTION, MOTION_TORQUE_ADVANCED_PI)
        .await
        .map_err(|_| CommissionError::Spi)?;
    backend
        .write_register(PIDOUT_UQ_UD_LIMITS, backend.operational_voltage_limit())
        .await
        .map_err(|_| CommissionError::Spi)?;

    enable_if_interlock_clear(backend)?;

    let torque_step = PidTorqueFluxTarget::new()
        .with_pid_flux_target(0)
        .with_pid_torque_target(i_target);
    if backend
        .write_register(PID_TORQUE_FLUX_TARGET, torque_step.into())
        .await
        .is_err()
    {
        backend.set_enable_pin(false);
        return Err(CommissionError::Spi);
    }

    let capture_result =
        capture::run_capture(backend, capture, i_target, resistance_current_ceiling_count).await;
    let zero_target_result = backend.write_register(PID_TORQUE_FLUX_TARGET, 0).await;
    let limit_result = backend
        .write_register(
            PIDOUT_UQ_UD_LIMITS,
            super::current_loop::CURRENT_SAFE_PIDOUT_LIMIT,
        )
        .await;
    backend.set_enable_pin(false);

    capture_result?;
    if zero_target_result.is_err() || limit_result.is_err() {
        return Err(CommissionError::Spi);
    }
    Ok(())
}

fn enable_if_interlock_clear<B>(backend: &mut B) -> Result<(), CommissionError>
where
    B: CommissioningBackend,
{
    if backend.interlock_tripped() {
        return Err(CommissionError::FaultInterlock);
    }
    backend.set_enable_pin(true);
    Ok(())
}

const CAPTURE_LOG_CHUNK: usize = 28;

fn measure_step(
    capture: &capture::CaptureBuffer,
    i_target: i16,
) -> Result<GainFloorPoint, CommissionError> {
    for chunk in capture.chunks(CAPTURE_LOG_CHUNK) {
        defmt::info!(
            "gain-floor capture: i_target={} samples={=[?]}",
            i_target,
            chunk
        );
    }
    let position_0 = capture.first().map_or(0, |s| s.position);
    let position_samples: heapless::Vec<(u32, i32), 200> = capture
        .iter()
        .map(|s| (s.t_us, s.position.wrapping_sub(position_0)))
        .collect();
    let commanded_sign: i8 = if i_target >= 0 { 1 } else { -1 };
    let peak_to_peak = position_samples.iter().map(|(_, p)| *p).max().unwrap_or(0)
        - position_samples.iter().map(|(_, p)| *p).min().unwrap_or(0);
    let torque_peak = capture.iter().map(|s| s.torque_actual).max().unwrap_or(0);
    let accel_raw_q32 = match quadratic_fit::fit_accel_raw(&position_samples, commanded_sign) {
        Ok(v) => v,
        Err(e) => {
            defmt::warn!(
                "gain-floor step rejected: i_target={} torque_peak={} p2p={} reason={}",
                i_target,
                torque_peak,
                peak_to_peak,
                e
            );
            return Err(match e {
                quadratic_fit::GainFloorFitError::WrongSign => {
                    CommissionError::GainFloorNonPositiveSlope
                }
                quadratic_fit::GainFloorFitError::TooFewSamples
                | quadratic_fit::GainFloorFitError::InsufficientExcitation
                | quadratic_fit::GainFloorFitError::HighResidual => {
                    CommissionError::GainFloorInsufficientLinearPoints
                }
            });
        }
    };
    let window = quadratic_fit::integration_window(&position_samples);
    let point = window_displacement_and_impulse(
        &capture[window.start.min(capture.len())..window.end.min(capture.len())],
        commanded_sign,
    );
    defmt::info!(
        "gain-floor step: i_target={} torque_peak={} p2p={} accel_raw_q32={} displacement={} impulse={}",
        i_target,
        torque_peak,
        peak_to_peak,
        accel_raw_q32,
        point.displacement_units,
        point.current_impulse
    );
    Ok(point)
}

fn window_displacement_and_impulse(
    window: &[capture::CaptureSample],
    commanded_sign: i8,
) -> GainFloorPoint {
    let (Some(first), Some(last)) = (window.first(), window.last()) else {
        return GainFloorPoint::default();
    };
    let sign = i64::from(commanded_sign);
    let displacement_units = last.position.wrapping_sub(first.position) * i32::from(commanded_sign);
    let mut velocity_integral: i64 = 0;
    let mut current_impulse: i64 = 0;
    for pair in window.windows(2) {
        let dt = i64::from(pair[1].t_us - pair[0].t_us);
        let i_prev = i64::from(pair[0].torque_actual) * sign;
        let i_next = i64::from(pair[1].torque_actual) * sign;
        let next_velocity_integral = velocity_integral + (i_prev + i_next) * dt / 2;
        current_impulse += (velocity_integral + next_velocity_integral) * dt / 2;
        velocity_integral = next_velocity_integral;
    }
    GainFloorPoint {
        displacement_units,
        current_impulse,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::autotune::{CommissionProfile, CommissionStatus, InnerSequence, TransientBuffer};
    use crate::commission_dispatch::test_support::{GainFloorCaptureScript, MockBackend, SpiOp};
    use crate::commissioning::backend::{CommissionBoardConfig, CommissionTiming};
    use crate::commissioning::types::{
        AdcResidualEvidence, CommissionTimingSummary, CurrentValidationAxisEvidence,
        InductanceEvidence, InnerResult,
    };
    use crate::selftest::{CommissionDiagnostics, SelftestResults};

    #[test]
    fn decaying_current_inside_the_window_does_not_bias_the_step_ratio() {
        let k = 1e-6_f64;
        let mut capture: capture::CaptureBuffer = heapless::Vec::new();
        for i in 0..84u32 {
            let t = f64::from(i * 60);
            let current = 1000.0 - 0.1 * t;
            let position = k * (1000.0 * t * t / 2.0 - 0.1 * t * t * t / 6.0);
            capture
                .push(capture::CaptureSample {
                    t_us: i * 60,
                    torque_actual: current as i16,
                    flux_actual: 0,
                    position: position as i32,
                })
                .unwrap();
        }
        let point = measure_step(&capture, 1280).expect("measures");
        let ratio = f64::from(point.displacement_units) / point.current_impulse as f64;
        assert!(
            ((ratio - k) / k).abs() < 0.06,
            "ratio {ratio} against plant {k}"
        );
    }

    fn sample_inner_result(resistance_current_ceiling_count: u16) -> InnerResult {
        InnerResult {
            status: CommissionStatus::Accepted,
            r_count_milli: 1706,
            l_count_micro: 1245,
            flux_p: 100,
            flux_i: 10,
            torque_p: 200,
            torque_i: 20,
            candidate_gains_source: crate::autotune::CURRENT_GAINS_SOURCE_MEASURED,
            candidate_axis_split_source: crate::autotune::AXIS_SPLIT_SOURCE_NONE,
            candidate_gains_tier: crate::autotune::CURRENT_GAINS_TIER_MEASURED_SYMMETRIC,
            candidate_measured_axis_split_permille: 1000,
            candidate_applied_axis_split_permille: 1000,
            candidate_axis_split_clamped: 0,
            candidate_flux_p: 100,
            candidate_flux_i: 10,
            candidate_torque_p: 200,
            candidate_torque_i: 20,
            candidate_attempt: 0,
            lambda_us: 100,
            theta_e_us: 50,
            theta_source: 0,
            bandwidth_hz: 600,
            inductance: InductanceEvidence::EMPTY,
            inner_warning_flags: 0,
            timing: [CommissionTimingSummary::NOT_RUN; 3],
            timing_summary: 0,
            resistance_selected_count_slope_milli: 1706,
            resistance_gain_path_count_slope_milli: 1706,
            resistance_axis0_count_slope_milli: 1700,
            resistance_axis1_count_slope_milli: 1712,
            resistance_axis0_intercept_count: 0,
            resistance_axis1_intercept_count: 0,
            resistance_axis0_rmse_permille: 0,
            resistance_axis1_rmse_permille: 0,
            resistance_selected_mask_axis0: 0b1111_1000,
            resistance_selected_mask_axis1: 0b1111_1000,
            resistance_axis0_signed_count_slope_milli: 1702,
            resistance_axis1_signed_count_slope_milli: 1710,
            resistance_axis0_signed_asymmetry_permille: 0,
            resistance_axis1_signed_asymmetry_permille: 0,
            resistance_axis0_drift_permille: 0,
            resistance_axis1_drift_permille: 0,
            resistance_status_flags_or: 0,
            resistance_warning_flags: 0,
            resistance_peak_abs_current_count: 1_440,
            resistance_max_abs_steady_mean_current_count: 1_280,
            resistance_current_ceiling_count,
            resistance_power_stage_tripped: false,
            encoder_alignment_count: 0,
            encoder_alignment_electrical_residual_counts: 0,
            encoder_alignment_stability_counts: 0,
            encoder_alignment_movement_counts: 0,
            encoder_alignment_min_movement_counts: 0,
            encoder_alignment_counts_per_electrical_rev: 0,
            adc_residual_chop0: AdcResidualEvidence::EMPTY,
            adc_residual_chop7: AdcResidualEvidence::EMPTY,
            current_gains_source: crate::autotune::CURRENT_GAINS_SOURCE_MEASURED,
            axis_split_source: crate::autotune::AXIS_SPLIT_SOURCE_NONE,
            gains_tier: crate::autotune::CURRENT_GAINS_TIER_MEASURED_SYMMETRIC,
            measured_axis_split_permille: 1000,
            applied_axis_split_permille: 1000,
            axis_split_clamped: 0,
            current_validation_axes: crate::autotune::CURRENT_VALIDATION_AXIS_TORQUE,
            retry_budget_exhausted: 0,
            current_loop_failure_reason: crate::autotune::CURRENT_LOOP_FAILURE_NONE,
            closed_loop_activation_status: crate::autotune::CLOSED_LOOP_ACTIVATION_STATUS_NOT_RUN,
            closed_loop_activation_position_1: 0,
            closed_loop_activation_position_2: 0,
            closed_loop_activation_drift_count: 0,
            closed_loop_activation_threshold_count: 0,
            closed_loop_activation_runaway: 0,
            current_hold_status: crate::autotune::HOLD_GATE_STATUS_NOT_RUN,
            current_hold_warning_flags: 0,
            current_hold_elapsed_us: 0,
            current_hold_requested_sample_period_us: crate::autotune::HOLD_GATE_SAMPLE_PERIOD_US
                as u16,
            current_hold_sample_count: 0,
            current_hold_position_span_count: 0,
            current_hold_position_drift_count: 0,
            current_hold_torque_mean_count: 0,
            current_hold_torque_rms_count: 0,
            current_hold_torque_peak_to_peak_count: 0,
            current_hold_torque_crossing_count: 0,
            current_hold_flux_mean_count: 0,
            current_hold_flux_rms_count: 0,
            current_hold_flux_peak_to_peak_count: 0,
            current_hold_flux_crossing_count: 0,
            current_hold_status_flags_or: 0,
            current_hold_actionable_status_count: 0,
            current_validation_step_amplitude: 128,
            current_validation_flux_step_amplitude: 128,
            current_validation_torque_step_amplitude: 128,
            current_validation_pidout_limit: 2400,
            current_validation_current_limited: 422,
            current_validation_voltage_limited: 128,
            current_validation_max_p: 3474,
            current_settled_validation_count: 0,
            current_settled_validation:
                [crate::commissioning::types::CurrentValidationSettledSampleEvidence::EMPTY;
                    crate::autotune::CURRENT_VALIDATION_SETTLED_MAX_SAMPLES],
            current_flux_validation_count: 0,
            current_flux_validation: [CurrentValidationAxisEvidence::EMPTY;
                crate::autotune::CURRENT_VALIDATION_MAX_SAMPLES_PER_AXIS],
            current_torque_validation_count: 0,
            current_torque_validation: [CurrentValidationAxisEvidence::EMPTY;
                crate::autotune::CURRENT_VALIDATION_MAX_SAMPLES_PER_AXIS],
        }
    }

    fn gain_floor_phase(i_target_high: i16, resistance_current_ceiling_count: u16) -> InnerPhase {
        InnerPhase::GainFloorMeasurement {
            state: GainFloorState::Seat,
            i_targets: [i_target_high / 4 * 3, i_target_high / 8 * 7, i_target_high],
            resistance_current_ceiling_count,
            inner_result: Some(sample_inner_result(resistance_current_ceiling_count)),
            capture: capture::CaptureBuffer::new(),
        }
    }

    fn ideal_plant() -> GainFloorCaptureScript {
        GainFloorCaptureScript {
            accel_raw_per_us2_per_count: 1e-6,
            accel_raw_offset_per_us2: 0.0,
            tau_current_us: 1.0,
            overshoot_ratio: 1.0,
            creep_below_target: 0,
            creep_excess_accel_per_us2: 0.0,
        }
    }

    fn points(raw: [(i32, i64); 3]) -> [GainFloorPoint; 3] {
        raw.map(|(displacement_units, current_impulse)| GainFloorPoint {
            displacement_units,
            current_impulse,
        })
    }

    #[test]
    fn pooled_plant_gain_is_the_ratio_of_summed_displacement_to_summed_impulse() {
        let k = fit_plant_gain(&points([
            (300, 3_000_000),
            (350, 3_500_000),
            (400, 4_000_000),
        ]))
        .expect("proportional points fit");
        assert!((k - 1e-4).abs() < 1e-12, "{k}");
    }

    #[test]
    fn a_small_friction_offset_stays_inside_the_pooled_gain_bound() {
        let k = fit_plant_gain(&points([
            (280, 3_000_000),
            (330, 3_500_000),
            (380, 4_000_000),
        ]))
        .expect("offset points still fit");
        assert!((k - 0.943e-4).abs() < 0.01e-4, "{k}");
    }

    #[test]
    fn a_step_off_the_pooled_gain_fails_as_insufficient_linear_points() {
        assert_eq!(
            fit_plant_gain(&points([
                (200, 3_000_000),
                (450, 3_500_000),
                (460, 4_000_000)
            ])),
            Err(CommissionError::GainFloorInsufficientLinearPoints)
        );
    }

    #[test]
    fn a_step_just_past_the_deviation_bound_is_rejected_without_truncation() {
        assert_eq!(
            fit_plant_gain(&points([
                (1000, 1_000_000),
                (2000, 2_000_000),
                (5010, 3_000_000)
            ])),
            Err(CommissionError::GainFloorInsufficientLinearPoints)
        );
    }

    #[test]
    fn equal_quantized_displacements_on_growing_impulses_still_pool() {
        let k = fit_plant_gain(&points([
            (770, 352_806_030),
            (770, 419_474_340),
            (1016, 469_650_000),
        ]))
        .expect("quantized displacements pool");
        assert!((k - 2.058e-6).abs() < 0.01e-6, "{k}");
    }

    #[test]
    fn a_step_without_motion_or_a_flat_impulse_fails_as_non_positive_slope() {
        assert_eq!(
            fit_plant_gain(&points([
                (0, 3_000_000),
                (350, 3_500_000),
                (400, 4_000_000)
            ])),
            Err(CommissionError::GainFloorNonPositiveSlope)
        );
        assert_eq!(
            fit_plant_gain(&points([
                (300, 3_000_000),
                (350, 3_000_000),
                (400, 4_000_000)
            ])),
            Err(CommissionError::GainFloorNonPositiveSlope)
        );
    }

    #[test]
    fn a_lowest_step_that_out_accelerates_the_next_fails_as_non_positive_slope() {
        pollster::block_on(async {
            let mut phase = gain_floor_phase(2000, 4_000);
            let mut backend = MockBackend::new();
            backend.advance_elapsed_on_delay = true;
            backend.script_gain_floor_capture = Some(GainFloorCaptureScript {
                creep_below_target: 1500,
                creep_excess_accel_per_us2: 6e-4,
                ..ideal_plant()
            });

            let result = drive_to_completion(&mut phase, &mut backend).await;

            assert!(
                matches!(
                    result,
                    StepResult::InnerFailed(CommissionError::GainFloorNonPositiveSlope)
                ),
                "{result:?}"
            );
            assert!(!backend.enable_pin);
        });
    }

    #[test]
    fn two_soft_steps_under_a_stiff_one_fail_as_insufficient_linear_points() {
        pollster::block_on(async {
            let mut phase = gain_floor_phase(2000, 4_000);
            let mut backend = MockBackend::new();
            backend.advance_elapsed_on_delay = true;
            backend.script_gain_floor_capture = Some(GainFloorCaptureScript {
                creep_below_target: 1750,
                creep_excess_accel_per_us2: -6e-4,
                ..ideal_plant()
            });

            let result = drive_to_completion(&mut phase, &mut backend).await;

            assert!(
                matches!(
                    result,
                    StepResult::InnerFailed(CommissionError::GainFloorInsufficientLinearPoints)
                ),
                "{result:?}"
            );
            assert!(!backend.enable_pin);
        });
    }

    #[test]
    fn ringing_torque_step_fails_with_the_overshoot_reason_below_the_current_ceiling() {
        pollster::block_on(async {
            let mut phase = gain_floor_phase(2000, 4_000);
            let mut backend = MockBackend::new();
            backend.advance_elapsed_on_delay = true;
            backend.script_gain_floor_capture = Some(GainFloorCaptureScript {
                overshoot_ratio: 1.22,
                ..ideal_plant()
            });

            let result = drive_to_completion(&mut phase, &mut backend).await;

            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::GainFloorStepOvershoot)
            ));
            assert!(matches!(
                phase,
                InnerPhase::Failed(CommissionError::GainFloorStepOvershoot)
            ));
            assert!(!backend.enable_pin, "must de-energize on the overshoot");
            let steps_commanded = backend
                .spi_log
                .iter()
                .filter(|op| {
                    matches!(op, SpiOp::Write(addr, val)
                    if *addr == foci_tmc4671::registers::PID_TORQUE_FLUX_TARGET.addr()
                        && (*val >> 16) != 0)
                })
                .count();
            assert_eq!(steps_commanded, 1, "the first ringing step already fails");
        });
    }

    #[test]
    fn mild_overshoot_within_the_gate_still_commissions() {
        pollster::block_on(async {
            let mut phase = gain_floor_phase(2000, 4_000);
            let mut backend = MockBackend::new();
            backend.advance_elapsed_on_delay = true;
            backend.script_gain_floor_capture = Some(GainFloorCaptureScript {
                overshoot_ratio: 1.10,
                ..ideal_plant()
            });

            let result = drive_to_completion(&mut phase, &mut backend).await;

            assert!(matches!(
                result,
                StepResult::PhaseChanged(CommissionPhase::ClosedLoopActivation)
            ));
        });
    }

    async fn drive_to_completion(phase: &mut InnerPhase, backend: &mut MockBackend) -> StepResult {
        let mut result = step_gain_floor_measurement(phase, backend).await;
        while matches!(result, StepResult::At(_)) {
            result = step_gain_floor_measurement(phase, backend).await;
        }
        result
    }

    #[test]
    fn gain_floor_measurement_success_writes_computed_gains() {
        pollster::block_on(async {
            let mut phase = gain_floor_phase(2000, 4_000);
            let mut backend = MockBackend::new();
            backend.advance_elapsed_on_delay = true;
            backend.script_gain_floor_capture = Some(ideal_plant());

            let result = drive_to_completion(&mut phase, &mut backend).await;

            assert!(matches!(
                result,
                StepResult::PhaseChanged(CommissionPhase::ClosedLoopActivation)
            ));
            assert!(
                !backend.enable_pin,
                "must de-energize after a successful measurement"
            );
            match phase {
                InnerPhase::ClosedLoopActivation {
                    velocity_p,
                    velocity_i,
                    position_p,
                    position_i,
                    ..
                } => {
                    assert_ne!(
                        velocity_p.raw(),
                        crate::pid::DEFAULT_OUTER_GAINS.velocity_p.raw()
                    );
                    assert_ne!(
                        position_p.raw(),
                        crate::pid::DEFAULT_OUTER_GAINS.position_p.raw()
                    );
                    assert_eq!(velocity_p.raw(), 100);
                    assert_eq!(velocity_i.raw(), 300);
                    assert_eq!(position_i, PidIGain::ZERO);
                    assert_eq!(position_p.raw(), 164);
                }
                other => panic!("expected ClosedLoopActivation phase, got {other:?}"),
            }
        });
    }

    #[test]
    fn outer_floor_does_not_scale_with_the_current_loop_bandwidth() {
        let mut floors = std::vec::Vec::new();
        for bandwidth_hz in [600.0, 1000.0, 1600.0] {
            pollster::block_on(async {
                let mut phase = gain_floor_phase(2000, 4_000);
                if let InnerPhase::GainFloorMeasurement {
                    inner_result: Some(result),
                    ..
                } = &mut phase
                {
                    result.bandwidth_hz = bandwidth_hz as u32;
                }
                let mut backend = MockBackend::new();
                backend.advance_elapsed_on_delay = true;
                backend.script_gain_floor_capture = Some(ideal_plant());
                let result = drive_to_completion(&mut phase, &mut backend).await;
                assert!(matches!(
                    result,
                    StepResult::PhaseChanged(CommissionPhase::ClosedLoopActivation)
                ));
                match phase {
                    InnerPhase::ClosedLoopActivation {
                        velocity_p,
                        position_p,
                        ..
                    } => floors.push((velocity_p.raw(), position_p.raw())),
                    other => panic!("expected ClosedLoopActivation phase, got {other:?}"),
                }
            });
        }
        assert_eq!(floors[0], floors[1]);
        assert_eq!(floors[1], floors[2]);
    }

    #[test]
    fn floor_applies_the_pwm_relative_default_current_filters() {
        pollster::block_on(async {
            let mut phase = gain_floor_phase(2000, 4_000);
            let mut backend = MockBackend::new();
            backend.advance_elapsed_on_delay = true;
            backend.script_gain_floor_capture = Some(ideal_plant());
            let result = drive_to_completion(&mut phase, &mut backend).await;
            assert!(matches!(
                result,
                StepResult::PhaseChanged(CommissionPhase::ClosedLoopActivation)
            ));
            match phase {
                InnerPhase::ClosedLoopActivation {
                    velocity_filter_hz,
                    torque_filter_hz,
                    position_filter_hz,
                    flux_filter_hz,
                    ..
                } => {
                    assert_eq!(velocity_filter_hz, 0);
                    assert_eq!(torque_filter_hz, 10_000);
                    assert_eq!(position_filter_hz, 0);
                    assert_eq!(flux_filter_hz, 10_000);
                }
                other => panic!("expected ClosedLoopActivation phase, got {other:?}"),
            }
        });
    }

    #[test]
    fn floor_honors_configured_current_filter_overrides() {
        pollster::block_on(async {
            let mut phase = gain_floor_phase(2000, 4_000);
            let mut backend = MockBackend::new();
            backend.advance_elapsed_on_delay = true;
            backend.script_gain_floor_capture = Some(ideal_plant());
            backend.configured_torque_filter_hz = Some(0);
            backend.configured_flux_filter_hz = Some(3000);
            let result = drive_to_completion(&mut phase, &mut backend).await;
            assert!(matches!(
                result,
                StepResult::PhaseChanged(CommissionPhase::ClosedLoopActivation)
            ));
            match phase {
                InnerPhase::ClosedLoopActivation {
                    torque_filter_hz,
                    flux_filter_hz,
                    ..
                } => {
                    assert_eq!(torque_filter_hz, 0);
                    assert_eq!(flux_filter_hz, 3000);
                }
                other => panic!("expected ClosedLoopActivation phase, got {other:?}"),
            }
        });
    }

    #[test]
    fn seating_stroke_precedes_the_three_measured_steps() {
        pollster::block_on(async {
            let mut phase = gain_floor_phase(2000, 4_000);
            let mut backend = MockBackend::new();
            backend.advance_elapsed_on_delay = true;
            backend.script_gain_floor_capture = Some(ideal_plant());

            let result = drive_to_completion(&mut phase, &mut backend).await;

            assert!(matches!(
                result,
                StepResult::PhaseChanged(CommissionPhase::ClosedLoopActivation)
            ));
            let target_addr = foci_tmc4671::registers::PID_TORQUE_FLUX_TARGET.addr();
            let targets: heapless::Vec<u32, 8> = backend
                .spi_log
                .iter()
                .filter_map(|op| match op {
                    SpiOp::Write(a, v) if *a == target_addr && *v != 0 => Some(*v),
                    _ => None,
                })
                .collect();
            assert_eq!(
                targets.len(),
                4,
                "seating stroke, then three measured steps"
            );
            assert_eq!(
                targets[0], targets[1],
                "the seating stroke repeats the lowest step"
            );
            assert!(targets[1] < targets[2] && targets[2] < targets[3]);
            match phase {
                InnerPhase::ClosedLoopActivation {
                    velocity_p,
                    position_p,
                    ..
                } => {
                    assert_eq!(velocity_p.raw(), 100);
                    assert_eq!(position_p.raw(), 164);
                }
                other => panic!("expected ClosedLoopActivation phase, got {other:?}"),
            }
        });
    }

    #[test]
    fn torque_step_runs_under_the_operational_voltage_limit_and_restores_the_safe_one() {
        pollster::block_on(async {
            let mut phase = gain_floor_phase(2000, 4_000);
            let mut backend = MockBackend::new();
            backend.advance_elapsed_on_delay = true;
            backend.script_gain_floor_capture = Some(ideal_plant());
            let operational = backend.operational_voltage_limit();
            assert_ne!(
                operational,
                super::super::current_loop::CURRENT_SAFE_PIDOUT_LIMIT
            );

            let result = step_gain_floor_measurement(&mut phase, &mut backend).await;
            assert!(matches!(result, StepResult::At(_)), "got {result:?}");

            let limit_addr = foci_tmc4671::registers::PIDOUT_UQ_UD_LIMITS.addr();
            let target_addr = foci_tmc4671::registers::PID_TORQUE_FLUX_TARGET.addr();
            let operational_write = backend
                .spi_log
                .iter()
                .position(
                    |op| matches!(op, SpiOp::Write(a, v) if *a == limit_addr && *v == operational),
                )
                .expect("operational voltage limit must be written for the step");
            let step_write = backend
                .spi_log
                .iter()
                .position(|op| matches!(op, SpiOp::Write(a, v) if *a == target_addr && *v != 0))
                .expect("torque step must be commanded");
            let safe_restore = backend
                .spi_log
                .iter()
                .rposition(|op| {
                    matches!(
                        op,
                        SpiOp::Write(a, v)
                            if *a == limit_addr
                                && *v == super::super::current_loop::CURRENT_SAFE_PIDOUT_LIMIT
                    )
                })
                .expect("safe voltage limit must be restored after the capture");
            let disable = backend
                .spi_log
                .iter()
                .rposition(|op| matches!(op, SpiOp::EnablePin(false)))
                .expect("must de-energize after the capture");
            assert!(operational_write < step_write);
            assert!(step_write < safe_restore);
            assert!(safe_restore < disable);
        });
    }

    #[test]
    fn seat_and_three_steps_are_commanded_in_order_with_a_settle_between() {
        pollster::block_on(async {
            let mut phase = gain_floor_phase(2000, 4_000);
            let mut backend = MockBackend::new();
            backend.advance_elapsed_on_delay = true;
            backend.script_gain_floor_capture = Some(ideal_plant());

            for step in ["seating stroke", "first step", "second step"] {
                let result = step_gain_floor_measurement(&mut phase, &mut backend).await;
                let settle_deadline = match result {
                    StepResult::At(deadline) => deadline,
                    other => panic!("expected At(_) after the {step}, got {other:?}"),
                };
                assert!(
                    settle_deadline >= backend.elapsed_us() + INTER_STEP_SETTLE_US,
                    "the next step must wait for the axis to settle after the {step}"
                );
                assert!(!backend.enable_pin, "must be de-energized while settling");
            }

            let last = step_gain_floor_measurement(&mut phase, &mut backend).await;
            assert!(matches!(
                last,
                StepResult::PhaseChanged(CommissionPhase::ClosedLoopActivation)
            ));

            let target_addr = foci_tmc4671::registers::PID_TORQUE_FLUX_TARGET.addr();
            let commanded: std::vec::Vec<i16> = backend
                .spi_log
                .iter()
                .filter_map(|op| match op {
                    SpiOp::Write(a, v) if *a == target_addr && *v != 0 => {
                        Some((*v >> 16) as u16 as i16)
                    }
                    _ => None,
                })
                .collect();
            assert_eq!(commanded, std::vec![1500, 1500, 1750, 2000]);
        });
    }

    #[test]
    fn equal_acceleration_at_both_amplitudes_routes_through_restore_machine() {
        let mut phase = gain_floor_phase(2000, 4000);
        let mut backend = MockBackend::new();
        backend.advance_elapsed_on_delay = true;
        backend.script_gain_floor_capture = Some(GainFloorCaptureScript {
            accel_raw_per_us2_per_count: 0.0,
            accel_raw_offset_per_us2: 2e-3,
            ..ideal_plant()
        });
        assert_routes_through_restore_machine(
            &mut phase,
            &mut backend,
            CommissionError::GainFloorNonPositiveSlope,
        );
    }

    #[test]
    fn computed_gains_do_not_depend_on_the_encoder_resolution() {
        let run = |counts_per_rev: u32| {
            pollster::block_on(async {
                let mut phase = gain_floor_phase(2000, 4_000);
                let mut backend = MockBackend::new();
                backend.advance_elapsed_on_delay = true;
                backend.mechanical_encoder_quadrature_counts_per_rev_value = counts_per_rev;
                backend.script_gain_floor_capture = Some(ideal_plant());
                let _ = drive_to_completion(&mut phase, &mut backend).await;
                match phase {
                    InnerPhase::ClosedLoopActivation { velocity_p, .. } => velocity_p.raw(),
                    other => panic!("expected ClosedLoopActivation, got {other:?}"),
                }
            })
        };
        assert_eq!(run(4000), run(1000));
    }

    #[test]
    fn step_gain_floor_measurement_enters_torque_mode_before_commanding_the_step() {
        pollster::block_on(async {
            let mut phase = gain_floor_phase(2000, 4_000);
            let mut backend = MockBackend::new();
            backend.advance_elapsed_on_delay = true;
            backend.script_gain_floor_capture = Some(ideal_plant());

            let _ = step_gain_floor_measurement(&mut phase, &mut backend).await;

            let mode_write_index = backend.spi_log.iter().position(|op| {
                matches!(
                    op,
                    SpiOp::Write(addr, val)
                        if *addr == foci_tmc4671::registers::MODE_RAMP_MODE_MOTION.addr()
                            && *val == crate::tmc_register_values::MOTION_TORQUE_ADVANCED_PI
                )
            });
            let torque_target_write_index = backend.spi_log.iter().position(|op| {
                matches!(
                    op,
                    SpiOp::Write(addr, val)
                        if *addr == foci_tmc4671::registers::PID_TORQUE_FLUX_TARGET.addr()
                            && *val != 0
                )
            });
            assert!(
                mode_write_index.is_some(),
                "must write MODE_RAMP_MODE_MOTION=MOTION_TORQUE_ADVANCED_PI before commanding a torque step"
            );
            assert!(
                mode_write_index < torque_target_write_index,
                "torque mode must be entered before the nonzero torque step is commanded"
            );
        });
    }

    #[test]
    fn gain_floor_measurement_cleanup_zero_write_failure_fails_commissioning() {
        pollster::block_on(async {
            let mut phase = gain_floor_phase(2000, 4_000);
            let mut backend = MockBackend::new();
            backend.advance_elapsed_on_delay = true;
            backend.script_gain_floor_capture = Some(ideal_plant());
            backend.fail_on_write_occurrence =
                Some((foci_tmc4671::registers::PID_TORQUE_FLUX_TARGET.addr(), 2));

            let result = step_gain_floor_measurement(&mut phase, &mut backend).await;

            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::Spi)
            ));
            assert!(
                !backend.enable_pin,
                "must de-energize even when the cleanup zero-write fails"
            );
        });
    }

    #[test]
    fn gain_floor_measurement_failure_fails_commissioning() {
        pollster::block_on(async {
            let i_target: i16 = 2000;
            let ceiling = 100u16;
            let mut phase = gain_floor_phase(i_target, ceiling);
            let mut backend = MockBackend::new();
            let overcurrent = foci_tmc4671::registers::PidTorqueFluxActual::new()
                .with_pid_flux_actual(0)
                .with_pid_torque_actual(200);
            let _ = backend.registers.insert(
                foci_tmc4671::registers::PID_TORQUE_FLUX_ACTUAL.addr(),
                overcurrent.into(),
            );

            let result = step_gain_floor_measurement(&mut phase, &mut backend).await;

            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::GainFloorPeakCurrentExceeded)
            ));
            assert!(matches!(
                phase,
                InnerPhase::Failed(CommissionError::GainFloorPeakCurrentExceeded)
            ));
            assert!(
                !backend.enable_pin,
                "must de-energize on measurement failure"
            );
        });
    }

    fn assert_routes_through_restore_machine(
        phase: &mut InnerPhase,
        backend: &mut MockBackend,
        expected: CommissionError,
    ) {
        let mut buf = TransientBuffer::new();
        let mut adc_cal = None;
        let mut selftest_results = SelftestResults::default();
        let mut diagnostics = CommissionDiagnostics::default();
        let mut result = StepResult::At(0);
        while matches!(result, StepResult::At(_)) {
            result = pollster::block_on(super::super::step_inner(
                phase,
                CommissionProfile::Balanced,
                InnerSequence::Commission,
                super::super::InnerStepContext {
                    buf: &mut buf,
                    adc_cal: &mut adc_cal,
                    calibrate_entry_gains: None,
                    selftest_results: &mut selftest_results,
                    diagnostics: &mut diagnostics,
                },
                backend,
            ));
        }
        assert!(
            matches!(result, StepResult::InnerFailed(e) if e == expected),
            "expected InnerFailed({expected:?}), got {result:?}"
        );
        assert!(
            matches!(phase, InnerPhase::Failed(e) if *e == expected),
            "expected InnerPhase::Failed({expected:?}), got {phase:?}"
        );
        assert!(
            !backend.enable_pin,
            "{expected:?} must de-energize on measurement failure"
        );
        let last_disable = backend
            .spi_log
            .iter()
            .rposition(|op| matches!(op, SpiOp::EnablePin(false)))
            .expect("must de-energize");
        assert!(
            backend.spi_log[last_disable..].iter().any(|op| matches!(
                op,
                SpiOp::Write(addr, val)
                    if *addr == foci_tmc4671::registers::PIDOUT_UQ_UD_LIMITS.addr()
                        && *val == 1024
            )),
            "{expected:?} must route through restore_safe_state_via_backend \
             (no PIDOUT_UQ_UD_LIMITS=1024 write after the last enable-pin drop)"
        );
    }

    #[test]
    fn gain_floor_insufficient_coverage_routes_through_restore_machine() {
        let mut phase = gain_floor_phase(2000, 4000);
        let mut backend = MockBackend::new();
        backend.advance_elapsed_on_delay = true;
        backend.spi_elapsed_step_us = 35;
        assert_routes_through_restore_machine(
            &mut phase,
            &mut backend,
            CommissionError::GainFloorInsufficientCoverage,
        );
    }

    #[test]
    fn gain_floor_peak_current_exceeded_routes_through_restore_machine() {
        let mut phase = gain_floor_phase(2000, 100);
        let mut backend = MockBackend::new();
        let overcurrent = foci_tmc4671::registers::PidTorqueFluxActual::new()
            .with_pid_flux_actual(0)
            .with_pid_torque_actual(200);
        let _ = backend.registers.insert(
            foci_tmc4671::registers::PID_TORQUE_FLUX_ACTUAL.addr(),
            overcurrent.into(),
        );
        assert_routes_through_restore_machine(
            &mut phase,
            &mut backend,
            CommissionError::GainFloorPeakCurrentExceeded,
        );
    }

    #[test]
    fn gain_floor_motion_envelope_exceeded_routes_through_restore_machine() {
        let mut phase = gain_floor_phase(2000, 4000);
        let mut backend = MockBackend::new();
        backend.read_position_actual_sequence.push(0).unwrap();
        backend.read_position_actual_sequence.push(40_000).unwrap();
        assert_routes_through_restore_machine(
            &mut phase,
            &mut backend,
            CommissionError::GainFloorMotionEnvelopeExceeded,
        );
    }

    #[test]
    fn gain_floor_non_positive_slope_routes_through_restore_machine() {
        let mut phase = gain_floor_phase(2000, 4000);
        let mut backend = MockBackend::new();
        backend.advance_elapsed_on_delay = true;
        backend.script_gain_floor_capture = Some(GainFloorCaptureScript {
            accel_raw_per_us2_per_count: -1e-6,
            ..ideal_plant()
        });
        assert_routes_through_restore_machine(
            &mut phase,
            &mut backend,
            CommissionError::GainFloorNonPositiveSlope,
        );
    }

    #[test]
    fn gain_floor_insufficient_linear_points_routes_through_restore_machine() {
        let mut phase = gain_floor_phase(2000, 4000);
        let mut backend = MockBackend::new();
        backend.advance_elapsed_on_delay = true;
        backend.script_gain_floor_capture = Some(GainFloorCaptureScript {
            accel_raw_per_us2_per_count: 0.0,
            ..ideal_plant()
        });
        assert_routes_through_restore_machine(
            &mut phase,
            &mut backend,
            CommissionError::GainFloorInsufficientLinearPoints,
        );
    }

    #[test]
    fn gain_floor_velocity_gain_unrepresentable_routes_through_restore_machine() {
        let mut phase = gain_floor_phase(20_000, 30_000);
        let mut backend = MockBackend::new();
        backend.advance_elapsed_on_delay = true;
        backend.script_gain_floor_capture = Some(GainFloorCaptureScript {
            accel_raw_per_us2_per_count: 3e-9,
            ..ideal_plant()
        });
        assert_routes_through_restore_machine(
            &mut phase,
            &mut backend,
            CommissionError::GainFloorVelocityGainUnrepresentable,
        );
    }

    #[test]
    fn gain_floor_step_overshoot_routes_through_restore_machine() {
        let mut phase = gain_floor_phase(2000, 4_000);
        let mut backend = MockBackend::new();
        backend.advance_elapsed_on_delay = true;
        backend.script_gain_floor_capture = Some(GainFloorCaptureScript {
            overshoot_ratio: 1.35,
            ..ideal_plant()
        });
        assert_routes_through_restore_machine(
            &mut phase,
            &mut backend,
            CommissionError::GainFloorStepOvershoot,
        );
    }
}
