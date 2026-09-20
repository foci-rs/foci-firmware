//! Commissioning fallback gain-floor measurement.
//!
//! Runs two fixed-grid current-step captures (`capture::run_capture`) at a
//! low and a high torque amplitude, fits each position response
//! (`quadratic_fit::fit_accel_raw`), takes the differential acceleration
//! per measured torque count between them, then derives
//! `velocity_p`/`position_p` (`gains::compute_velocity_p`/
//! `compute_position_p`) from it and the current-loop bandwidth CurrentTune
//! already measured, and hands off to `InnerPhase::ClosedLoopActivation`.
//! Reached only when `current_loop.rs`'s bandwidth gate has already
//! confirmed the board's current-loop bandwidth is inside the supported
//! range; a measurement failure here fails commissioning outright (no
//! fallback substitution -- that only happens for the static
//! bandwidth-out-of-range case, handled before this phase is ever entered).
//!
//! Two amplitudes rather than one because on a real axis a single torque
//! step measures `(T - T_friction) / J`, not `T / J`: Coulomb friction and
//! breakaway shift every fitted acceleration by the same current-independent
//! amount, which biases a single-step `accel_per_count` low and therefore the
//! derived `velocity_p` high. The difference between two steps cancels that
//! offset.

pub(crate) mod capture;
pub(crate) mod gains;
pub(crate) mod quadratic_fit;

use crate::autotune::{ClosedLoopActivationState, CommissionError, CommissionPhase};
use crate::commissioning::backend::CommissioningBackend;
use crate::commissioning::inner::InnerPhase;
use crate::commissioning::types::StepResult;
use foci_tmc4671::pid::{PidIGain, PidPGain};

/// Coast time so the high step starts from rest with the same stiction
/// history as the low one.
const INTER_STEP_SETTLE_US: u64 = 100_000;

/// `step_gain_floor_measurement` sub-state.
#[derive(Debug, Clone, Copy, defmt::Format)]
pub(crate) enum GainFloorState {
    Start,
    HighStep {
        low_accel_raw_q32: i64,
        low_mean_torque_actual: i32,
    },
}

/// Drive the commissioning fallback gain-floor measurement phase.
pub(crate) async fn step_gain_floor_measurement<B>(
    phase: &mut InnerPhase,
    backend: &mut B,
) -> StepResult
where
    B: CommissioningBackend,
{
    let (state, i_target_low, i_target_high, resistance_current_ceiling_count) = match phase {
        InnerPhase::GainFloorMeasurement {
            state,
            i_target_low,
            i_target_high,
            resistance_current_ceiling_count,
            ..
        } => (
            *state,
            *i_target_low,
            *i_target_high,
            *resistance_current_ceiling_count,
        ),
        _ => return StepResult::ReadyNow,
    };

    let (low_accel_raw_q32, low_mean_torque_actual) =
        match state {
            GainFloorState::Start => {
                let capture =
                    match run_torque_step(backend, i_target_low, resistance_current_ceiling_count)
                        .await
                    {
                        Ok(buf) => buf,
                        Err(e) => return super::fail_inner(phase, e),
                    };
                let (accel_raw_q32, mean_torque_actual) =
                    match fit_capture_accel(&capture, i_target_low) {
                        Ok(v) => v,
                        Err(e) => return super::fail_inner(phase, e),
                    };
                if let InnerPhase::GainFloorMeasurement { state, .. } = phase {
                    *state = GainFloorState::HighStep {
                        low_accel_raw_q32: accel_raw_q32,
                        low_mean_torque_actual: mean_torque_actual,
                    };
                }
                return StepResult::At(backend.elapsed_us() + INTER_STEP_SETTLE_US);
            }
            GainFloorState::HighStep {
                low_accel_raw_q32,
                low_mean_torque_actual,
            } => (low_accel_raw_q32, low_mean_torque_actual),
        };

    let capture =
        match run_torque_step(backend, i_target_high, resistance_current_ceiling_count).await {
            Ok(buf) => buf,
            Err(e) => return super::fail_inner(phase, e),
        };
    let (high_accel_raw_q32, high_mean_torque_actual) =
        match fit_capture_accel(&capture, i_target_high) {
            Ok(v) => v,
            Err(e) => return super::fail_inner(phase, e),
        };
    // The current loop does not hold the commanded target while the rotor
    // moves, so the differential divides by the current the motor actually
    // saw, not by the command.
    let mean_torque_delta = high_mean_torque_actual - low_mean_torque_actual;
    if high_accel_raw_q32 <= low_accel_raw_q32 || mean_torque_delta <= 0 {
        defmt::warn!(
            "gain-floor: high step did not out-accelerate low step (accel {} <= {}, current delta {})",
            high_accel_raw_q32,
            low_accel_raw_q32,
            mean_torque_delta
        );
        return super::fail_inner(phase, CommissionError::GainFloorNonPositiveSlope);
    }

    let accel_per_count: f32 = match gains::accel_raw_q32_to_accel_per_count(
        high_accel_raw_q32 - low_accel_raw_q32,
        crate::motion_scale::POSITION_UNITS_PER_REV as f64,
        mean_torque_delta.min(i32::from(i16::MAX)) as i16,
    ) {
        Some(v) => v,
        None => {
            return super::fail_inner(phase, CommissionError::GainFloorInsufficientLinearPoints);
        }
    };

    let bw_current_hz = match phase {
        InnerPhase::GainFloorMeasurement {
            inner_result: Some(result),
            ..
        } => result.bandwidth_hz,
        _ => 0,
    };
    if bw_current_hz == 0 {
        return super::fail_inner(phase, CommissionError::GainFloorCurrentFitRejected);
    }
    let velocity_p = match gains::compute_velocity_p(accel_per_count, bw_current_hz) {
        Ok(v) => v,
        Err(e) => return super::fail_inner(phase, e),
    };
    let velocity_i = match gains::compute_velocity_i(velocity_p) {
        Ok(v) => v,
        Err(e) => return super::fail_inner(phase, e),
    };
    let position_p = match gains::compute_position_p(bw_current_hz) {
        Ok(v) => v,
        Err(e) => return super::fail_inner(phase, e),
    };
    defmt::info!(
        "gain-floor: accel_per_count_milli={} bw_current_hz={} velocity_p={} velocity_i={} position_p={}",
        (accel_per_count * 1000.0) as i32,
        bw_current_hz,
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

    // The current-loop flux/torque gains only survive into this phase as
    // InnerResult's raw register values (Finalize already validated them as
    // PidPGain/PidIGain before writing InnerResult); re-validating here
    // instead of threading the CurrentGains struct through the phase avoids
    // widening InnerPhase::GainFloorMeasurement beyond what the bandwidth
    // gate needs, and a previously-valid raw value can never fail this
    // re-check.
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

async fn run_torque_step<B>(
    backend: &mut B,
    i_target: i16,
    resistance_current_ceiling_count: u16,
) -> Result<capture::CaptureBuffer, CommissionError>
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

    // Current-validate teardown leaves the chip stopped under the safe
    // voltage limit; the first ignores the torque target and the second
    // clips the current loop below anything that moves a loaded axis.
    backend
        .write_register(
            MODE_RAMP_MODE_MOTION,
            super::current_loop::CURRENT_TORQUE_MODE,
        )
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

    // ClosedLoopActivation never re-zeroes the target itself.
    let capture_result =
        capture::run_capture(backend, i_target, resistance_current_ceiling_count).await;
    let zero_target_result = backend.write_register(PID_TORQUE_FLUX_TARGET, 0).await;
    let limit_result = backend
        .write_register(
            PIDOUT_UQ_UD_LIMITS,
            super::current_loop::CURRENT_SAFE_PIDOUT_LIMIT,
        )
        .await;
    backend.set_enable_pin(false);

    let capture = capture_result?;
    if zero_target_result.is_err() || limit_result.is_err() {
        return Err(CommissionError::Spi);
    }
    Ok(capture)
}

/// Synchronous so no fault can latch between the interlock check and the
/// enable-pin assertion.
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

fn fit_capture_accel(
    capture: &capture::CaptureBuffer,
    i_target: i16,
) -> Result<(i64, i32), CommissionError> {
    let position_0 = capture.first().map_or(0, |s| s.position);
    let position_samples: heapless::Vec<(u32, i32), 200> = capture
        .iter()
        .map(|s| (s.t_us, s.position.wrapping_sub(position_0)))
        .collect();
    let commanded_sign: i8 = if i_target >= 0 { 1 } else { -1 };
    let peak_to_peak = position_samples.iter().map(|(_, p)| *p).max().unwrap_or(0)
        - position_samples.iter().map(|(_, p)| *p).min().unwrap_or(0);
    let torque_peak = capture.iter().map(|s| s.torque_actual).max().unwrap_or(0);
    let mean_torque_actual = capture
        .iter()
        .map(|s| i32::from(s.torque_actual))
        .sum::<i32>()
        / capture.len().max(1) as i32;
    match quadratic_fit::fit_accel_raw(&position_samples, commanded_sign) {
        Ok(accel_raw_q32) => {
            defmt::info!(
                "gain-floor step: i_target={} torque_peak={} torque_mean={} p2p={} accel_raw_q32={}",
                i_target,
                torque_peak,
                mean_torque_actual,
                peak_to_peak,
                accel_raw_q32
            );
            Ok((accel_raw_q32, mean_torque_actual))
        }
        Err(e) => {
            defmt::warn!(
                "gain-floor step rejected: i_target={} torque_peak={} p2p={} reason={}",
                i_target,
                torque_peak,
                peak_to_peak,
                e
            );
            Err(match e {
                quadratic_fit::GainFloorFitError::WrongSign => {
                    CommissionError::GainFloorNonPositiveSlope
                }
                quadratic_fit::GainFloorFitError::TooFewSamples
                | quadratic_fit::GainFloorFitError::InsufficientExcitation
                | quadratic_fit::GainFloorFitError::HighResidual => {
                    CommissionError::GainFloorInsufficientLinearPoints
                }
            })
        }
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
            ringing_count: 0,
            bandwidth_hz: 600,
            tau_e_us: 730,
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
            state: GainFloorState::Start,
            i_target_low: i_target_high / 2,
            i_target_high,
            resistance_current_ceiling_count,
            inner_result: Some(sample_inner_result(resistance_current_ceiling_count)),
        }
    }

    fn ideal_plant() -> GainFloorCaptureScript {
        GainFloorCaptureScript {
            accel_raw_per_us2_per_count: 1e-6,
            accel_raw_offset_per_us2: 0.0,
            tau_current_us: 1.0,
        }
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
            let mut backend = MockBackend::new_with_bandwidth_hz(600.0);
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
                    assert_eq!(velocity_i.raw(), 54);
                    assert_eq!(position_i, PidIGain::ZERO);
                    // Steps at 1000 and 2000 counts give accel_raw 1e-3 and
                    // 2e-3 units/us^2: 1e-6 per count = 95.9 rad/s^2 per
                    // count; tau 1000us gives ~158 Hz, omega_target ~99.3
                    // rad/s; velocity_p = ceil(256*99.3*2pi/(60*95.9)) = 28,
                    // position_p = ceil(9.93*60*256/65536) = 3.
                    // Steps at 1000 and 2000 counts (the mock's current
                    // reaches target on the second sample, mean ~988/1976)
                    // give accel_raw 1e-3 and 2e-3 units/us^2: ~97 rad/s^2
                    // per measured count; InnerResult's 600 Hz current loop
                    // targets 10 Hz, velocity_p = ceil(256*62.8*2pi/(60*97))
                    // = 18, position_p = ceil(418.9*60*256/65536) = 99.
                    assert_eq!(velocity_p.raw(), 18);
                    assert_eq!(position_p.raw(), 99);
                }
                other => panic!("expected ClosedLoopActivation phase, got {other:?}"),
            }
        });
    }

    #[test]
    fn torque_step_runs_under_the_operational_voltage_limit_and_restores_the_safe_one() {
        pollster::block_on(async {
            let mut phase = gain_floor_phase(2000, 4_000);
            let mut backend = MockBackend::new_with_bandwidth_hz(600.0);
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
    fn low_and_high_steps_are_commanded_in_order_with_a_settle_between() {
        pollster::block_on(async {
            let mut phase = gain_floor_phase(2000, 4_000);
            let mut backend = MockBackend::new_with_bandwidth_hz(600.0);
            backend.advance_elapsed_on_delay = true;
            backend.script_gain_floor_capture = Some(ideal_plant());

            let first = step_gain_floor_measurement(&mut phase, &mut backend).await;
            let settle_deadline = match first {
                StepResult::At(deadline) => deadline,
                other => panic!("expected At(_) after the low step, got {other:?}"),
            };
            assert!(
                settle_deadline >= backend.elapsed_us() + INTER_STEP_SETTLE_US,
                "high step must wait for the axis to settle"
            );
            assert!(!backend.enable_pin, "must be de-energized while settling");

            let second = step_gain_floor_measurement(&mut phase, &mut backend).await;
            assert!(matches!(
                second,
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
            assert_eq!(commanded, std::vec![1000, 2000]);
        });
    }

    #[test]
    fn current_independent_acceleration_offset_cancels_in_the_differential() {
        let run = |script: GainFloorCaptureScript| {
            pollster::block_on(async {
                let mut phase = gain_floor_phase(2000, 4_000);
                let mut backend = MockBackend::new_with_bandwidth_hz(600.0);
                backend.advance_elapsed_on_delay = true;
                backend.script_gain_floor_capture = Some(script);
                let result = drive_to_completion(&mut phase, &mut backend).await;
                assert!(matches!(
                    result,
                    StepResult::PhaseChanged(CommissionPhase::ClosedLoopActivation)
                ));
                match phase {
                    InnerPhase::ClosedLoopActivation { velocity_p, .. } => velocity_p.raw(),
                    other => panic!("expected ClosedLoopActivation, got {other:?}"),
                }
            })
        };
        let ideal = run(ideal_plant());
        let with_offset = run(GainFloorCaptureScript {
            accel_raw_offset_per_us2: -4e-4,
            ..ideal_plant()
        });
        assert_eq!(with_offset, ideal);
    }

    #[test]
    fn equal_acceleration_at_both_amplitudes_routes_through_restore_machine() {
        let mut phase = gain_floor_phase(2000, 4000);
        let mut backend = MockBackend::new_with_bandwidth_hz(600.0);
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
                let mut backend = MockBackend::new_with_bandwidth_hz(600.0);
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
            let mut backend = MockBackend::new_with_bandwidth_hz(600.0);
            backend.advance_elapsed_on_delay = true;
            backend.script_gain_floor_capture = Some(ideal_plant());

            let _ = step_gain_floor_measurement(&mut phase, &mut backend).await;

            // `CurrentValidate`'s own teardown leaves `MODE_RAMP_MODE_MOTION`
            // at 0 (stopped) -- see `restore_current_validation_safe_state`.
            // Without explicitly re-entering torque mode here, a commanded
            // `PID_TORQUE_FLUX_TARGET` has no effect on the real chip: it
            // stays in stopped mode and ignores the target entirely. This is
            // exactly the failure found live on Ouroboros hardware:
            // `torque_actual` stayed noise-level regardless of the commanded
            // target, and the rotor never moved.
            let mode_write_index = backend.spi_log.iter().position(|op| {
                matches!(
                    op,
                    SpiOp::Write(addr, val)
                        if *addr == foci_tmc4671::registers::MODE_RAMP_MODE_MOTION.addr()
                            && *val == super::super::current_loop::CURRENT_TORQUE_MODE
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
                "must write MODE_RAMP_MODE_MOTION=CURRENT_TORQUE_MODE before commanding a torque step"
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
            let mut backend = MockBackend::new_with_bandwidth_hz(600.0);
            backend.advance_elapsed_on_delay = true;
            backend.script_gain_floor_capture = Some(ideal_plant());
            // PID_TORQUE_FLUX_TARGET is written twice before the post-capture
            // cleanup write (the initial zero, then the commanded torque
            // step); failing the third (zero-based occurrence 2) targets
            // only that cleanup write, with the capture itself succeeding.
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
            // Ceiling well below the default (unscripted) torque register
            // value of 200, so `run_capture` trips
            // `GainFloorPeakCurrentExceeded` on its very first slot.
            let ceiling = 100u16;
            let mut phase = gain_floor_phase(i_target, ceiling);
            let mut backend = MockBackend::new_with_bandwidth_hz(600.0);
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

    /// Shared assertion for the `gain_floor_*_routes_through_restore_machine`
    /// tests below: drives the failure through `super::super::step_inner`
    /// (not the phase-local `step_gain_floor_measurement` handler directly),
    /// following `coil.rs`'s `coil_fault_records_raw_current_sample` calling
    /// pattern. `step_inner` unconditionally calls
    /// `tmc_safe_state::restore_safe_state_via_backend` whenever the
    /// dispatched result is `StepResult::InnerFailed(_)`, so routing through
    /// it (rather than the phase-local handler alone) is what actually
    /// proves the restore-machine boundary ran for this failure.
    ///
    /// `!backend.enable_pin` alone does not distinguish the wrapper's restore
    /// from `gain_floor`'s own `set_enable_pin(false)` calls on every exit
    /// path, so this also checks for `PIDOUT_UQ_UD_LIMITS = 1024`, a register
    /// write `restore_safe_state_via_backend` performs (`tmc_safe_state.rs`,
    /// `SafeStateRestoreStep::advance` step 7) that no `gain_floor` code path
    /// ever touches -- the same `SpiOp::Write` idiom
    /// `commission_dispatch/tests.rs`'s safety-fault-teardown tests use to
    /// confirm a specific register reload happened.
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
                &mut buf,
                CommissionProfile::Balanced,
                InnerSequence::Commission,
                &mut adc_cal,
                None,
                &mut selftest_results,
                &mut diagnostics,
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
        // The phase's own safe-limit restore precedes its enable-pin drop;
        // only the wrapper's restore machine writes the limit after it.
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
        // Slot 0 arrives on time (now = 0) but its two reads do not
        // complete within the 60us slot budget (post-read elapsed = 70) --
        // the actual safety-relevant timing violation, not mere arrival
        // lateness (which alone is not a trip condition; see capture.rs's
        // own `late_arrival_that_still_completes_within_budget_does_not_trip`).
        backend.set_elapsed_us_sequence(&[0, 0, 70]);
        assert_routes_through_restore_machine(
            &mut phase,
            &mut backend,
            CommissionError::GainFloorInsufficientCoverage,
        );
    }

    #[test]
    fn gain_floor_peak_current_exceeded_routes_through_restore_machine() {
        let mut phase = gain_floor_phase(2000, 100);
        let mut backend = MockBackend::new_with_bandwidth_hz(600.0);
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
        let mut backend = MockBackend::new_with_bandwidth_hz(600.0);
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
        let mut backend = MockBackend::new_with_bandwidth_hz(600.0);
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
    fn missing_current_bandwidth_routes_through_restore_machine() {
        let mut phase = gain_floor_phase(2000, 4_000);
        if let InnerPhase::GainFloorMeasurement {
            inner_result: Some(result),
            ..
        } = &mut phase
        {
            result.bandwidth_hz = 0;
        }
        let mut backend = MockBackend::new_with_bandwidth_hz(600.0);
        backend.advance_elapsed_on_delay = true;
        backend.script_gain_floor_capture = Some(ideal_plant());
        assert_routes_through_restore_machine(
            &mut phase,
            &mut backend,
            CommissionError::GainFloorCurrentFitRejected,
        );
    }

    #[test]
    fn gain_floor_velocity_gain_unrepresentable_routes_through_restore_machine() {
        // accel_per_count is divided by both i_target and encoder counts-per-rev
        // (`accel_raw_q32_to_accel_per_count`); maxing both out shrinks it far
        // enough that velocity_p_raw overflows PidPGain::MAX, without touching
        // the position/current fits' own excitation or residual checks (both
        // depend only on the dimensionless current ratio, not on these scales).
        let mut phase = gain_floor_phase(2000, 4_000);
        let mut backend = MockBackend::new_with_bandwidth_hz(600.0);
        backend.advance_elapsed_on_delay = true;
        backend.script_gain_floor_capture = Some(GainFloorCaptureScript {
            accel_raw_per_us2_per_count: 3e-10,
            accel_raw_offset_per_us2: 1e-4,
            ..ideal_plant()
        });
        assert_routes_through_restore_machine(
            &mut phase,
            &mut backend,
            CommissionError::GainFloorVelocityGainUnrepresentable,
        );
    }
}
