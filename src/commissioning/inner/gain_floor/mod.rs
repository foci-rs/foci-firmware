//! Commissioning fallback gain-floor measurement.
//!
//! Runs the fixed-grid current-step capture (`capture::run_capture`), fits
//! the position response (`quadratic_fit::fit_accel_raw`) and the current
//! response (`current_response_fit::fit_current_response`), then derives
//! `velocity_p`/`position_p` (`gains::compute_velocity_p`/
//! `compute_position_p`) and hands off to `InnerPhase::ClosedLoopActivation`.
//! Reached only when `current_loop.rs`'s bandwidth gate has already
//! confirmed the board's current-loop bandwidth is inside the supported
//! range; a measurement failure here fails commissioning outright (no
//! fallback substitution -- that only happens for the static
//! bandwidth-out-of-range case, handled before this phase is ever entered).

pub(crate) mod capture;
pub(crate) mod current_response_fit;
pub(crate) mod gains;
pub(crate) mod quadratic_fit;

use crate::autotune::{ClosedLoopActivationState, CommissionError, CommissionPhase};
use crate::commissioning::backend::CommissioningBackend;
use crate::commissioning::inner::InnerPhase;
use crate::commissioning::types::StepResult;
use foci_tmc4671::pid::{PidIGain, PidPGain};

/// `step_gain_floor_measurement` sub-state.
///
/// Unlike `CurrentValidate`'s `AxisSampleDelay`/`AxisSampleSettle` states
/// (which return `StepResult::At(deadline_us)` so the outer dispatcher
/// re-drives them across a multi-millisecond mechanical settle),
/// `capture::run_capture` awaits its own fixed-grid slot delays directly via
/// `backend.delay_us().await` -- the same cooperative-yield primitive
/// `current_loop.rs`'s `read_axis_directional_evidence_for_gate` already
/// uses for its own multi-sample loop, per `CommissionTiming::delay_us`'s
/// own doc comment ("used by ... any other firmware-side wait that must
/// yield to higher-priority RTIC tasks"). A single dispatcher tick can
/// therefore run the whole safety sequence, capture, and fit pipeline to
/// completion in one `step_gain_floor_measurement` call, so this phase needs
/// no additional per-tick polling state of its own.
#[derive(Debug, defmt::Format)]
pub(crate) enum GainFloorState {
    Start,
}

/// Drive the commissioning fallback gain-floor measurement phase.
///
/// Safety sequence (mirrors `current_loop.rs`'s `AxisStepSetup` and
/// `electrical_identification.rs`'s inductance open-loop-ramp arming for the
/// interlock-check/enable ordering, and
/// `closed_loop_activation.rs`'s `ZeroAndClear` for the
/// `write_sub_register_checked` integrator-clear pattern): zero the
/// torque/flux target, clear STATUS_FLAGS via the race-safe helper, clear
/// the torque/flux PID integrators, a final `interlock_tripped()` check with
/// no await before `set_enable_pin(true)`, command the bounded torque step,
/// run the capture, then unconditionally re-zero the target and de-energize
/// on every exit path before mapping the outcome.
pub(crate) async fn step_gain_floor_measurement<B>(
    phase: &mut InnerPhase,
    backend: &mut B,
) -> StepResult
where
    B: CommissioningBackend,
{
    let (i_target, resistance_current_ceiling_count, inner_result) = match phase {
        InnerPhase::GainFloorMeasurement {
            i_target,
            resistance_current_ceiling_count,
            inner_result,
            ..
        } => (
            *i_target,
            *resistance_current_ceiling_count,
            inner_result.take(),
        ),
        _ => return StepResult::ReadyNow,
    };

    // Zero the torque/flux target first -- no interlock guard needed, since
    // this write does not clear a fault or enable the motor and so carries
    // none of the race the interlock check below exists to prevent.
    let zero_target = foci_tmc4671::registers::PidTorqueFluxTarget::new()
        .with_pid_flux_target(0)
        .with_pid_torque_target(0);
    if backend
        .write_register(
            foci_tmc4671::registers::PID_TORQUE_FLUX_TARGET,
            zero_target.into(),
        )
        .await
        .is_err()
    {
        return super::fail_inner(phase, CommissionError::Spi);
    }

    // Clear STATUS_FLAGS via the race-safe helper -- not a direct write --
    // because a fault can latch during any awaited operation, and this is
    // the codebase's one designated place that check-and-clear happens
    // atomically with respect to that race.
    if let Err(e) = crate::tmc_control::interlock::clear_status_flags_via_backend(backend).await {
        return super::fail_inner(phase, e);
    }

    // Clear the torque/flux PID integrators before enabling, the same
    // `write_sub_register_checked` pattern `ClosedLoopActivationState::ZeroAndClear`
    // uses for its own (different) closed-position-loop entry. This phase
    // never enters velocity or position mode, so only the torque/flux sums
    // are cleared here.
    if crate::commissioning::helpers::write_sub_register_checked(
        backend,
        foci_tmc4671::registers::PID_ERROR_PID_TORQUE_ERROR_SUM,
        0,
    )
    .await
    .is_err()
        || crate::commissioning::helpers::write_sub_register_checked(
            backend,
            foci_tmc4671::registers::PID_ERROR_PID_FLUX_ERROR_SUM,
            0,
        )
        .await
        .is_err()
    {
        return super::fail_inner(phase, CommissionError::Spi);
    }

    // Final interlock check immediately before enabling -- a fault can have
    // latched during either awaited integrator-clear write above, and this
    // is the last point before the motor is actually energized. No await
    // between this check and `set_enable_pin`.
    if backend.interlock_tripped() {
        return super::fail_inner(phase, CommissionError::FaultInterlock);
    }
    backend.set_enable_pin(true);

    // Command the torque step: flux target held at zero, torque target set
    // to i_target -- already bounded to the resistance current ceiling by
    // the caller in current_loop.rs's bandwidth-gate transition.
    let torque_step = foci_tmc4671::registers::PidTorqueFluxTarget::new()
        .with_pid_flux_target(0)
        .with_pid_torque_target(i_target);
    if backend
        .write_register(
            foci_tmc4671::registers::PID_TORQUE_FLUX_TARGET,
            torque_step.into(),
        )
        .await
        .is_err()
    {
        backend.set_enable_pin(false);
        return super::fail_inner(phase, CommissionError::Spi);
    }

    // Run the capture, then unconditionally zero the target and de-energize
    // on every exit path -- success or failure -- before mapping the
    // capture's own result. `run_capture` already de-energizes on its own
    // exit; this additionally re-zeroes the target so ClosedLoopActivation's
    // entry never inherits a nonzero torque command left over from here --
    // ClosedLoopActivation never re-zeroes this register itself, so a
    // silently-dropped failure on this write would otherwise leak a stale
    // nonzero torque command into the next phase.
    let capture_result =
        capture::run_capture(backend, i_target, resistance_current_ceiling_count).await;
    let zero_target_result = backend
        .write_register(foci_tmc4671::registers::PID_TORQUE_FLUX_TARGET, 0)
        .await;
    backend.set_enable_pin(false);

    let capture = match capture_result {
        Ok(buf) => buf,
        Err(e) => return super::fail_inner(phase, e),
    };
    if zero_target_result.is_err() {
        return super::fail_inner(phase, CommissionError::Spi);
    }

    let commanded_sign: i8 = if i_target >= 0 { 1 } else { -1 };
    // fit_accel_raw requires centered_position (wrapping_sub against the
    // first sample), not the raw register value, so a signed
    // position-register wrap does not appear as a large discontinuity in
    // the fitted trace.
    let position_0 = capture.first().map(|s| s.position).unwrap_or(0);
    let position_samples: heapless::Vec<(u32, i32), 200> = capture
        .iter()
        .map(|s| (s.t_us, s.position.wrapping_sub(position_0)))
        .collect();
    let accel_raw_q32 = match quadratic_fit::fit_accel_raw(&position_samples, commanded_sign) {
        Ok(v) => v,
        Err(quadratic_fit::GainFloorFitError::WrongSign) => {
            return super::fail_inner(phase, CommissionError::GainFloorNonPositiveSlope);
        }
        Err(_) => {
            return super::fail_inner(phase, CommissionError::GainFloorInsufficientLinearPoints);
        }
    };

    // Convert accel_raw (Q32-scaled) to accel_per_count. The
    // counts-per-revolution input is
    // `mechanical_encoder_quadrature_counts_per_rev()`, the same ABN-encoder
    // resolution value `closed_loop_activation.rs` already uses.
    let encoder_counts_per_rev: f64 =
        f64::from(backend.mechanical_encoder_quadrature_counts_per_rev());
    let accel_per_count: f32 = match gains::accel_raw_q32_to_accel_per_count(
        accel_raw_q32,
        encoder_counts_per_rev,
        i_target,
    ) {
        Some(v) => v,
        None => {
            return super::fail_inner(phase, CommissionError::GainFloorInsufficientLinearPoints);
        }
    };

    let torque_samples: heapless::Vec<(u32, i16), 200> =
        capture.iter().map(|s| (s.t_us, s.torque_actual)).collect();
    let bw_current_floor_hz =
        match current_response_fit::fit_current_response(&torque_samples, i_target) {
            Ok(v) => v,
            Err(current_response_fit::CurrentFitError::HighResidual) => {
                return super::fail_inner(phase, CommissionError::GainFloorCurrentFitRejected);
            }
            Err(_) => {
                return super::fail_inner(phase, CommissionError::GainFloorInsufficientCoverage);
            }
        };

    let (velocity_p, omega_target) =
        match gains::compute_velocity_p(accel_per_count, bw_current_floor_hz) {
            Ok(v) => v,
            Err(e) => return super::fail_inner(phase, e),
        };
    let position_p = match gains::compute_position_p(omega_target) {
        Ok(v) => v,
        Err(e) => return super::fail_inner(phase, e),
    };

    let vel_limit = backend.operational_velocity_limit();
    let default_current_filter_hz = backend.tmc_config().default_current_filter_hz();
    let torque_filter_hz = backend
        .configured_torque_filter_hz()
        .unwrap_or(default_current_filter_hz);
    let flux_filter_hz = backend
        .configured_flux_filter_hz()
        .unwrap_or(default_current_filter_hz);
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
        velocity_i: PidIGain::ZERO,
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::autotune::{CommissionProfile, CommissionStatus, InnerSequence, TransientBuffer};
    use crate::commission_dispatch::test_support::{GainFloorCaptureScript, MockBackend, SpiOp};
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

    fn gain_floor_phase(i_target: i16, resistance_current_ceiling_count: u16) -> InnerPhase {
        InnerPhase::GainFloorMeasurement {
            state: GainFloorState::Start,
            i_target,
            resistance_current_ceiling_count,
            inner_result: Some(sample_inner_result(resistance_current_ceiling_count)),
        }
    }

    #[test]
    fn gain_floor_measurement_success_writes_computed_gains() {
        pollster::block_on(async {
            let i_target: i16 = 2000;
            let ceiling = 4_000u16;
            let mut phase = gain_floor_phase(i_target, ceiling);
            let mut backend = MockBackend::new_with_bandwidth_hz(600.0);
            backend.advance_elapsed_on_delay = true;
            // Synthetic accel_raw=0.002 counts/us^2 (the same fixture
            // `quadratic_fit`'s own unit tests validate independently) and
            // tau_current=1000us, so both fits are exercised end to end
            // through the real `run_capture` primitive. A shorter tau (e.g.
            // the 265us Ouroboros bandwidth-derived value used in isolation
            // by `current_response_fit`'s own unit tests, which only sample
            // out to ~4.4*tau) saturates for most of this capture's full
            // ~5ms/166-sample window: with a perfectly noiseless synthetic
            // trace, the many identical near-saturated integer-truncated
            // samples dominate the log-linear regression and blow the
            // residual past its threshold. 1000us keeps the fit inside its
            // residual bound across the whole window while still exercising
            // the same code path a real (noisy) capture would take.
            backend.script_gain_floor_capture = Some(GainFloorCaptureScript {
                accel_raw_per_us2: 0.002,
                tau_current_us: 1000.0,
                i_target,
            });

            let result = step_gain_floor_measurement(&mut phase, &mut backend).await;

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
                    assert_eq!(velocity_i, PidIGain::ZERO);
                    assert_eq!(position_i, PidIGain::ZERO);
                    // Worked by hand from the fixture above: accel_per_count
                    // ~= 1570.8 rad/s^2 per count, bw_current_floor_hz ~= 158
                    // Hz -> velocity_p_raw = 2, position_p_raw = 3.
                    assert_eq!(velocity_p.raw(), 2);
                    assert_eq!(position_p.raw(), 3);
                }
                other => panic!("expected ClosedLoopActivation phase, got {other:?}"),
            }
        });
    }

    #[test]
    fn gain_floor_measurement_cleanup_zero_write_failure_fails_commissioning() {
        pollster::block_on(async {
            let i_target: i16 = 2000;
            let ceiling = 4_000u16;
            let mut phase = gain_floor_phase(i_target, ceiling);
            let mut backend = MockBackend::new_with_bandwidth_hz(600.0);
            backend.advance_elapsed_on_delay = true;
            backend.script_gain_floor_capture = Some(GainFloorCaptureScript {
                accel_raw_per_us2: 0.002,
                tau_current_us: 1000.0,
                i_target,
            });
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
        let result = pollster::block_on(super::super::step_inner(
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
        assert!(
            backend.spi_log.iter().any(|op| matches!(
                op,
                SpiOp::Write(addr, val)
                    if *addr == foci_tmc4671::registers::PIDOUT_UQ_UD_LIMITS.addr()
                        && *val == 1024
            )),
            "{expected:?} must route through restore_safe_state_via_backend \
             (no PIDOUT_UQ_UD_LIMITS=1024 write observed)"
        );
    }

    // `GainFloorPositionGainUnrepresentable` has no test here: its only
    // input, omega_target, is derived from bw_current_floor_hz, which the
    // current-response fit's own coverage/sample-count gates keep inside
    // [95.5 Hz, 796 Hz] for any capture that reaches a successful fit --
    // across that whole range position_p_raw stays near [2, 12], nowhere
    // near PidPGain::ZERO or MAX. `gains::position_p_max_overflow_rejected`
    // covers its trigger condition directly; its fail_inner call site in
    // this file is the same arm used for compute_velocity_p just above it,
    // exercised by `gain_floor_velocity_gain_unrepresentable_routes_through_restore_machine`
    // below.

    #[test]
    fn gain_floor_insufficient_coverage_routes_through_restore_machine() {
        let mut phase = gain_floor_phase(2000, 4000);
        let mut backend = MockBackend::new();
        backend.set_elapsed_us_sequence(&[0, 5]);
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
            accel_raw_per_us2: -0.002,
            tau_current_us: 1000.0,
            i_target: 2000,
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
            accel_raw_per_us2: 0.0,
            tau_current_us: 1000.0,
            i_target: 2000,
        });
        assert_routes_through_restore_machine(
            &mut phase,
            &mut backend,
            CommissionError::GainFloorInsufficientLinearPoints,
        );
    }

    #[test]
    fn gain_floor_current_fit_rejected_routes_through_restore_machine() {
        let mut phase = gain_floor_phase(2000, 4000);
        let mut backend = MockBackend::new_with_bandwidth_hz(600.0);
        backend.advance_elapsed_on_delay = true;
        // tau_current_us=1.0 saturates almost the whole capture at i_target,
        // tripping the current-response fit's saturated-trace rejection.
        backend.script_gain_floor_capture = Some(GainFloorCaptureScript {
            accel_raw_per_us2: 0.002,
            tau_current_us: 1.0,
            i_target: 2000,
        });
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
        let i_target = i16::MAX;
        let mut phase = gain_floor_phase(i_target, 40_000);
        let mut backend = MockBackend::new_with_bandwidth_hz(600.0);
        backend.advance_elapsed_on_delay = true;
        backend.mechanical_encoder_quadrature_counts_per_rev_value = u32::MAX;
        backend.script_gain_floor_capture = Some(GainFloorCaptureScript {
            accel_raw_per_us2: 0.002,
            tau_current_us: 1000.0,
            i_target,
        });
        assert_routes_through_restore_machine(
            &mut phase,
            &mut backend,
            CommissionError::GainFloorVelocityGainUnrepresentable,
        );
    }
}
