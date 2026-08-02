//! Shared motor lifecycle helpers for terminal TMC control paths.

use crate::commission_dispatch::DispatchState;
use crate::commissioning::types::CommissioningBackend;

const CLOSED_LOOP_POSITION_MODE: u32 = 0x8000_0003;

pub(crate) async fn settle_closed_loop_hold_before_enable<B: CommissioningBackend>(
    backend: &mut B,
) -> Result<(), crate::autotune::CommissionError> {
    use foci_tmc4671::registers::{
        MODE_RAMP_MODE_MOTION, PID_ERROR_PID_FLUX_ERROR_SUM, PID_ERROR_PID_POSITION_ERROR_SUM,
        PID_ERROR_PID_TORQUE_ERROR_SUM, PID_ERROR_PID_VELOCITY_ERROR_SUM, PID_POSITION_ACTUAL,
        STATUS_FLAGS,
    };

    let mode = backend
        .read_register(MODE_RAMP_MODE_MOTION)
        .await
        .map_err(|_| crate::autotune::CommissionError::Spi)?;
    if mode != CLOSED_LOOP_POSITION_MODE {
        return Ok(());
    }

    let actual = backend
        .read_register(PID_POSITION_ACTUAL)
        .await
        .map_err(|_| crate::autotune::CommissionError::Spi)?;
    backend
        .write_register(PID_POSITION_ACTUAL, actual)
        .await
        .map_err(|_| crate::autotune::CommissionError::Spi)?;

    crate::commissioning::helpers::write_sub_register_checked(
        backend,
        PID_ERROR_PID_TORQUE_ERROR_SUM,
        0,
    )
    .await
    .map_err(|_| crate::autotune::CommissionError::Spi)?;
    crate::commissioning::helpers::write_sub_register_checked(
        backend,
        PID_ERROR_PID_FLUX_ERROR_SUM,
        0,
    )
    .await
    .map_err(|_| crate::autotune::CommissionError::Spi)?;
    crate::commissioning::helpers::write_sub_register_checked(
        backend,
        PID_ERROR_PID_VELOCITY_ERROR_SUM,
        0,
    )
    .await
    .map_err(|_| crate::autotune::CommissionError::Spi)?;
    crate::commissioning::helpers::write_sub_register_checked(
        backend,
        PID_ERROR_PID_POSITION_ERROR_SUM,
        0,
    )
    .await
    .map_err(|_| crate::autotune::CommissionError::Spi)?;
    backend
        .write_register(STATUS_FLAGS, 0)
        .await
        .map_err(|_| crate::autotune::CommissionError::Spi)?;
    Ok(())
}

/// Mark the motor electrically energized after verified calibration.
///
/// Asserts `state.calibrated`; verifies the calibration sentinel register;
/// sets the enable pin high; updates `state.motor_enabled`. Does NOT touch
/// channel motor arming — stepper arming is a separate concern (see
/// [`arm_motor_for_steps`]).
///
/// `assert!`, not `debug_assert!`: this is a hard motor-safety
/// contract and must enforce in release builds. Costs one branch
/// per arming (not on the hot path). On failure: panic -> RTIC fault
/// handler -> safe-state via the GPIO Drop and `PIDOUT_UQ_UD_LIMITS` cap.
///
/// # Errors
///
/// Returns [`crate::autotune::CommissionError::ChipResetDetected`] if the
/// sentinel is inactive or does not match the TMC4671 readback.
pub async fn arm_motor_electrical<B: CommissioningBackend>(
    backend: &mut B,
    state: &mut DispatchState<'_>,
) -> Result<(), crate::autotune::CommissionError> {
    use core::sync::atomic::Ordering::Relaxed;
    use foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS;

    assert!(
        *state.calibrated,
        "FATAL: arm_motor_electrical called with calibrated=false; \
         SPI restore must precede arm"
    );

    let expected = state.tmc.motor_sentinel.load(Relaxed);
    if expected == 0 {
        defmt::error!("arm_motor_electrical: missing MOTOR_TYPE sentinel");
        return Err(crate::autotune::CommissionError::ChipResetDetected);
    }

    backend
        .read_register_verified(MOTOR_TYPE_N_POLE_PAIRS, expected)
        .await
        .map_err(|_| crate::autotune::CommissionError::ChipResetDetected)?;
    crate::tmc_control::pi_representation::verify_pi_representation(backend, state.tmc)
        .await
        .map_err(|_| crate::autotune::CommissionError::ChipResetDetected)?;

    settle_closed_loop_hold_before_enable(backend).await?;

    backend.set_enable_pin(true);
    *state.motor_enabled = true;
    Ok(())
}

/// Arm the stepper for step-timer processing.
///
/// Asserts `state.motor_enabled` (set only by `arm_motor_electrical`);
/// stores the channel motor-armed flag. Caller is responsible for any
/// prerequisite state (e.g. zeroing the stepper channel position) before
/// invoking. Always-on assert.
pub fn arm_motor_for_steps(state: &mut DispatchState<'_>) {
    assert!(
        *state.motor_enabled,
        "FATAL: arm_motor_for_steps called with motor_enabled=false"
    );
    state
        .tmc
        .motor_armed
        .store(true, core::sync::atomic::Ordering::Relaxed);
}

/// Electrically arm the motor for open-loop voltage-vector diagnostics.
///
/// This path deliberately does not call [`arm_motor_for_steps`]. It verifies
/// the same calibration sentinel and enable-state contract as
/// [`arm_motor_electrical`], then leaves the terminal stepper-armed flag clear
/// so STEP/DIR pulses remain rejected while the diagnostic owns voltage
/// authority.
///
/// # Errors
///
/// Returns [`crate::autotune::CommissionError::ChipResetDetected`] if the
/// sentinel is inactive or does not match the TMC4671 readback.
pub async fn arm_motor_for_voltage_vectors<B: CommissioningBackend>(
    backend: &mut B,
    state: &mut DispatchState<'_>,
) -> Result<(), crate::autotune::CommissionError> {
    arm_motor_electrical(backend, state).await?;
    state
        .tmc
        .motor_armed
        .store(false, core::sync::atomic::Ordering::Relaxed);
    Ok(())
}

/// De-energize the motor and clear all armed/enabled state.
///
/// Clears the channel motor-armed flag first so the step-timer ISR stops
/// accepting step events before async SPI cleanup begins, then routes the
/// safe-state register sequence through
/// [`crate::tmc_safe_state::restore_safe_state_via_backend`] and clears
/// `state.calibrated` and `state.motor_enabled`.
///
/// Idempotent: safe to call when the motor is already disarmed.
/// Does not touch the stepper queue or invoke `stepper_stop_from_p1` —
/// those remain board-side responsibilities.
pub async fn disarm_motor<B: CommissioningBackend>(backend: &mut B, state: &mut DispatchState<'_>) {
    state
        .tmc
        .motor_armed
        .store(false, core::sync::atomic::Ordering::Relaxed);
    crate::tmc_safe_state::restore_safe_state_via_backend(backend).await;
    *state.calibrated = false;
    *state.motor_enabled = false;
}

/// Apply the shared shutdown safe-state policy for one TMC channel.
///
/// Restores phase advance, de-energizes the motor through [`disarm_motor`],
/// then stops any active commissioning engine. A retained breakaway campaign
/// is completed only after both safe-state operations succeed.
///
/// Returns `true` when the phase-advance restore either was unnecessary or
/// succeeded. Returns `false` when restore failed; callers should keep retrying
/// this helper while shutdown remains active.
pub async fn apply_shutdown_safe_state<B: CommissioningBackend>(
    runtime: &mut crate::tmc_control::TmcControlRuntime,
    backend: &mut B,
    tmc: &'static crate::state::tmc::TmcChannelState,
    stepper: &'static crate::state::stepper::StepperChannelState,
) -> bool {
    let phase_advance_restored = runtime.restore_phase_advance_via_backend(backend).await;
    if !phase_advance_restored {
        defmt::warn!("phase advance: failed to restore ABN offset on shutdown");
    }
    let mut state = runtime.dispatch_state(tmc, stepper);
    disarm_motor(backend, &mut state).await;
    if phase_advance_restored {
        runtime.take_commission_engine_after_handled_stop();
    } else {
        runtime.take_commission_engine();
    }
    phase_advance_restored
}

#[cfg(test)]
mod tests {
    use super::*;
    #[cfg(feature = "trace-velocity-sweep")]
    use crate::commission_dispatch::test_support::breakaway_campaign_engine;
    use crate::commission_dispatch::test_support::{MockBackend, fresh_state_locals};

    fn dispatch_state<'a>(
        calibrated: &'a mut bool,
        motor_enabled: &'a mut bool,
        stored_gains: &'a mut Option<crate::pid::InnerLoopGains>,
    ) -> DispatchState<'a> {
        crate::state::tmc::tmc_channel0().mark_pi_representation_established();
        DispatchState::new(
            crate::state::tmc::tmc_channel0(),
            crate::state::stepper::stepper_channel0(),
            calibrated,
            motor_enabled,
            stored_gains,
        )
    }

    fn tmc_state() -> &'static crate::state::tmc::TmcChannelState {
        crate::state::tmc::tmc_channel0()
    }

    #[test]
    #[should_panic(expected = "calibrated=false")]
    fn arm_motor_electrical_panics_when_uncalibrated() {
        pollster::block_on(async {
            let mut backend = MockBackend::new();
            let (mut cal, mut en, mut gains) = fresh_state_locals();
            let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
            let _ = arm_motor_electrical(&mut backend, &mut state).await;
        });
    }

    #[test]
    #[should_panic(expected = "motor_enabled=false")]
    fn arm_motor_for_steps_panics_when_motor_disabled() {
        let (mut cal, mut en, mut gains) = fresh_state_locals();
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        arm_motor_for_steps(&mut state);
    }

    #[test]
    fn arm_then_disarm_returns_to_baseline() {
        pollster::block_on(async {
            use core::sync::atomic::Ordering;
            let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
                .lock()
                .await;
            let mut backend = MockBackend::new();
            let (_, mut en, mut gains) = fresh_state_locals();
            let mut cal = true;
            let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
            tmc_state()
                .motor_sentinel
                .store(0x0002_0032, Ordering::Relaxed);
            let _ = backend
                .write_register(
                    foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS,
                    0x0002_0032,
                )
                .await;
            arm_motor_electrical(&mut backend, &mut state)
                .await
                .expect("sentinel should match");
            arm_motor_for_steps(&mut state);
            assert!(backend.enable_pin);
            assert!(tmc_state().motor_armed.load(Ordering::Relaxed));
            disarm_motor(&mut backend, &mut state).await;
            assert!(!backend.enable_pin);
            assert!(!*state.calibrated);
            assert!(!*state.motor_enabled);
            assert!(!tmc_state().motor_armed.load(Ordering::Relaxed));
            tmc_state().motor_sentinel.store(0, Ordering::Relaxed);
        });
    }

    #[test]
    fn arm_motor_success_path() {
        pollster::block_on(async {
            use core::sync::atomic::Ordering;
            let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
                .lock()
                .await;
            let mut backend = MockBackend::new();
            let (_, mut en, mut gains) = fresh_state_locals();
            let mut cal = true;
            let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
            tmc_state()
                .motor_sentinel
                .store(0x0002_0032, Ordering::Relaxed);
            let _ = backend
                .write_register(
                    foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS,
                    0x0002_0032,
                )
                .await;
            arm_motor_electrical(&mut backend, &mut state)
                .await
                .expect("sentinel should match");
            assert!(backend.enable_pin);
            assert!(*state.motor_enabled);
            assert!(!tmc_state().motor_armed.load(Ordering::Relaxed));
            arm_motor_for_steps(&mut state);
            assert!(tmc_state().motor_armed.load(Ordering::Relaxed));
            tmc_state().motor_armed.store(false, Ordering::Relaxed);
            tmc_state().motor_sentinel.store(0, Ordering::Relaxed);
        });
    }

    #[test]
    fn arm_motor_for_voltage_vectors_keeps_stepper_disarmed() {
        pollster::block_on(async {
            use core::sync::atomic::Ordering;
            let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
                .lock()
                .await;
            let mut backend = MockBackend::new();
            let (_, mut en, mut gains) = fresh_state_locals();
            let mut cal = true;
            let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
            tmc_state()
                .motor_sentinel
                .store(0x0002_0032, Ordering::Relaxed);
            let _ = backend
                .write_register(
                    foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS,
                    0x0002_0032,
                )
                .await;

            arm_motor_for_voltage_vectors(&mut backend, &mut state)
                .await
                .expect("sentinel should match");

            assert!(backend.enable_pin);
            assert!(*state.motor_enabled);
            assert!(!tmc_state().motor_armed.load(Ordering::Relaxed));

            disarm_motor(&mut backend, &mut state).await;
            tmc_state().motor_sentinel.store(0, Ordering::Relaxed);
        });
    }

    #[test]
    fn arm_motor_electrical_refuses_when_sentinel_mismatches() {
        pollster::block_on(async {
            let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
                .lock()
                .await;
            let mut backend = MockBackend::new();
            tmc_state()
                .motor_sentinel
                .store(0x0002_0032, core::sync::atomic::Ordering::Relaxed);
            backend.read_override =
                Some((foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS.addr(), 0));
            let (_, mut en, mut gains) = fresh_state_locals();
            let mut cal = true;
            let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
            let result = arm_motor_electrical(&mut backend, &mut state).await;
            assert_eq!(
                result,
                Err(crate::autotune::CommissionError::ChipResetDetected)
            );
            assert!(
                !backend.enable_pin,
                "enable pin must remain low on mismatch"
            );
            tmc_state()
                .motor_sentinel
                .store(0, core::sync::atomic::Ordering::Relaxed);
        });
    }

    #[test]
    fn arm_motor_electrical_refuses_when_pi_representation_mismatches() {
        pollster::block_on(async {
            let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
                .lock()
                .await;
            let mut backend = MockBackend::new();
            tmc_state()
                .motor_sentinel
                .store(0x0002_0032, core::sync::atomic::Ordering::Relaxed);
            tmc_state().mark_pi_representation_established();
            let _ = backend
                .write_register(
                    foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS,
                    0x0002_0032,
                )
                .await;
            backend.read_override = Some((
                foci_tmc4671::registers::CONFIG_ADVANCED_PI_REPRESENT
                    .data
                    .addr(),
                0,
            ));
            let (_, mut en, mut gains) = fresh_state_locals();
            let mut cal = true;
            let mut state = dispatch_state(&mut cal, &mut en, &mut gains);

            let result = arm_motor_electrical(&mut backend, &mut state).await;

            assert_eq!(
                result,
                Err(crate::autotune::CommissionError::ChipResetDetected)
            );
            assert!(!backend.enable_pin);
            tmc_state()
                .motor_sentinel
                .store(0, core::sync::atomic::Ordering::Relaxed);
            tmc_state().clear_pi_representation_established();
        });
    }

    #[test]
    fn arm_motor_electrical_succeeds_when_sentinel_matches() {
        pollster::block_on(async {
            let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
                .lock()
                .await;
            let mut backend = MockBackend::new();
            tmc_state()
                .motor_sentinel
                .store(0x0002_0032, core::sync::atomic::Ordering::Relaxed);
            let _ = backend
                .write_register(
                    foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS,
                    0x0002_0032,
                )
                .await;
            let (_, mut en, mut gains) = fresh_state_locals();
            let mut cal = true;
            let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
            let result = arm_motor_electrical(&mut backend, &mut state).await;
            assert_eq!(result, Ok(()));
            assert!(backend.enable_pin);
            tmc_state()
                .motor_sentinel
                .store(0, core::sync::atomic::Ordering::Relaxed);
        });
    }

    #[test]
    fn arm_motor_electrical_sets_hold_target_before_enable_in_position_mode() {
        pollster::block_on(async {
            use foci_tmc4671::registers::{
                MODE_RAMP_MODE_MOTION, MOTOR_TYPE_N_POLE_PAIRS, PID_POSITION_ACTUAL,
                PID_POSITION_TARGET, STATUS_FLAGS,
            };

            let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
                .lock()
                .await;
            let mut backend = MockBackend::new();
            tmc_state()
                .motor_sentinel
                .store(0x0002_0032, core::sync::atomic::Ordering::Relaxed);
            let _ = backend
                .write_register(MOTOR_TYPE_N_POLE_PAIRS, 0x0002_0032)
                .await;
            let _ = backend
                .write_register(MODE_RAMP_MODE_MOTION, 0x8000_0003)
                .await;
            let _ = backend.write_register(PID_POSITION_TARGET, 0).await;
            let _ = backend.write_register(PID_POSITION_ACTUAL, 11).await;
            backend.spi_log.clear();

            let (_, mut en, mut gains) = fresh_state_locals();
            let mut cal = true;
            let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
            let result = arm_motor_electrical(&mut backend, &mut state).await;

            assert_eq!(result, Ok(()));
            let enable_index = backend
                .spi_log
                .iter()
                .position(|op| {
                    matches!(
                        op,
                        crate::commission_dispatch::test_support::SpiOp::EnablePin(true)
                    )
                })
                .expect("arm should assert enable");
            let target_rewrite_index = backend
                .spi_log
                .iter()
                .position(|op| {
                    matches!(
                        op,
                        crate::commission_dispatch::test_support::SpiOp::Write(addr, 11)
                            if *addr == PID_POSITION_ACTUAL.addr()
                    )
                })
                .expect("arm should rewrite PID_POSITION_ACTUAL to observed actual");
            let status_clear_index = backend
                .spi_log
                .iter()
                .position(|op| {
                    matches!(
                        op,
                        crate::commission_dispatch::test_support::SpiOp::Write(addr, 0)
                            if *addr == STATUS_FLAGS.addr()
                    )
                })
                .expect("arm should clear STATUS_FLAGS");
            assert!(target_rewrite_index < enable_index);
            assert!(status_clear_index < enable_index);
            tmc_state()
                .motor_sentinel
                .store(0, core::sync::atomic::Ordering::Relaxed);
        });
    }

    #[test]
    fn shutdown_safe_state_restores_phase_advance_and_disarms() {
        pollster::block_on(async {
            use core::sync::atomic::Ordering;

            let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
                .lock()
                .await;
            let mut runtime = crate::tmc_control::TmcControlRuntime::new_for_channel(0);
            runtime.set_phase_advance_for_test(true, 0x1111_2222, 0x3333_4444);
            let mut backend = MockBackend::new();
            {
                let state = runtime.dispatch_state(
                    crate::state::tmc::tmc_channel0(),
                    crate::state::stepper::stepper_channel0(),
                );
                *state.calibrated = true;
                *state.motor_enabled = true;
            }
            tmc_state().motor_armed.store(true, Ordering::Relaxed);

            let complete = apply_shutdown_safe_state(
                &mut runtime,
                &mut backend,
                crate::state::tmc::tmc_channel0(),
                crate::state::stepper::stepper_channel0(),
            )
            .await;

            assert!(complete);
            assert!(!runtime.phase_advance_active_for_test());
            assert_eq!(runtime.phase_advance_last_register_for_test(), 0x1111_2222);
            assert_eq!(
                backend
                    .registers
                    .get(&foci_tmc4671::registers::ABN_DECODER_PHI_E_PHI_M_OFFSET.addr()),
                Some(&0x1111_2222)
            );
            assert!(!tmc_state().motor_armed.load(Ordering::Relaxed));
            assert!(!runtime.is_calibrated());
            assert!(!runtime.is_motor_enabled());
        });
    }

    #[test]
    fn shutdown_safe_state_reports_incomplete_when_phase_restore_fails() {
        pollster::block_on(async {
            use core::sync::atomic::Ordering;

            let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
                .lock()
                .await;
            let mut runtime = crate::tmc_control::TmcControlRuntime::new_for_channel(0);
            runtime.set_phase_advance_for_test(true, 0x1111_2222, 0x3333_4444);
            let mut backend = MockBackend::new();
            backend.fail_on_write =
                Some(foci_tmc4671::registers::ABN_DECODER_PHI_E_PHI_M_OFFSET.addr());
            {
                let state = runtime.dispatch_state(
                    crate::state::tmc::tmc_channel0(),
                    crate::state::stepper::stepper_channel0(),
                );
                *state.calibrated = true;
                *state.motor_enabled = true;
            }
            tmc_state().motor_armed.store(true, Ordering::Relaxed);

            let complete = apply_shutdown_safe_state(
                &mut runtime,
                &mut backend,
                crate::state::tmc::tmc_channel0(),
                crate::state::stepper::stepper_channel0(),
            )
            .await;

            assert!(!complete);
            assert!(runtime.phase_advance_active_for_test());
            assert_eq!(runtime.phase_advance_last_register_for_test(), 0x3333_4444);
            assert!(!tmc_state().motor_armed.load(Ordering::Relaxed));
            assert!(!runtime.is_calibrated());
            assert!(!runtime.is_motor_enabled());
        });
    }

    #[cfg(feature = "trace-velocity-sweep")]
    #[test]
    fn shutdown_breakaway_cancellation_completes_after_safe_state() {
        use core::sync::atomic::Ordering;

        let _guard =
            pollster::block_on(crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.lock());
        let mut runtime = crate::tmc_control::TmcControlRuntime::new_for_channel(0);
        runtime.set_commission_engine(breakaway_campaign_engine());
        runtime.set_phase_advance_for_test(true, 0x1111_2222, 0x3333_4444);
        let mut backend = MockBackend::new();
        backend.enable_pin = true;
        {
            let state = runtime.dispatch_state(
                crate::state::tmc::tmc_channel0(),
                crate::state::stepper::stepper_channel0(),
            );
            *state.calibrated = true;
            *state.motor_enabled = true;
        }
        tmc_state().motor_armed.store(true, Ordering::Relaxed);
        crate::diagnostic::set_breakaway_breadcrumb_raw_for_test(0);
        crate::diagnostic::begin_breakaway_campaign();
        crate::diagnostic::panic_after_breakaway_completion_for_test();

        let reset = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            pollster::block_on(apply_shutdown_safe_state(
                &mut runtime,
                &mut backend,
                crate::state::tmc::tmc_channel0(),
                crate::state::stepper::stepper_channel0(),
            ));
        }));

        assert!(reset.is_err(), "completion must reach the reset injection");
        assert!(!runtime.phase_advance_active_for_test());
        assert!(!backend.enable_pin, "safe disarm must precede completion");
        assert!(!tmc_state().motor_armed.load(Ordering::Relaxed));
        assert!(
            runtime.is_commission_active(),
            "engine removal must follow completion"
        );
        assert_eq!(
            crate::diagnostic::current_breakaway_breadcrumb_for_test(),
            Some(crate::diagnostic::BreakawayBreadcrumb {
                checkpoint: crate::diagnostic::BreakawayCheckpoint::CampaignTerminalHandled,
                interrupted: false,
            })
        );

        assert!(pollster::block_on(apply_shutdown_safe_state(
            &mut runtime,
            &mut backend,
            crate::state::tmc::tmc_channel0(),
            crate::state::stepper::stepper_channel0(),
        )));
        assert!(!runtime.is_commission_active());
    }

    #[cfg(feature = "trace-velocity-sweep")]
    #[test]
    fn shutdown_breakaway_cancellation_restore_failure_stays_active() {
        pollster::block_on(async {
            let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
                .lock()
                .await;
            let mut runtime = crate::tmc_control::TmcControlRuntime::new_for_channel(0);
            runtime.set_commission_engine(breakaway_campaign_engine());
            runtime.set_phase_advance_for_test(true, 0x1111_2222, 0x3333_4444);
            let mut backend = MockBackend::new();
            backend.fail_on_write =
                Some(foci_tmc4671::registers::ABN_DECODER_PHI_E_PHI_M_OFFSET.addr());
            backend.enable_pin = true;
            crate::diagnostic::set_breakaway_breadcrumb_raw_for_test(0);
            crate::diagnostic::begin_breakaway_campaign();

            let complete = apply_shutdown_safe_state(
                &mut runtime,
                &mut backend,
                crate::state::tmc::tmc_channel0(),
                crate::state::stepper::stepper_channel0(),
            )
            .await;

            assert!(!complete);
            assert!(!runtime.is_commission_active());
            assert!(!backend.enable_pin, "failed restoration must still disarm");
            assert_eq!(
                crate::diagnostic::current_breakaway_breadcrumb_for_test(),
                Some(crate::diagnostic::BreakawayBreadcrumb {
                    checkpoint: crate::diagnostic::BreakawayCheckpoint::CampaignInstalled,
                    interrupted: true,
                })
            );
        });
    }
}
