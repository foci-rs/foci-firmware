//! Shared motor lifecycle helpers for terminal TMC control paths.

use crate::commission_dispatch::DispatchState;
use crate::commissioning::types::CommissioningBackend;

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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::commission_dispatch::test_support::{MockBackend, fresh_state_locals};

    fn dispatch_state<'a>(
        calibrated: &'a mut bool,
        motor_enabled: &'a mut bool,
        stored_gains: &'a mut Option<crate::pid::InnerLoopGains>,
    ) -> DispatchState<'a> {
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
}
