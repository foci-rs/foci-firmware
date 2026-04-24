//! Board-agnostic TMC4671 operations extracted from the RTIC task body.
//!
//! Functions here are monomorphized over `B: SpiDevice<u8>` rather than
//! dyn-dispatched: the hot-path SPI access does not benefit from vtable
//! indirection, and `async fn` in traits is not viable in `no_std`. The RTIC
//! task shell (`#[task]` declaration, shared-resource lock, `spawn()`, EXTI
//! setup) stays board-local; the body calls into this module.
//!
//! ## Contained operations
//!
//! - [`clear_pid_error_sums`] — zero all four TMC4671 PID integrator sums.
//! - [`settle_hold_and_clear_pid_errors`] — snap the hold target to the
//!   measured actual position when within the deadband, then clear PID
//!   error accumulators and STATUS_FLAGS.
//! - [`apply_stored_filters`] — write the four biquad filter sections from
//!   the `state::tmc` filter-Hz atomics back to TMC4671 registers.
//! - [`apply_set_current`] — convert RMS current to TMC4671 peak counts and
//!   write `PID_TORQUE_FLUX_LIMITS`; stores the result in `CURRENT_LIMIT_REG`.
//! - [`apply_set_encoder`] — validate PPR, write `ABN_DECODER_PPR` (×4 for
//!   quadrature), and log the computed step width.
//! - [`apply_set_encoder_dir`] — write `ABN_DECODER_MODE` and update the
//!   `ENCODER_REVERSED` / `FEEDFORWARD_RESET` atomics.
//!
//! Populated progressively by Phase A-7 Tasks 26–27 of the internal-
//! restructure plan (`docs/plans/archive/2026-04-20-phase-a-internal-
//! restructure.md`). See the extraction blueprint in this session's
//! logbook entry for commit-by-commit scope.

use crate::commission_dispatch::DispatchState;
use crate::commissioning::types::CommissioningBackend;
use embedded_hal_async::spi::SpiDevice;
use foci_tmc4671::spi::Tmc4671Spi;

/// Acceptable residual following error after a completed move, in ABN encoder
/// counts. Within this window the controller is fighting encoder quantization
/// and belt/motor compliance more than useful position error.
pub const HOLD_SETTLE_DEADBAND_COUNTS: u32 = 4;

/// Clear TMC4671 PID error accumulators.
///
/// The current-loop sums can saturate during commissioning and step
/// transients, then keep STATUS_FLAGS asserted even after the commanded
/// position has returned to zero. Clearing all four sums at lifecycle
/// boundaries removes stale integral state without changing configured
/// gains or the active position target.
pub async fn clear_pid_error_sums<B>(tmc: &mut Tmc4671Spi<B>) -> Result<(), ()>
where
    B: SpiDevice<u8>,
{
    use foci_tmc4671::registers::*;

    tmc.write_sub_register(PID_ERROR_PID_TORQUE_ERROR_SUM, 0)
        .await
        .map_err(|_| ())?;
    tmc.write_sub_register(PID_ERROR_PID_FLUX_ERROR_SUM, 0)
        .await
        .map_err(|_| ())?;
    tmc.write_sub_register(PID_ERROR_PID_VELOCITY_ERROR_SUM, 0)
        .await
        .map_err(|_| ())?;
    tmc.write_sub_register(PID_ERROR_PID_POSITION_ERROR_SUM, 0)
        .await
        .map_err(|_| ())?;
    Ok(())
}

/// Snap the TMC4671 hold target to the measured actual position when within
/// the settle deadband, then clear all PID error accumulators and STATUS_FLAGS.
///
/// With ABN feedback active, `PID_POSITION_ACTUAL` is driven by encoder phase.
/// Accepting tiny settled error by moving the internal TMC hold target to the
/// measured actual position keeps firmware and Klipper logical step counts
/// unchanged while preventing stale error sums from driving unnecessary
/// corrective current.
pub async fn settle_hold_and_clear_pid_errors<B>(tmc: &mut Tmc4671Spi<B>) -> Result<(), ()>
where
    B: SpiDevice<u8>,
{
    use core::sync::atomic::Ordering::Relaxed;
    use foci_tmc4671::registers::*;

    let target = tmc
        .read_register(PID_POSITION_TARGET)
        .await
        .map_err(|_| ())?;
    let actual = tmc
        .read_register(PID_POSITION_ACTUAL)
        .await
        .map_err(|_| ())?;
    let step_width = crate::state::tmc::STEP_WIDTH_VALUE
        .load(Relaxed)
        .unsigned_abs()
        .max(1);
    let deadband = step_width.saturating_mul(HOLD_SETTLE_DEADBAND_COUNTS);
    let error = (actual as i32).wrapping_sub(target as i32);
    if error.unsigned_abs() <= deadband {
        // With ABN feedback active, PID_POSITION_ACTUAL is driven by
        // encoder phase. Accept tiny settled error by moving the internal
        // TMC hold target to the measured actual position; firmware and
        // Klipper logical step counts remain unchanged.
        tmc.write_register(PID_POSITION_TARGET, actual)
            .await
            .map_err(|_| ())?;
    }

    clear_pid_error_sums(tmc).await?;
    tmc.write_register(STATUS_FLAGS, 0).await.map_err(|_| ())?;
    Ok(())
}

/// Re-apply biquad filters from stored atomics.
///
/// Called after `CalibrateDone` to restore filters that the commissioning
/// engine's `ClosedLoopEntry` phase disabled (it has no access to the filter
/// Hz atomics). Best-effort: warns on failure, does not abort.
///
/// `pwm_maxcnt` is board-specific and must be supplied by the caller
/// (e.g. `TMC_CONFIG.pwm_maxcnt`).
pub async fn apply_stored_filters<B>(tmc: &mut Tmc4671Spi<B>, pwm_maxcnt: u16)
where
    B: SpiDevice<u8>,
{
    use core::sync::atomic::Ordering::Relaxed;
    use foci_tmc4671::registers::*;

    // Velocity biquad.
    let filter_hz = crate::state::tmc::VELOCITY_FILTER_HZ.load(Relaxed);
    if filter_hz > 0
        && let Some(coeffs) = crate::biquad::butterworth_lowpass(filter_hz, pwm_maxcnt)
    {
        let _ = tmc
            .write_sub_register(CONFIG_BIQUAD_V_A_1, coeffs.a_1 as u32)
            .await;
        let _ = tmc
            .write_sub_register(CONFIG_BIQUAD_V_A_2, coeffs.a_2 as u32)
            .await;
        let _ = tmc
            .write_sub_register(CONFIG_BIQUAD_V_B_0, coeffs.b_0 as u32)
            .await;
        let _ = tmc
            .write_sub_register(CONFIG_BIQUAD_V_B_1, coeffs.b_1 as u32)
            .await;
        let _ = tmc
            .write_sub_register(CONFIG_BIQUAD_V_B_2, coeffs.b_2 as u32)
            .await;
        let _ = tmc.write_sub_register(CONFIG_BIQUAD_V_ENABLE, 1).await;
    }

    // Torque biquad.
    let filter_hz = crate::state::tmc::TORQUE_FILTER_HZ.load(Relaxed);
    if filter_hz > 0
        && let Some(coeffs) = crate::biquad::butterworth_lowpass(filter_hz, pwm_maxcnt)
    {
        let _ = tmc
            .write_sub_register(CONFIG_BIQUAD_T_A_1, coeffs.a_1 as u32)
            .await;
        let _ = tmc
            .write_sub_register(CONFIG_BIQUAD_T_A_2, coeffs.a_2 as u32)
            .await;
        let _ = tmc
            .write_sub_register(CONFIG_BIQUAD_T_B_0, coeffs.b_0 as u32)
            .await;
        let _ = tmc
            .write_sub_register(CONFIG_BIQUAD_T_B_1, coeffs.b_1 as u32)
            .await;
        let _ = tmc
            .write_sub_register(CONFIG_BIQUAD_T_B_2, coeffs.b_2 as u32)
            .await;
        let _ = tmc.write_sub_register(CONFIG_BIQUAD_T_ENABLE, 1).await;
    }

    // Position biquad.
    let filter_hz = crate::state::tmc::POSITION_FILTER_HZ.load(Relaxed);
    if filter_hz > 0
        && let Some(coeffs) = crate::biquad::butterworth_lowpass(filter_hz, pwm_maxcnt)
    {
        let _ = tmc
            .write_sub_register(CONFIG_BIQUAD_X_A_1, coeffs.a_1 as u32)
            .await;
        let _ = tmc
            .write_sub_register(CONFIG_BIQUAD_X_A_2, coeffs.a_2 as u32)
            .await;
        let _ = tmc
            .write_sub_register(CONFIG_BIQUAD_X_B_0, coeffs.b_0 as u32)
            .await;
        let _ = tmc
            .write_sub_register(CONFIG_BIQUAD_X_B_1, coeffs.b_1 as u32)
            .await;
        let _ = tmc
            .write_sub_register(CONFIG_BIQUAD_X_B_2, coeffs.b_2 as u32)
            .await;
        let _ = tmc.write_sub_register(CONFIG_BIQUAD_X_ENABLE, 1).await;
    }

    // Flux biquad.
    let filter_hz = crate::state::tmc::FLUX_FILTER_HZ.load(Relaxed);
    if filter_hz > 0
        && let Some(coeffs) = crate::biquad::butterworth_lowpass(filter_hz, pwm_maxcnt)
    {
        let _ = tmc
            .write_sub_register(CONFIG_BIQUAD_F_A_1, coeffs.a_1 as u32)
            .await;
        let _ = tmc
            .write_sub_register(CONFIG_BIQUAD_F_A_2, coeffs.a_2 as u32)
            .await;
        let _ = tmc
            .write_sub_register(CONFIG_BIQUAD_F_B_0, coeffs.b_0 as u32)
            .await;
        let _ = tmc
            .write_sub_register(CONFIG_BIQUAD_F_B_1, coeffs.b_1 as u32)
            .await;
        let _ = tmc
            .write_sub_register(CONFIG_BIQUAD_F_B_2, coeffs.b_2 as u32)
            .await;
        let _ = tmc.write_sub_register(CONFIG_BIQUAD_F_ENABLE, 1).await;
    }
}

/// Apply a new RMS current limit to the TMC4671.
///
/// Converts the Klipper-convention RMS milliamp value to the TMC4671 peak
/// amplitude count (datasheet eq. 19/21: `I_phase = I_D * sin(phi_e)`),
/// writes `PID_TORQUE_FLUX_LIMITS`, and stores the packed register value in
/// `CURRENT_LIMIT_REG` for use by the commissioning engine.
///
/// # Parameters
///
/// - `tmc` — mutable reference to the TMC4671 SPI driver.
/// - `run_ma` — target RMS current in milliamps.
/// - `current_scale_ma_per_lsb` — board-specific milliamps-per-LSB scaling
///   factor (e.g. `1.155` for OpenFFBoard v1.2.x).
///
/// # Errors
///
/// Returns `Err(())` if the SPI write fails.
pub async fn apply_set_current<B>(
    tmc: &mut Tmc4671Spi<B>,
    run_ma: u32,
    current_scale_ma_per_lsb: f32,
) -> Result<(), ()>
where
    B: SpiDevice<u8>,
{
    use foci_tmc4671::registers::*;

    // User configures RMS current (Klipper convention). The TMC4671
    // PID limits operate on peak amplitude in the d-q frame
    // (datasheet eq. 19/21: I_phase = I_D * sin(phi_e)).
    // Convert: peak = RMS * sqrt(2).
    let peak_ma = run_ma as f32 * core::f32::consts::SQRT_2;
    let counts = (peak_ma / current_scale_ma_per_lsb) as u32;
    let limit = (counts << 16) | counts;
    if tmc
        .write_register(PID_TORQUE_FLUX_LIMITS, limit)
        .await
        .is_err()
    {
        defmt::error!("TMC4671 SPI write failed: PID_TORQUE_FLUX_LIMITS");
        return Err(());
    }
    crate::state::tmc::CURRENT_LIMIT_REG.store(limit, core::sync::atomic::Ordering::Relaxed);
    defmt::info!(
        "TMC4671 current: {}mA RMS -> {} counts (peak)",
        run_ma,
        counts
    );
    Ok(())
}

/// Apply a new encoder pulses-per-revolution value to the TMC4671.
///
/// Validates that `ppr` is non-zero, multiplies by 4 to obtain the
/// quadrature count (ABN encoders produce four edges per mechanical cycle),
/// writes `ABN_DECODER_PPR`, and logs the derived step width.
///
/// # Parameters
///
/// - `tmc` — mutable reference to the TMC4671 SPI driver.
/// - `ppr` — encoder pulses per revolution (mechanical, pre-quadrature).
///
/// # Errors
///
/// Returns `Err(())` if `ppr` is zero or if the SPI write fails.
pub async fn apply_set_encoder<B>(tmc: &mut Tmc4671Spi<B>, ppr: u32) -> Result<(), ()>
where
    B: SpiDevice<u8>,
{
    use foci_tmc4671::registers::*;

    if ppr == 0 {
        defmt::error!("Invalid encoder PPR: 0");
        return Err(());
    }
    let quadrature_ppr = ppr.saturating_mul(4);
    if tmc
        .write_register(ABN_DECODER_PPR, quadrature_ppr)
        .await
        .is_err()
    {
        defmt::error!("TMC4671 SPI write failed: ABN_DECODER_PPR");
        return Err(());
    }
    let step_width = 65536u64 * 50 / quadrature_ppr as u64;
    defmt::info!("TMC4671 encoder: {} PPR, step_width={}", ppr, step_width);
    Ok(())
}

/// Apply an encoder direction setting to the TMC4671.
///
/// Writes `ABN_DECODER_MODE` (bit 12 selects reversed/default direction),
/// stores the inversion flag in `ENCODER_REVERSED`, and sets `FEEDFORWARD_RESET`
/// to prevent a one-tick velocity impulse from the sign-flipped delta in the
/// feedforward path on the next tick.
///
/// # Parameters
///
/// - `tmc` — mutable reference to the TMC4671 SPI driver.
/// - `invert` — `true` to reverse the encoder direction, `false` for default.
///
/// # Errors
///
/// Returns `Err(())` if the SPI write fails.
pub async fn apply_set_encoder_dir<B>(tmc: &mut Tmc4671Spi<B>, invert: bool) -> Result<(), ()>
where
    B: SpiDevice<u8>,
{
    use foci_tmc4671::registers::*;

    let mode = if invert { 1u32 << 12 } else { 0 };
    if tmc.write_register(ABN_DECODER_MODE, mode).await.is_err() {
        defmt::error!("TMC4671 SPI write failed: ABN_DECODER_MODE");
        return Err(());
    }
    crate::state::tmc::ENCODER_REVERSED.store(invert, core::sync::atomic::Ordering::Relaxed);
    // Signal the tick loop to reset feedforward state: prev_position_target
    // has the wrong sign after a direction flip, which would produce a
    // one-tick velocity impulse in the feedforward delta computation.
    crate::state::tmc::FEEDFORWARD_RESET.store(true, core::sync::atomic::Ordering::Relaxed);
    defmt::info!(
        "TMC4671 encoder direction: {}",
        if invert { "reversed" } else { "default" }
    );
    Ok(())
}

/// Mark the motor electrically energized after verified calibration.
///
/// Asserts `state.calibrated`; sets the enable pin high; updates
/// `state.motor_enabled`. Does NOT touch `MOTOR_ARMED` — stepper
/// arming is a separate concern (see `arm_motor_for_steps`).
///
/// `assert!`, not `debug_assert!`: this is a hard motor-safety
/// contract and must enforce in release builds. Costs one branch
/// per arming (not on the hot path). On failure: panic -> RTIC fault
/// handler -> safe-state via the GPIO Drop and `PIDOUT_UQ_UD_LIMITS` cap.
pub fn arm_motor_electrical<B: CommissioningBackend>(
    backend: &mut B,
    state: &mut DispatchState<'_>,
) {
    assert!(
        *state.calibrated,
        "FATAL: arm_motor_electrical called with calibrated=false; \
         SPI restore must precede arm"
    );
    backend.set_enable_pin(true);
    *state.motor_enabled = true;
}

/// Arm the stepper for step-timer processing.
///
/// Asserts `state.motor_enabled` (set only by `arm_motor_electrical`);
/// stores `MOTOR_ARMED=true`. Caller is responsible for any
/// prerequisite state (e.g. zeroing `STEPPER_POSITION`) before
/// invoking. Always-on assert.
pub fn arm_motor_for_steps(state: &mut DispatchState<'_>) {
    assert!(
        *state.motor_enabled,
        "FATAL: arm_motor_for_steps called with motor_enabled=false"
    );
    crate::state::tmc::MOTOR_ARMED.store(true, core::sync::atomic::Ordering::Relaxed);
}

/// Apply biquad filters via [`CommissioningBackend`].
///
/// Mirror of [`apply_stored_filters`] (which uses raw `Tmc4671Spi<B>`),
/// but routes all writes through the backend trait so the dispatch can
/// call it without holding a raw SPI device. Best-effort: SPI failures
/// are ignored (filters affect loop shaping; the loop runs unfiltered
/// if writes fail).
///
/// Each [`SubRegister`] write reproduces the addr-then-data sequence that
/// `Tmc4671Spi::write_sub_register` does internally: write `sub_addr`
/// (the selector) to the `addr` register, then write the value to the
/// `data` register.
///
/// # Parameters
///
/// - `backend` — mutable reference to a [`CommissioningBackend`] implementation.
/// - `pwm_maxcnt` — board-specific PWM counter maximum, used to derive
///   the biquad sampling frequency for coefficient calculation.
pub async fn apply_stored_filters_via_backend<B: CommissioningBackend>(
    backend: &mut B,
    pwm_maxcnt: u16,
) {
    use core::sync::atomic::Ordering::Relaxed;
    use foci_tmc4671::registers::{SubRegister, *};

    /// The six SubRegister addresses that make up one biquad channel.
    struct BiquadFilterSubRegs {
        a_1: SubRegister,
        a_2: SubRegister,
        b_0: SubRegister,
        b_1: SubRegister,
        b_2: SubRegister,
        enable: SubRegister,
    }

    async fn write_sub<B: CommissioningBackend>(
        backend: &mut B,
        sub: SubRegister,
        val: u32,
    ) -> Result<(), B::Error> {
        backend
            .write_register(sub.addr, sub.sub_addr as u32)
            .await?;
        backend.write_register(sub.data, val).await?;
        Ok(())
    }

    async fn apply_one<B: CommissioningBackend>(
        backend: &mut B,
        filter_hz: u16,
        pwm_maxcnt: u16,
        regs: BiquadFilterSubRegs,
    ) {
        if filter_hz == 0 {
            return;
        }
        let Some(coeffs) = crate::biquad::butterworth_lowpass(filter_hz, pwm_maxcnt) else {
            return;
        };
        let _ = write_sub(backend, regs.a_1, coeffs.a_1 as u32).await;
        let _ = write_sub(backend, regs.a_2, coeffs.a_2 as u32).await;
        let _ = write_sub(backend, regs.b_0, coeffs.b_0 as u32).await;
        let _ = write_sub(backend, regs.b_1, coeffs.b_1 as u32).await;
        let _ = write_sub(backend, regs.b_2, coeffs.b_2 as u32).await;
        let _ = write_sub(backend, regs.enable, 1).await;
    }

    apply_one(
        backend,
        crate::state::tmc::VELOCITY_FILTER_HZ.load(Relaxed),
        pwm_maxcnt,
        BiquadFilterSubRegs {
            a_1: CONFIG_BIQUAD_V_A_1,
            a_2: CONFIG_BIQUAD_V_A_2,
            b_0: CONFIG_BIQUAD_V_B_0,
            b_1: CONFIG_BIQUAD_V_B_1,
            b_2: CONFIG_BIQUAD_V_B_2,
            enable: CONFIG_BIQUAD_V_ENABLE,
        },
    )
    .await;
    apply_one(
        backend,
        crate::state::tmc::TORQUE_FILTER_HZ.load(Relaxed),
        pwm_maxcnt,
        BiquadFilterSubRegs {
            a_1: CONFIG_BIQUAD_T_A_1,
            a_2: CONFIG_BIQUAD_T_A_2,
            b_0: CONFIG_BIQUAD_T_B_0,
            b_1: CONFIG_BIQUAD_T_B_1,
            b_2: CONFIG_BIQUAD_T_B_2,
            enable: CONFIG_BIQUAD_T_ENABLE,
        },
    )
    .await;
    apply_one(
        backend,
        crate::state::tmc::POSITION_FILTER_HZ.load(Relaxed),
        pwm_maxcnt,
        BiquadFilterSubRegs {
            a_1: CONFIG_BIQUAD_X_A_1,
            a_2: CONFIG_BIQUAD_X_A_2,
            b_0: CONFIG_BIQUAD_X_B_0,
            b_1: CONFIG_BIQUAD_X_B_1,
            b_2: CONFIG_BIQUAD_X_B_2,
            enable: CONFIG_BIQUAD_X_ENABLE,
        },
    )
    .await;
    apply_one(
        backend,
        crate::state::tmc::FLUX_FILTER_HZ.load(Relaxed),
        pwm_maxcnt,
        BiquadFilterSubRegs {
            a_1: CONFIG_BIQUAD_F_A_1,
            a_2: CONFIG_BIQUAD_F_A_2,
            b_0: CONFIG_BIQUAD_F_B_0,
            b_1: CONFIG_BIQUAD_F_B_1,
            b_2: CONFIG_BIQUAD_F_B_2,
            enable: CONFIG_BIQUAD_F_ENABLE,
        },
    )
    .await;
}

/// De-energize the motor and clear all armed/enabled state.
///
/// Routes the safe-state register sequence through
/// [`crate::selftest::restore_safe_state_via_backend`], then clears
/// `state.calibrated`, `state.motor_enabled`, and
/// [`crate::state::tmc::MOTOR_ARMED`].
///
/// Idempotent: safe to call when the motor is already disarmed.
/// Does not touch the stepper queue or invoke `stepper_stop_from_p1` —
/// those remain board-side responsibilities.
pub async fn disarm_motor<B: CommissioningBackend>(backend: &mut B, state: &mut DispatchState<'_>) {
    crate::selftest::restore_safe_state_via_backend(backend).await;
    *state.calibrated = false;
    *state.motor_enabled = false;
    crate::state::tmc::MOTOR_ARMED.store(false, core::sync::atomic::Ordering::Relaxed);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::commission_dispatch::DispatchState;
    use crate::commission_dispatch::test_support::{MockBackend, fresh_state_locals};

    #[test]
    #[should_panic(expected = "calibrated=false")]
    fn arm_motor_electrical_panics_when_uncalibrated() {
        let mut backend = MockBackend::new();
        let (mut cal, mut en, mut gains) = fresh_state_locals();
        let mut state = DispatchState {
            calibrated: &mut cal,
            motor_enabled: &mut en,
            stored_gains: &mut gains,
        };
        // calibrated is false; this MUST panic.
        arm_motor_electrical(&mut backend, &mut state);
    }

    #[test]
    #[should_panic(expected = "motor_enabled=false")]
    fn arm_motor_for_steps_panics_when_motor_disabled() {
        let (mut cal, mut en, mut gains) = fresh_state_locals();
        let mut state = DispatchState {
            calibrated: &mut cal,
            motor_enabled: &mut en,
            stored_gains: &mut gains,
        };
        // motor_enabled is false; this MUST panic.
        arm_motor_for_steps(&mut state);
    }

    #[test]
    fn arm_then_disarm_returns_to_baseline() {
        pollster::block_on(async {
            use core::sync::atomic::Ordering;
            let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
                .lock()
                .unwrap();
            let mut backend = MockBackend::new();
            let (mut cal, mut en, mut gains) = fresh_state_locals();
            cal = true;
            let mut state = DispatchState {
                calibrated: &mut cal,
                motor_enabled: &mut en,
                stored_gains: &mut gains,
            };
            arm_motor_electrical(&mut backend, &mut state);
            arm_motor_for_steps(&mut state);
            assert!(backend.enable_pin);
            assert!(crate::state::tmc::MOTOR_ARMED.load(Ordering::Relaxed));
            disarm_motor(&mut backend, &mut state).await;
            assert!(!backend.enable_pin);
            assert!(!*state.calibrated);
            assert!(!*state.motor_enabled);
            assert!(!crate::state::tmc::MOTOR_ARMED.load(Ordering::Relaxed));
        });
    }

    #[test]
    fn arm_motor_success_path() {
        use core::sync::atomic::Ordering;
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
            .lock()
            .unwrap();
        let mut backend = MockBackend::new();
        let (mut cal, mut en, mut gains) = fresh_state_locals();
        cal = true;
        let mut state = DispatchState {
            calibrated: &mut cal,
            motor_enabled: &mut en,
            stored_gains: &mut gains,
        };
        arm_motor_electrical(&mut backend, &mut state);
        assert!(backend.enable_pin);
        assert!(*state.motor_enabled);
        assert!(!crate::state::tmc::MOTOR_ARMED.load(Ordering::Relaxed));
        arm_motor_for_steps(&mut state);
        assert!(crate::state::tmc::MOTOR_ARMED.load(Ordering::Relaxed));
        // Reset the static for downstream tests.
        crate::state::tmc::MOTOR_ARMED.store(false, Ordering::Relaxed);
    }

    #[test]
    fn apply_stored_filters_via_backend_writes_enabled_filters() {
        pollster::block_on(async {
            use core::sync::atomic::Ordering;
            let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
                .lock()
                .unwrap();
            // Set the velocity filter to a non-zero Hz so it gets applied.
            crate::state::tmc::VELOCITY_FILTER_HZ.store(100, Ordering::Relaxed);
            crate::state::tmc::TORQUE_FILTER_HZ.store(0, Ordering::Relaxed);
            crate::state::tmc::POSITION_FILTER_HZ.store(0, Ordering::Relaxed);
            crate::state::tmc::FLUX_FILTER_HZ.store(0, Ordering::Relaxed);

            let mut backend = MockBackend::new();
            apply_stored_filters_via_backend(&mut backend, 4095).await;

            // All four biquad SubRegisters share the same data register (0x4D)
            // and the same addr register (0x4E). The only way to distinguish
            // which sub-register was targeted is by checking the selector value
            // written to the addr register immediately before each data write.
            // Check consecutive (selector, data) write pairs.
            use crate::commission_dispatch::test_support::SpiOp;
            use foci_tmc4671::registers::*;
            let writes: std::vec::Vec<(u8, u32)> = backend
                .spi_log
                .iter()
                .filter_map(|op| {
                    if let SpiOp::Write(addr, val) = op {
                        Some((*addr, *val))
                    } else {
                        None
                    }
                })
                .collect();

            // Returns true when the write log contains the consecutive pair:
            //   Write(addr_reg, sub_addr_val) immediately followed by
            //   Write(data_reg, data_val).
            let has_sub_write = |sub: foci_tmc4671::registers::SubRegister, data_val: u32| {
                writes.windows(2).any(|pair| {
                    pair[0].0 == sub.addr.addr()
                        && pair[0].1 == sub.sub_addr as u32
                        && pair[1].0 == sub.data.addr()
                        && pair[1].1 == data_val
                })
            };

            // Velocity biquad was Hz=100; its enable=1 pair must appear.
            assert!(
                has_sub_write(CONFIG_BIQUAD_V_ENABLE, 1),
                "velocity filter enable=1 write pair not found"
            );
            // Torque/position/flux were Hz=0; their enable=1 pairs must NOT appear.
            assert!(
                !has_sub_write(CONFIG_BIQUAD_T_ENABLE, 1),
                "torque filter should not have been enabled"
            );
            assert!(
                !has_sub_write(CONFIG_BIQUAD_X_ENABLE, 1),
                "position filter should not have been enabled"
            );
            assert!(
                !has_sub_write(CONFIG_BIQUAD_F_ENABLE, 1),
                "flux filter should not have been enabled"
            );

            // Cleanup.
            crate::state::tmc::VELOCITY_FILTER_HZ.store(0, Ordering::Relaxed);
        });
    }
}
