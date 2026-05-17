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
//!   the channel filter-Hz atomics back to TMC4671 registers.
//! - [`apply_set_current`] — convert RMS current to TMC4671 peak counts, write
//!   `PID_TORQUE_FLUX_LIMITS`, and store the result in channel state.
//! - [`apply_set_encoder`] — validate PPR, write `ABN_DECODER_PPR` (×4 for
//!   quadrature), and log the computed step width.
//! - [`apply_set_encoder_dir`] — write `ABN_DECODER_MODE` and update the
//!   channel encoder direction / feedforward reset atomics.
//!
//! Populated progressively by Phase A-7 Tasks 26–27 of the internal-
//! restructure plan (`docs/plans/archive/2026-04-20-phase-a-internal-
//! restructure.md`). See the extraction blueprint in this session's
//! logbook entry for commit-by-commit scope.

use crate::commission_dispatch::{DispatchState, StepperPostAction};
use crate::commissioning::types::CommissioningBackend;
use crate::state::tmc::TmcChannelState;
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
pub async fn settle_hold_and_clear_pid_errors<B>(
    tmc: &mut Tmc4671Spi<B>,
    tmc_state: &TmcChannelState,
) -> Result<(), ()>
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
    let step_width = tmc_state
        .step_width_value
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
pub async fn apply_stored_filters<B>(
    tmc: &mut Tmc4671Spi<B>,
    tmc_state: &TmcChannelState,
    pwm_maxcnt: u16,
) where
    B: SpiDevice<u8>,
{
    use core::sync::atomic::Ordering::Relaxed;
    use foci_tmc4671::registers::*;

    // Velocity biquad.
    let filter_hz = tmc_state.velocity_filter_hz.load(Relaxed);
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
    let filter_hz = tmc_state.torque_filter_hz.load(Relaxed);
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
    let filter_hz = tmc_state.position_filter_hz.load(Relaxed);
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
    let filter_hz = tmc_state.flux_filter_hz.load(Relaxed);
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
/// writes `PID_TORQUE_FLUX_LIMITS`, and stores the register value in
/// channel current-limit state for use by the commissioning engine.
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
    tmc_state: &TmcChannelState,
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
    let limit = counts;
    if tmc
        .write_register(PID_TORQUE_FLUX_LIMITS, limit)
        .await
        .is_err()
    {
        defmt::error!("TMC4671 SPI write failed: PID_TORQUE_FLUX_LIMITS");
        return Err(());
    }
    tmc_state
        .current_limit_reg
        .store(limit, core::sync::atomic::Ordering::Relaxed);
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
/// writes `ABN_DECODER_PPR`, and logs the mechanical-angle step width.
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
    let step_width = 65536u64 / quadrature_ppr as u64;
    defmt::info!("TMC4671 encoder: {} PPR, step_width={}", ppr, step_width);
    Ok(())
}

/// Apply an encoder direction setting to the TMC4671.
///
/// Writes `ABN_DECODER_MODE` (bit 12 selects reversed/default direction),
/// stores the inversion flag, and sets the channel feedforward reset flag to
/// prevent a one-tick velocity impulse from the sign-flipped delta in the
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
pub async fn apply_set_encoder_dir<B>(
    tmc: &mut Tmc4671Spi<B>,
    tmc_state: &TmcChannelState,
    invert: bool,
) -> Result<(), ()>
where
    B: SpiDevice<u8>,
{
    use foci_tmc4671::registers::*;

    let mode = if invert { 1u32 << 12 } else { 0 };
    if tmc.write_register(ABN_DECODER_MODE, mode).await.is_err() {
        defmt::error!("TMC4671 SPI write failed: ABN_DECODER_MODE");
        return Err(());
    }
    tmc_state
        .encoder_reversed
        .store(invert, core::sync::atomic::Ordering::Relaxed);
    // Signal the tick loop to reset feedforward state: prev_position_target
    // has the wrong sign after a direction flip, which would produce a
    // one-tick velocity impulse in the feedforward delta computation.
    tmc_state
        .feedforward_reset
        .store(true, core::sync::atomic::Ordering::Relaxed);
    defmt::info!(
        "TMC4671 encoder direction: {}",
        if invert { "reversed" } else { "default" }
    );
    Ok(())
}

/// Mark the motor electrically energized after verified calibration.
///
/// Asserts `state.calibrated`; sets the enable pin high; updates
/// `state.motor_enabled`. Does NOT touch channel motor arming — stepper
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

/// Board task to run after a stepper post-action state transition.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum StepperTaskDirective {
    /// No board task should be spawned.
    None,
    /// Spawn the board-local arm-check task.
    ArmCheck,
    /// Spawn the board-local stop-check task.
    StopCheck,
}

/// Apply P1-visible stepper state for a terminal commissioning action.
///
/// Returns the board task directive only after all shared atomics and step
/// epoch state are updated, so the board cannot spawn `arm_check` or
/// `stop_check` before those tasks can observe the intended state.
pub fn apply_stepper_post_action_state(
    action: StepperPostAction,
    state: &mut DispatchState<'_>,
) -> StepperTaskDirective {
    match action {
        StepperPostAction::None => StepperTaskDirective::None,
        StepperPostAction::Stop => {
            state
                .tmc
                .motor_armed
                .store(false, core::sync::atomic::Ordering::Relaxed);
            StepperTaskDirective::StopCheck
        }
        StepperPostAction::ArmZeroPosition => {
            state
                .stepper
                .position
                .store(0, core::sync::atomic::Ordering::Relaxed);
            arm_motor_for_steps(state);
            StepperTaskDirective::ArmCheck
        }
        StepperPostAction::ArmPreservePosition => {
            arm_motor_for_steps(state);
            StepperTaskDirective::ArmCheck
        }
    }
}

/// Apply biquad filters via `CommissioningBackend`.
///
/// Mirror of [`apply_stored_filters`] (which uses raw `Tmc4671Spi<B>`),
/// but routes all writes through the backend trait so the dispatch can
/// call it without holding a raw SPI device.
///
/// Each `SubRegister` write reproduces the addr-then-data sequence that
/// `Tmc4671Spi::write_sub_register` does internally: write `sub_addr`
/// (the selector) to the `addr` register, then write the value to the
/// `data` register.
///
/// # Parameters
///
/// - `backend` — mutable reference to a `CommissioningBackend` implementation.
/// - `pwm_maxcnt` — board-specific PWM counter maximum, used to derive
///   the biquad sampling frequency for coefficient calculation.
pub async fn apply_stored_filters_via_backend<B: CommissioningBackend>(
    backend: &mut B,
    tmc_state: &TmcChannelState,
    pwm_maxcnt: u16,
) -> Result<(), B::Error> {
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
    ) -> Result<(), B::Error> {
        if filter_hz == 0 {
            return Ok(());
        }
        let Some(coeffs) = crate::biquad::butterworth_lowpass(filter_hz, pwm_maxcnt) else {
            return Ok(());
        };
        write_sub(backend, regs.a_1, coeffs.a_1 as u32).await?;
        write_sub(backend, regs.a_2, coeffs.a_2 as u32).await?;
        write_sub(backend, regs.b_0, coeffs.b_0 as u32).await?;
        write_sub(backend, regs.b_1, coeffs.b_1 as u32).await?;
        write_sub(backend, regs.b_2, coeffs.b_2 as u32).await?;
        write_sub(backend, regs.enable, 1).await?;
        Ok(())
    }

    apply_one(
        backend,
        tmc_state.velocity_filter_hz.load(Relaxed),
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
    .await?;
    apply_one(
        backend,
        tmc_state.torque_filter_hz.load(Relaxed),
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
    .await?;
    apply_one(
        backend,
        tmc_state.position_filter_hz.load(Relaxed),
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
    .await?;
    apply_one(
        backend,
        tmc_state.flux_filter_hz.load(Relaxed),
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
    .await?;
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

/// Pack TMC4671 outer-loop gain registers from raw Q8.8 gain values.
///
/// The TMC4671 stores the P term in the upper 16 bits and the I term in the
/// lower 16 bits for both velocity and position outer-loop registers.
pub fn pack_outer_gain_registers(
    position_p: u16,
    position_i: u16,
    velocity_p: u16,
    velocity_i: u16,
) -> (u32, u32) {
    let velocity_reg = ((velocity_p as u32) << 16) | velocity_i as u32;
    let position_reg = ((position_p as u32) << 16) | position_i as u32;
    (velocity_reg, position_reg)
}

/// Pack TMC4671 inner current-loop gain registers from raw gain values.
///
/// The TMC4671 stores the P term in the upper 16 bits and the I term in the
/// lower 16 bits for both flux and torque PI registers.
pub fn pack_inner_gain_registers(gains: crate::pid::InnerLoopGains) -> (u32, u32) {
    let flux_reg = ((gains.flux_p as u32) << 16) | gains.flux_i as u32;
    let torque_reg = ((gains.torque_p as u32) << 16) | gains.torque_i as u32;
    (flux_reg, torque_reg)
}

/// SPI restore sequence for the OuterDone success path.
///
/// Writes tuned velocity and position gains, the operational velocity limit,
/// current limit from channel state, the
/// safety-net `PIDOUT_UQ_UD_LIMITS` cap (1024 counts), clears the
/// position-actual integrator, snaps the position target to the current
/// encoder count scaled by the channel step width, and
/// enters position mode via `MODE_RAMP_MODE_MOTION = 0x8000_0003`.
///
/// The write order matches the OuterDone restore block in `main.rs` exactly.
/// Motor safety depends on setting limits (`PID_TORQUE_FLUX_LIMITS`,
/// `PIDOUT_UQ_UD_LIMITS`) before entering motion mode.
///
/// # Parameters
///
/// - `backend` — mutable reference to a `CommissioningBackend` implementation.
/// - `outer` — outer commissioning result containing tuned gain values.
///
/// # Errors
///
/// Returns the backend's SPI error on the first failed write.
pub async fn restore_position_mode_after_outer<B: CommissioningBackend>(
    backend: &mut B,
    tmc_state: &TmcChannelState,
    outer: &crate::commissioning::types::OuterResult,
) -> Result<(), B::Error> {
    use core::sync::atomic::Ordering::Relaxed;
    use foci_tmc4671::registers::*;

    // Tuned velocity + position gains.
    let (velocity_reg, position_reg) = pack_outer_gain_registers(
        outer.position_p,
        outer.position_i,
        outer.velocity_p,
        outer.velocity_i,
    );
    backend
        .write_register(PID_VELOCITY_P_VELOCITY_I, velocity_reg)
        .await?;
    backend
        .write_register(PID_POSITION_P_POSITION_I, position_reg)
        .await?;

    // Operational limits (not safety caps).
    backend
        .write_register(PID_VELOCITY_LIMIT, outer.velocity_limit)
        .await?;

    let current_limit = tmc_state.current_limit_reg.load(Relaxed);
    backend
        .write_register(PID_TORQUE_FLUX_LIMITS, current_limit)
        .await?;

    // Safety-net voltage cap — must be written before entering motion mode.
    backend.write_register(PIDOUT_UQ_UD_LIMITS, 1024).await?;

    // Clear PID error sums before mode switch.
    backend.write_register(PID_POSITION_ACTUAL, 0).await?;

    let target = backend.read_register(ABN_DECODER_COUNT).await?;
    let target_scaled = target.wrapping_mul(tmc_state.step_width_value.load(Relaxed) as u32);
    backend
        .write_register(PID_POSITION_TARGET, target_scaled)
        .await?;

    // Enter position mode (advanced PID cascade).
    backend
        .write_register(MODE_RAMP_MODE_MOTION, 0x8000_0003)
        .await?;

    Ok(())
}

/// SPI restore sequence for the CalibrateDone path.
///
/// Reapplies host-stored inner gains (if any) to `PID_FLUX_P_FLUX_I` and
/// `PID_TORQUE_P_TORQUE_I`, then reads outer gain values from the
/// [`crate::state::tmc`] atomics and writes velocity gains, position gains,
/// velocity limit, and current limit back to the TMC4671.
///
/// The write order matches the CalibrateDone gain-restore block in `main.rs`
/// exactly. SPI failures abort immediately; reporting success with wrong
/// gains active is worse than failing.
///
/// # Parameters
///
/// - `backend` — mutable reference to a `CommissioningBackend` implementation.
/// - `stored` — optional host-stored inner loop gains; skipped when `None`.
///
/// # Errors
///
/// Returns the backend's SPI error on the first failed write.
pub async fn restore_calibrate_gains<B: CommissioningBackend>(
    backend: &mut B,
    tmc_state: &TmcChannelState,
    stored: Option<crate::pid::InnerLoopGains>,
) -> Result<(), B::Error> {
    use core::sync::atomic::Ordering::Relaxed;
    use foci_tmc4671::registers::*;

    // Current-loop gains from prior commissioning.
    if let Some(gains) = stored {
        let (flux_reg, torque_reg) = pack_inner_gain_registers(gains);
        backend.write_register(PID_FLUX_P_FLUX_I, flux_reg).await?;
        backend
            .write_register(PID_TORQUE_P_TORQUE_I, torque_reg)
            .await?;
    }

    let velocity_p = tmc_state.velocity_p_value.load(Relaxed);
    let velocity_i = tmc_state.velocity_i_value.load(Relaxed);
    let position_p = tmc_state.position_p_value.load(Relaxed);
    let position_i = tmc_state.position_i_value.load(Relaxed);
    let (velocity_reg, position_reg) =
        pack_outer_gain_registers(position_p, position_i, velocity_p, velocity_i);

    // Velocity gains.
    backend
        .write_register(PID_VELOCITY_P_VELOCITY_I, velocity_reg)
        .await?;

    // Position gains.
    backend
        .write_register(PID_POSITION_P_POSITION_I, position_reg)
        .await?;

    // Velocity limit.
    let velocity_limit = tmc_state.velocity_limit_cfg.load(Relaxed);
    backend
        .write_register(PID_VELOCITY_LIMIT, velocity_limit)
        .await?;

    // Current limit (TMC4671 resets on mode change).
    let current_limit = tmc_state.current_limit_reg.load(Relaxed);
    backend
        .write_register(PID_TORQUE_FLUX_LIMITS, current_limit)
        .await?;

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::commission_dispatch::DispatchState;
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

    fn stepper_state() -> &'static crate::state::stepper::StepperChannelState {
        crate::state::stepper::stepper_channel0()
    }

    #[derive(Default)]
    struct MockRawSpi {
        writes: std::vec::Vec<std::vec::Vec<u8>>,
    }

    impl embedded_hal_async::spi::ErrorType for MockRawSpi {
        type Error = core::convert::Infallible;
    }

    impl SpiDevice<u8> for MockRawSpi {
        async fn transaction(
            &mut self,
            operations: &mut [embedded_hal_async::spi::Operation<'_, u8>],
        ) -> Result<(), Self::Error> {
            for op in operations {
                match op {
                    embedded_hal_async::spi::Operation::Read(buf) => buf.fill(0),
                    embedded_hal_async::spi::Operation::Write(buf) => {
                        self.writes.push(buf.to_vec())
                    }
                    embedded_hal_async::spi::Operation::Transfer(read, write) => {
                        read.copy_from_slice(write);
                    }
                    embedded_hal_async::spi::Operation::TransferInPlace(_) => {}
                    embedded_hal_async::spi::Operation::DelayNs(_) => {}
                }
            }
            Ok(())
        }
    }

    #[test]
    fn apply_set_current_writes_single_current_limit_register() {
        pollster::block_on(async {
            use core::sync::atomic::Ordering;
            use foci_tmc4671::registers::PID_TORQUE_FLUX_LIMITS;

            let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
                .lock()
                .await;
            tmc_state().current_limit_reg.store(0, Ordering::Relaxed);

            let raw_spi = MockRawSpi::default();
            let mut tmc = Tmc4671Spi::new(raw_spi);
            apply_set_current(&mut tmc, tmc_state(), 1500, 1.155)
                .await
                .expect("current write should succeed");

            assert_eq!(
                tmc_state().current_limit_reg.load(Ordering::Relaxed),
                0x0000_072c
            );
            let raw_spi = tmc.into_inner();
            assert_eq!(raw_spi.writes.len(), 1);
            assert_eq!(
                raw_spi.writes[0],
                [PID_TORQUE_FLUX_LIMITS.addr() | 0x80, 0x00, 0x00, 0x07, 0x2c]
            );

            tmc_state().current_limit_reg.store(0, Ordering::Relaxed);
        });
    }

    #[test]
    #[should_panic(expected = "calibrated=false")]
    fn arm_motor_electrical_panics_when_uncalibrated() {
        let mut backend = MockBackend::new();
        let (mut cal, mut en, mut gains) = fresh_state_locals();
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        // calibrated is false; this MUST panic.
        arm_motor_electrical(&mut backend, &mut state);
    }

    #[test]
    #[should_panic(expected = "motor_enabled=false")]
    fn arm_motor_for_steps_panics_when_motor_disabled() {
        let (mut cal, mut en, mut gains) = fresh_state_locals();
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        // motor_enabled is false; this MUST panic.
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
            arm_motor_electrical(&mut backend, &mut state);
            arm_motor_for_steps(&mut state);
            assert!(backend.enable_pin);
            assert!(tmc_state().motor_armed.load(Ordering::Relaxed));
            disarm_motor(&mut backend, &mut state).await;
            assert!(!backend.enable_pin);
            assert!(!*state.calibrated);
            assert!(!*state.motor_enabled);
            assert!(!tmc_state().motor_armed.load(Ordering::Relaxed));
        });
    }

    #[test]
    fn arm_motor_success_path() {
        use core::sync::atomic::Ordering;
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
        let mut backend = MockBackend::new();
        let (_, mut en, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        arm_motor_electrical(&mut backend, &mut state);
        assert!(backend.enable_pin);
        assert!(*state.motor_enabled);
        assert!(!tmc_state().motor_armed.load(Ordering::Relaxed));
        arm_motor_for_steps(&mut state);
        assert!(tmc_state().motor_armed.load(Ordering::Relaxed));
        // Reset the static for downstream tests.
        tmc_state().motor_armed.store(false, Ordering::Relaxed);
    }

    #[test]
    fn stepper_post_action_zeroes_before_arm_check_directive() {
        use core::sync::atomic::Ordering;
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
        let (mut cal, _, mut gains) = fresh_state_locals();
        let mut en = true;
        stepper_state().position.store(123, Ordering::Relaxed);
        tmc_state().motor_armed.store(false, Ordering::Relaxed);
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);

        let directive = apply_stepper_post_action_state(
            crate::commission_dispatch::StepperPostAction::ArmZeroPosition,
            &mut state,
        );

        assert_eq!(directive, StepperTaskDirective::ArmCheck);
        assert_eq!(stepper_state().position.load(Ordering::Relaxed), 0);
        assert!(tmc_state().motor_armed.load(Ordering::Relaxed));
        tmc_state().motor_armed.store(false, Ordering::Relaxed);
    }

    #[test]
    fn stepper_post_action_stop_clears_before_stop_check_directive() {
        use core::sync::atomic::Ordering;
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
        let (mut cal, mut en, mut gains) = fresh_state_locals();
        tmc_state().motor_armed.store(true, Ordering::Relaxed);
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);

        let directive = apply_stepper_post_action_state(
            crate::commission_dispatch::StepperPostAction::Stop,
            &mut state,
        );

        assert_eq!(directive, StepperTaskDirective::StopCheck);
        assert!(!tmc_state().motor_armed.load(Ordering::Relaxed));
    }

    #[test]
    fn apply_stored_filters_via_backend_writes_enabled_filters() {
        pollster::block_on(async {
            use core::sync::atomic::Ordering;
            let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
                .lock()
                .await;
            // Set the velocity filter to a non-zero Hz so it gets applied.
            tmc_state().velocity_filter_hz.store(100, Ordering::Relaxed);
            tmc_state().torque_filter_hz.store(0, Ordering::Relaxed);
            tmc_state().position_filter_hz.store(0, Ordering::Relaxed);
            tmc_state().flux_filter_hz.store(0, Ordering::Relaxed);

            let mut backend = MockBackend::new();
            apply_stored_filters_via_backend(&mut backend, tmc_state(), 4095)
                .await
                .expect("filter writes should succeed");

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
            tmc_state().velocity_filter_hz.store(0, Ordering::Relaxed);
        });
    }

    #[test]
    fn restore_position_mode_after_outer_writes_six_registers() {
        pollster::block_on(async {
            use crate::commission_dispatch::test_support::SpiOp;
            use foci_tmc4671::registers::*;
            let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
                .lock()
                .await;
            let mut backend = MockBackend::new();
            tmc_state()
                .current_limit_reg
                .store(0x0010_0010, core::sync::atomic::Ordering::Relaxed);
            tmc_state()
                .step_width_value
                .store(50, core::sync::atomic::Ordering::Relaxed);
            // Pre-seed ABN_DECODER_COUNT for the position-target compute.
            let _ = backend.write_register(ABN_DECODER_COUNT, 1234).await;
            backend.spi_log.clear();

            let outer = sample_outer_result();
            let result = restore_position_mode_after_outer(&mut backend, tmc_state(), &outer).await;
            assert!(result.is_ok());

            let writes: heapless::Vec<u8, 32> = backend
                .spi_log
                .iter()
                .filter_map(|op| {
                    if let SpiOp::Write(a, _) = op {
                        Some(*a)
                    } else {
                        None
                    }
                })
                .collect();
            assert!(writes.contains(&PID_VELOCITY_P_VELOCITY_I.addr()));
            assert!(writes.contains(&PID_POSITION_P_POSITION_I.addr()));
            assert!(writes.contains(&PID_VELOCITY_LIMIT.addr()));
            assert!(writes.contains(&PID_TORQUE_FLUX_LIMITS.addr()));
            assert!(writes.contains(&PIDOUT_UQ_UD_LIMITS.addr()));
            assert!(writes.contains(&MODE_RAMP_MODE_MOTION.addr()));

            // Cleanup atomics.
            tmc_state()
                .current_limit_reg
                .store(0, core::sync::atomic::Ordering::Relaxed);
            tmc_state()
                .step_width_value
                .store(0, core::sync::atomic::Ordering::Relaxed);
        });
    }

    #[test]
    fn restore_position_mode_after_outer_propagates_spi_failure() {
        pollster::block_on(async {
            use foci_tmc4671::registers::*;
            let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
                .lock()
                .await;
            let mut backend = MockBackend::new();
            backend.fail_on_write = Some(MODE_RAMP_MODE_MOTION.addr());
            let outer = sample_outer_result();
            let result = restore_position_mode_after_outer(&mut backend, tmc_state(), &outer).await;
            assert!(result.is_err());
        });
    }

    fn sample_outer_result() -> crate::commissioning::types::OuterResult {
        crate::commissioning::types::OuterResult {
            status: crate::autotune::CommissionStatus::Accepted,
            velocity_p: 1000,
            velocity_i: 100,
            velocity_limit: 500_000,
            position_p: 640,
            position_i: 0,
            velocity_filter_hz: 0,
            position_filter_hz: 0,
            flux_filter_hz: 0,
            torque_filter_hz: 0,
            j_eff: 0,
            b_eff: 0,
            warning_code: 0,
            mech_torque_step: 0,
            mech_fwd_accel: 0,
            mech_rev_accel: 0,
            mech_fwd_peak_velocity: 0,
            mech_rev_peak_velocity: 0,
            mech_fwd_travel: 0,
            mech_rev_travel: 0,
            synth_velocity_p: 0,
            synth_velocity_i: 0,
            synth_position_p: 0,
            synth_position_i: 0,
        }
    }

    #[test]
    fn restore_calibrate_gains_writes_all_gains_when_stored_present() {
        pollster::block_on(async {
            use crate::commission_dispatch::test_support::SpiOp;
            use foci_tmc4671::registers::*;
            let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
                .lock()
                .await;
            let mut backend = MockBackend::new();
            let stored = Some(crate::pid::InnerLoopGains {
                flux_p: 100,
                flux_i: 10,
                torque_p: 200,
                torque_i: 20,
            });
            let result = restore_calibrate_gains(&mut backend, tmc_state(), stored).await;
            assert!(result.is_ok());
            let writes: heapless::Vec<u8, 32> = backend
                .spi_log
                .iter()
                .filter_map(|op| {
                    if let SpiOp::Write(a, _) = op {
                        Some(*a)
                    } else {
                        None
                    }
                })
                .collect();
            assert!(writes.contains(&PID_FLUX_P_FLUX_I.addr()));
            assert!(writes.contains(&PID_TORQUE_P_TORQUE_I.addr()));
            assert!(writes.contains(&PID_VELOCITY_P_VELOCITY_I.addr()));
            assert!(writes.contains(&PID_POSITION_P_POSITION_I.addr()));
            assert!(writes.contains(&PID_VELOCITY_LIMIT.addr()));
            assert!(writes.contains(&PID_TORQUE_FLUX_LIMITS.addr()));
        });
    }

    #[test]
    fn restore_calibrate_gains_skips_inner_when_stored_none() {
        pollster::block_on(async {
            use crate::commission_dispatch::test_support::SpiOp;
            use foci_tmc4671::registers::*;
            let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
                .lock()
                .await;
            let mut backend = MockBackend::new();
            let result = restore_calibrate_gains(&mut backend, tmc_state(), None).await;
            assert!(result.is_ok());
            let writes: heapless::Vec<u8, 32> = backend
                .spi_log
                .iter()
                .filter_map(|op| {
                    if let SpiOp::Write(a, _) = op {
                        Some(*a)
                    } else {
                        None
                    }
                })
                .collect();
            assert!(!writes.contains(&PID_FLUX_P_FLUX_I.addr()));
            assert!(writes.contains(&PID_VELOCITY_P_VELOCITY_I.addr()));
        });
    }

    #[test]
    fn pack_outer_gain_registers_places_p_in_upper_half() {
        let (velocity_reg, position_reg) = pack_outer_gain_registers(256, 3, 6656, 4);

        assert_eq!(velocity_reg, 0x1a00_0004);
        assert_eq!(position_reg, 0x0100_0003);
    }

    #[test]
    fn pack_inner_gain_registers_places_p_in_upper_half() {
        let gains = crate::pid::InnerLoopGains {
            flux_p: 707,
            flux_i: 162,
            torque_p: 707,
            torque_i: 162,
        };

        let (flux_reg, torque_reg) = pack_inner_gain_registers(gains);

        assert_eq!(flux_reg, 0x02c3_00a2);
        assert_eq!(torque_reg, 0x02c3_00a2);
    }
}
