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
//!
//! Populated progressively by Phase A-7 Tasks 26–27 of the internal-
//! restructure plan (`docs/plans/archive/2026-04-20-phase-a-internal-
//! restructure.md`). See the extraction blueprint in this session's
//! logbook entry for commit-by-commit scope.

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
