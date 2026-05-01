//! TMC4671 command handlers: `tmc_init`, `tmc_read_register`,
//! `tmc_write_register`, `tmc_set_current`, `tmc_set_encoder`,
//! `tmc_set_encoder_dir`, `tmc_set_flux_filter`, `tmc_set_pid_gains`,
//! `tmc_set_position_filter`, `tmc_set_position_gains`,
//! `tmc_set_torque_filter`, `tmc_set_velocity_feedforward`,
//! `tmc_set_velocity_filter`, `tmc_set_velocity_limit`,
//! `tmc_set_auto_calibrate_on_enable`, `tmc_test_spin`.
//!
//! Lifted from `boards/openffboard-fw/src/ankyra/handlers/tmc.rs` in
//! Task 16 of the foci-firmware handler-lift plan. Handler bodies bind to
//! the capability traits declared in [`crate::context`]; the board crate
//! wires up the trait impls on its own `FirmwareCtx`.
//!
//! See the handler audit at
//! `docs/notes/foci-firmware-lift-handler-audit.md` for the exact
//! capability bound used by each handler.

use ankyra::prelude::klipper_command;
use core::sync::atomic::Ordering;

use crate::context::{ShutdownRecovery, StateAccess, TmcHost};
use crate::shutdown::ShutdownReason;
use crate::state::tmc::{
    FLUX_FILTER_HZ, POSITION_FILTER_HZ, STEP_WIDTH_VALUE, TORQUE_FILTER_HZ, VELOCITY_FEEDFORWARD,
    VELOCITY_FILTER_HZ, VELOCITY_LIMIT_CFG,
};

/// Number of TMC4671 motor channels supported by this firmware.
///
/// Single-channel even on future multi-motor boards; parameterize if that
/// ever changes.
const TMC_MOTOR_COUNT: u8 = 1;

/// Trigger TMC4671 initialization sequence (dev-only).
#[cfg(feature = "dev")]
#[klipper_command]
pub fn tmc_init(_ctx: &mut dyn TmcHost, _oid: u8) {
    let _ = _ctx.enqueue_request(crate::tmc::tmc_init());
}

/// Test motor rotation by sweeping PHI_E_EXT through N revolutions (dev-only).
#[cfg(feature = "dev")]
#[klipper_command]
pub fn tmc_test_spin(_ctx: &mut dyn TmcHost, _oid: u8, revolutions: u8) {
    let _ = _ctx.enqueue_request(crate::tmc::tmc_test_spin(revolutions));
}

/// Write a TMC4671 register via SPI (dev-only).
#[cfg(feature = "dev")]
#[klipper_command]
pub fn tmc_write_register(_ctx: &mut dyn TmcHost, _oid: u8, addr: u8, value: u32) {
    let _ = _ctx.enqueue_request(crate::tmc::tmc_write_register(addr, value));
}

/// Read a TMC4671 register via SPI (dev-only).
///
/// The response is sent asynchronously as a `tmc_register_value` reply.
/// If the queue is full the request is dropped silently; the Python
/// query_command timeout handles the missing reply naturally.
#[cfg(feature = "dev")]
#[klipper_command]
pub fn tmc_read_register(_ctx: &mut dyn TmcHost, oid: u8, addr: u8) {
    if _ctx
        .enqueue_request(crate::tmc::tmc_read_register(oid, addr))
        .is_err()
    {
        defmt::warn!("tmc_read_register dropped: addr={:#04x}", addr);
    }
}

/// Set motor run current. The firmware converts milliamps to
/// `PID_TORQUE_FLUX_LIMITS` register counts using the board-specific
/// current scaling factor.
#[klipper_command]
pub fn tmc_set_current(ctx: &mut dyn TmcWriteHost, _oid: u8, run_ma: u32) {
    if ctx
        .enqueue_request(crate::tmc::tmc_set_current(run_ma))
        .is_err()
        && ctx.enter_shutdown()
    {
        let clock = ctx.now();
        ctx.send_shutdown(ShutdownReason::TmcSetCurrentLostQueueFull, clock);
    }
}

/// Configure ABN encoder resolution and compute step width.
///
/// Uses `tmc_set_encoder_hw` to compute the step width from PPR and
/// pole pairs, then stores it in the `STEP_WIDTH_VALUE` atomic for
/// the TMC4671 STEP_WIDTH register. The TMC4671 position PID operates
/// on phi_e, not encoder counts:
///   `step_width = 65536 * pole_pairs / quadrature_ppr`
#[klipper_command]
pub fn tmc_set_encoder(ctx: &mut dyn TmcWriteHost, _oid: u8, channel: u8, ppr: u32) {
    if channel >= TMC_MOTOR_COUNT {
        if ctx.enter_shutdown() {
            let clock = ctx.now();
            ctx.send_shutdown(ShutdownReason::InvalidTmcChannel, clock);
        }
        return;
    }
    let (req, step_width) = crate::tmc::tmc_set_encoder_hw(ppr, 50);
    STEP_WIDTH_VALUE.store(step_width, Ordering::Relaxed);
    defmt::info!("channel={} ppr={} step_width={}", channel, ppr, step_width);
    if ctx.enqueue_request(req).is_err() && ctx.enter_shutdown() {
        let clock = ctx.now();
        ctx.send_shutdown(ShutdownReason::TmcSetEncoderLostQueueFull, clock);
    }
}

/// Set ABN encoder count direction. `invert=1` flips the count direction
/// so the position PID has correct (negative) feedback. Depends on
/// motor/encoder wiring — not board-specific.
#[klipper_command]
pub fn tmc_set_encoder_dir(ctx: &mut dyn TmcWriteHost, _oid: u8, channel: u8, invert: u8) {
    if channel >= TMC_MOTOR_COUNT {
        if ctx.enter_shutdown() {
            let clock = ctx.now();
            ctx.send_shutdown(ShutdownReason::InvalidTmcChannel, clock);
        }
        return;
    }
    if ctx
        .enqueue_request(crate::tmc::tmc_set_encoder_dir(invert != 0))
        .is_err()
        && ctx.enter_shutdown()
    {
        let clock = ctx.now();
        ctx.send_shutdown(ShutdownReason::TmcSetEncoderDirLostQueueFull, clock);
    }
}

/// Set inner loop PI gains from host (explicit override or saved values).
#[klipper_command]
pub fn tmc_set_pid_gains(
    ctx: &mut dyn TmcWriteHost,
    _oid: u8,
    flux_p: u16,
    flux_i: u16,
    torque_p: u16,
    torque_i: u16,
) {
    if ctx
        .enqueue_request(crate::tmc::tmc_set_pid_gains(
            flux_p, flux_i, torque_p, torque_i,
        ))
        .is_err()
        && ctx.enter_shutdown()
    {
        let clock = ctx.now();
        ctx.send_shutdown(ShutdownReason::TmcSetPidGainsLostQueueFull, clock);
    }
}

/// Set velocity biquad filter cutoff frequency from host config.
#[klipper_command]
pub fn tmc_set_velocity_filter(_ctx: &mut dyn StateAccess, _oid: u8, filter_hz: u16) {
    VELOCITY_FILTER_HZ.store(filter_hz, Ordering::Relaxed);
    defmt::info!("Velocity filter: {} Hz", filter_hz);
}

/// Set torque biquad filter cutoff frequency from host config.
#[klipper_command]
pub fn tmc_set_torque_filter(_ctx: &mut dyn StateAccess, _oid: u8, filter_hz: u16) {
    TORQUE_FILTER_HZ.store(filter_hz, Ordering::Relaxed);
    defmt::info!("Torque filter: {} Hz", filter_hz);
}

/// Set position target biquad filter cutoff frequency from host config.
#[klipper_command]
pub fn tmc_set_position_filter(_ctx: &mut dyn StateAccess, _oid: u8, filter_hz: u16) {
    POSITION_FILTER_HZ.store(filter_hz, Ordering::Relaxed);
    defmt::info!("Position filter: {} Hz", filter_hz);
}

/// Set flux biquad filter cutoff frequency from host config.
#[klipper_command]
pub fn tmc_set_flux_filter(_ctx: &mut dyn StateAccess, _oid: u8, filter_hz: u16) {
    FLUX_FILTER_HZ.store(filter_hz, Ordering::Relaxed);
    defmt::info!("Flux filter: {} Hz", filter_hz);
}

/// Set position/velocity PI gains from host config.
#[klipper_command]
pub fn tmc_set_position_gains(
    ctx: &mut dyn TmcWriteHost,
    _oid: u8,
    position_p: u16,
    position_i: u16,
    velocity_p: u16,
    velocity_i: u16,
) {
    if ctx
        .enqueue_request(crate::tmc::tmc_set_position_gains(
            position_p, position_i, velocity_p, velocity_i,
        ))
        .is_err()
        && ctx.enter_shutdown()
    {
        let clock = ctx.now();
        ctx.send_shutdown(ShutdownReason::TmcSetPositionGainsLostQueueFull, clock);
    }
}

/// Enable or disable velocity feedforward from host config.
#[klipper_command]
pub fn tmc_set_velocity_feedforward(_ctx: &mut dyn StateAccess, _oid: u8, enable: u8) {
    VELOCITY_FEEDFORWARD.store(enable != 0, Ordering::Relaxed);
    defmt::info!("Velocity feedforward: {}", enable != 0);
}

/// Set PID velocity limit from host config.
///
/// Caps position PID output to prevent integrator-driven velocity runaway.
/// Firmware default: 500,000. Override via `pid_velocity_limit` in
/// `printer.cfg`.
#[klipper_command]
pub fn tmc_set_velocity_limit(_ctx: &mut dyn StateAccess, _oid: u8, limit: u32) {
    VELOCITY_LIMIT_CFG.store(limit, Ordering::Relaxed);
    defmt::info!("Velocity limit: {}", limit);
}

/// Set whether a raw enable may trigger firmware-side auto-calibration.
#[klipper_command]
pub fn tmc_set_auto_calibrate_on_enable(_ctx: &mut dyn StateAccess, _oid: u8, enable: u8) {
    crate::tmc::set_auto_calibrate_on_enable_allowed(enable != 0);
    defmt::info!("Auto-calibrate on enable: {}", enable != 0);
}

/// Composite bound for handlers that both enqueue TMC requests and may
/// trigger shutdown on queue-full errors.
///
/// `&mut dyn Trait1 + Trait2` is not a valid trait-object type in stable
/// Rust for two non-auto traits; this named supertrait is the dyn-safe way
/// to compose `ShutdownRecovery` (shutdown + clock) and `TmcHost` (request
/// enqueue). Mirrors the `LifecycleReset` pattern introduced in Task 12.
pub trait TmcWriteHost: ShutdownRecovery + TmcHost {}
impl<T: ?Sized + ShutdownRecovery + TmcHost> TmcWriteHost for T {}
