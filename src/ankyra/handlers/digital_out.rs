//! Digital-output command handlers: `config_digital_out`,
//! `queue_digital_out`, `update_digital_out`.
//!
//! Lifted out of `boards/openffboard-fw/src/ankyra/handlers/digital_out.rs`
//! in Task 11 of the foci-firmware handler-lift plan. Handler bodies now
//! bind to the capability traits declared in [`crate::context`] instead
//! of concrete board types; the board crate wires up the trait impls on
//! its own `FirmwareCtx`.
//!
//! See the handler audit at
//! `docs/notes/foci-firmware-lift-handler-audit.md` for the exact
//! capability bound used by each handler.

use ankyra::prelude::klipper_command;

use crate::context::{ShutdownRecovery, TmcHost};
use crate::shutdown::ShutdownReason;
use crate::tmc::TmcRequest;

/// Register a digital output object for motor enable/disable.
///
/// Pin parameters are accepted for protocol compatibility but ignored;
/// the TMC4671 enable line is controlled directly.
#[klipper_command]
pub fn config_digital_out(
    ctx: &mut dyn ShutdownRecovery,
    oid: u8,
    _pin: u32,
    _value: u8,
    _default_value: u8,
    _max_duration: u32,
) {
    if let Err(e) = foci_klipper::commands::digital_out::config_digital_out(ctx.state(), oid) {
        defmt::error!("config_digital_out: {}", e);
        if ctx.enter_shutdown() {
            let clock = ctx.now();
            ctx.send_shutdown(ShutdownReason::ConfigDigitalOutFailed, clock);
        }
        return;
    }
    defmt::info!(
        "config_digital_out: oid={} (enable control, pin ignored)",
        oid
    );
}

/// Schedule a digital output change at a given clock time.
///
/// Used by Klipper for timed motor enable/disable.
#[klipper_command]
pub fn queue_digital_out(ctx: &mut dyn DigitalOutHost, oid: u8, clock: u32, on_ticks: u32) {
    let _ = clock;
    match foci_klipper::commands::digital_out::queue_digital_out(ctx.state(), oid, on_ticks) {
        Ok(enabled) => {
            if ctx.enqueue_request(TmcRequest::SetEnable(enabled)).is_err() && ctx.enter_shutdown()
            {
                let c = ctx.now();
                ctx.send_shutdown(ShutdownReason::MotorEnableDisableLostQueueFull, c);
            }
        }
        Err(e) => {
            defmt::error!("queue_digital_out: {}", e);
            if ctx.enter_shutdown() {
                let clock = ctx.now();
                ctx.send_shutdown(ShutdownReason::QueueDigitalOutFailed, clock);
            }
        }
    }
}

/// Immediately update a digital output value.
#[klipper_command]
pub fn update_digital_out(ctx: &mut dyn DigitalOutHost, oid: u8, value: u8) {
    match foci_klipper::commands::digital_out::update_digital_out(ctx.state(), oid, value) {
        Ok(enabled) => {
            if ctx.enqueue_request(TmcRequest::SetEnable(enabled)).is_err() && ctx.enter_shutdown()
            {
                let c = ctx.now();
                ctx.send_shutdown(ShutdownReason::MotorEnableDisableLostQueueFull, c);
            }
        }
        Err(e) => {
            defmt::error!("update_digital_out: {}", e);
            if ctx.enter_shutdown() {
                let clock = ctx.now();
                ctx.send_shutdown(ShutdownReason::UpdateDigitalOutFailed, clock);
            }
        }
    }
}

/// Composite bound for `queue_digital_out` and `update_digital_out`:
/// both handlers need shutdown recovery (to enter shutdown on error),
/// a clock source (for the shutdown clock argument), and the TMC host
/// (to enqueue the enable/disable request).
///
/// `&mut dyn ShutdownRecovery + TmcHost` is not a valid trait-object
/// type in stable Rust for two non-auto traits; this named supertrait
/// is the dyn-safe way to compose the capabilities.
pub trait DigitalOutHost: ShutdownRecovery + TmcHost {}
impl<T: ?Sized + ShutdownRecovery + TmcHost> DigitalOutHost for T {}
