//! Endstop command handlers: `config_endstop`, `endstop_home`,
//! `endstop_query_state`.
//!
//! Lifted out of `boards/openffboard-fw/src/ankyra/handlers/endstop.rs`
//! in Task 12 of the foci-firmware handler-lift plan. Handler bodies now
//! bind to the capability traits declared in [`crate::context`] instead
//! of concrete board types; the board crate wires up the trait impls on
//! its own `FirmwareCtx`.
//!
//! See the handler audit at
//! `docs/notes/foci-firmware-lift-handler-audit.md` for the exact
//! capability bound used by each handler.

use ankyra::prelude::klipper_command;

use crate::context::{EndstopHost, ShutdownRecovery, TrsyncHost};
use crate::shutdown::ShutdownReason;
use crate::state::trsync::{ENDSTOP_PIN_IDS, GPIO_MAX_PIN_VALUE};

/// Register an endstop object.
///
/// Allocates the next free endstop slot and maps the Klipper OID to it.
/// Configures the specified GPIO pin as input with optional pull-up
/// resistor for endstop switch reading. Virtual pins (values at or above
/// [`GPIO_MAX_PIN_VALUE`]) are accepted but skipped for GPIO
/// configuration.
#[klipper_command]
pub fn config_endstop(ctx: &mut dyn EndstopConfigHost, oid: u8, pin: u8, pull_up: u8) {
    let idx = match ctx.register_endstop(oid) {
        Ok(idx) => idx,
        Err(()) => {
            defmt::error!("config_endstop: oid={}", oid);
            if ctx.enter_shutdown() {
                let clock = ctx.now();
                ctx.send_shutdown(ShutdownReason::ConfigEndstopFailed, clock);
            }
            return;
        }
    };

    // Configure GPIO pin as input (only for real GPIO pins, not virtual).
    if pin < GPIO_MAX_PIN_VALUE {
        ctx.configure_input(pin, pull_up != 0);
        ENDSTOP_PIN_IDS[idx].store(pin, core::sync::atomic::Ordering::Relaxed);
        defmt::info!(
            "config_endstop: oid={} idx={} pin=P{}{}{}",
            oid,
            idx,
            ['A', 'B', 'C', 'D', 'E'][(pin / 16) as usize],
            pin % 16,
            if pull_up != 0 { " (pull-up)" } else { "" }
        );
    } else {
        defmt::info!(
            "config_endstop: oid={} idx={} pin={} (virtual, no GPIO)",
            oid,
            idx,
            pin
        );
    }
}

/// Arm an endstop for homing.
///
/// Configures sampling parameters and links to a TrSync for trigger
/// notification. Schedules the first endstop sample event via the
/// trigger scheduler (TIM5 CC2). A `sample_count` of zero acts as a
/// disarm request; no `trsync_oid` validation is performed in that case.
#[klipper_command]
#[allow(clippy::too_many_arguments)]
pub fn endstop_home(
    ctx: &mut dyn EndstopHomeHost,
    oid: u8,
    clock: u32,
    sample_ticks: u32,
    sample_count: u8,
    rest_ticks: u32,
    pin_value: u8,
    trsync_oid: u8,
    trigger_reason: u8,
) {
    let result = ctx.endstop_start_homing(
        oid,
        clock,
        sample_ticks,
        sample_count,
        rest_ticks,
        pin_value != 0,
        trsync_oid,
        trigger_reason,
    );
    if result.is_err() {
        defmt::error!("endstop_home: oid={}", oid);
        if ctx.enter_shutdown() {
            let shutdown_clock = ctx.now();
            ctx.send_shutdown(ShutdownReason::EndstopHomeFailed, shutdown_clock);
        }
    }
}

/// Query the current state of an endstop.
///
/// Returns homing status, next sample clock, and current pin value.
#[klipper_command]
pub fn endstop_query_state(ctx: &mut dyn EndstopConfigHost, oid: u8) {
    match ctx.endstop_query(oid) {
        Ok((homing, next_clock, pin_id)) => {
            let pin_state = if pin_id != 0xFF {
                ctx.read_pin(pin_id)
            } else {
                false
            };
            ::ankyra::klipper_reply!(
                crate::ankyra::replies::EndstopState,
                oid: u8 = oid,
                homing: u8 = u8::from(homing),
                next_clock: u32 = next_clock,
                pin_value: u8 = u8::from(pin_state)
            );
        }
        Err(()) => {
            defmt::error!("endstop_query_state: unknown oid={}", oid);
            if ctx.enter_shutdown() {
                let clock = ctx.now();
                ctx.send_shutdown(ShutdownReason::EndstopQueryStateFailed, clock);
            }
        }
    }
}

/// Composite bound for `config_endstop` and `endstop_query_state`:
/// both handlers need shutdown recovery (to enter shutdown on error),
/// a clock source (for the shutdown clock argument), the trigger-domain
/// bridge (to register/query the endstop slot), and GPIO access (to
/// configure or read the endstop pin).
///
/// `&mut dyn ShutdownRecovery + TrsyncHost + EndstopHost` is not a
/// valid trait-object type in stable Rust for multiple non-auto traits;
/// this named supertrait is the dyn-safe way to compose the capabilities.
pub trait EndstopConfigHost: ShutdownRecovery + TrsyncHost + EndstopHost {}
impl<T: ?Sized + ShutdownRecovery + TrsyncHost + EndstopHost> EndstopConfigHost for T {}

/// Composite bound for `endstop_home`: needs shutdown recovery (to
/// enter shutdown on error), a clock source (for the shutdown clock
/// argument), and the trigger-domain bridge (to configure the homing
/// runtime and schedule sample events).
///
/// `EndstopHost` is not required here — the pin ID is retrieved inside
/// the `endstop_start_homing` trait-method body from
/// `foci_firmware::state::trsync::ENDSTOP_PIN_IDS`.
pub trait EndstopHomeHost: ShutdownRecovery + TrsyncHost {}
impl<T: ?Sized + ShutdownRecovery + TrsyncHost> EndstopHomeHost for T {}
