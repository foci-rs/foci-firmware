//! Trsync command handlers: `config_trsync`, `trsync_start`,
//! `trsync_set_timeout`, `trsync_trigger`.
//!
//! Lifted out of `boards/openffboard-fw/src/ankyra/handlers/trsync.rs`
//! in Task 13 of the foci-firmware handler-lift plan. Handler bodies now
//! bind to the capability traits declared in [`crate::context`] instead
//! of concrete board types; the board crate wires up the trait impls on
//! its own `FirmwareCtx`.
//!
//! See the handler audit at
//! `docs/notes/foci-firmware-lift-handler-audit.md` (lines 281–319) for
//! the exact capability bound used by each handler.

use ankyra::prelude::klipper_command;

use crate::context::{ShutdownRecovery, TrsyncHost};
use crate::shutdown::ShutdownReason;
use crate::state::trsync::{HOMING_STATE, HOMING_TRIGGERED};

/// Register a TrSync (trigger synchronization) object.
///
/// Allocates the next free TrSync slot and maps the Klipper OID to it.
#[klipper_command]
pub fn config_trsync(ctx: &mut dyn TrsyncLifecycleHost, oid: u8) {
    if ctx.register_trsync(oid).is_err() {
        defmt::error!("config_trsync: oid={}", oid);
        if ctx.enter_shutdown() {
            let clock = ctx.now();
            ctx.send_shutdown(ShutdownReason::ConfigTrsyncFailed, clock);
        }
        return;
    }
    defmt::info!("config_trsync: oid={}", oid);
}

/// Arm a TrSync for synchronized triggering.
///
/// Configures the reporting interval and timeout reason, then transitions
/// the TrSync to the Armed state. Schedules the first report timer event.
#[klipper_command]
pub fn trsync_start(
    ctx: &mut dyn TrsyncLifecycleHost,
    oid: u8,
    report_clock: u32,
    report_ticks: u32,
    expire_reason: u8,
) {
    if ctx
        .trsync_arm(oid, report_clock, report_ticks, expire_reason)
        .is_err()
    {
        defmt::error!("trsync_start: oid={}", oid);
        if ctx.enter_shutdown() {
            let clock = ctx.now();
            ctx.send_shutdown(ShutdownReason::TrsyncStartFailed, clock);
        }
    }
}

/// Update the expiry deadline for a TrSync.
///
/// Schedules the expire timer event. The TrSync will auto-trigger with
/// the configured expire_reason if the clock reaches this value before
/// a manual trigger.
#[klipper_command]
pub fn trsync_set_timeout(ctx: &mut dyn TrsyncLifecycleHost, oid: u8, clock: u32) {
    if ctx.trsync_set_timeout(oid, clock).is_err() {
        defmt::error!("trsync_set_timeout: oid={}", oid);
        if ctx.enter_shutdown() {
            let shutdown_clock = ctx.now();
            ctx.send_shutdown(ShutdownReason::TrsyncSetTimeoutFailed, shutdown_clock);
        }
    }
}

/// Manually trigger a TrSync and clean up.
///
/// Mirrors Klipper C's `command_trsync_trigger`: attempts to fire the
/// trigger (no-op if already triggered), then unconditionally stops
/// report/expire timers, clears blocked_by_trigger on steppers, and
/// sends one final `trsync_state` reply with `can_trigger=0`.
#[klipper_command]
pub fn trsync_trigger(ctx: &mut dyn TrsyncLifecycleHost, oid: u8, reason: u8) {
    match ctx.trsync_trigger(oid, reason) {
        Ok(trigger_reason) => {
            // Clear homing trigger sentinel and advance suppression
            // state machine. If P1 is in state 2 (Suppressing), the
            // host trsync_trigger cleanup must advance it to state 3
            // (RestorePending) so I-suppression is released even if
            // P1 hasn't polled HOMING_TRIGGERED yet.
            use core::sync::atomic::Ordering::Relaxed;
            HOMING_TRIGGERED.store(0xFF, Relaxed);
            let _ = HOMING_STATE.compare_exchange(2, 3, Relaxed, Relaxed);
            // Send final trsync_state reply (can_trigger=0, clock=0)
            ::ankyra::klipper_reply!(
                crate::ankyra::replies::TrsyncState,
                oid: u8 = oid,
                can_trigger: u8 = 0,
                trigger_reason: u8 = trigger_reason,
                clock: u32 = 0
            );
        }
        Err(()) => {
            defmt::error!("trsync_trigger: unknown oid={}", oid);
            if ctx.enter_shutdown() {
                let clock = ctx.now();
                ctx.send_shutdown(ShutdownReason::TrsyncTriggerFailed, clock);
            }
        }
    }
}

/// Composite bound for all four trsync handlers: needs shutdown recovery
/// (to enter shutdown on error, which also implies `ClockSource` via its
/// supertrait) and the trigger-domain bridge (to register, arm, update,
/// and fire the trsync runtime).
///
/// `&mut dyn ShutdownRecovery + TrsyncHost` is not a valid trait-object
/// type in stable Rust for two non-auto traits; this named supertrait is
/// the dyn-safe way to compose the capabilities.
pub trait TrsyncLifecycleHost: ShutdownRecovery + TrsyncHost {}
impl<T: ?Sized + ShutdownRecovery + TrsyncHost> TrsyncLifecycleHost for T {}
