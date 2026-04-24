//! Lifecycle helpers: board-agnostic shutdown transition.
//!
//! The Klipper-level shutdown state change and the pre-quiesce atomics
//! that must fire before the trigger-domain is quiesced live here.
//! The trigger-domain quiesce (scheduler clear, stepper stop, TIM5
//! disable) must happen on the board — it touches board-local types
//! and memory-mapped peripherals. [`ShutdownRecovery::enter_shutdown`]
//! on a board's `FirmwareCtx` runs this helper first, then its own
//! board-local quiesce when this returns `true`, then the post-quiesce
//! atomics ([`REPORT_PENDING`], [`HOMING_TRIGGERED`], [`HOMING_STATE`]).
//!
//! Atomics set here (pre-quiesce): [`TMC_DISABLE_SIGNAL`] and
//! [`SHUTDOWN_ACTIVE`]. Post-quiesce atomics are the board impl's
//! responsibility to preserve the pre-lift ordering exactly.

use core::sync::atomic::Ordering::Relaxed;

use foci_klipper::state::State;

use crate::state::lifecycle::SHUTDOWN_ACTIVE;
use crate::state::tmc::TMC_DISABLE_SIGNAL;

/// Transition Klipper state to shutdown and set board-independent
/// pre-quiesce shutdown flags. Returns `true` when this call performed
/// the transition — callers should follow up with their board-local
/// quiesce and post-quiesce atomics on a `true` return and skip them
/// on `false` (already in shutdown).
///
/// Order of operations enforced here:
/// 1. `foci_klipper::commands::lifecycle::enter_shutdown` — marks
///    Klipper state as shutdown.
/// 2. [`TMC_DISABLE_SIGNAL`] — wakes `tmc_control` to perform one-shot
///    motor safe-state.
/// 3. [`SHUTDOWN_ACTIVE`] — tells `tmc_control` tick to inhibit motion.
///
/// The board impl then quiesces the trigger domain and clears
/// [`REPORT_PENDING`], [`HOMING_TRIGGERED`], [`HOMING_STATE`] in that
/// order to match the pre-lift behaviour.
pub fn enter_shutdown(state: &mut State) -> bool {
    let entered = foci_klipper::commands::lifecycle::enter_shutdown(state);
    if entered {
        TMC_DISABLE_SIGNAL.signal(());
        SHUTDOWN_ACTIVE.store(true, Relaxed);
    }
    entered
}
