//! Board-agnostic shutdown transition (pre-quiesce half).

use core::sync::atomic::Ordering::Relaxed;

use foci_klipper::state::State;

use crate::state::lifecycle::SHUTDOWN_ACTIVE;
use crate::state::tmc::TMC_DISABLE_SIGNAL;

/// Moves Klipper state to shutdown, then signals [`TMC_DISABLE_SIGNAL`] and sets
/// [`SHUTDOWN_ACTIVE`]. Returns `true` only when this call performed the transition; the board
/// then runs its own quiesce and post-quiesce atomics.
pub fn enter_shutdown(state: &mut State) -> bool {
    let entered = foci_klipper::commands::lifecycle::enter_shutdown(state);
    if entered {
        TMC_DISABLE_SIGNAL.signal(());
        SHUTDOWN_ACTIVE.store(true, Relaxed);
    }
    entered
}
