//! Lossless pending-flag drain handshake.
//!
//! A producer (ISR or P1 code) sets a shared `AtomicBool` and spawns a
//! single-capacity P2 task to drain whatever it produced. If a second
//! producer races in after the task's drain pass has already scanned for
//! work but before the task actually returns, a naive "drain once, clear
//! the flag, return" body can strand that second arrival: the task's
//! `spawn()` call finds the capacity slot still occupied, and the flag it
//! set gets clobbered by the running task's own unconditional clear.

use core::sync::atomic::{AtomicBool, Ordering::Relaxed};

/// Run `drain_once` until a pass completes without `pending` having been
/// re-armed, so work that arrives during a pass is never stranded.
///
/// Clears `pending` before each pass, then runs `drain_once`. If a producer
/// sets `pending` again before the pass returns, that signals new work may
/// have arrived that this pass already missed scanning for, so another pass
/// runs immediately rather than returning with nothing left to notice the
/// flag. Terminates once a pass completes with `pending` still `false`.
pub fn drain_until_quiescent(pending: &AtomicBool, mut drain_once: impl FnMut()) {
    loop {
        pending.store(false, Relaxed);
        drain_once();
        if !pending.load(Relaxed) {
            break;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn runs_exactly_once_when_nothing_races() {
        let pending = AtomicBool::new(true);
        let mut passes = 0;

        drain_until_quiescent(&pending, || passes += 1);

        assert_eq!(passes, 1);
        assert!(!pending.load(Relaxed));
    }

    #[test]
    fn reruns_when_pending_is_reraised_during_the_pass() {
        let pending = AtomicBool::new(true);
        let mut passes = 0;

        drain_until_quiescent(&pending, || {
            passes += 1;
            if passes == 1 {
                // Simulate a producer racing in while this pass is running:
                // it wrote new work and re-armed pending before we return.
                pending.store(true, Relaxed);
            }
        });

        assert_eq!(
            passes, 2,
            "a re-armed flag during the first pass must force a second pass"
        );
        assert!(!pending.load(Relaxed));
    }

    #[test]
    fn always_drains_at_least_once_even_if_pending_was_already_false() {
        // A task is only ever spawned in response to a producer setting
        // `pending`, so this should not happen in practice -- but the first
        // pass must still run so a stray pre-existing item is never missed.
        let pending = AtomicBool::new(false);
        let mut passes = 0;

        drain_until_quiescent(&pending, || passes += 1);

        assert_eq!(passes, 1);
    }
}
