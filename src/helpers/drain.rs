//! Lossless pending-flag drain handshake.

use core::sync::atomic::{AtomicBool, Ordering::Relaxed};

/// Clears `pending`, runs `drain_once`, and repeats until a pass finishes without `pending`
/// having been re-armed, so work arriving mid-pass is never stranded.
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
        let pending = AtomicBool::new(false);
        let mut passes = 0;

        drain_until_quiescent(&pending, || passes += 1);

        assert_eq!(passes, 1);
    }
}
