//! Board-agnostic USB pump: bounded per-activation drain loop, budget and
//! yield constants, and best-effort diagnostic counters. Board crates supply
//! the transport, protocol dispatch, wake, and yield through `UsbPumpBackend`.

use core::sync::atomic::{AtomicU32, Ordering::Relaxed};

/// Max RX bytes one drain quantum processes before yielding to lower-priority
/// work. HW-tunable; the actual per-activation bound is this plus one 128-byte
/// chunk (the budget is checked after a chunk is dispatched).
pub const USB_RX_BYTE_BUDGET: usize = 256;

/// Exhaustion-path yield duration, microseconds, on the 1 MHz `Mono`. Non-zero
/// so the delay future cannot complete synchronously and genuinely parks the
/// P2 dispatcher.
pub const USB_YIELD_DELAY_US: u32 = 50;

/// Best-effort, lock-free per-drain-quantum counters. Snapshot callers may see
/// fields from slightly different instants; this matches the stepper counters.
pub struct UsbPumpCounters {
    activation_count: AtomicU32,
    exhaustion_count: AtomicU32,
    last_activation_bytes: AtomicU32,
    max_activation_bytes: AtomicU32,
}

/// Point-in-time view of [`UsbPumpCounters`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct UsbPumpSnapshot {
    pub activation_count: u32,
    pub exhaustion_count: u32,
    pub last_activation_bytes: u32,
    pub max_activation_bytes: u32,
}

impl UsbPumpCounters {
    pub const fn new() -> Self {
        Self {
            activation_count: AtomicU32::new(0),
            exhaustion_count: AtomicU32::new(0),
            last_activation_bytes: AtomicU32::new(0),
            max_activation_bytes: AtomicU32::new(0),
        }
    }

    /// Record one drain quantum that consumed `activation_bytes` bytes.
    pub fn record(&self, activation_bytes: u32, exhausted: bool) {
        saturating_inc(&self.activation_count);
        if exhausted {
            saturating_inc(&self.exhaustion_count);
        }
        self.last_activation_bytes.store(activation_bytes, Relaxed);
        self.max_activation_bytes
            .fetch_max(activation_bytes, Relaxed);
    }

    pub fn snapshot(&self) -> UsbPumpSnapshot {
        UsbPumpSnapshot {
            activation_count: self.activation_count.load(Relaxed),
            exhaustion_count: self.exhaustion_count.load(Relaxed),
            last_activation_bytes: self.last_activation_bytes.load(Relaxed),
            max_activation_bytes: self.max_activation_bytes.load(Relaxed),
        }
    }

    #[cfg(test)]
    fn force_counts_for_test(&self, activations: u32, exhaustions: u32) {
        self.activation_count.store(activations, Relaxed);
        self.exhaustion_count.store(exhaustions, Relaxed);
    }
}

impl Default for UsbPumpCounters {
    fn default() -> Self {
        Self::new()
    }
}

fn saturating_inc(counter: &AtomicU32) {
    let _ = counter.fetch_update(Relaxed, Relaxed, |v| Some(v.saturating_add(1)));
}

/// Minimum defmt-emit interval, in 1 MHz `Mono` ticks (~1 s).
pub const USB_EMIT_MIN_INTERVAL_US: u32 = 1_000_000;

/// Pure throttle policy for the board-side defmt summary. Emit only when the
/// interval has elapsed or `max_activation_bytes` grew (a bounded, self-limiting
/// event). NOT keyed on `exhaustion_count`, which rises every exhaustion and
/// would emit once per activation under a sustained stream.
pub fn should_emit(
    now_ticks: u32,
    last_emit_ticks: u32,
    snap: &UsbPumpSnapshot,
    last_emitted_max: u32,
) -> bool {
    let elapsed = now_ticks.wrapping_sub(last_emit_ticks);
    elapsed >= USB_EMIT_MIN_INTERVAL_US || snap.max_activation_bytes > last_emitted_max
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn record_tracks_last_max_and_counts() {
        let c = UsbPumpCounters::new();
        c.record(100, false);
        c.record(300, true);
        c.record(50, false);
        let s = c.snapshot();
        assert_eq!(s.activation_count, 3);
        assert_eq!(s.exhaustion_count, 1);
        assert_eq!(s.last_activation_bytes, 50);
        assert_eq!(s.max_activation_bytes, 300);
    }

    #[test]
    fn counts_saturate_instead_of_wrapping() {
        let c = UsbPumpCounters::new();
        c.force_counts_for_test(u32::MAX, u32::MAX);
        c.record(10, true);
        let s = c.snapshot();
        assert_eq!(s.activation_count, u32::MAX);
        assert_eq!(s.exhaustion_count, u32::MAX);
    }

    fn snap(max: u32, exh: u32) -> UsbPumpSnapshot {
        UsbPumpSnapshot {
            activation_count: 0,
            exhaustion_count: exh,
            last_activation_bytes: 0,
            max_activation_bytes: max,
        }
    }

    #[test]
    fn should_emit_throttles_repeated_exhaustions() {
        // Same max, only exhaustion_count rising, inside the interval => no emit.
        assert!(!should_emit(500, 0, &snap(256, 5), 256));
        assert!(!should_emit(999_999, 0, &snap(256, 9999), 256));
    }

    #[test]
    fn should_emit_on_interval_or_new_max() {
        assert!(should_emit(1_000_000, 0, &snap(256, 1), 256)); // interval elapsed
        assert!(should_emit(10, 0, &snap(300, 1), 256)); // max grew
    }
}
