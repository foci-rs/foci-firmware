//! Board-agnostic USB pump: bounded per-activation drain loop, budget and
//! yield constants, and best-effort diagnostic counters. Board crates supply
//! the transport, protocol dispatch, wake, and yield through `UsbPumpBackend`.

use core::sync::atomic::{AtomicU32, Ordering::Relaxed};

/// Max RX bytes per drain quantum before yielding; the actual bound is this plus one 128-byte
/// chunk.
pub const USB_RX_BYTE_BUDGET: usize = 256;

/// Exhaustion-path yield in microseconds on the 1 MHz `Mono`; non-zero so the delay genuinely
/// parks the P2 dispatcher.
pub const USB_YIELD_DELAY_US: u32 = 50;

/// Best-effort, lock-free per-drain-quantum counters; snapshot fields may come from slightly
/// different instants.
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
    /// Construct a zeroed counters instance.
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

    /// Point-in-time snapshot; see [`UsbPumpSnapshot`].
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

/// Throttle for the board defmt summary: emit when the interval elapsed or `max_activation_bytes`
/// grew; not keyed on `exhaustion_count`, which would emit once per activation.
pub fn should_emit(
    now_ticks: u32,
    last_emit_ticks: u32,
    snap: &UsbPumpSnapshot,
    last_emitted_max: u32,
) -> bool {
    let elapsed = now_ticks.wrapping_sub(last_emit_ticks);
    elapsed >= USB_EMIT_MIN_INTERVAL_US || snap.max_activation_bytes > last_emitted_max
}

/// Board-supplied hooks for the shared drain loop.
#[allow(async_fn_in_trait)]
pub trait UsbPumpBackend {
    async fn wait_wake(&mut self);
    fn poll(&mut self) -> bool;
    fn rx_data(&self) -> &[u8];
    fn rx_consume(&mut self, n: usize);
    /// Feed `chunk` to the protocol; returns bytes consumed (`0..=chunk.len()`, clamped by the loop).
    fn dispatch(&mut self, chunk: &[u8]) -> usize;
    fn report_pending(&self) -> bool;
    /// Re-signal the USB wake so the next `run_once` does not sleep.
    fn wake(&mut self);
    /// Non-zero suspension that cannot complete synchronously, so P2 parks and P1 runs.
    async fn yield_now(&mut self);
    fn final_poll(&mut self);
    /// Board hook at the head of a drain quantum (trace/configured detection).
    fn on_wake(&mut self);
    /// Board hook at the tail of a drain quantum (trace drains).
    fn on_drain_complete(&mut self);
}

/// One drain quantum: await wake, drain up to the byte budget, flush TX, record counters. On
/// budget exhaustion with RX remaining, re-signal and yield so P1 makes progress.
pub async fn run_once<B: UsbPumpBackend>(backend: &mut B, counters: &UsbPumpCounters) {
    backend.wait_wake().await;
    backend.on_wake();

    let mut budget = USB_RX_BYTE_BUDGET;
    let mut activation_bytes: u32 = 0;
    let mut exhausted = false;

    while backend.poll() {
        let len = backend.rx_data().len().min(128);
        if len == 0 {
            break;
        }
        let mut chunk = [0u8; 128];
        chunk[..len].copy_from_slice(&backend.rx_data()[..len]);

        let consumed = backend.dispatch(&chunk[..len]).min(len);
        backend.rx_consume(consumed);
        if consumed == 0 {
            break;
        }
        activation_bytes = activation_bytes.saturating_add(consumed as u32);
        if backend.report_pending() {
            break;
        }
        budget = budget.saturating_sub(consumed);
        if budget == 0 {
            if backend.poll() && !backend.rx_data().is_empty() {
                exhausted = true;
                backend.wake();
                backend.yield_now().await;
            }
            break;
        }
    }

    backend.final_poll();
    backend.on_drain_complete();
    if activation_bytes > 0 {
        counters.record(activation_bytes, exhausted);
    }
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
        assert!(!should_emit(500, 0, &snap(256, 5), 256));
        assert!(!should_emit(999_999, 0, &snap(256, 9999), 256));
    }

    #[test]
    fn should_emit_on_interval_or_new_max() {
        assert!(should_emit(1_000_000, 0, &snap(256, 1), 256));
        assert!(should_emit(10, 0, &snap(300, 1), 256));
    }
}

#[cfg(test)]
mod loop_tests {
    use super::*;
    use core::cell::Cell;
    use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};

    fn block_on<F: core::future::Future>(mut fut: F) -> F::Output {
        fn noop(_: *const ()) {}
        fn clone(_: *const ()) -> RawWaker {
            RawWaker::new(core::ptr::null(), &VT)
        }
        static VT: RawWakerVTable = RawWakerVTable::new(clone, noop, noop, noop);
        let waker = unsafe { Waker::from_raw(RawWaker::new(core::ptr::null(), &VT)) };
        let mut cx = Context::from_waker(&waker);
        let mut fut = unsafe { core::pin::Pin::new_unchecked(&mut fut) };
        loop {
            if let Poll::Ready(v) = fut.as_mut().poll(&mut cx) {
                return v;
            }
        }
    }

    /// Scripted backend. `rx` is the current buffer `rx_data` returns.
    /// `consume_plan` scripts how many bytes each `dispatch` reports consumed
    /// (front-popped, clamped to the slice length); when empty, `dispatch`
    /// consumes the whole chunk (a stream of complete commands). `over_consume`
    /// forces a return greater than the slice to exercise the clamp.
    struct FakeBackend {
        rx: std::vec::Vec<u8>,
        consume_plan: std::collections::VecDeque<usize>,
        report_after: Option<usize>,
        over_consume: bool,
        dispatches: usize,
        pub woke: Cell<bool>,
        pub yielded: Cell<bool>,
        pub yield_was_pending: Cell<bool>,
    }
    impl FakeBackend {
        fn new(rx: std::vec::Vec<u8>) -> Self {
            Self {
                rx,
                consume_plan: std::collections::VecDeque::new(),
                report_after: None,
                over_consume: false,
                dispatches: 0,
                woke: Cell::new(false),
                yielded: Cell::new(false),
                yield_was_pending: Cell::new(false),
            }
        }
        fn with_plan(rx: std::vec::Vec<u8>, plan: &[usize]) -> Self {
            let mut b = Self::new(rx);
            b.consume_plan = plan.iter().copied().collect();
            b
        }
    }
    impl UsbPumpBackend for FakeBackend {
        async fn wait_wake(&mut self) {}
        fn poll(&mut self) -> bool {
            !self.rx.is_empty()
        }
        fn rx_data(&self) -> &[u8] {
            &self.rx
        }
        fn rx_consume(&mut self, n: usize) {
            self.rx.drain(..n.min(self.rx.len()));
        }
        fn dispatch(&mut self, chunk: &[u8]) -> usize {
            self.dispatches += 1;
            if self.over_consume {
                return chunk.len() + 10;
            }
            match self.consume_plan.pop_front() {
                Some(n) => n.min(chunk.len()),
                None => chunk.len(),
            }
        }
        fn report_pending(&self) -> bool {
            self.report_after.is_some_and(|n| self.dispatches >= n)
        }
        fn wake(&mut self) {
            self.woke.set(true);
        }
        async fn yield_now(&mut self) {
            let mut polled = false;
            core::future::poll_fn(|_cx| {
                if !polled {
                    polled = true;
                    self.yield_was_pending.set(true);
                    core::task::Poll::Pending
                } else {
                    core::task::Poll::Ready(())
                }
            })
            .await;
            self.yielded.set(true);
        }
        fn final_poll(&mut self) {}
        fn on_wake(&mut self) {}
        fn on_drain_complete(&mut self) {}
    }

    #[test]
    fn sustained_stream_stops_at_budget_and_yields() {
        let c = UsbPumpCounters::new();
        let mut b = FakeBackend::new(std::vec![0u8; 1280]);
        block_on(run_once(&mut b, &c));
        let s = c.snapshot();
        assert_eq!(s.exhaustion_count, 1);
        assert!(s.last_activation_bytes >= USB_RX_BYTE_BUDGET as u32);
        assert!(s.last_activation_bytes <= (USB_RX_BYTE_BUDGET + 127) as u32);
        assert!(b.woke.get() && b.yielded.get() && b.yield_was_pending.get());
    }

    #[test]
    fn backlog_drains_across_quanta_without_loss() {
        let c = UsbPumpCounters::new();
        let mut b = FakeBackend::new(std::vec![0u8; 1280]);
        let mut quanta = 0;
        while b.poll() {
            block_on(run_once(&mut b, &c));
            quanta += 1;
            if quanta > 100 {
                break;
            }
        }
        assert!(b.rx.is_empty());
    }

    #[test]
    fn exact_budget_drain_with_empty_rx_is_not_exhaustion() {
        let c = UsbPumpCounters::new();
        let mut b = FakeBackend::new(std::vec![0u8; 256]);
        block_on(run_once(&mut b, &c));
        let s = c.snapshot();
        assert_eq!(s.exhaustion_count, 0);
        assert!(!b.woke.get() && !b.yielded.get());
    }

    #[test]
    fn over_consumption_is_clamped_to_slice() {
        let c = UsbPumpCounters::new();
        let mut b = FakeBackend::new(std::vec![0u8; 128]);
        b.over_consume = true;
        block_on(run_once(&mut b, &c));
        assert!(b.rx.is_empty());
        assert!(c.snapshot().last_activation_bytes <= 128);
    }

    #[test]
    fn report_pending_breaks_before_budget() {
        let c = UsbPumpCounters::new();
        let mut b = FakeBackend::new(std::vec![0u8; 1280]);
        b.report_after = Some(1);
        block_on(run_once(&mut b, &c));
        assert_eq!(c.snapshot().exhaustion_count, 0);
        assert!(!b.woke.get());
    }

    #[test]
    fn positive_partial_consumption_continues_then_makes_progress() {
        let c = UsbPumpCounters::new();
        let mut b = FakeBackend::with_plan(std::vec![0u8; 200], &[40]);
        block_on(run_once(&mut b, &c));
        assert!(b.rx.is_empty());
        assert!(b.dispatches >= 2);
        assert_eq!(c.snapshot().exhaustion_count, 0);
    }

    #[test]
    fn zero_consumption_retains_all_bytes_without_wake() {
        let c = UsbPumpCounters::new();
        let mut b = FakeBackend::with_plan(std::vec![0u8; 200], &[0]);
        block_on(run_once(&mut b, &c));
        assert_eq!(b.rx.len(), 200);
        assert!(!b.woke.get() && !b.yielded.get());
        assert_eq!(c.snapshot().activation_count, 0);
    }

    #[test]
    fn empty_rx_quantum_is_not_counted() {
        let c = UsbPumpCounters::new();
        struct EmptyOnce {
            polled: bool,
        }
        impl UsbPumpBackend for EmptyOnce {
            async fn wait_wake(&mut self) {}
            fn poll(&mut self) -> bool {
                let p = !self.polled;
                self.polled = true;
                p
            }
            fn rx_data(&self) -> &[u8] {
                &[]
            }
            fn rx_consume(&mut self, _n: usize) {}
            fn dispatch(&mut self, _c: &[u8]) -> usize {
                0
            }
            fn report_pending(&self) -> bool {
                false
            }
            fn wake(&mut self) {}
            async fn yield_now(&mut self) {}
            fn final_poll(&mut self) {}
            fn on_wake(&mut self) {}
            fn on_drain_complete(&mut self) {}
        }
        block_on(run_once(&mut EmptyOnce { polled: false }, &c));
        assert_eq!(c.snapshot().activation_count, 0);
    }
}
