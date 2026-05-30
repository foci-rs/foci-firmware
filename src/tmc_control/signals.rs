//! Injected cross-priority signals used by the shared TMC control runtime.

use core::sync::atomic::AtomicBool;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;

/// Cross-priority signals consumed by `TmcControlRuntime`.
pub struct TmcControlSignals<'a> {
    /// Shutdown active flag.
    pub shutdown_active: &'a AtomicBool,
    /// Config-reset queue flush request.
    pub queue_flush: &'a AtomicBool,
    /// Latched TMC fault flag.
    pub fault_active: &'a AtomicBool,
    /// Emergency disable signal produced by interrupt/shutdown code and
    /// consumed directly by the runtime.
    pub disable_signal: &'a Signal<CriticalSectionRawMutex, ()>,
}

impl<'a> TmcControlSignals<'a> {
    /// Build an injected signal bundle.
    pub const fn new(
        shutdown_active: &'a AtomicBool,
        queue_flush: &'a AtomicBool,
        fault_active: &'a AtomicBool,
        disable_signal: &'a Signal<CriticalSectionRawMutex, ()>,
    ) -> Self {
        Self {
            shutdown_active,
            queue_flush,
            fault_active,
            disable_signal,
        }
    }
}
