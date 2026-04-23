//! Per-capability context traits that board crates implement on their
//! concrete `FirmwareCtx<'a>` struct. Handlers in `ankyra::handlers`
//! take `ctx: &mut dyn <TraitName>` — the narrowest trait covering
//! the operations that handler performs.
//!
//! All traits must be dyn-safe: no generic methods, no closure
//! parameters, no associated items. If a caller needs composed
//! capabilities (`StateAccess + TmcHost`), use a supertrait bound
//! (`trait X: StateAccess + TmcHost {}` with blanket impl).

use foci_klipper::state::State;

/// Signal returned when an enqueue operation finds the channel full.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct QueueFull;

/// Access the shared `foci_klipper::state::State` instance. Every
/// handler that mutates Klipper protocol state goes through here.
pub trait StateAccess {
    fn state(&mut self) -> &mut State;
}

/// Klipper MCU clock source (typically a CPU timer counter).
pub trait ClockSource {
    fn now(&self) -> u32;
}

/// MCU shutdown + recovery operations.
pub trait ShutdownRecovery: StateAccess {
    /// Reboot into the Katapult bootloader. Writes a board-specific
    /// signature to RAM then triggers `NVIC_SystemReset`.
    fn reset_to_bootloader(&mut self) -> !;

    /// Transition to shutdown state; returns `true` if the transition
    /// was performed (vs. already in shutdown).
    fn enter_shutdown(&mut self) -> bool;
}

/// Stepper subsystem: timer arming, stop-check scheduling, step-queue
/// ops. The three stepper handle statics (`STEP_QUEUE`, `P2_STEPPER`,
/// `ISR_STEPPER`) stay in the board crate; access is mediated by the
/// methods below. Concrete method surface will be tuned by the Stage-4
/// handler audit; the initial signatures below match the pre-lift
/// call patterns documented in `docs/notes/foci-firmware-lift-handler-audit.md`.
pub trait StepperHost: StateAccess {
    fn arm_step_timer(&mut self, clock: u32) -> bool;
    fn schedule_stop_check(&mut self);
    fn queue_has_room(&self) -> bool;
    fn step_position(&self) -> i32;
}

/// Endstop subsystem: GPIO pin configuration and reads.
pub trait EndstopHost: StateAccess {
    fn configure_input(&mut self, pin: u8, pull_up: bool);
    fn read_pin(&self, pin: u8) -> bool;
}

/// Trsync + endstop-trigger subsystem. These replace the closure-based
/// `with_trigger_domain(|td| ...)` call sites from the pre-lift code;
/// each method corresponds to a specific operation the callers
/// performed through the closure. `TriggerDomain` stays in the board
/// crate; this trait is the dyn-safe bridge.
pub trait TrsyncHost: StateAccess {
    fn set_step_timer_active(&mut self, active: bool);
    fn record_trigger(&mut self, reason: u8);
    fn clear_triggers(&mut self);
    fn stop_physical_stepper(&mut self);
}

/// TMC4671 request bus. Send-only; the `tmc_control` task on the
/// board side consumes from the channel and performs the actual
/// register writes.
pub trait TmcHost: StateAccess {
    fn enqueue_request(&self, req: crate::tmc::TmcRequest) -> Result<(), QueueFull>;
}

/// Trace ring buffer accessor. Implementation picks the backing
/// storage (CCMRAM on OpenFFBoard, possibly different memory region
/// on other boards).
pub trait TraceHost: StateAccess {
    fn trace_buffer(&self) -> &'static crate::trace::TraceBuffer;
}

/// FOCI autotune / commissioning callbacks. Populated during Stage 4
/// if the handler audit reveals a genuine need beyond TmcHost /
/// StepperHost.
pub trait FociHost: StateAccess {}
