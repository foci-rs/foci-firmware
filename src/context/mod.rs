//! Per-capability context traits that board crates implement on their
//! concrete `FirmwareCtx<'a>` struct. Handlers in `ankyra::handlers`
//! take `ctx: &mut dyn <TraitName>` — the narrowest trait covering
//! the operations that handler performs.
//!
//! All traits must be dyn-safe: no generic methods, no closure
//! parameters, no associated items. If a caller needs composed
//! capabilities (`StateAccess + TmcHost`), use a supertrait bound
//! (`trait X: StateAccess + TmcHost {}` with blanket impl).
//!
//! Each trait method exists because at least one handler body requires
//! it.

pub mod stepper;
pub mod stepper_reconcile;
pub mod tmc;
pub mod trsync;

pub use stepper::{StepperEnqueueError, StepperHost};
pub use tmc::TmcHost;
pub use trsync::TrsyncHost;

use foci_klipper::state::State;

/// Signal returned when an enqueue operation finds the channel full.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct QueueFull;

/// Access the shared `foci_klipper::state::State` instance. Every
/// handler that mutates Klipper protocol state goes through here.
pub trait StateAccess {
    fn state(&mut self) -> &mut State;
}

/// Board-local motor channel and virtual pin mapping.
pub trait BoardChannels {
    /// Number of TMC4671 motor channels supported by this board.
    fn tmc_channel_count(&self) -> u8;

    /// Return the motor channel represented by a virtual STEP pin.
    fn step_pin_channel(&self, pin: u8) -> Option<u8>;

    /// Return the motor channel represented by a virtual DIR pin.
    fn dir_pin_channel(&self, pin: u8) -> Option<u8>;

    /// Return the motor channel represented by a virtual enable pin.
    fn enable_pin_channel(&self, pin: u32) -> Option<u8>;
}

/// Klipper MCU clock source (typically a CPU timer counter).
///
/// Separated from `ShutdownRecovery` so the pure clock-reporting
/// handlers (`get_clock`, `get_uptime`) bind to the minimum surface.
pub trait ClockSource {
    fn now(&self) -> u32;
}

/// MCU shutdown + recovery operations.
pub trait ShutdownRecovery: StateAccess + ClockSource {
    /// Transition to shutdown state and quiesce dependent subsystems.
    /// Returns `true` if the transition actually happened (first call),
    /// `false` if the MCU was already in shutdown.
    fn enter_shutdown(&mut self) -> bool;

    /// Write the Katapult "bypass" signature and perform a system
    /// reset. Matches Klipper's `command_reset` semantics.
    fn reset_to_bootloader(&mut self) -> !;

    /// Emit a `klipper_shutdown` message on the firmware's transport.
    ///
    /// Cross-crate bridge: lifted handlers (living in `foci-firmware`)
    /// cannot invoke `::ankyra::klipper_shutdown!` directly because the
    /// macro expands to a `crate::_ankyra_config::…` path that only
    /// resolves in the board binary. The board-side impl dispatches
    /// through [`crate::dispatch_shutdown!`] to the matching
    /// `klipper_shutdown_from!` call.
    fn send_shutdown(&mut self, reason: crate::shutdown::ShutdownReason, clock: u32);
}

/// Endstop GPIO configuration and sampling. Pure hardware access;
/// runtime state lives inside the trigger domain and is reached via
/// `TrsyncHost`.
pub trait EndstopHost: StateAccess {
    fn configure_input(&mut self, pin: u8, pull_up: bool);
    fn read_pin(&self, pin: u8) -> bool;
    /// Map a board virtual STALL pin id to its TMC channel, `None` for any other pin.
    fn stall_channel_for_pin(&self, pin: u8) -> Option<u8>;
}

/// Board-specific analog-in pin resolution. Stateless by design -- the
/// implementing board's context type just forwards to its own
/// `board_config::analog_pin_channel`. All *stateful* sampling data lives
/// in `analog_in::ANALOG_CHANNELS`, a process-wide static reachable
/// without going through this trait at all (each board's heartbeat, which
/// samples the channels and sends `analog_in_state`, never implements or
/// needs this trait).
pub trait AnalogInHost {
    fn resolve_analog_pin(&self, pin: u8) -> Option<u8>;
}

/// Composite bound for the `config_reset` handler: needs shutdown
/// recovery (to enter shutdown on error), clock reporting (for the
/// `klipper_shutdown!` clock arg), and full trigger-domain reset.
///
/// `&mut dyn Trait1 + Trait2` is not a valid trait-object type in
/// stable Rust for two non-auto traits; this named supertrait is the
/// dyn-safe way to compose the capabilities the handler needs.
pub trait LifecycleReset: ShutdownRecovery + TrsyncHost {}
impl<T: ?Sized + ShutdownRecovery + TrsyncHost> LifecycleReset for T {}
