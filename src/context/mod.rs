//! Per-capability context traits that board crates implement on `FirmwareCtx`.
//! All traits must be dyn-safe.

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
    fn send_shutdown(&mut self, reason: crate::shutdown::ShutdownReason, clock: u32);
}

/// Endstop GPIO configuration and sampling.
pub trait EndstopHost: StateAccess {
    fn configure_input(&mut self, pin: u8, pull_up: bool);
    fn read_pin(&self, pin: u8) -> bool;
    /// Map a board virtual STALL pin id to its TMC channel, `None` for any other pin.
    fn stall_channel_for_pin(&self, pin: u8) -> Option<u8>;
}

/// Board-specific analog-in pin resolution.
pub trait AnalogInHost {
    fn resolve_analog_pin(&self, pin: u8) -> Option<u8>;
}

/// Composite bound for the `config_reset` handler.
pub trait LifecycleReset: ShutdownRecovery + TrsyncHost {}
impl<T: ?Sized + ShutdownRecovery + TrsyncHost> LifecycleReset for T {}
