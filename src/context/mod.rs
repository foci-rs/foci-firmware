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
//! Trait surface is driven by the Stage-4 handler audit
//! (`docs/notes/foci-firmware-lift-handler-audit.md`). Each trait
//! method exists because at least one handler body requires it —
//! methods from the initial scaffold that no handler used have been
//! removed.

pub mod trsync;

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

// Blanket `&mut T` impl. The assembler's `Config::dispatch` passes
// `ctx: &mut Self::Context<'c>` into per-handler dispatch wrappers that
// expect `&mut dyn Trait`. When `Context<'c> = &'c mut FirmwareCtx<'c>`
// (the shape the board binaries use), the trait-object coercion is
// `&mut &'c mut FirmwareCtx: &mut dyn Trait`, which requires
// `&mut FirmwareCtx: Trait`. This blanket supplies it by forwarding to
// the underlying `T: Trait` impl. The same rationale applies to every
// capability trait below.
impl<T: ?Sized + StateAccess> StateAccess for &mut T {
    fn state(&mut self) -> &mut State {
        (**self).state()
    }
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

impl<T: ?Sized + BoardChannels> BoardChannels for &mut T {
    fn tmc_channel_count(&self) -> u8 {
        (**self).tmc_channel_count()
    }

    fn step_pin_channel(&self, pin: u8) -> Option<u8> {
        (**self).step_pin_channel(pin)
    }

    fn dir_pin_channel(&self, pin: u8) -> Option<u8> {
        (**self).dir_pin_channel(pin)
    }

    fn enable_pin_channel(&self, pin: u32) -> Option<u8> {
        (**self).enable_pin_channel(pin)
    }
}

/// Klipper MCU clock source (typically a CPU timer counter).
///
/// Separated from `ShutdownRecovery` so the pure clock-reporting
/// handlers (`get_clock`, `get_uptime`) bind to the minimum surface.
pub trait ClockSource {
    fn now(&self) -> u32;
}

impl<T: ?Sized + ClockSource> ClockSource for &mut T {
    fn now(&self) -> u32 {
        (**self).now()
    }
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

impl<T: ?Sized + ShutdownRecovery> ShutdownRecovery for &mut T {
    fn enter_shutdown(&mut self) -> bool {
        (**self).enter_shutdown()
    }
    fn reset_to_bootloader(&mut self) -> ! {
        (**self).reset_to_bootloader()
    }
    fn send_shutdown(&mut self, reason: crate::shutdown::ShutdownReason, clock: u32) {
        (**self).send_shutdown(reason, clock)
    }
}

/// Stepper subsystem: direction/queue/clock operations exposed at the
/// granularity the handler bodies actually need.
///
/// The step-timer / stop-check helpers exercised only by the RTIC
/// task graph are intentionally absent — they are not reachable via
/// `&mut dyn StepperHost`.
pub trait StepperHost: StateAccess {
    /// Capture the direction bit for the next enqueued move on `channel`.
    fn set_next_step_dir(&mut self, channel: u8, forward: bool);

    /// Enqueue a trapezoidal move onto the hardware step queue for `channel`.
    fn queue_step_move(
        &mut self,
        channel: u8,
        move_: foci_klipper::stepper::StepMove,
    ) -> Result<(), foci_klipper::commands::CommandError>;

    /// Reset the step-clock reference for `channel`.
    fn reset_step_clock(
        &mut self,
        channel: u8,
        clock: u32,
    ) -> Result<(), foci_klipper::commands::CommandError>;
}

impl<T: ?Sized + StepperHost> StepperHost for &mut T {
    fn set_next_step_dir(&mut self, channel: u8, forward: bool) {
        (**self).set_next_step_dir(channel, forward)
    }

    fn queue_step_move(
        &mut self,
        channel: u8,
        move_: foci_klipper::stepper::StepMove,
    ) -> Result<(), foci_klipper::commands::CommandError> {
        (**self).queue_step_move(channel, move_)
    }

    fn reset_step_clock(
        &mut self,
        channel: u8,
        clock: u32,
    ) -> Result<(), foci_klipper::commands::CommandError> {
        (**self).reset_step_clock(channel, clock)
    }
}

/// Endstop GPIO configuration and sampling. Pure hardware access;
/// runtime state lives inside the trigger domain and is reached via
/// `TrsyncHost`.
pub trait EndstopHost: StateAccess {
    fn configure_input(&mut self, pin: u8, pull_up: bool);
    fn read_pin(&self, pin: u8) -> bool;
}

impl<T: ?Sized + EndstopHost> EndstopHost for &mut T {
    fn configure_input(&mut self, pin: u8, pull_up: bool) {
        (**self).configure_input(pin, pull_up)
    }
    fn read_pin(&self, pin: u8) -> bool {
        (**self).read_pin(pin)
    }
}

/// TMC4671 request bus. Send-only; the `tmc_control` task on the
/// board side consumes from the channel and performs the actual
/// register writes.
pub trait TmcHost: StateAccess {
    fn enqueue_request(&self, req: crate::tmc::TmcRequest) -> Result<(), QueueFull>;
}

impl<T: ?Sized + TmcHost> TmcHost for &mut T {
    fn enqueue_request(&self, req: crate::tmc::TmcRequest) -> Result<(), QueueFull> {
        (**self).enqueue_request(req)
    }
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
