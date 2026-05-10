//! Stepper command handlers: `config_stepper`, `queue_step`,
//! `set_next_step_dir`, `reset_step_clock`, `stepper_get_position`,
//! `stepper_get_commanded_position`, `stepper_stop_on_trigger`.
//!
//! Lifted out of `boards/openffboard-fw/src/ankyra/handlers/stepper.rs`
//! in Task 15 of the foci-firmware handler-lift plan. Handler bodies now
//! bind to the capability traits declared in [`crate::context`] instead
//! of concrete board types; the board crate wires up the trait impls on
//! its own `FirmwareCtx`.
//!
//! See the handler audit at
//! `docs/notes/foci-firmware-lift-handler-audit.md` for the exact
//! capability bound used by each handler.

use ankyra::prelude::klipper_command;

use crate::context::{BoardChannels, ShutdownRecovery, StateAccess, StepperHost, TrsyncHost};
use crate::shutdown::ShutdownReason;
use crate::state::stepper::stepper_channel;
use crate::state::trsync::HOMING_STATE;

/// Register a stepper motor object.
///
/// STEP/DIR pins must be FOCI virtual pins that map to the same board-local
/// TMC4671 channel. The TMC4671 handles stepping internally after config.
#[klipper_command]
pub fn config_stepper(
    ctx: &mut dyn StepperConfigHost,
    oid: u8,
    step_pin: u8,
    dir_pin: u8,
    _invert_step: u8,
    _step_pulse_ticks: u32,
) {
    let step_channel = match ctx.step_pin_channel(step_pin) {
        Some(channel) => channel,
        None => {
            defmt::error!("config_stepper: unknown STEP pin {}", step_pin);
            if ctx.enter_shutdown() {
                let clock = ctx.now();
                ctx.send_shutdown(ShutdownReason::ConfigStepperFailed, clock);
            }
            return;
        }
    };
    let dir_channel = match ctx.dir_pin_channel(dir_pin) {
        Some(channel) => channel,
        None => {
            defmt::error!("config_stepper: unknown DIR pin {}", dir_pin);
            if ctx.enter_shutdown() {
                let clock = ctx.now();
                ctx.send_shutdown(ShutdownReason::ConfigStepperFailed, clock);
            }
            return;
        }
    };
    if step_channel != dir_channel {
        defmt::error!(
            "config_stepper: STEP/DIR channel mismatch step={} dir={}",
            step_channel,
            dir_channel
        );
        if ctx.enter_shutdown() {
            let clock = ctx.now();
            ctx.send_shutdown(ShutdownReason::ConfigStepperFailed, clock);
        }
        return;
    }

    if let Err(e) =
        foci_klipper::commands::stepper::config_stepper_channel(ctx.state(), oid, step_channel)
    {
        defmt::error!("config_stepper: {}", e);
        if ctx.enter_shutdown() {
            let clock = ctx.now();
            ctx.send_shutdown(ShutdownReason::ConfigStepperFailed, clock);
        }
        return;
    }
    defmt::info!(
        "config_stepper: oid={} channel={} (TMC4671 mode)",
        oid,
        step_channel
    );
}

/// Enqueue a trapezoidal move segment for a stepper.
///
/// Delegates to [`StepperHost::queue_step_move`] which handles the
/// trigger-domain critical section, need-reset check, direction
/// preparation, SPSC queue push, and step-timer arming.
#[klipper_command]
pub fn queue_step(ctx: &mut dyn StepperQueue, oid: u8, interval: u32, count: u16, add: i16) {
    let move_ = foci_klipper::stepper::StepMove::new(interval, count, add);
    if let Err(e) = ctx.queue_step_move(move_) {
        defmt::error!("queue_step: oid={} {}", oid, e);
        if ctx.enter_shutdown() {
            let clock = ctx.now();
            ctx.send_shutdown(ShutdownReason::QueueStepFailed, clock);
        }
    }
}

/// Set the direction for subsequent steps on a stepper.
///
/// Direction is captured per-move in `queue_step` via `prepare_move()`.
#[klipper_command]
pub fn set_next_step_dir(ctx: &mut dyn StepperHost, _oid: u8, dir: u8) {
    ctx.set_next_step_dir(dir != 0);
}

/// Reset the step clock reference for a stepper.
///
/// Only valid when the stepper is inactive (timer not armed).
/// Triggers shutdown if called while active (matches Klipper stepper.c).
#[klipper_command]
pub fn reset_step_clock(ctx: &mut dyn StepperReset, _oid: u8, clock: u32) {
    if let Err(e) = ctx.reset_step_clock(clock) {
        defmt::error!("reset_step_clock: {}", e);
        if ctx.enter_shutdown() {
            let shutdown_clock = ctx.now();
            ctx.send_shutdown(
                ShutdownReason::CannotResetTimeWhenStepperActive,
                shutdown_clock,
            );
        }
    }
}

/// Report the current position of a stepper.
#[klipper_command]
pub fn stepper_get_position(ctx: &mut dyn StateAccess, oid: u8) {
    let pos = stepper_position(ctx, oid);
    ::ankyra::klipper_reply!(
        crate::ankyra::replies::StepperPosition,
        oid: u8 = oid,
        pos: i32 = pos
    );
}

/// Report the commanded (target) position of a stepper.
///
/// For hardware step/dir this is equivalent to `stepper_get_position`
/// since the firmware tracks only the logical step count.
#[klipper_command]
pub fn stepper_get_commanded_position(ctx: &mut dyn StateAccess, oid: u8) {
    let pos = stepper_position(ctx, oid);
    ::ankyra::klipper_reply!(
        crate::ankyra::replies::StepperPosition,
        oid: u8 = oid,
        pos: i32 = pos
    );
}

fn stepper_position(ctx: &mut dyn StateAccess, oid: u8) -> i32 {
    let channel = match ctx.state().stepper_channel(oid) {
        Some(channel) => channel,
        None => {
            defmt::error!("stepper_get_position: unknown stepper oid={}", oid);
            return 0;
        }
    };
    let state = match stepper_channel(channel) {
        Some(state) => state,
        None => {
            defmt::error!("stepper_get_position: invalid channel={}", channel);
            return 0;
        }
    };
    state.position.load(core::sync::atomic::Ordering::Relaxed)
}

/// Link a stepper to a TrSync so the stepper stops when the TrSync triggers.
///
/// Multiple steppers can be linked to the same TrSync. When a trigger fires,
/// all linked steppers are stopped.
#[klipper_command]
pub fn stepper_stop_on_trigger(ctx: &mut dyn StepperStopOnTrigger, oid: u8, trsync_oid: u8) {
    // Resolve stepper OID to index via State (immutable borrow; copy
    // the index before the mutable ctx methods are called).
    let stepper_idx = match ctx.state().steppers_by_oid.get(&oid).copied() {
        Some(idx) => idx,
        None => {
            defmt::error!("stepper_stop_on_trigger: unknown stepper oid={}", oid);
            if ctx.enter_shutdown() {
                let clock = ctx.now();
                ctx.send_shutdown(ShutdownReason::StepperStopOnTriggerUnknownStepper, clock);
            }
            return;
        }
    };

    if let Err(()) = ctx.link_stepper_stop(stepper_idx, trsync_oid) {
        defmt::error!(
            "stepper_stop_on_trigger: oid={} trsync_oid={} failed",
            oid,
            trsync_oid,
        );
        if ctx.enter_shutdown() {
            let clock = ctx.now();
            ctx.send_shutdown(ShutdownReason::StepperStopOnTriggerFailed, clock);
        }
    } else {
        // Begin homing I-suppression: transition Idle -> SuppressPending.
        // HOMING_TRIGGERED is NOT set here — it is set by
        // TriggerDomain::trigger() when the endstop/expire fires.
        use core::sync::atomic::Ordering::Relaxed;
        let _ = HOMING_STATE.compare_exchange(0, 1, Relaxed, Relaxed);
    }
}

/// Composite bound for `queue_step`: needs shutdown recovery (to enter
/// shutdown on error, which also implies `ClockSource` and `StateAccess`
/// via its supertraits) and the stepper host (to delegate the move
/// enqueue operation).
///
/// `&mut dyn ShutdownRecovery + StepperHost` is not a valid trait-object
/// type in stable Rust for two non-auto traits; this named supertrait is
/// the dyn-safe way to compose the capabilities.
pub trait StepperQueue: ShutdownRecovery + StepperHost {}
impl<T: ?Sized + ShutdownRecovery + StepperHost> StepperQueue for T {}

/// Composite bound for `config_stepper`: needs shutdown recovery and the
/// board's virtual STEP/DIR pin-to-channel mapping.
pub trait StepperConfigHost: ShutdownRecovery + BoardChannels {}
impl<T: ?Sized + ShutdownRecovery + BoardChannels> StepperConfigHost for T {}

/// Composite bound for `reset_step_clock`: needs shutdown recovery (to
/// enter shutdown on error) and the stepper host (to delegate the
/// clock-reset operation).
///
/// `&mut dyn ShutdownRecovery + StepperHost` is not valid as a
/// trait-object type; this named supertrait is the dyn-safe composition.
pub trait StepperReset: ShutdownRecovery + StepperHost {}
impl<T: ?Sized + ShutdownRecovery + StepperHost> StepperReset for T {}

/// Composite bound for `stepper_stop_on_trigger`: needs shutdown recovery
/// (to enter shutdown on error, implying `StateAccess` and `ClockSource`
/// via its supertraits) and the trsync host (to call `link_stepper_stop`
/// inside the trigger-domain critical section).
///
/// `&mut dyn ShutdownRecovery + TrsyncHost` is not a valid trait-object
/// type in stable Rust for two non-auto traits; this named supertrait is
/// the dyn-safe way to compose the capabilities.
pub trait StepperStopOnTrigger: ShutdownRecovery + TrsyncHost {}
impl<T: ?Sized + ShutdownRecovery + TrsyncHost> StepperStopOnTrigger for T {}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::context::{BoardChannels, ClockSource, StateAccess};
    use crate::shutdown::ShutdownReason;
    use foci_klipper::state::State;

    struct MockCtx {
        state: State,
        last_shutdown: Option<ShutdownReason>,
    }

    impl MockCtx {
        fn new() -> Self {
            Self {
                state: State::new(),
                last_shutdown: None,
            }
        }
    }

    impl StateAccess for MockCtx {
        fn state(&mut self) -> &mut State {
            &mut self.state
        }
    }

    impl ClockSource for MockCtx {
        fn now(&self) -> u32 {
            123
        }
    }

    impl ShutdownRecovery for MockCtx {
        fn enter_shutdown(&mut self) -> bool {
            true
        }

        fn reset_to_bootloader(&mut self) -> ! {
            panic!("reset_to_bootloader not used in tests")
        }

        fn send_shutdown(&mut self, reason: ShutdownReason, _clock: u32) {
            self.last_shutdown = Some(reason);
        }
    }

    impl BoardChannels for MockCtx {
        fn tmc_channel_count(&self) -> u8 {
            2
        }

        fn step_pin_channel(&self, pin: u8) -> Option<u8> {
            match pin {
                1 => Some(0),
                3 => Some(1),
                _ => None,
            }
        }

        fn dir_pin_channel(&self, pin: u8) -> Option<u8> {
            match pin {
                2 => Some(0),
                4 => Some(1),
                _ => None,
            }
        }

        fn enable_pin_channel(&self, _pin: u32) -> Option<u8> {
            None
        }
    }

    #[test]
    fn config_stepper_registers_matching_step_dir_channel() {
        let mut ctx = MockCtx::new();
        let mut sender = ();

        config_stepper(&mut ctx, &mut sender, 10, 1, 2, 0, 0);

        assert_eq!(ctx.state.stepper_channel(10), Some(0));
        assert_eq!(ctx.last_shutdown, None);
    }

    #[test]
    fn config_stepper_rejects_step_dir_channel_mismatch() {
        let mut ctx = MockCtx::new();
        let mut sender = ();

        config_stepper(&mut ctx, &mut sender, 10, 1, 4, 0, 0);

        assert_eq!(ctx.state.stepper_channel(10), None);
        assert_eq!(ctx.last_shutdown, Some(ShutdownReason::ConfigStepperFailed));
    }
}
