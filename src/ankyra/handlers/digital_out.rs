//! Digital-output command handlers.

use ankyra::prelude::klipper_command;

use crate::context::{BoardChannels, ShutdownRecovery, TmcHost};
use crate::shutdown::ShutdownReason;
use crate::tmc::tmc_set_enable;

/// Register a digital output for motor enable/disable.
/// The pin must be a FOCI virtual enable pin mapped to a board-local TMC4671 channel.
#[klipper_command]
pub fn config_digital_out(
    ctx: &mut dyn DigitalOutConfigHost,
    oid: u8,
    pin: u32,
    _value: u8,
    _default_value: u8,
    _max_duration: u32,
) {
    let channel = match ctx.enable_pin_channel(pin) {
        Some(channel) => channel,
        None => {
            defmt::error!("config_digital_out: unknown ENA pin {}", pin);
            if ctx.enter_shutdown() {
                let clock = ctx.now();
                ctx.send_shutdown(ShutdownReason::ConfigDigitalOutFailed, clock);
            }
            return;
        }
    };

    if let Err(e) =
        foci_klipper::commands::digital_out::config_enable_channel(ctx.state(), oid, channel)
    {
        defmt::error!("config_digital_out: {}", e);
        if ctx.enter_shutdown() {
            let clock = ctx.now();
            ctx.send_shutdown(ShutdownReason::ConfigDigitalOutFailed, clock);
        }
        return;
    }
    defmt::info!(
        "config_digital_out: oid={} channel={} (enable control)",
        oid,
        channel
    );
}

/// Schedule a digital output change at a given clock time.
#[klipper_command]
pub fn queue_digital_out(ctx: &mut dyn DigitalOutHost, oid: u8, clock: u32, on_ticks: u32) {
    let _ = clock;
    match foci_klipper::commands::digital_out::queue_digital_out(ctx.state(), oid, on_ticks) {
        Ok(enabled) => {
            let Some(channel) = enable_channel_or_shutdown(ctx, oid) else {
                return;
            };
            if ctx
                .enqueue_request(tmc_set_enable(channel, enabled))
                .is_err()
                && ctx.enter_shutdown()
            {
                let c = ctx.now();
                ctx.send_shutdown(ShutdownReason::MotorEnableDisableLostQueueFull, c);
            }
        }
        Err(e) => {
            defmt::error!("queue_digital_out: {}", e);
            if ctx.enter_shutdown() {
                let clock = ctx.now();
                ctx.send_shutdown(ShutdownReason::QueueDigitalOutFailed, clock);
            }
        }
    }
}

/// Immediately update a digital output value.
#[klipper_command]
pub fn update_digital_out(ctx: &mut dyn DigitalOutHost, oid: u8, value: u8) {
    match foci_klipper::commands::digital_out::update_digital_out(ctx.state(), oid, value) {
        Ok(enabled) => {
            let Some(channel) = enable_channel_or_shutdown(ctx, oid) else {
                return;
            };
            if ctx
                .enqueue_request(tmc_set_enable(channel, enabled))
                .is_err()
                && ctx.enter_shutdown()
            {
                let c = ctx.now();
                ctx.send_shutdown(ShutdownReason::MotorEnableDisableLostQueueFull, c);
            }
        }
        Err(e) => {
            defmt::error!("update_digital_out: {}", e);
            if ctx.enter_shutdown() {
                let clock = ctx.now();
                ctx.send_shutdown(ShutdownReason::UpdateDigitalOutFailed, clock);
            }
        }
    }
}

/// Composite bound for `queue_digital_out` and `update_digital_out`:
/// shutdown recovery plus the TMC host.
pub trait DigitalOutHost: ShutdownRecovery + TmcHost {}
impl<T: ?Sized + ShutdownRecovery + TmcHost> DigitalOutHost for T {}

/// Composite bound for `config_digital_out`: needs shutdown recovery and the
/// board's virtual enable pin-to-channel mapping.
pub trait DigitalOutConfigHost: ShutdownRecovery + BoardChannels {}
impl<T: ?Sized + ShutdownRecovery + BoardChannels> DigitalOutConfigHost for T {}

fn enable_channel_or_shutdown(ctx: &mut dyn ShutdownRecovery, oid: u8) -> Option<u8> {
    match ctx.state().enable_channel(oid) {
        Some(channel) => Some(channel),
        None => {
            if ctx.enter_shutdown() {
                let clock = ctx.now();
                ctx.send_shutdown(ShutdownReason::InvalidTmcChannel, clock);
            }
            None
        }
    }
}

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
            1
        }

        fn step_pin_channel(&self, _pin: u8) -> Option<u8> {
            None
        }

        fn dir_pin_channel(&self, _pin: u8) -> Option<u8> {
            None
        }

        fn enable_pin_channel(&self, pin: u32) -> Option<u8> {
            match pin {
                100 => Some(0),
                _ => None,
            }
        }
    }

    #[test]
    fn config_digital_out_registers_enable_channel() {
        let mut ctx = MockCtx::new();
        let mut sender = ();

        config_digital_out(&mut ctx, &mut sender, 11, 100, 0, 0, 0);

        assert_eq!(ctx.state.enable_channel(11), Some(0));
        assert_eq!(ctx.last_shutdown, None);
    }

    #[test]
    fn config_digital_out_rejects_unknown_enable_pin() {
        let mut ctx = MockCtx::new();
        let mut sender = ();

        config_digital_out(&mut ctx, &mut sender, 11, 101, 0, 0, 0);

        assert_eq!(ctx.state.enable_channel(11), None);
        assert_eq!(
            ctx.last_shutdown,
            Some(ShutdownReason::ConfigDigitalOutFailed)
        );
    }
}
