//! Generic ADC command handlers.

use ankyra::prelude::klipper_command;

use crate::analog_in::{AnalogInConfig, analog_channel, find_channel_by_oid};
use crate::context::AnalogInHost;

/// Register an analog-in object mapped to the board channel for `pin`.
/// Unknown pins are silently ignored; a later `query_analog_in` for this `oid` finds no channel.
#[klipper_command]
pub fn config_analog_in(ctx: &mut dyn AnalogInHost, oid: u8, pin: u8) {
    if let Some(index) = ctx.resolve_analog_pin(pin)
        && let Some(channel) = analog_channel(index)
    {
        channel.configure_oid(oid);
    }
}

/// Start (or restart) sampling on the channel assigned to `oid`.
/// A `sample_count` of zero is a no-op, not a disarm.
#[klipper_command]
#[allow(clippy::too_many_arguments)]
pub fn query_analog_in(
    _ctx: &mut dyn AnalogInHost,
    oid: u8,
    clock: u32,
    sample_ticks: u32,
    sample_count: u8,
    rest_ticks: u32,
    min_value: u16,
    max_value: u16,
    range_check_count: u8,
) {
    if sample_count == 0 {
        return;
    }
    if let Some(channel) = find_channel_by_oid(oid) {
        let config = AnalogInConfig {
            sample_ticks,
            sample_count,
            rest_ticks,
            min_value,
            max_value,
            range_check_count,
        };
        channel.configure_sampling(&config, clock);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::analog_in::{analog_channel, find_channel_by_oid};

    struct MockAnalogInHost;
    impl AnalogInHost for MockAnalogInHost {
        fn resolve_analog_pin(&self, pin: u8) -> Option<u8> {
            Some(pin)
        }
    }

    #[test]
    fn config_analog_in_records_the_oid_on_the_given_channel() {
        let mut ctx = MockAnalogInHost;
        let mut sender = ();
        config_analog_in(&mut ctx, &mut sender, 3, 0);
        assert_eq!(analog_channel(0).unwrap().oid(), Some(3));
    }

    #[test]
    fn query_analog_in_finds_the_channel_config_analog_in_assigned() {
        let mut ctx = MockAnalogInHost;
        let mut sender = ();
        config_analog_in(&mut ctx, &mut sender, 4, 1);
        query_analog_in(&mut ctx, &mut sender, 4, 1000, 10, 4, 5000, 100, 10_000, 3);
        let channel = find_channel_by_oid(4)
            .expect("channel must be found by the oid config_analog_in assigned");
        assert!(channel.is_due(1000));
    }
}
