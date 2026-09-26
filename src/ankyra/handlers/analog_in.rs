//! Generic ADC command handlers: `config_analog_in`, `query_analog_in`.
//!
//! Storage lives in `crate::analog_in::ANALOG_CHANNELS`, a process-wide
//! static reachable directly via `analog_channel`/`find_channel_by_oid` --
//! no host trait needed for it. The only board-specific piece is pin
//! resolution, reached through [`crate::context::AnalogInHost`].

use ankyra::prelude::klipper_command;

use crate::analog_in::{AnalogInConfig, analog_channel, find_channel_by_oid};
use crate::context::AnalogInHost;

/// Register an analog-in object and remember which channel it maps to.
///
/// Resolves the wire `pin` to a board-local channel index via
/// [`AnalogInHost::resolve_analog_pin`] and records `oid` on that
/// channel's slot. Unknown pins are silently ignored -- no channel is
/// mapped and a subsequent `query_analog_in` for this `oid` will not find
/// one.
#[klipper_command]
pub fn config_analog_in(ctx: &mut dyn AnalogInHost, oid: u8, pin: u8) {
    if let Some(index) = ctx.resolve_analog_pin(pin)
        && let Some(channel) = analog_channel(index)
    {
        channel.configure_oid(oid);
    }
}

/// Start (or restart) sampling on the channel `config_analog_in` already
/// assigned to `oid`.
///
/// `query_analog_in` carries no `pin` on the wire, so the channel is
/// recovered via [`find_channel_by_oid`] instead of pin resolution. A
/// `sample_count` of zero is a no-op: Klipper's reference firmware treats
/// it as an invalid request rather than a disarm.
#[klipper_command]
#[allow(clippy::too_many_arguments)]
pub fn query_analog_in(
    // No pin resolution needed here -- query_analog_in carries no `pin` on
    // the wire, only `oid`. Keep this first parameter regardless: the
    // macro unconditionally treats the first parameter as "ctx" and
    // splices the injected sender immediately after it -- dropping this
    // parameter would shift the sender into the wire-argument position.
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

    /// Maps every pin to channel `pin as u8` -- a trivial identity mock,
    /// sufficient since these tests exercise oid/channel bookkeeping, not
    /// pin-resolution logic (each board's `analog_pin_channel`, tested there).
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
