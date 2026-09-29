//! Stepper protocol state shared across priorities.

use core::sync::atomic::{AtomicBool, AtomicI32, AtomicU8, AtomicU32};

/// Maximum number of board-local stepper channels supported by shared firmware.
pub const MAX_STEPPER_CHANNELS: usize = 2;

/// Sentinel indicating that no stepper channel owns a deadline-miss shutdown.
pub const NO_STEPPER_CHANNEL: u8 = u8::MAX;

/// Channel that first observed a fatal step deadline miss.
pub static STEPPER_DEADLINE_MISS_CHANNEL: AtomicU8 = AtomicU8::new(NO_STEPPER_CHANNEL);

/// Cross-priority runtime state for one stepper channel.
pub struct StepperChannelState {
    pub position_target: AtomicI32,
    pub position: AtomicI32,
    /// Current step interval in board timer ticks.
    pub interval: AtomicU32,
    /// Interval delta in board timer ticks per step.
    pub interval_add: AtomicI32,
    pub direction: AtomicBool,
    pub timer_active_mirror: AtomicBool,
    planner_steps_per_rev: AtomicU32,
}

impl StepperChannelState {
    /// Create a stepper channel state with firmware defaults.
    pub const fn new() -> Self {
        Self {
            position_target: AtomicI32::new(0),
            position: AtomicI32::new(0),
            interval: AtomicU32::new(0),
            interval_add: AtomicI32::new(0),
            direction: AtomicBool::new(true),
            timer_active_mirror: AtomicBool::new(false),
            planner_steps_per_rev: AtomicU32::new(0),
        }
    }

    /// Reset channel-local stepper state to firmware defaults.
    pub fn reset(&self) {
        use core::sync::atomic::Ordering::Relaxed;

        self.position_target.store(0, Relaxed);
        self.position.store(0, Relaxed);
        self.interval.store(0, Relaxed);
        self.interval_add.store(0, Relaxed);
        self.direction.store(true, Relaxed);
        self.timer_active_mirror.store(false, Relaxed);
        self.planner_steps_per_rev.store(0, Relaxed);
    }

    pub(crate) fn store_planner_steps_per_rev(&self, value: u32) {
        self.planner_steps_per_rev
            .store(value, core::sync::atomic::Ordering::Relaxed);
    }

    /// Configured logical planner steps per mechanical revolution; `None` until the motion scale is
    /// configured.
    pub fn planner_steps_per_rev(
        &self,
        tmc_state: &crate::state::tmc::TmcChannelState,
    ) -> Option<u32> {
        tmc_state.motion_scale_configured().then(|| {
            self.planner_steps_per_rev
                .load(core::sync::atomic::Ordering::Relaxed)
        })
    }
}

impl Default for StepperChannelState {
    fn default() -> Self {
        Self::new()
    }
}

/// Channel-indexed stepper runtime state.
pub static STEPPER_CHANNELS: [StepperChannelState; MAX_STEPPER_CHANNELS] =
    [const { StepperChannelState::new() }; MAX_STEPPER_CHANNELS];

/// Return runtime state for a board-local stepper channel.
pub fn stepper_channel(channel: u8) -> Option<&'static StepperChannelState> {
    STEPPER_CHANNELS.get(channel as usize)
}

/// Runtime state for board-local channel 0.
pub fn stepper_channel0() -> &'static StepperChannelState {
    &STEPPER_CHANNELS[0]
}

/// Reset all stepper channel states to firmware defaults.
pub fn reset_all_stepper_channels() {
    for channel in &STEPPER_CHANNELS {
        channel.reset();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::sync::atomic::Ordering::Relaxed;
    use foci_firmware_macros::fixture_test;

    #[fixture_test]
    fn stepper_channels_have_independent_positions() {
        let channel0 = stepper_channel(0).unwrap();
        let channel1 = stepper_channel(1).unwrap();

        channel0.position.store(11, Relaxed);
        channel1.position.store(22, Relaxed);

        assert_eq!(channel0.position.load(Relaxed), 11);
        assert_eq!(channel1.position.load(Relaxed), 22);

        channel0.position.store(0, Relaxed);
        channel1.position.store(0, Relaxed);
    }

    #[test]
    fn stepper_channel_rejects_out_of_range_index() {
        assert!(stepper_channel(MAX_STEPPER_CHANNELS as u8).is_none());
    }
}
