//! Stepper protocol state: position target, step interval/direction,
//! timer-active mirror, actual position. The heapless step-queue and
//! HardwareStepper handles stay board-local.

use core::sync::atomic::{AtomicBool, AtomicI32, AtomicU32};

/// Maximum number of board-local stepper channels supported by shared firmware.
pub const MAX_STEPPER_CHANNELS: usize = 2;

/// Cross-priority runtime state for one board-local stepper channel.
pub struct StepperChannelState {
    /// Stepper position target, updated by protocol dispatch.
    pub position_target: AtomicI32,
    /// Step count incremented by the hardware step ISR.
    pub position: AtomicI32,
    /// Current step interval in board timer ticks.
    pub interval: AtomicU32,
    /// Current step direction.
    pub direction: AtomicBool,
    /// P1-readable mirror of board-local step timer activity.
    pub timer_active_mirror: AtomicBool,
}

impl StepperChannelState {
    /// Create a stepper channel state with firmware defaults.
    pub const fn new() -> Self {
        Self {
            position_target: AtomicI32::new(0),
            position: AtomicI32::new(0),
            interval: AtomicU32::new(0),
            direction: AtomicBool::new(true),
            timer_active_mirror: AtomicBool::new(false),
        }
    }

    /// Reset channel-local stepper state to firmware defaults.
    pub fn reset(&self) {
        use core::sync::atomic::Ordering::Relaxed;

        self.position_target.store(0, Relaxed);
        self.position.store(0, Relaxed);
        self.interval.store(0, Relaxed);
        self.direction.store(true, Relaxed);
        self.timer_active_mirror.store(false, Relaxed);
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

/// Return the OpenFFBoard-compatible channel-0 runtime state.
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

    #[test]
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
