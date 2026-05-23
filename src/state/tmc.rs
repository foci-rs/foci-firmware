//! TMC4671 protocol state primitives. The mpsc channel sender
//! (TMC_REQUEST_SENDER) stays in the board crate — its Sender type's
//! generic params bind to the board's RTIC app declaration.

use core::sync::atomic::{AtomicBool, AtomicI32, AtomicU16, AtomicU32};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;

/// Maximum number of board-local TMC4671 channels supported by shared firmware.
pub const MAX_TMC_CHANNELS: usize = 2;
/// Conservative operational PIDOUT_UQ_UD_LIMITS value used after closed-loop entry.
pub const DEFAULT_OPERATIONAL_PIDOUT_UQ_UD_LIMIT: u32 = 16_000;
/// Lowest accepted operational PIDOUT_UQ_UD_LIMITS value.
pub const MIN_OPERATIONAL_PIDOUT_UQ_UD_LIMIT: u32 = 1_024;
/// Highest accepted diagnostic PIDOUT_UQ_UD_LIMITS value.
pub const MAX_OPERATIONAL_PIDOUT_UQ_UD_LIMIT: u32 = 32_767;

/// Runtime state for one board-local TMC4671 channel.
pub struct TmcChannelState {
    /// Current limit for PID_TORQUE_FLUX_LIMITS (0x5E).
    pub current_limit_reg: AtomicU32,
    /// Set by P1 after calibration, cleared on disable/fault.
    pub motor_armed: AtomicBool,
    /// STEP_WIDTH value computed by tmc_set_encoder_hw.
    pub step_width_value: AtomicI32,
    /// Velocity biquad filter cutoff frequency in Hz.
    pub velocity_filter_hz: AtomicU16,
    /// Torque biquad filter cutoff frequency in Hz.
    pub torque_filter_hz: AtomicU16,
    /// Position target biquad filter cutoff frequency in Hz.
    pub position_filter_hz: AtomicU16,
    /// Flux biquad filter cutoff frequency in Hz.
    pub flux_filter_hz: AtomicU16,
    /// Velocity feedforward enable flag.
    pub velocity_feedforward: AtomicBool,
    /// Runtime multiplier for the computed velocity feedforward offset.
    pub velocity_feedforward_multiplier: AtomicU16,
    /// Acceleration feedforward enable flag.
    pub accel_feedforward: AtomicBool,
    /// Runtime gain in permille for the computed acceleration torque offset.
    pub accel_feedforward_gain_permille: AtomicU16,
    /// One-tick feedforward reset request after direction changes.
    pub feedforward_reset: AtomicBool,
    /// PID error-sum clear request after motion drains.
    pub pid_error_sums_clear: AtomicBool,
    /// Configurable PID velocity limit.
    pub velocity_limit_cfg: AtomicU32,
    /// Configurable operational PIDOUT_UQ_UD_LIMITS value.
    pub voltage_limit_cfg: AtomicU32,
    /// Whether the encoder direction is reversed.
    pub encoder_reversed: AtomicBool,
    /// Guard for single-in-flight commissioning on this channel.
    pub commission_in_flight: AtomicBool,
    /// Host-controlled gate for raw-enable auto-calibration.
    pub auto_calibrate_on_enable_allowed: AtomicBool,
    /// Position P gain for cross-priority access (Q8.8).
    pub position_p_value: AtomicU16,
    /// Position I gain for cross-priority access (Q8.8).
    pub position_i_value: AtomicU16,
    /// Velocity P gain for cross-priority access (Q8.8).
    pub velocity_p_value: AtomicU16,
    /// Velocity I gain for cross-priority access (Q8.8).
    pub velocity_i_value: AtomicU16,
}

impl TmcChannelState {
    /// Create a channel state with firmware defaults.
    pub const fn new() -> Self {
        Self {
            current_limit_reg: AtomicU32::new(0),
            motor_armed: AtomicBool::new(false),
            step_width_value: AtomicI32::new(0),
            velocity_filter_hz: AtomicU16::new(0),
            torque_filter_hz: AtomicU16::new(0),
            position_filter_hz: AtomicU16::new(0),
            flux_filter_hz: AtomicU16::new(0),
            velocity_feedforward: AtomicBool::new(false),
            velocity_feedforward_multiplier: AtomicU16::new(1),
            accel_feedforward: AtomicBool::new(false),
            accel_feedforward_gain_permille: AtomicU16::new(1000),
            feedforward_reset: AtomicBool::new(false),
            pid_error_sums_clear: AtomicBool::new(false),
            velocity_limit_cfg: AtomicU32::new(500_000),
            voltage_limit_cfg: AtomicU32::new(DEFAULT_OPERATIONAL_PIDOUT_UQ_UD_LIMIT),
            encoder_reversed: AtomicBool::new(false),
            commission_in_flight: AtomicBool::new(false),
            auto_calibrate_on_enable_allowed: AtomicBool::new(false),
            position_p_value: AtomicU16::new(640),
            position_i_value: AtomicU16::new(0),
            velocity_p_value: AtomicU16::new(1152),
            velocity_i_value: AtomicU16::new(0),
        }
    }

    /// Reset channel-local runtime/configuration state to firmware defaults.
    pub fn reset(&self) {
        use core::sync::atomic::Ordering::Relaxed;

        self.current_limit_reg.store(0, Relaxed);
        self.motor_armed.store(false, Relaxed);
        self.step_width_value.store(0, Relaxed);
        self.velocity_filter_hz.store(0, Relaxed);
        self.torque_filter_hz.store(0, Relaxed);
        self.position_filter_hz.store(0, Relaxed);
        self.flux_filter_hz.store(0, Relaxed);
        self.velocity_feedforward.store(false, Relaxed);
        self.velocity_feedforward_multiplier.store(1, Relaxed);
        self.accel_feedforward.store(false, Relaxed);
        self.accel_feedforward_gain_permille.store(1000, Relaxed);
        self.feedforward_reset.store(false, Relaxed);
        self.pid_error_sums_clear.store(false, Relaxed);
        self.velocity_limit_cfg.store(500_000, Relaxed);
        self.voltage_limit_cfg
            .store(DEFAULT_OPERATIONAL_PIDOUT_UQ_UD_LIMIT, Relaxed);
        self.encoder_reversed.store(false, Relaxed);
        self.commission_in_flight.store(false, Relaxed);
        self.auto_calibrate_on_enable_allowed.store(false, Relaxed);
        self.position_p_value.store(640, Relaxed);
        self.position_i_value.store(0, Relaxed);
        self.velocity_p_value.store(1152, Relaxed);
        self.velocity_i_value.store(0, Relaxed);
    }
}

impl Default for TmcChannelState {
    fn default() -> Self {
        Self::new()
    }
}

/// Channel-indexed TMC runtime state.
pub static TMC_CHANNELS: [TmcChannelState; MAX_TMC_CHANNELS] =
    [const { TmcChannelState::new() }; MAX_TMC_CHANNELS];

/// Return runtime state for a board-local TMC channel.
pub fn tmc_channel(channel: u8) -> Option<&'static TmcChannelState> {
    TMC_CHANNELS.get(channel as usize)
}

/// Return the OpenFFBoard-compatible channel-0 runtime state.
pub fn tmc_channel0() -> &'static TmcChannelState {
    &TMC_CHANNELS[0]
}

/// Reset all TMC channel states to firmware defaults.
pub fn reset_all_tmc_channels() {
    for channel in &TMC_CHANNELS {
        channel.reset();
    }
}

/// Emergency motor disable signal. Set by enter_shutdown(), consumed
/// by tmc_control. Uses a Signal so tmc_control wakes immediately
/// even if blocked on an empty channel.
pub static TMC_DISABLE_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Signal tmc_control to drain all pending requests. Set by config_reset
/// (P2), cleared by tmc_control (P1) after draining.
pub static TMC_QUEUE_FLUSH: AtomicBool = AtomicBool::new(false);

#[cfg(test)]
mod tests {
    use super::*;
    use core::sync::atomic::Ordering::Relaxed;

    #[test]
    fn tmc_channels_have_independent_current_limits() {
        let channel0 = tmc_channel(0).unwrap();
        let channel1 = tmc_channel(1).unwrap();

        channel0.current_limit_reg.store(0x1111, Relaxed);
        channel1.current_limit_reg.store(0x2222, Relaxed);

        assert_eq!(channel0.current_limit_reg.load(Relaxed), 0x1111);
        assert_eq!(channel1.current_limit_reg.load(Relaxed), 0x2222);

        channel0.current_limit_reg.store(0, Relaxed);
        channel1.current_limit_reg.store(0, Relaxed);
    }

    #[test]
    fn tmc_channel_rejects_out_of_range_index() {
        assert!(tmc_channel(MAX_TMC_CHANNELS as u8).is_none());
    }
}
