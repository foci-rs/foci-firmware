//! TMC4671 protocol state primitives. The mpsc channel sender
//! (TMC_REQUEST_SENDER) stays in the board crate — its Sender type's
//! generic params bind to the board's RTIC app declaration.

use core::sync::atomic::{AtomicBool, AtomicI32, AtomicU16, AtomicU32};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;

/// Emergency motor disable signal. Set by enter_shutdown(), consumed
/// by tmc_control. Uses a Signal so tmc_control wakes immediately
/// even if blocked on an empty channel.
pub static TMC_DISABLE_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Current limit for PID_TORQUE_FLUX_LIMITS (0x5E).
/// Datasheet: single u16 field at bits `[15:0]`; upper 16 bits ignored by hardware.
/// Set by tmc_set_current, re-applied during calibration to ensure
/// the limit is correct regardless of TMC4671 register state.
pub static CURRENT_LIMIT_REG: AtomicU32 = AtomicU32::new(0);

/// Velocity biquad filter cutoff frequency in Hz (set by tmc_set_velocity_filter).
/// 0 = disabled, 10..=1000 = enabled. Applied during closed-loop entry.
pub static VELOCITY_FILTER_HZ: AtomicU16 = AtomicU16::new(0);

/// Torque biquad filter cutoff frequency in Hz (set by tmc_set_torque_filter).
/// 0 = disabled, 10..=1000 = enabled. Applied during closed-loop entry.
pub static TORQUE_FILTER_HZ: AtomicU16 = AtomicU16::new(0);

/// Position target biquad filter cutoff frequency in Hz (set by tmc_set_position_filter).
/// 0 = disabled, 10..=1000 = enabled. Applied during closed-loop entry.
pub static POSITION_FILTER_HZ: AtomicU16 = AtomicU16::new(0);

/// Flux biquad filter cutoff frequency in Hz (set by tmc_set_flux_filter).
/// 0 = disabled, 10..=1000 = enabled. Applied during closed-loop entry.
pub static FLUX_FILTER_HZ: AtomicU16 = AtomicU16::new(0);

/// Velocity feedforward enable flag (set by tmc_set_velocity_feedforward).
/// When true, the 1ms tick loop computes velocity from the step
/// interval and writes PID_VELOCITY_OFFSET.
pub static VELOCITY_FEEDFORWARD: AtomicBool = AtomicBool::new(false);

/// Signal the tick loop to zero feedforward for one tick after a
/// direction change. Avoids a sign-flip transient in
/// PID_VELOCITY_OFFSET. Set by handle_tmc_request when encoder
/// direction changes, consumed by tmc_control on next iteration.
pub static FEEDFORWARD_RESET: AtomicBool = AtomicBool::new(false);

/// Signal the tick loop to clear TMC4671 PID error sums after motion.
/// The step ISR cannot use SPI, so it sets this flag when the hardware
/// step queue drains and P1 performs the register writes on the next tick.
pub static PID_ERROR_SUMS_CLEAR: AtomicBool = AtomicBool::new(false);

/// Configurable PID velocity limit (caps position PID output).
/// Prevents integrator-driven velocity runaway when position I > 0.
/// Override via tmc_set_velocity_limit command from host config.
/// Written to PID_VELOCITY_LIMIT (0x60) during closedloop entry.
pub static VELOCITY_LIMIT_CFG: AtomicU32 = AtomicU32::new(500_000);

/// Whether the encoder direction is reversed (set by tmc_set_encoder_dir).
/// When true, position targets are negated before writing to the TMC4671.
pub static ENCODER_REVERSED: AtomicBool = AtomicBool::new(false);

/// Guard for single-in-flight commissioning. Set at command ingress (P2),
/// cleared after commissioning completes or fails (P1).
pub static COMMISSION_IN_FLIGHT: AtomicBool = AtomicBool::new(false);

/// Signal tmc_control to drain all pending requests. Set by config_reset
/// (P2), cleared by tmc_control (P1) after draining.
pub static TMC_QUEUE_FLUSH: AtomicBool = AtomicBool::new(false);

/// Set by P1 after calibration, cleared on disable/fault.
/// Gates timer arming and ISR execution.
pub static MOTOR_ARMED: AtomicBool = AtomicBool::new(false);

/// STEP_WIDTH value computed by tmc_set_encoder_hw, written to TMC4671
/// register 0x78 during closedloop entry.
pub static STEP_WIDTH_VALUE: AtomicI32 = AtomicI32::new(0);

/// Position P gain for cross-priority access (Q8.8).
pub static POSITION_P_VALUE: AtomicU16 = AtomicU16::new(640);

/// Position I gain for cross-priority access (Q8.8).
pub static POSITION_I_VALUE: AtomicU16 = AtomicU16::new(0);

/// Velocity P gain for cross-priority access (Q8.8).
/// Default 1152 = 4.5 in Q8.8 (conservative).
pub static VELOCITY_P_VALUE: AtomicU16 = AtomicU16::new(1152);

/// Velocity I gain for cross-priority access (Q8.8).
pub static VELOCITY_I_VALUE: AtomicU16 = AtomicU16::new(0);
