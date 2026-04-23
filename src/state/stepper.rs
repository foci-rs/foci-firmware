//! Stepper protocol state: position target, step interval/direction,
//! timer-active mirror, actual position. The heapless step-queue and
//! HardwareStepper handles stay board-local.

use core::sync::atomic::{AtomicBool, AtomicI32, AtomicU32};

/// Stepper position target, updated by usb_receive (P2) after each
/// protocol dispatch, read by tmc_control (P1) for SPI writes.
/// Lockfree: AtomicI32 avoids priority ceiling lock that would
/// block P2 protocol tasks.
pub static STEPPER_POSITION_TARGET: AtomicI32 = AtomicI32::new(0);

/// Step count (incremented by P3 ISR, read by P1/P2 for stepper_get_position).
pub static STEPPER_POSITION: AtomicI32 = AtomicI32::new(0);

/// Current step interval in TIM5 clock ticks, written by P3 after
/// every step event. Written to 0 when the step queue drains (both
/// the ISR empty-queue branch and stepper_stop). Read by tmc_control
/// at 1 kHz for velocity feedforward computation.
pub static STEP_INTERVAL: AtomicU32 = AtomicU32::new(0);

/// Current step direction, written by P3 when direction changes.
/// true = forward, false = reverse. Read by tmc_control for
/// feedforward sign.
pub static STEP_DIRECTION: AtomicBool = AtomicBool::new(true);

/// Read-only mirror of `stepper_stop[0].timer_active` for P1.
/// Written inside TriggerDomain lock via `set_timer_active()`,
/// read by P1 for advisory purposes (PID settle timing).
/// P1 must never enter TriggerDomain.
pub static STEP_TIMER_ACTIVE_MIRROR: AtomicBool = AtomicBool::new(false);
