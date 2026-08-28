//! Lifecycle state primitives: shutdown / fault flags, stats
//! timestamps, deferred-report pending signals.

use core::sync::atomic::{AtomicBool, AtomicU32};

/// 64-bit uptime tracking. Updated by the stats task every 5 seconds.
/// `stats_send_time` is the CYCCNT value at last stats send.
/// `stats_send_time_high` counts 32-bit CYCCNT wraps (~25.5s at 168MHz).
pub static STATS_SEND_TIME: AtomicU32 = AtomicU32::new(0);
pub static STATS_SEND_TIME_HIGH: AtomicU32 = AtomicU32::new(0);

/// Shutdown active flag. Set by enter_shutdown(), cleared by clear_shutdown().
/// Checked by tmc_control to perform one-shot motor safe-state on first tick.
pub static SHUTDOWN_ACTIVE: AtomicBool = AtomicBool::new(false);

/// Tick of the most recently received valid `get_clock` request. Updated
/// only by the `get_clock` handler (P2); read by the P1 tmc_control loop's
/// host-liveness deadman.
pub static LAST_HOST_LIVENESS_TICKS: AtomicU32 = AtomicU32::new(0);

/// Whether at least one `get_clock` has ever been received. Distinguishes
/// "genuinely recent" from "boot-time timestamp that happens to look
/// recent" — a channel enabled before any get_clock has been seen is
/// treated as an immediate lease expiry, not granted a fresh grace window.
pub static HAS_RECEIVED_VALID_CLOCK: AtomicBool = AtomicBool::new(false);

/// Set by fault EXTI, cleared by P1 fault handler after cleanup.
pub static FAULT_ACTIVE: AtomicBool = AtomicBool::new(false);

/// Indicates trsync_state reports are pending delivery.
/// Set by TIM5 CC2 ISR, cleared by report_task.
pub static REPORT_PENDING: AtomicBool = AtomicBool::new(false);

/// Indicates stepper diagnostic events are pending delivery.
/// Set by `record_stepper_event`, cleared by stepper_diag_task.
pub static STEPPER_DIAG_PENDING: AtomicBool = AtomicBool::new(false);
