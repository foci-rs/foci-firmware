//! Shutdown and fault flags, stats timestamps, deferred-report pending flags.

use core::sync::atomic::{AtomicBool, AtomicU32};

/// CYCCNT value at the last stats send; the `_HIGH` counter counts 32-bit CYCCNT wraps.
pub static STATS_SEND_TIME: AtomicU32 = AtomicU32::new(0);
pub static STATS_SEND_TIME_HIGH: AtomicU32 = AtomicU32::new(0);

/// Shutdown active flag. Set by enter_shutdown(), cleared by clear_shutdown().
pub static SHUTDOWN_ACTIVE: AtomicBool = AtomicBool::new(false);

/// Tick of the most recently received valid `get_clock` request. Updated
/// only by the `get_clock` handler (P2); read by the P1 tmc_control loop's
/// host-liveness deadman.
pub static LAST_HOST_LIVENESS_TICKS: AtomicU32 = AtomicU32::new(0);

/// Whether any `get_clock` has been received; a channel enabled before then gets an immediate lease
/// expiry.
pub static HAS_RECEIVED_VALID_CLOCK: AtomicBool = AtomicBool::new(false);

/// Set by fault EXTI, cleared by P1 fault handler after cleanup.
pub static FAULT_ACTIVE: AtomicBool = AtomicBool::new(false);

/// trsync_state reports pending delivery.
pub static REPORT_PENDING: AtomicBool = AtomicBool::new(false);

/// Stepper diagnostic events pending delivery; set by `record_stepper_event`.
pub static STEPPER_DIAG_PENDING: AtomicBool = AtomicBool::new(false);
