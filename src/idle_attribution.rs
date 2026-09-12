//! Temporary per-task idle-residency attribution for the ongoing residual
//! investigation.
//!
//! Splits `cpu_load`'s aggregate busy/idle total into the fast/maintenance
//! buckets a prior investigation established plus four periodic board
//! tasks it did not cover, leaving an explicit unattributed residual.
//! Throwaway instrumentation: revert once the residual note is written.

use core::sync::atomic::{AtomicU32, Ordering::Relaxed};

static FAST_TICKS: AtomicU32 = AtomicU32::new(0);
static MAINTENANCE_TICKS: AtomicU32 = AtomicU32::new(0);
static WATCHDOG_TICKS: AtomicU32 = AtomicU32::new(0);
static STATS_TICKS: AtomicU32 = AtomicU32::new(0);
static REPORT_TICKS: AtomicU32 = AtomicU32::new(0);
static STEPPER_DIAG_TICKS: AtomicU32 = AtomicU32::new(0);

/// One stats window's busy time, split across the recorded buckets.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct Attribution {
    pub fast: u32,
    pub maintenance: u32,
    pub watchdog: u32,
    pub stats: u32,
    pub report: u32,
    pub stepper_diag: u32,
    /// `busy - (fast + maintenance + watchdog + stats + report +
    /// stepper_diag)`, saturating at zero. Reported explicitly rather than
    /// folded into a bucket: a large residual means these six buckets do
    /// not explain most of the busy time, not that it is negligible noise.
    pub residual: u32,
}

/// Inputs to [`attribute`]: one stats window's wall/idle totals plus the
/// recorded bucket values. A plain struct rather than positional
/// parameters, matching this project's convention for functions needing
/// more than five arguments.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AttributionInputs {
    pub wall: u32,
    pub idle: u32,
    pub fast: u32,
    pub maintenance: u32,
    pub watchdog: u32,
    pub stats: u32,
    pub report: u32,
    pub stepper_diag: u32,
}

/// Split one stats window's wall/idle totals across the recorded buckets.
pub fn attribute(inputs: AttributionInputs) -> Attribution {
    let busy = inputs.wall.saturating_sub(inputs.idle);
    let attributed = inputs
        .fast
        .saturating_add(inputs.maintenance)
        .saturating_add(inputs.watchdog)
        .saturating_add(inputs.stats)
        .saturating_add(inputs.report)
        .saturating_add(inputs.stepper_diag);
    Attribution {
        fast: inputs.fast,
        maintenance: inputs.maintenance,
        watchdog: inputs.watchdog,
        stats: inputs.stats,
        report: inputs.report,
        stepper_diag: inputs.stepper_diag,
        residual: busy.saturating_sub(attributed),
    }
}

/// Record cycles spent on one pick of a fast-control unit.
pub fn record_fast(ticks: u32) {
    FAST_TICKS.fetch_add(ticks, Relaxed);
}

/// Record cycles spent on one pick of a maintenance or maintenance-finalize
/// unit.
pub fn record_maintenance(ticks: u32) {
    MAINTENANCE_TICKS.fetch_add(ticks, Relaxed);
}

/// Record cycles spent in one `watchdog` task tick.
pub fn record_watchdog(ticks: u32) {
    WATCHDOG_TICKS.fetch_add(ticks, Relaxed);
}

/// Record cycles spent in one `stats` task tick. Because this task's own
/// cost can only be measured once it has already finished, and it emits
/// this module's `take()` call as part of that same execution, the
/// recorded value necessarily lands in the *next* window's snapshot, not
/// the one during which it ran -- an accepted one-window lag for this one
/// bucket, documented in the residual note rather than engineered
/// around (its own cost is small and roughly constant per window, so the
/// lag does not change which bucket dominates).
pub fn record_stats(ticks: u32) {
    STATS_TICKS.fetch_add(ticks, Relaxed);
}

/// Record cycles spent in one `report_task` drain pass.
pub fn record_report(ticks: u32) {
    REPORT_TICKS.fetch_add(ticks, Relaxed);
}

/// Record cycles spent in one `stepper_diag_task` drain pass.
pub fn record_stepper_diag(ticks: u32) {
    STEPPER_DIAG_TICKS.fetch_add(ticks, Relaxed);
}

/// Snapshot and reset the accumulators for a stats window of `wall` ticks,
/// given that window's already-known idle ticks.
pub fn take(wall: u32, idle: u32) -> Attribution {
    attribute(AttributionInputs {
        wall,
        idle,
        fast: FAST_TICKS.swap(0, Relaxed),
        maintenance: MAINTENANCE_TICKS.swap(0, Relaxed),
        watchdog: WATCHDOG_TICKS.swap(0, Relaxed),
        stats: STATS_TICKS.swap(0, Relaxed),
        report: REPORT_TICKS.swap(0, Relaxed),
        stepper_diag: STEPPER_DIAG_TICKS.swap(0, Relaxed),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn residual_is_busy_minus_attributed_buckets() {
        let a = attribute(AttributionInputs {
            wall: 1_000_000,
            idle: 400_000,
            fast: 100_000,
            maintenance: 80_000,
            watchdog: 50_000,
            stats: 30_000,
            report: 10_000,
            stepper_diag: 5_000,
        });
        // busy = wall - idle = 600_000; attributed = 275_000
        assert_eq!(a.residual, 325_000);
    }

    #[test]
    fn take_partitions_windows_and_resets() {
        record_fast(10_000);
        record_maintenance(20_000);
        record_watchdog(1_000);
        record_stats(2_000);
        let w1 = take(1_000_000, 700_000);
        // busy = 300_000; attributed = 33_000
        assert_eq!(w1.fast, 10_000);
        assert_eq!(w1.maintenance, 20_000);
        assert_eq!(w1.watchdog, 1_000);
        assert_eq!(w1.stats, 2_000);
        assert_eq!(w1.report, 0);
        assert_eq!(w1.residual, 267_000);

        record_report(500);
        let w2 = take(1_000_000, 700_000);
        assert_eq!(w2.fast, 0);
        assert_eq!(w2.report, 500);
        assert_eq!(w2.residual, 299_500);
    }
}
