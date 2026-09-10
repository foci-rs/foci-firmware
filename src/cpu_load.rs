//! Idle-residency CPU-load accounting for the Klipper `stats` message.
//!
//! The board `idle` task times each `wfi` sleep in `CLOCK_FREQ`-domain
//! ticks and reports it here; `take` converts a stats window's accumulated
//! idle time into the busy figure Klipper's host derives load from.

use core::sync::atomic::{AtomicU32, Ordering::Relaxed};

static IDLE_TICKS: AtomicU32 = AtomicU32::new(0);
static IDLE_LAPS: AtomicU32 = AtomicU32::new(0);

/// Klipper `stats` payload fields.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct StatsSample {
    pub count: u32,
    pub sum: u32,
    pub sumsq: u32,
}

/// Derive the `stats` fields from one window's totals.
///
/// `sum` is busy ticks (`wall - idle`, saturating: a lap that straddles the
/// stats boundary can push idle past wall). `count` is the idle-lap count
/// floored at 1 so the host never divides by zero. `sumsq` is 0: idle
/// bracketing yields no per-pass busy series, and feeding idle variance to a
/// host formula that expects busy variance would misreport stddev.
pub fn accumulate(wall: u32, idle: u32, laps: u32) -> StatsSample {
    StatsSample {
        count: laps.max(1),
        sum: wall.saturating_sub(idle),
        sumsq: 0,
    }
}

/// Record one completed `wfi` sleep, in `CLOCK_FREQ`-domain ticks.
pub fn record_idle_lap(idle_ticks: u32) {
    IDLE_TICKS.fetch_add(idle_ticks, Relaxed);
    IDLE_LAPS.fetch_add(1, Relaxed);
}

/// Snapshot and reset the accumulators for a stats window of `wall` ticks.
pub fn take(wall: u32) -> StatsSample {
    let idle = IDLE_TICKS.swap(0, Relaxed);
    let laps = IDLE_LAPS.swap(0, Relaxed);
    accumulate(wall, idle, laps)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn busy_is_wall_minus_idle() {
        let s = accumulate(1_000_000, 900_000, 42);
        assert_eq!(s.sum, 100_000);
        assert_eq!(s.count, 42);
        assert_eq!(s.sumsq, 0);
    }

    #[test]
    fn idle_exceeding_wall_saturates_to_zero_busy() {
        // A lap can straddle the stats boundary, so idle may exceed wall.
        let s = accumulate(1_000, 1_500, 3);
        assert_eq!(s.sum, 0);
    }

    #[test]
    fn zero_laps_reports_count_one() {
        // Fully saturated window: idle never reached. count must stay >= 1
        // so the host's `1/(count*freq)` never divides by zero.
        let s = accumulate(1_300_000_000, 0, 0);
        assert_eq!(s.count, 1);
        assert_eq!(s.sum, 1_300_000_000);
    }

    #[test]
    fn sumsq_is_always_zero() {
        assert_eq!(accumulate(500, 100, 7).sumsq, 0);
        assert_eq!(accumulate(0, 0, 0).sumsq, 0);
    }

    #[test]
    fn take_partitions_windows_and_resets() {
        // Sole test that touches the module's global accumulators; it runs the
        // whole record/take sequence itself so nothing else races it (the
        // other tests call only the pure `accumulate`). Asserts each window
        // reports only the laps recorded before its `take`, and that `take`
        // resets the accumulators.
        record_idle_lap(300_000);
        record_idle_lap(200_000);
        let w1 = take(1_000_000);
        assert_eq!(w1.count, 2);
        assert_eq!(w1.sum, 500_000); // 1_000_000 - (300_000 + 200_000)

        // A lap recorded only after the first take lands in the next window.
        record_idle_lap(100_000);
        let w2 = take(1_000_000);
        assert_eq!(w2.count, 1);
        assert_eq!(w2.sum, 900_000);
    }
}
