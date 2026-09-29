//! Idle-residency CPU-load accounting for the Klipper `stats` message.

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

/// `sum` is busy ticks (`wall - idle`, saturating); `count` is `laps` floored at 1 (host
/// divides by it); `sumsq` is always 0.
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
        let s = accumulate(1_000, 1_500, 3);
        assert_eq!(s.sum, 0);
    }

    #[test]
    fn zero_laps_reports_count_one() {
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
        record_idle_lap(300_000);
        record_idle_lap(200_000);
        let w1 = take(1_000_000);
        assert_eq!(w1.count, 2);
        assert_eq!(w1.sum, 500_000);

        record_idle_lap(100_000);
        let w2 = take(1_000_000);
        assert_eq!(w2.count, 1);
        assert_eq!(w2.sum, 900_000);
    }
}
