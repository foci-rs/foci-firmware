//! Following-error stall detector for sensorless homing.

/// Absolute following error in TMC position units.
///
/// Both registers are 32-bit two's-complement positions; the difference is
/// taken wrapping so a revolution boundary between them does not blow up.
pub fn following_error_units(target: u32, actual: u32) -> u32 {
    (target.wrapping_sub(actual) as i32).unsigned_abs()
}

/// Consecutive-tick following-error detector with a sticky latch.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct StallDetector {
    threshold_units: u32,
    persistence: u8,
    consecutive: u8,
    ticks: u32,
    peak_error_units: u32,
    trigger_tick: u32,
    latched: bool,
}

impl StallDetector {
    /// Create an armed detector. `persistence` is clamped to at least 1.
    pub const fn new(threshold_units: u32, persistence: u8) -> Self {
        Self {
            threshold_units,
            persistence: if persistence == 0 { 1 } else { persistence },
            consecutive: 0,
            ticks: 0,
            peak_error_units: 0,
            trigger_tick: 0,
            latched: false,
        }
    }

    /// Feed one tick of target and actual position. Returns the latch state.
    pub fn observe(&mut self, target: u32, actual: u32) -> bool {
        self.ticks = self.ticks.wrapping_add(1);
        let error = following_error_units(target, actual);
        if error > self.peak_error_units {
            self.peak_error_units = error;
        }
        if self.latched {
            return true;
        }
        if error >= self.threshold_units {
            self.consecutive = self.consecutive.saturating_add(1);
            if self.consecutive >= self.persistence {
                self.latched = true;
                self.trigger_tick = self.ticks;
            }
        } else {
            self.consecutive = 0;
        }
        self.latched
    }

    /// Whether the latch has set since arming.
    pub const fn latched(&self) -> bool {
        self.latched
    }

    /// Largest following error seen since arming.
    pub const fn peak_error_units(&self) -> u32 {
        self.peak_error_units
    }

    /// Tick count at which the latch set, 0 if it never did.
    pub const fn trigger_tick(&self) -> u32 {
        self.trigger_tick
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn following_error_wraps_across_revolution_boundary() {
        assert_eq!(following_error_units(10, u32::MAX - 5), 16);
        assert_eq!(following_error_units(u32::MAX - 5, 10), 16);
        assert_eq!(following_error_units(0x8000_0000, 0), 0x8000_0000);
    }

    #[test]
    fn latch_requires_consecutive_ticks_over_threshold() {
        let mut det = StallDetector::new(100, 3);
        assert!(!det.observe(100, 0));
        assert!(!det.observe(100, 0));
        assert!(!det.observe(50, 0));
        assert!(!det.observe(100, 0));
        assert!(!det.observe(100, 0));
        assert!(det.observe(100, 0));
        assert_eq!(det.trigger_tick(), 6);
    }

    #[test]
    fn latch_is_sticky_and_tracks_peak() {
        let mut det = StallDetector::new(10, 1);
        assert!(det.observe(500, 0));
        assert!(det.observe(0, 0));
        assert!(det.latched());
        assert_eq!(det.peak_error_units(), 500);
    }

    #[test]
    fn zero_persistence_behaves_as_one() {
        let mut det = StallDetector::new(10, 0);
        assert!(det.observe(10, 0));
    }
}
