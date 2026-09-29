//! Following-error stall detector for sensorless homing.

/// Absolute following error in TMC position units.
///
/// Both registers are wrapping 32-bit positions; the difference wraps.
pub fn following_error_units(target: u32, actual: u32) -> u32 {
    (target.wrapping_sub(actual) as i32).unsigned_abs()
}

/// Which check latched the detector, for validation telemetry. `None`
/// before any latch.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum StallTriggerPath {
    None,
    Ceiling,
    Margin,
}

impl StallTriggerPath {
    /// Wire encoding: 0 = None, 1 = Ceiling, 2 = Margin.
    pub const fn as_u8(self) -> u8 {
        match self {
            StallTriggerPath::None => 0,
            StallTriggerPath::Ceiling => 1,
            StallTriggerPath::Margin => 2,
        }
    }
}

const STALLED_ENCODER_JITTER_COUNTS: u32 = 3;
const MOTION_NOISE_FLOOR_COUNTS: u32 = STALLED_ENCODER_JITTER_COUNTS + 1;

const MOTION_WARMUP_TICKS: u16 = 8;

const EMA_SHIFT: u32 = 3;

const BASELINE_FRAC_BITS: u32 = 8;

/// Consecutive-tick following-error detector with a sticky latch.
///
/// Runs an unconditional absolute `ceiling` check and a `margin` check against a
/// motion-gated EMA baseline. Either latches the detector; a same-tick tie favors `Ceiling`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct StallDetector {
    ceiling_units: u32,
    margin_units: u32,
    persistence: u8,
    motion_noise_floor_units: u32,
    ceiling_consecutive: u8,
    margin_consecutive: u8,
    ticks: u32,
    peak_error_units: u32,
    peak_margin_delta_scaled: u64,
    trigger_tick: u32,
    trigger_path: StallTriggerPath,
    latched: bool,
    prev_actual: u32,
    baseline_scaled: u64,
    motion_warmup_ticks: u16,
}

impl StallDetector {
    /// Create an armed detector. `persistence` is clamped to at least 1;
    /// `encoder_counts_per_rev` comes from a published motion scale, which
    /// is never zero.
    pub const fn new(
        ceiling_units: u32,
        margin_units: u32,
        persistence: u8,
        encoder_counts_per_rev: u32,
    ) -> Self {
        Self {
            ceiling_units,
            margin_units,
            persistence: if persistence == 0 { 1 } else { persistence },
            motion_noise_floor_units: (MOTION_NOISE_FLOOR_COUNTS
                * crate::motion_scale::POSITION_UNITS_PER_REV as u32)
                .div_ceil(encoder_counts_per_rev),
            ceiling_consecutive: 0,
            margin_consecutive: 0,
            ticks: 0,
            peak_error_units: 0,
            peak_margin_delta_scaled: 0,
            trigger_tick: 0,
            trigger_path: StallTriggerPath::None,
            latched: false,
            prev_actual: 0,
            baseline_scaled: 0,
            motion_warmup_ticks: 0,
        }
    }

    /// Feed one tick of target and actual position. Returns the latch state.
    pub fn observe(&mut self, target: u32, actual: u32) -> bool {
        let first_tick = self.ticks == 0;
        self.ticks = self.ticks.wrapping_add(1);
        let error = following_error_units(target, actual);
        if error > self.peak_error_units {
            self.peak_error_units = error;
        }
        if self.latched {
            return true;
        }

        let motion_confirmed = !first_tick
            && (actual.wrapping_sub(self.prev_actual) as i32).unsigned_abs()
                >= self.motion_noise_floor_units;
        self.prev_actual = actual;
        let mut baseline_valid = self.motion_warmup_ticks >= MOTION_WARMUP_TICKS;
        let error_scaled = (error as u64) << BASELINE_FRAC_BITS;
        if motion_confirmed {
            if self.motion_warmup_ticks < MOTION_WARMUP_TICKS {
                self.motion_warmup_ticks += 1;
                baseline_valid = self.motion_warmup_ticks >= MOTION_WARMUP_TICKS;
            }
            let diff = error_scaled as i64 - self.baseline_scaled as i64;
            let step = diff >> EMA_SHIFT;
            self.baseline_scaled = (self.baseline_scaled as i64 + step).max(0) as u64;
        }

        let ceiling_hit = error >= self.ceiling_units;
        self.ceiling_consecutive = if ceiling_hit {
            self.ceiling_consecutive.saturating_add(1)
        } else {
            0
        };

        let margin_delta_scaled = error_scaled.saturating_sub(self.baseline_scaled);
        if baseline_valid && margin_delta_scaled > self.peak_margin_delta_scaled {
            self.peak_margin_delta_scaled = margin_delta_scaled;
        }
        let margin_units_scaled = (self.margin_units as u64) << BASELINE_FRAC_BITS;
        let margin_hit = baseline_valid && margin_delta_scaled >= margin_units_scaled;
        self.margin_consecutive = if margin_hit {
            self.margin_consecutive.saturating_add(1)
        } else {
            0
        };

        let ceiling_latches = self.ceiling_consecutive >= self.persistence;
        let margin_latches = self.margin_consecutive >= self.persistence;
        if ceiling_latches || margin_latches {
            self.latched = true;
            self.trigger_tick = self.ticks;
            self.trigger_path = if ceiling_latches {
                StallTriggerPath::Ceiling
            } else {
                StallTriggerPath::Margin
            };
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

    /// Largest `error - baseline` seen while the baseline was valid, in position units rounded
    /// up. 0 if the baseline never warmed up.
    pub const fn peak_margin_delta_units(&self) -> u32 {
        let round_up_bias = (1u64 << BASELINE_FRAC_BITS) - 1;
        ((self.peak_margin_delta_scaled + round_up_bias) >> BASELINE_FRAC_BITS) as u32
    }

    /// Tick count at which the latch set, 0 if it never did.
    pub const fn trigger_tick(&self) -> u32 {
        self.trigger_tick
    }

    /// Which check latched the detector, `None` if it never did.
    pub const fn trigger_path(&self) -> StallTriggerPath {
        self.trigger_path
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const COUNTS_PER_REV: u32 = 4_000;

    fn units_from_counts(counts: u32, counts_per_rev: u32) -> u32 {
        counts * crate::motion_scale::POSITION_UNITS_PER_REV as u32 / counts_per_rev
    }

    #[test]
    fn following_error_wraps_across_revolution_boundary() {
        assert_eq!(following_error_units(10, u32::MAX - 5), 16);
        assert_eq!(following_error_units(u32::MAX - 5, 10), 16);
        assert_eq!(following_error_units(0x8000_0000, 0), 0x8000_0000);
    }

    #[test]
    fn ceiling_check_latches_on_gross_error_before_baseline_warms_up() {
        let mut det = StallDetector::new(
            /* ceiling */ 100,
            /* margin */ 10,
            /* persistence */ 2,
            COUNTS_PER_REV,
        );
        assert!(!det.observe(50, 0));
        assert!(!det.observe(100, 0));
        assert!(det.observe(150, 0));
        assert!(det.latched());
        assert_eq!(det.trigger_path(), StallTriggerPath::Ceiling);
    }

    #[test]
    fn ceiling_check_latches_under_encoder_jitter_within_the_noise_floor() {
        let mut det = StallDetector::new(
            /* ceiling */ 200,
            /* margin */ 10,
            /* persistence */ 2,
            COUNTS_PER_REV,
        );
        let jitter = units_from_counts(STALLED_ENCODER_JITTER_COUNTS, COUNTS_PER_REV);
        assert!(!det.observe(100, 0));
        assert!(!det.observe(150, jitter));
        assert!(!det.observe(205, 0));
        assert!(det.observe(250, jitter));
        assert!(det.latched());
        assert_eq!(det.trigger_path(), StallTriggerPath::Ceiling);
        assert_eq!(det.peak_margin_delta_units(), 0);
    }

    #[test]
    fn margin_check_stays_masked_while_axis_is_jammed_from_tick_1() {
        let mut det = StallDetector::new(
            /* ceiling */ 10_000,
            /* margin */ 10,
            /* persistence */ 2,
            COUNTS_PER_REV,
        );
        for _ in 0..50 {
            assert!(!det.observe(1000, 0));
        }
        assert!(!det.latched());
    }

    #[test]
    fn margin_check_latches_once_baseline_warms_up_and_error_grows() {
        let mut det = StallDetector::new(
            /* ceiling */ 10_000,
            /* margin */ 20,
            /* persistence */ 2,
            COUNTS_PER_REV,
        );
        let mut actual = 0u32;
        for _ in 0..MOTION_WARMUP_TICKS + 2 {
            actual += 70;
            assert!(!det.observe(actual + 5, actual));
        }
        assert!(!det.latched());
        assert!(!det.observe(actual + 30, actual));
        assert!(det.observe(actual + 30, actual));
        assert!(det.latched());
        assert_eq!(det.trigger_path(), StallTriggerPath::Margin);
    }

    #[test]
    fn slow_motion_on_a_fine_encoder_warms_the_baseline() {
        const FINE_COUNTS_PER_REV: u32 = 40_000;
        let mut det = StallDetector::new(
            /* ceiling */ 10_000,
            /* margin */ 20,
            /* persistence */ 2,
            FINE_COUNTS_PER_REV,
        );
        let mut actual = 0u32;
        for _ in 0..MOTION_WARMUP_TICKS + 2 {
            actual += units_from_counts(MOTION_NOISE_FLOOR_COUNTS * 3, FINE_COUNTS_PER_REV);
            assert!(!det.observe(actual + 5, actual));
        }
        assert!(!det.observe(actual + 30, actual));
        assert!(det.observe(actual + 30, actual));
        assert_eq!(det.trigger_path(), StallTriggerPath::Margin);
    }

    #[test]
    fn jitter_on_a_coarse_encoder_never_confirms_motion() {
        const COARSE_COUNTS_PER_REV: u32 = 1_000;
        let mut det = StallDetector::new(
            /* ceiling */ 10_000,
            /* margin */ 10,
            /* persistence */ 2,
            COARSE_COUNTS_PER_REV,
        );
        let jitter = units_from_counts(STALLED_ENCODER_JITTER_COUNTS, COARSE_COUNTS_PER_REV);
        for tick in 0..50u32 {
            let actual = if tick % 2 == 0 { 0 } else { jitter };
            assert!(!det.observe(1_000, actual));
        }
        assert_eq!(det.peak_margin_delta_units(), 0);
    }

    #[test]
    fn normal_cruise_within_ceiling_and_margin_never_latches() {
        let mut det = StallDetector::new(
            /* ceiling */ 10_000,
            /* margin */ 20,
            /* persistence */ 2,
            COUNTS_PER_REV,
        );
        let mut actual = 0u32;
        for _ in 0..MOTION_WARMUP_TICKS + 20 {
            actual += 70;
            assert!(!det.observe(actual + 5, actual));
        }
        assert!(!det.latched());
    }

    #[test]
    fn same_tick_tie_between_ceiling_and_margin_reports_ceiling() {
        let mut det = StallDetector::new(
            /* ceiling */ 40,
            /* margin */ 5,
            /* persistence */ 1,
            COUNTS_PER_REV,
        );
        let mut actual = 0u32;
        for _ in 0..MOTION_WARMUP_TICKS + 1 {
            actual += 70;
            det.observe(actual + 2, actual);
        }
        assert!(!det.latched());
        assert!(det.observe(actual + 40, actual));
        assert_eq!(det.trigger_path(), StallTriggerPath::Ceiling);
    }

    #[test]
    fn gross_failure_latches_via_ceiling_even_after_baseline_valid() {
        let mut det = StallDetector::new(
            /* ceiling */ 500,
            /* margin */ 20,
            /* persistence */ 1,
            COUNTS_PER_REV,
        );
        let mut actual = 0u32;
        for _ in 0..MOTION_WARMUP_TICKS + 5 {
            actual += 70;
            det.observe(actual + 5, actual);
        }
        assert!(!det.latched());
        assert!(det.observe(actual + 600, actual));
        assert_eq!(det.trigger_path(), StallTriggerPath::Ceiling);
    }

    #[test]
    fn latch_is_sticky_and_tracks_peak() {
        let mut det = StallDetector::new(10, 5, 1, COUNTS_PER_REV);
        assert!(det.observe(500, 0));
        assert!(det.observe(0, 0));
        assert!(det.latched());
        assert_eq!(det.peak_error_units(), 500);
    }

    #[test]
    fn zero_persistence_behaves_as_one() {
        let mut det = StallDetector::new(10, 5, 0, COUNTS_PER_REV);
        assert!(det.observe(10, 0));
    }

    #[test]
    fn unlatched_detector_reports_no_trigger_path() {
        let det = StallDetector::new(10_000, 20, 1, COUNTS_PER_REV);
        assert_eq!(det.trigger_path(), StallTriggerPath::None);
    }

    #[test]
    fn baseline_converges_for_a_sub_shift_error_instead_of_sticking_at_zero() {
        let mut det = StallDetector::new(
            /* ceiling */ 10_000,
            /* margin */ 1,
            /* persistence */ 1,
            COUNTS_PER_REV,
        );
        let mut actual = 0u32;
        for _ in 0..MOTION_WARMUP_TICKS + 40 {
            actual += 70;
            det.observe(actual + 1, actual);
        }
        assert!(!det.latched());
    }

    #[test]
    fn slow_creep_under_the_ceiling_stays_masked_by_the_tracking_baseline() {
        let mut det = StallDetector::new(
            /* ceiling */ 10_000,
            /* margin */ 20,
            /* persistence */ 2,
            COUNTS_PER_REV,
        );
        let mut actual = 0u32;
        for _ in 0..MOTION_WARMUP_TICKS + 5 {
            actual += 70;
            det.observe(actual + 5, actual);
        }
        assert!(!det.latched());
        let mut error_offset = 5u32;
        for _ in 0..300 {
            actual += 70;
            error_offset += 1;
            det.observe(actual + error_offset, actual);
        }
        assert!(!det.latched());
        assert_eq!(det.trigger_path(), StallTriggerPath::None);
    }

    #[test]
    fn peak_margin_delta_reports_at_least_the_true_observed_gap() {
        let mut det = StallDetector::new(
            /* ceiling */ 10_000,
            /* margin */ 10_000,
            /* persistence */ 1,
            COUNTS_PER_REV,
        );
        let mut actual = 0u32;
        for _ in 0..MOTION_WARMUP_TICKS + 5 {
            actual += 70;
            det.observe(actual + 5, actual);
        }
        det.observe(actual + 1000, actual);
        assert!(det.peak_margin_delta_units() >= 990);
    }
}
