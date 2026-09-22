//! Following-error stall detector for sensorless homing.

/// Absolute following error in TMC position units.
///
/// Both registers are 32-bit two's-complement positions; the difference is
/// taken wrapping so a revolution boundary between them does not blow up.
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

/// A following-error delta a tick must exceed to count as "real motion" for
/// baseline purposes. Firmware constant, not host-configurable (see spec's
/// Config surface). Position units (65536 per revolution).
///
/// Sized against two bounds, not picked as a round number:
///
/// - **Above realistic encoder jitter.** The ABN encoder is 4000 counts/rev
///   (`docs/notes/openffboard-hardware.md`), so one count is
///   `65536 / 4000 ~= 16.384` position units. The project's own
///   "+/-2 counts jitter" precedent for a stationary axis
///   (`docs/notes/decisions.md`, 2026-04-05) is therefore `~=33` units of
///   realistic settling noise, with observed excursions up to `~=49` units
///   (3 counts). A stalled axis that is genuinely not moving can still show
///   deltas in that range every tick; the floor must sit above it or a
///   stall never stops "confirming motion" and the EMA baseline keeps
///   chasing the rising following error instead of staying gated.
/// - **Far below real per-tick motion at the slowest homing speed in use.**
///   The homing maintenance tick runs on the shared 1 ms maintenance grid
///   (`scheduler::MAINTENANCE_PERIOD_US`), so 1 tick = 1 ms. At this
///   printer's slowest homing speed, 100 mm/s, and its `rotation_distance`
///   of 40 mm/rev (`POSITION_UNITS_PER_REV = 65536` in `config.py`), one
///   tick of real cruise motion is `0.1 mm * 65536 / 40 ~= 164` position
///   units -- comfortably above this floor, so genuine motion still
///   registers on essentially every tick and the baseline still warms up
///   promptly.
///
/// 64 sits well clear of both bounds: ~1.9x the ~33-unit documented jitter
/// precedent, ~1.3x the ~49-unit observed excursion, and about 2.5x below
/// per-tick motion at the slowest homing speed (100 mm/s; 175 mm/s gives
/// even more headroom).
const MOTION_NOISE_FLOOR_UNITS: u32 = 64;

/// Consecutive motion-confirmed ticks required before the baseline is
/// trusted. Firmware constant; see spec's residual-risk section for why
/// this needs hardware tuning, not just a reasonable-looking default.
const MOTION_WARMUP_TICKS: u16 = 8;

/// EMA weight for the baseline update, as a right-shift on the fixed-point
/// accumulator: new baseline = old + (error - old) >> EMA_SHIFT. Firmware
/// constant.
const EMA_SHIFT: u32 = 3;

/// Extra fractional bits kept in the baseline accumulator beyond
/// `EMA_SHIFT`. Without them, a plain-integer EMA truncates to a zero step
/// whenever `|error - baseline| < 2^EMA_SHIFT`, so any error smaller than
/// that (a realistic case at this printer's mm-to-position-unit scale)
/// would never move the baseline off its initial value. Every comparison
/// against the baseline (the margin check, and the reported margin-delta
/// peak) must stay in this scaled representation too -- downscaling the
/// baseline to real units before comparing reintroduces the exact same
/// truncation dead zone one level up, at the comparison rather than the
/// accumulation. Only the final `peak_margin_delta_units()` accessor,
/// reporting to the host for telemetry, downscales, and does so once.
const BASELINE_FRAC_BITS: u32 = 8;

/// Consecutive-tick following-error detector with a sticky latch.
///
/// Runs two independent checks per tick: an unconditional absolute
/// `ceiling` (always active, catches a stall present before any baseline
/// exists) and a `margin` check relative to a motion-gated EMA baseline
/// (the primary mechanism, only active once warmed up). Either latches the
/// detector; a same-tick tie favors `Ceiling`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct StallDetector {
    ceiling_units: u32,
    margin_units: u32,
    persistence: u8,
    ceiling_consecutive: u8,
    margin_consecutive: u8,
    ticks: u32,
    peak_error_units: u32,
    /// Largest scaled margin delta observed, in the same `2^BASELINE_FRAC_BITS`
    /// fixed-point representation as `baseline_scaled`. Only downscaled to
    /// real units in the public `peak_margin_delta_units()` accessor.
    peak_margin_delta_scaled: u64,
    trigger_tick: u32,
    trigger_path: StallTriggerPath,
    latched: bool,
    prev_actual: u32,
    /// Baseline, scaled by `2^BASELINE_FRAC_BITS`. Never downscaled for a
    /// comparison -- only the public accessor downscales, once, for
    /// telemetry.
    baseline_scaled: u64,
    motion_warmup_ticks: u16,
}

impl StallDetector {
    /// Create an armed detector. `persistence` is clamped to at least 1.
    pub const fn new(ceiling_units: u32, margin_units: u32, persistence: u8) -> Self {
        Self {
            ceiling_units,
            margin_units,
            persistence: if persistence == 0 { 1 } else { persistence },
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

        // Motion evidence and baseline update, before either check so a
        // margin breach on this same tick already sees the fresh baseline.
        let motion_confirmed = !first_tick
            && (actual.wrapping_sub(self.prev_actual) as i32).unsigned_abs()
                >= MOTION_NOISE_FLOOR_UNITS;
        self.prev_actual = actual;
        let mut baseline_valid = self.motion_warmup_ticks >= MOTION_WARMUP_TICKS;
        let error_scaled = (error as u64) << BASELINE_FRAC_BITS;
        if motion_confirmed {
            if self.motion_warmup_ticks < MOTION_WARMUP_TICKS {
                self.motion_warmup_ticks += 1;
                baseline_valid = self.motion_warmup_ticks >= MOTION_WARMUP_TICKS;
            }
            // Move the fixed-point baseline a 1/2^EMA_SHIFT fraction of the
            // way from its old value toward the current error, entirely in
            // scaled units so the step never truncates to zero for a small
            // real error. i64 avoids any overflow concern at these
            // magnitudes; clamp at 0 since error and baseline are both
            // conceptually unsigned.
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

        // Margin comparison stays in scaled space throughout -- comparing a
        // downscaled baseline against real-unit margin_units would
        // reintroduce the truncation dead zone the fixed-point accumulator
        // exists to avoid.
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
            // Tie rule: Ceiling wins.
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

    /// Largest `error - baseline` observed since arming, while the baseline
    /// was valid, downscaled from the internal fixed-point representation
    /// to real position units, **rounded up** (ceiling division). 0 if the
    /// baseline never warmed up. This is the quantity the hardware
    /// validation campaign's margin-headroom acceptance criterion measures
    /// directly, against a strict `<= 0.8 * margin_units` bound -- rounding
    /// down here could report a value one fractional unit under a real
    /// violation, letting an unsafe configuration pass. Rounding up is
    /// conservative in the direction that matters: it can only make the
    /// gate harder to pass, never easier. The downscaling happens only
    /// here, once, for telemetry -- the latch decision itself never
    /// downscales (see `BASELINE_FRAC_BITS`).
    pub const fn peak_margin_delta_units(&self) -> u32 {
        // Standard ceiling-division-by-power-of-2: adding (2^n - 1) before
        // the shift rounds any non-negative value up to the next unit.
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

    #[test]
    fn following_error_wraps_across_revolution_boundary() {
        assert_eq!(following_error_units(10, u32::MAX - 5), 16);
        assert_eq!(following_error_units(u32::MAX - 5, 10), 16);
        assert_eq!(following_error_units(0x8000_0000, 0), 0x8000_0000);
    }

    #[test]
    fn ceiling_check_latches_on_gross_error_before_baseline_warms_up() {
        // Jammed-from-tick-1: actual never advances, error grows past the
        // ceiling well before MOTION_WARMUP_TICKS worth of real motion could
        // ever be observed, so the margin check cannot be the one that fires.
        let mut det = StallDetector::new(
            /* ceiling */ 100, /* margin */ 10, /* persistence */ 2,
        );
        assert!(!det.observe(50, 0));
        assert!(!det.observe(100, 0));
        assert!(det.observe(150, 0));
        assert!(det.latched());
        assert_eq!(det.trigger_path(), StallTriggerPath::Ceiling);
    }

    #[test]
    fn ceiling_check_latches_under_encoder_jitter_within_the_new_noise_floor() {
        // Companion to `ceiling_check_latches_on_gross_error_before_baseline_
        // warms_up` above, but with `actual` jittering by 50 units a tick
        // (below MOTION_NOISE_FLOOR_UNITS's 64, above the old 4-unit floor,
        // and above the documented +/-2-count / ~33-unit encoder jitter
        // precedent) instead of holding perfectly still. A real stalled
        // axis's encoder reading jitters like this; it must not look like
        // "real motion" to the baseline, so only the ceiling backstop may
        // latch here, exactly as it does in the zero-jitter case.
        let mut det = StallDetector::new(
            /* ceiling */ 200, /* margin */ 10, /* persistence */ 2,
        );
        assert!(!det.observe(100, 0));
        assert!(!det.observe(150, 50));
        assert!(!det.observe(205, 0));
        assert!(det.observe(250, 50));
        assert!(det.latched());
        assert_eq!(det.trigger_path(), StallTriggerPath::Ceiling);
        // The baseline never warmed up: every tick-to-tick jitter delta (50
        // units) stayed under MOTION_NOISE_FLOOR_UNITS, so motion was never
        // confirmed and the margin path could not have been the one that
        // fired.
        assert_eq!(det.peak_margin_delta_units(), 0);
    }

    #[test]
    fn margin_check_stays_masked_while_axis_is_jammed_from_tick_1() {
        // Companion to the ceiling test above: the delta check never
        // activates for a stall present since tick 1, because the baseline
        // only updates on motion-confirmed ticks and this axis never moves.
        // A large ceiling means only the margin path could latch here, and
        // it must not.
        let mut det = StallDetector::new(
            /* ceiling */ 10_000, /* margin */ 10, /* persistence */ 2,
        );
        for _ in 0..50 {
            assert!(!det.observe(1000, 0));
        }
        assert!(!det.latched());
    }

    #[test]
    fn margin_check_latches_once_baseline_warms_up_and_error_grows() {
        // Real motion (actual advancing tick over tick) feeds the baseline;
        // once warmed up, a sustained rise above baseline by margin_units
        // latches via the margin path, well under a loose ceiling.
        let mut det = StallDetector::new(
            /* ceiling */ 10_000, /* margin */ 20, /* persistence */ 2,
        );
        let mut actual = 0u32;
        for _ in 0..MOTION_WARMUP_TICKS + 2 {
            actual += 70;
            assert!(!det.observe(actual + 5, actual));
        }
        assert!(!det.latched());
        // Contact: actual stops advancing, error above the settled baseline
        // (~5) grows past margin_units (20).
        assert!(!det.observe(actual + 30, actual));
        assert!(det.observe(actual + 30, actual));
        assert!(det.latched());
        assert_eq!(det.trigger_path(), StallTriggerPath::Margin);
    }

    #[test]
    fn normal_cruise_within_ceiling_and_margin_never_latches() {
        let mut det = StallDetector::new(
            /* ceiling */ 10_000, /* margin */ 20, /* persistence */ 2,
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
        // A ceiling low enough to coincide with an already-warmed-up
        // margin breach on the same tick: the tie rule picks Ceiling.
        let mut det = StallDetector::new(
            /* ceiling */ 40, /* margin */ 5, /* persistence */ 1,
        );
        let mut actual = 0u32;
        for _ in 0..MOTION_WARMUP_TICKS + 1 {
            actual += 70;
            det.observe(actual + 2, actual);
        }
        assert!(!det.latched());
        // Single tick: error 40 clears both the ceiling (>=40) and, since
        // baseline sits near 2, the margin (40 - 2 = 38 >= 5) at once.
        assert!(det.observe(actual + 40, actual));
        assert_eq!(det.trigger_path(), StallTriggerPath::Ceiling);
    }

    #[test]
    fn gross_failure_latches_via_ceiling_even_after_baseline_valid() {
        let mut det = StallDetector::new(
            /* ceiling */ 500, /* margin */ 20, /* persistence */ 1,
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
        let mut det = StallDetector::new(10, 5, 1);
        assert!(det.observe(500, 0));
        assert!(det.observe(0, 0));
        assert!(det.latched());
        assert_eq!(det.peak_error_units(), 500);
    }

    #[test]
    fn zero_persistence_behaves_as_one() {
        let mut det = StallDetector::new(10, 5, 0);
        assert!(det.observe(10, 0));
    }

    #[test]
    fn unlatched_detector_reports_no_trigger_path() {
        let det = StallDetector::new(10_000, 20, 1);
        assert_eq!(det.trigger_path(), StallTriggerPath::None);
    }

    #[test]
    fn baseline_converges_for_a_sub_shift_error_instead_of_sticking_at_zero() {
        // A naive integer EMA (baseline += (error - baseline) >> EMA_SHIFT
        // computed directly in real units) truncates to a zero step whenever
        // |error - baseline| < 2^EMA_SHIFT, so a small but real cruise error
        // would never move the baseline off zero. The fixed-point
        // accumulator must still converge for an error smaller than
        // 2^EMA_SHIFT.
        let mut det = StallDetector::new(
            /* ceiling */ 10_000, /* margin */ 1, /* persistence */ 1,
        );
        let mut actual = 0u32;
        for _ in 0..MOTION_WARMUP_TICKS + 40 {
            actual += 70;
            det.observe(actual + 1, actual);
        }
        // If the baseline were stuck at 0, error(1) - baseline(0) = 1 >=
        // margin_units(1) would have latched by persistence 1 on the very
        // first post-warmup tick. It must not: the baseline has to have
        // converged close enough to 1 that the margin never breaches.
        assert!(!det.latched());
    }

    #[test]
    fn slow_creep_under_the_ceiling_stays_masked_by_the_tracking_baseline() {
        // Documents the spec's accepted residual-risk limitation: a stall
        // that develops slower than the EMA can track (partial binding,
        // not a hard dead stop) is chased by the baseline the same way a
        // jammed-from-tick-1 stall would be -- this is expected, not a bug,
        // and this test exists so a future change to the EMA constants
        // cannot silently alter that property without a test noticing.
        let mut det = StallDetector::new(
            /* ceiling */ 10_000, /* margin */ 20, /* persistence */ 2,
        );
        let mut actual = 0u32;
        for _ in 0..MOTION_WARMUP_TICKS + 5 {
            actual += 70;
            det.observe(actual + 5, actual);
        }
        assert!(!det.latched());
        // Error grows by 1 unit per tick from here -- far slower than the
        // margin_units(20) threshold, and motion continues (actual still
        // advances), so the baseline keeps tracking it. Total error growth
        // over 300 ticks (300 units) vastly exceeds margin_units, yet the
        // detector must still never latch.
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
        // Regression guard for the rounding direction: peak_margin_delta_units()
        // must never report less than the real gap it measured -- rounding
        // toward zero could let a hardware validation run pass a
        // margin-headroom threshold it actually violated.
        let mut det = StallDetector::new(
            /* ceiling */ 10_000, /* margin */ 10_000, /* persistence */ 1,
        );
        let mut actual = 0u32;
        for _ in 0..MOTION_WARMUP_TICKS + 5 {
            actual += 70;
            det.observe(actual + 5, actual);
        }
        // One tick with a known, large jump while actual holds still: the
        // baseline (converged near 5) cannot have caught up, so the true
        // gap is close to 995 (1000 - ~5). Neither check latches (margin
        // and ceiling are both 10_000), so this only exercises peak
        // tracking, not the latch path.
        det.observe(actual + 1000, actual);
        assert!(det.peak_margin_delta_units() >= 990);
    }
}
