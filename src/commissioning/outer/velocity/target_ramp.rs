//! Linear target ramp that spaces velocity targets one per millisecond tick.

use super::sweep_plan::VELOCITY_SWEEP_SAMPLE_PERIOD_US;
use crate::autotune::CommissionError;
use crate::commissioning::schedule::next_grid_deadline;

/// Multiplier applied to the configured acceleration limit when shaping a ramp.
pub(crate) const RAMP_ACCEL_MULTIPLIER: u32 = 2;

/// Ramp slew rate in milli-RPM per millisecond tick. Never zero.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) struct RampRate(u64);

impl RampRate {
    /// Builds the rate from an acceleration limit in milli-rev/s^2.
    ///
    /// Returns `None` when the scaled rate rounds to zero.
    pub(crate) fn from_accel(max_accel_mrev_s2: u32) -> Option<RampRate> {
        let rate = u64::from(max_accel_mrev_s2) * u64::from(RAMP_ACCEL_MULTIPLIER) * 60 / 1000;
        (rate != 0).then_some(RampRate(rate))
    }

    /// Rebuilds a rate from its stored milli-RPM per millisecond value.
    ///
    /// Returns `None` for zero.
    pub(crate) fn from_milli_rpm_per_ms(rate: u32) -> Option<RampRate> {
        (rate != 0).then_some(RampRate(u64::from(rate)))
    }

    /// The stored rate in milli-RPM per millisecond.
    pub(crate) fn milli_rpm_per_ms(self) -> u32 {
        u32::try_from(self.0).expect("rates come from u32 inputs scaled by at most 0.12")
    }

    /// Ticks needed to traverse `span_rpm`, rounded up; zero for a zero span.
    pub(crate) fn ticks_for_span_rpm(self, span_rpm: u32) -> u32 {
        let ticks = (u64::from(span_rpm) * 1000).div_ceil(self.0);
        u32::try_from(ticks).unwrap_or(u32::MAX)
    }

    /// Ramp duration in microseconds for `span_rpm`, saturating at `u32::MAX`.
    pub(crate) fn ramp_us_for_span_rpm(self, span_rpm: u32) -> u32 {
        self.ticks_for_span_rpm(span_rpm).saturating_mul(1000)
    }
}

/// Iterator-style generator of per-tick velocity targets from one RPM to another.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) struct TargetRamp {
    from_rpm: i32,
    to_rpm: i32,
    rate: RampRate,
    tick: u32,
}

impl TargetRamp {
    /// Creates a ramp from `from_rpm` to `to_rpm` at `rate`.
    pub(crate) fn new(from_rpm: i32, to_rpm: i32, rate: RampRate) -> Self {
        Self {
            from_rpm,
            to_rpm,
            rate,
            tick: 0,
        }
    }

    fn span_rpm(&self) -> u32 {
        self.to_rpm.abs_diff(self.from_rpm)
    }

    /// Total number of targets the ramp produces.
    pub(crate) fn ticks(&self) -> u32 {
        self.rate.ticks_for_span_rpm(self.span_rpm())
    }

    /// Advances one tick and returns that tick's target, or `None` once all targets were produced.
    pub(crate) fn next(&mut self) -> Option<i32> {
        let ticks = self.ticks();
        if self.tick >= ticks {
            return None;
        }
        self.tick += 1;
        if self.tick == ticks {
            return Some(self.to_rpm);
        }
        let span_milli = i64::from(self.span_rpm()) * 1000;
        let travelled = i64::try_from(u64::from(self.tick) * self.rate.0)
            .unwrap_or(i64::MAX)
            .min(span_milli);
        let delta = travelled / 1000;
        let signed = if self.to_rpm >= self.from_rpm {
            delta
        } else {
            -delta
        };
        Some(
            i32::try_from(i64::from(self.from_rpm) + signed)
                .expect("the target lies between two i32 endpoints"),
        )
    }

    /// The target the ramp starts from.
    pub(crate) fn origin_rpm(&self) -> i32 {
        self.from_rpm
    }

    /// Whether at least one tick has been taken.
    pub(crate) fn is_started(&self) -> bool {
        self.tick > 0
    }

    /// Whether every target has been produced; true from construction for a zero-tick ramp.
    pub(crate) fn is_complete(&self) -> bool {
        self.tick >= self.ticks()
    }

    /// Whether the next call to [`TargetRamp::next`] completes the ramp.
    pub(crate) fn next_is_final(&self) -> bool {
        self.tick.saturating_add(1) >= self.ticks()
    }
}

/// Schedules the ramp's next 1 ms tick: one period after `now_us` on its first tick, else the
/// next grid point after `now_us`.
pub(crate) fn next_ramp_tick_us(
    next_tick_us: &mut u64,
    first: bool,
    now_us: u64,
) -> Result<u64, CommissionError> {
    let period_us = u64::from(VELOCITY_SWEEP_SAMPLE_PERIOD_US);
    *next_tick_us = if first {
        now_us
            .checked_add(period_us)
            .ok_or(CommissionError::InvalidSchedule)?
    } else {
        next_grid_deadline(*next_tick_us, period_us, now_us)?
    };
    Ok(*next_tick_us)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rate_from_accel_converts_mrev_s2_to_milli_rpm_per_ms() {
        assert_eq!(RampRate::from_accel(500_000).unwrap().0, 60_000);
        assert_eq!(RampRate::from_accel(8), None);
        assert_eq!(RampRate::from_accel(9).unwrap().0, 1);
        assert_eq!(
            RampRate::from_accel(u32::MAX).unwrap().0,
            u64::from(u32::MAX) * 6 / 50
        );
    }

    #[test]
    fn rebuilt_rate_rejects_zero_and_round_trips() {
        assert_eq!(RampRate::from_milli_rpm_per_ms(0), None);
        let rate = RampRate::from_accel(500_000).unwrap();
        assert_eq!(
            RampRate::from_milli_rpm_per_ms(rate.milli_rpm_per_ms()),
            Some(rate)
        );
    }

    #[test]
    fn ticks_round_up_and_a_zero_span_takes_no_ticks() {
        let rate = RampRate::from_accel(500_000).unwrap();
        assert_eq!(rate.ticks_for_span_rpm(274), 5);
        assert_eq!(rate.ramp_us_for_span_rpm(274), 5_000);
        assert_eq!(rate.ticks_for_span_rpm(300), 5);
        assert_eq!(rate.ticks_for_span_rpm(301), 6);
        assert_eq!(rate.ticks_for_span_rpm(0), 0);
        assert_eq!(
            RampRate::from_accel(9).unwrap().ticks_for_span_rpm(274),
            274_000
        );
    }

    #[test]
    fn reversal_sequence_changes_sign_between_ticks_and_ends_exactly_on_target() {
        let mut ramp = TargetRamp::new(153, -153, RampRate::from_accel(500_000).unwrap());
        assert_eq!(ramp.ticks(), 6);
        let seq: heapless::Vec<i32, 8> = core::iter::from_fn(|| ramp.next()).collect();
        assert_eq!(seq.as_slice(), &[93, 33, -27, -87, -147, -153]);
        assert!(ramp.is_complete());
    }

    #[test]
    fn step_up_from_rest_is_monotone_and_small_spans_take_one_tick() {
        let rate = RampRate::from_accel(500_000).unwrap();
        let mut up = TargetRamp::new(0, 274, rate);
        let seq: heapless::Vec<i32, 8> = core::iter::from_fn(|| up.next()).collect();
        assert_eq!(seq.as_slice(), &[60, 120, 180, 240, 274]);
        let mut small = TargetRamp::new(0, 60, rate);
        assert_eq!(small.ticks(), 1);
        assert_eq!(small.next(), Some(60));
        assert_eq!(small.next(), None);
        let none = TargetRamp::new(100, 100, rate);
        assert_eq!(none.ticks(), 0);
        assert!(none.is_complete());
    }
}
