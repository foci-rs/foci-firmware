//! Trapezoidal reversal-stroke planner for the step-queue stimulus.

/// Ramp runs per acceleration or deceleration phase. Boundaries sit at
/// equal velocity increments so the first run, where the interval changes
/// fastest, stays within the `i16` per-step `add`.
#[allow(dead_code)]
pub(crate) const RAMP_SEGMENTS: u64 = 16;

/// Smallest cruise interval accepted: below this the one-tick rounding of
/// `clock / speed` alone exceeds the two-percent cruise-speed bar.
#[allow(dead_code)]
pub(crate) const MIN_CRUISE_INTERVAL_TICKS: u64 = 50;

/// One reversal stroke as the phase requests it, in the host budget's units.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) struct StrokeRequest {
    /// Requested cruise speed, mrev/s.
    pub speed_mrev_s: u32,
    /// Ramp acceleration, mrev/s^2.
    pub accel_mrev_s2: u32,
    /// Distance one leg travels, mrev.
    pub excursion_mrev: u32,
    /// Physical steps per mechanical revolution (`65536 / STEP_WIDTH`).
    pub steps_per_rev: u32,
    /// Step timer frequency, Hz.
    pub step_timer_hz: u32,
    /// Shortest cruise the phase accepts, us.
    pub min_cruise_us: u32,
}

/// Solved geometry of one leg.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) struct LegPlan {
    /// Cruise speed actually planned, mrev/s; below the request when the
    /// excursion could not carry the minimum cruise.
    pub speed_mrev_s: u32,
    pub steps_per_leg: u32,
    pub ramp_steps: u32,
    pub cruise_steps: u32,
    pub ramp_us: u32,
    pub cruise_us: u32,
    pub cruise_interval_ticks: u32,
}

/// Why a stroke could not be planned.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) enum StrokePlanError {
    ZeroInput,
    NoCruise,
    IntervalTooCoarse,
    SegmentOverflow,
}

#[allow(dead_code)]
fn steps_per_second(speed_mrev_s: u64, steps_per_rev: u64) -> u64 {
    speed_mrev_s * steps_per_rev / 1_000
}

/// The discrete leg at `speed`, or `None` when its ramps plus the minimum
/// cruise (in emitted ticks) do not fit the excursion. `steps_per_leg` and
/// `accel_steps` are precomputed by the caller.
#[allow(dead_code)]
fn leg_at(
    request: &StrokeRequest,
    steps_per_leg: u64,
    accel_steps: u64,
    speed_mrev_s: u64,
) -> Option<(u64, u64, u64)> {
    let velocity_steps = steps_per_second(speed_mrev_s, u64::from(request.steps_per_rev));
    if velocity_steps == 0 {
        return None;
    }
    let ramp_steps = velocity_steps * velocity_steps / (2 * accel_steps);
    let cruise_interval = u64::from(request.step_timer_hz) / velocity_steps;
    if cruise_interval == 0 {
        return None;
    }
    let min_cruise_ticks =
        u64::from(request.min_cruise_us) * u64::from(request.step_timer_hz) / 1_000_000;
    let min_cruise_steps = min_cruise_ticks.div_ceil(cruise_interval).max(1);
    if 2 * ramp_steps + min_cruise_steps > steps_per_leg {
        return None;
    }
    Some((ramp_steps, steps_per_leg - 2 * ramp_steps, cruise_interval))
}

/// Whether a leg at exactly `speed_mrev_s` keeps the minimum cruise.
#[allow(dead_code)]
pub(crate) fn leg_fits(request: &StrokeRequest, speed_mrev_s: u32) -> bool {
    let steps_per_rev = u64::from(request.steps_per_rev);
    let steps_per_leg = u64::from(request.excursion_mrev) * steps_per_rev / 1_000;
    let accel_steps = steps_per_second(u64::from(request.accel_mrev_s2), steps_per_rev);
    accel_steps > 0
        && leg_at(request, steps_per_leg, accel_steps, u64::from(speed_mrev_s)).is_some()
}

#[allow(dead_code)]
pub(crate) fn plan_leg(request: &StrokeRequest) -> Result<LegPlan, StrokePlanError> {
    if request.speed_mrev_s == 0
        || request.accel_mrev_s2 == 0
        || request.excursion_mrev == 0
        || request.steps_per_rev == 0
        || request.step_timer_hz == 0
    {
        return Err(StrokePlanError::ZeroInput);
    }
    let steps_per_rev = u64::from(request.steps_per_rev);
    let steps_per_leg = u64::from(request.excursion_mrev) * steps_per_rev / 1_000;
    let accel_steps = steps_per_second(u64::from(request.accel_mrev_s2), steps_per_rev);
    if steps_per_leg == 0 || accel_steps == 0 {
        return Err(StrokePlanError::NoCruise);
    }
    let requested = u64::from(request.speed_mrev_s);
    let requested_velocity = steps_per_second(requested, steps_per_rev);
    if requested_velocity > 0
        && u64::from(request.step_timer_hz) / requested_velocity < MIN_CRUISE_INTERVAL_TICKS
    {
        return Err(StrokePlanError::IntervalTooCoarse);
    }
    // Smallest speed that moves at all: one step per second.
    let floor = 1_000_u64.div_ceil(steps_per_rev).max(1);
    let fits = |speed: u64| leg_at(request, steps_per_leg, accel_steps, speed).is_some();
    if !fits(floor) {
        return Err(StrokePlanError::NoCruise);
    }
    let speed = if fits(requested) {
        requested
    } else {
        // Feasibility is monotone in speed; find the largest fitting value.
        let (mut lo, mut hi) = (floor, requested);
        while hi - lo > 1 {
            let mid = lo + (hi - lo) / 2;
            if fits(mid) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        lo
    };
    let (ramp_steps, cruise_steps, cruise_interval) =
        leg_at(request, steps_per_leg, accel_steps, speed).expect("fits was checked");
    let ramp_us = speed * 1_000_000 / u64::from(request.accel_mrev_s2);
    let cruise_ticks_wide = u128::from(cruise_steps) * u128::from(cruise_interval);
    let cruise_us_wide = cruise_ticks_wide * 1_000_000 / u128::from(request.step_timer_hz);
    Ok(LegPlan {
        speed_mrev_s: narrow(speed)?,
        steps_per_leg: narrow(steps_per_leg)?,
        ramp_steps: narrow(ramp_steps)?,
        cruise_steps: narrow(cruise_steps)?,
        ramp_us: narrow(ramp_us)?,
        cruise_us: narrow(cruise_us_wide.try_into().unwrap_or(u64::MAX))?,
        cruise_interval_ticks: narrow(cruise_interval)?,
    })
}

#[allow(dead_code)]
fn narrow(value: u64) -> Result<u32, StrokePlanError> {
    u32::try_from(value).map_err(|_| StrokePlanError::SegmentOverflow)
}

/// Both legs' runs plus `u16` splitting headroom.
pub(crate) const MAX_STROKE_SEGMENTS: usize = 160;

/// One constant-`add` run of steps, in the format the step queue executes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) struct StrokeSegment {
    /// DIR level this run must be queued under.
    pub forward: bool,
    /// Initial interval, timer ticks.
    pub interval: u32,
    /// Steps in this run.
    pub count: u16,
    /// Interval change per step.
    pub add: i16,
}

impl StrokeSegment {
    /// The queue-format move; direction is applied by the consumer through
    /// `set_next_step_dir` before enqueueing, exactly as the host path does.
    #[allow(dead_code)]
    pub(crate) fn step_move(&self) -> foci_klipper::stepper::StepMove {
        foci_klipper::stepper::StepMove::new(self.interval, self.count, self.add)
    }
}

/// A planned reversal stroke: the solved leg plus every run of both legs,
/// forward leg first, with the exact tick boundaries of one leg.
#[allow(dead_code)]
#[derive(Debug, Clone, PartialEq, Eq, defmt::Format)]
pub(crate) struct StrokeProfile {
    pub leg: LegPlan,
    pub segments: heapless::Vec<StrokeSegment, MAX_STROKE_SEGMENTS>,
    pub accel_ticks: u64,
    pub cruise_ticks: u64,
    pub leg_ticks: u64,
    pub step_timer_hz: u32,
    pub accel_run_count: u8,
    pub cruise_run_count: u8,
}

#[allow(dead_code)]
impl StrokeProfile {
    fn ticks_to_us(&self, ticks: u64) -> u32 {
        u32::try_from(ticks * 1_000_000 / u64::from(self.step_timer_hz)).unwrap_or(u32::MAX)
    }

    /// End of the acceleration ramp within a leg, us.
    pub(crate) fn ramp_end_us(&self) -> u32 {
        self.ticks_to_us(self.accel_ticks)
    }

    /// End of the cruise within a leg, us.
    pub(crate) fn cruise_end_us(&self) -> u32 {
        self.ticks_to_us(self.accel_ticks + self.cruise_ticks)
    }

    /// Planned duration of one leg, us.
    pub(crate) fn leg_us(&self) -> u32 {
        self.ticks_to_us(self.leg_ticks)
    }

    /// Planned duration of the whole out-and-back stroke, us.
    pub(crate) fn stroke_us(&self) -> u32 {
        self.ticks_to_us(self.leg_ticks.saturating_mul(2))
    }
}

/// Exact timer ticks one `(interval, count, add)` run occupies, as the
/// step ISR walks it: the intervals form an arithmetic series.
#[allow(dead_code)]
pub(crate) fn run_ticks(interval: u32, count: u16, add: i16) -> u64 {
    let n = i128::from(count);
    let total = n * i128::from(interval) + i128::from(add) * n * (n - 1) / 2;
    u64::try_from(total.max(0)).unwrap_or(u64::MAX)
}

/// One leg's emitted acceleration runs (ramp runs, their `u16` splits,
/// and the join run), plus the cruise step count after the join took its
/// steps.
#[allow(dead_code)]
type RampRuns = heapless::Vec<StrokeSegment, { 2 * RAMP_SEGMENTS as usize + 18 }>;

/// One leg emitted at exactly `speed_mrev_s`: `Ok(None)` when the speed
/// does not fit (before or after the join) or exceeds what was requested,
/// `Err` on a representation overflow.
#[allow(dead_code)]
fn emit_leg(
    request: &StrokeRequest,
    speed_mrev_s: u32,
) -> Result<Option<(LegPlan, RampRuns, u64)>, StrokePlanError> {
    // `leg_fits`/`plan_leg` take `speed_mrev_s` as an independent target and
    // do not know it must stay within the phase's own request; enforce that
    // cap here so `stroke_fits` (and the search built on it) never reports a
    // speed above what the caller asked for as fitting.
    if speed_mrev_s > request.speed_mrev_s || !leg_fits(request, speed_mrev_s) {
        return Ok(None);
    }
    let leg = plan_leg(&StrokeRequest {
        speed_mrev_s,
        ..*request
    })?;
    debug_assert_eq!(leg.speed_mrev_s, speed_mrev_s);
    let steps_per_rev = u64::from(request.steps_per_rev);
    let accel_steps = steps_per_second(u64::from(request.accel_mrev_s2), steps_per_rev);
    let clock = u64::from(request.step_timer_hz);
    let (runs, cruise_steps) = acceleration_runs(&leg, accel_steps, clock)?;
    let min_cruise_ticks = u64::from(request.min_cruise_us) * clock / 1_000_000;
    if cruise_steps * u64::from(leg.cruise_interval_ticks) < min_cruise_ticks {
        return Ok(None);
    }
    Ok(Some((leg, runs, cruise_steps)))
}

/// Whether a full stroke at exactly `speed_mrev_s` keeps the minimum
/// cruise after the join has taken its steps.
#[allow(dead_code)]
pub(crate) fn stroke_fits(request: &StrokeRequest, speed_mrev_s: u32) -> bool {
    matches!(emit_leg(request, speed_mrev_s), Ok(Some(_)))
}

#[allow(dead_code)]
pub(crate) fn plan_stroke(request: &StrokeRequest) -> Result<StrokeProfile, StrokePlanError> {
    let upper = plan_leg(request)?.speed_mrev_s;
    let floor = u32::try_from(1_000_u64.div_ceil(u64::from(request.steps_per_rev)).max(1))
        .map_err(|_| StrokePlanError::SegmentOverflow)?;
    let speed = if stroke_fits(request, upper) {
        upper
    } else {
        if !stroke_fits(request, floor) {
            return Err(StrokePlanError::NoCruise);
        }
        let (mut lo, mut hi) = (floor, upper);
        while hi - lo > 1 {
            let mid = lo + (hi - lo) / 2;
            if stroke_fits(request, mid) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        lo
    };
    let (mut leg, accel_runs, cruise_steps) =
        emit_leg(request, speed)?.expect("stroke_fits was checked");
    let accel_ticks: u64 = accel_runs
        .iter()
        .map(|s| run_ticks(s.interval, s.count, s.add))
        .sum();
    // Widen before multiplying: both factors can approach u32::MAX, whose
    // product can approach u64::MAX.
    let cruise_ticks_wide = u128::from(leg.cruise_interval_ticks) * u128::from(cruise_steps);
    let cruise_ticks =
        u64::try_from(cruise_ticks_wide).map_err(|_| StrokePlanError::SegmentOverflow)?;
    let leg_ticks = accel_ticks
        .checked_mul(2)
        .and_then(|doubled| doubled.checked_add(cruise_ticks))
        .ok_or(StrokePlanError::SegmentOverflow)?;
    let ramp_steps: u64 = accel_runs.iter().map(|s| u64::from(s.count)).sum();

    let mut segments = heapless::Vec::new();
    let mut cruise_run_count = 0_u8;
    for forward in [true, false] {
        for run in accel_runs.iter() {
            segments
                .push(StrokeSegment { forward, ..*run })
                .map_err(|_| StrokePlanError::SegmentOverflow)?;
        }
        let before = segments.len();
        push_run(
            &mut segments,
            forward,
            u64::from(leg.cruise_interval_ticks),
            cruise_steps,
            0,
        )?;
        cruise_run_count =
            u8::try_from(segments.len() - before).map_err(|_| StrokePlanError::SegmentOverflow)?;
        for run in accel_runs.iter().rev() {
            let last_emitted =
                i64::from(run.interval) + i64::from(run.add) * i64::from(run.count - 1);
            let interval =
                u32::try_from(last_emitted).map_err(|_| StrokePlanError::SegmentOverflow)?;
            segments
                .push(StrokeSegment {
                    forward,
                    interval,
                    count: run.count,
                    add: -run.add,
                })
                .map_err(|_| StrokePlanError::SegmentOverflow)?;
        }
    }
    leg.ramp_steps = narrow(ramp_steps)?;
    leg.cruise_steps = narrow(cruise_steps)?;
    let profile = StrokeProfile {
        leg,
        segments,
        accel_ticks,
        cruise_ticks,
        leg_ticks,
        step_timer_hz: request.step_timer_hz,
        accel_run_count: u8::try_from(accel_runs.len())
            .map_err(|_| StrokePlanError::SegmentOverflow)?,
        cruise_run_count,
    };
    leg.ramp_us = profile.ramp_end_us();
    leg.cruise_us = profile.cruise_end_us() - profile.ramp_end_us();
    Ok(StrokeProfile { leg, ..profile })
}

/// Step index at the start of ramp run `k`: equal velocity increments,
/// since velocity grows with the square root of distance.
#[allow(dead_code)]
fn ramp_boundary(ramp_steps: u64, k: u64) -> u64 {
    ramp_steps * k * k / (RAMP_SEGMENTS * RAMP_SEGMENTS)
}

/// Exact interval at step `s` of a ramp from rest: `clock / sqrt(2 a s)`,
/// with step 0 evaluated at step 1 so the first interval is finite.
#[allow(dead_code)]
fn ramp_interval(clock: u64, accel_steps: u64, step: u64) -> u64 {
    let velocity = (2 * accel_steps * step.max(1)).isqrt().max(1);
    clock / velocity
}

/// Ceiling division for a non-positive numerator: the per-step change that
/// reaches at least `numerator` in `denominator` steps without overshooting
/// below it. Acceleration deltas are always non-positive.
#[allow(dead_code)]
fn ceil_div_toward_zero(numerator: i128, denominator: i128) -> i128 {
    debug_assert!(numerator <= 0 && denominator > 0);
    -((-numerator) / denominator)
}

/// Append one run to `runs`, splitting at the `u16` limit and carrying the
/// interval; returns the interval the schedule reaches after the run.
///
/// The `add` targets the run's own *last emitted step* (index `count - 1`),
/// not the position one step beyond it: dividing by `count` instead would
/// leave the last physical step of every run one `add` short of `target`,
/// which is exact for the queue's own bookkeeping (the next run still
/// starts from `interval + add * count`, per `push_ramp_run`) but drifts
/// the emitted cadence far from the ideal curve on short early runs, where
/// `add` is large relative to `count`. The final partition (`last`) keeps
/// the `count`-divisor instead, so its returned continuation lands at or
/// above the cruise interval exactly as `acceleration_runs`' join step
/// requires; on that run `count` is large enough that the two divisors
/// barely differ.
///
/// Representation is never a reason to refuse a speed: when the ideal
/// per-step change does not fit `i16` (only the first few steps from rest,
/// where the interval halves within a step or two), the partition is
/// emitted as single-step runs at the exact interval instead, so
/// `stroke_fits` stays monotone in speed and the binary search is sound. A
/// single-step partition has no room for an `add` at all, so it always
/// takes `target` directly.
#[allow(dead_code, clippy::too_many_arguments)]
fn push_ramp_partition(
    runs: &mut RampRuns,
    clock: u64,
    accel_steps: u64,
    start: u64,
    count: u64,
    interval: i128,
    target: i128,
    last: bool,
) -> Result<i128, StrokePlanError> {
    if count == 1 {
        let value = u32::try_from(target).map_err(|_| StrokePlanError::SegmentOverflow)?;
        runs.push(StrokeSegment {
            forward: true,
            interval: value,
            count: 1,
            add: 0,
        })
        .map_err(|_| StrokePlanError::SegmentOverflow)?;
        return Ok(target);
    }
    let divisor = if last { count } else { count - 1 };
    let add = ceil_div_toward_zero((target - interval).min(0), i128::from(divisor));
    if let Ok(add) = i16::try_from(add) {
        return push_ramp_run(runs, interval, count, add);
    }
    let mut interval = interval;
    for step in 0..count {
        let exact = i128::from(ramp_interval(clock, accel_steps, start + step + 1)).min(interval);
        runs.push(StrokeSegment {
            forward: true,
            interval: u32::try_from(exact).map_err(|_| StrokePlanError::SegmentOverflow)?,
            count: 1,
            add: 0,
        })
        .map_err(|_| StrokePlanError::SegmentOverflow)?;
        interval = exact;
    }
    Ok(interval)
}

#[allow(dead_code)]
fn push_ramp_run(
    runs: &mut RampRuns,
    mut interval: i128,
    mut count: u64,
    add: i16,
) -> Result<i128, StrokePlanError> {
    while count > 0 {
        let chunk = count.min(u64::from(u16::MAX));
        let start = u32::try_from(interval).map_err(|_| StrokePlanError::SegmentOverflow)?;
        runs.push(StrokeSegment {
            forward: true,
            interval: start,
            count: chunk as u16,
            add,
        })
        .map_err(|_| StrokePlanError::SegmentOverflow)?;
        interval += i128::from(add) * i128::from(chunk);
        count -= chunk;
    }
    Ok(interval)
}

/// The acceleration ramp as continuous constant-`add` runs. Each run starts
/// at the interval the previous run's schedule reaches next and rounds its
/// `add` as the ceiling toward the ideal interval at its own end, so a run
/// never ends faster than ideal and error does not accumulate; a final join
/// run of `diff` steps at `add = -1` lands exactly on the cruise interval,
/// taking those steps from the cruise.
/// Returns the runs and the cruise step count that remains.
#[allow(dead_code)]
fn acceleration_runs(
    leg: &LegPlan,
    accel_steps: u64,
    clock: u64,
) -> Result<(RampRuns, u64), StrokePlanError> {
    let ramp_steps = u64::from(leg.ramp_steps);
    let mut runs = RampRuns::new();
    // A ramp of zero steps starts at cruise speed outright; otherwise from
    // the interval of the first step from rest.
    let mut interval = if ramp_steps == 0 {
        i128::from(leg.cruise_interval_ticks)
    } else {
        i128::from(ramp_interval(clock, accel_steps, 1))
    };
    for k in 0..RAMP_SEGMENTS {
        let start = ramp_boundary(ramp_steps, k);
        let end = ramp_boundary(ramp_steps, k + 1);
        let count = end - start;
        if count == 0 {
            continue;
        }
        let target = i128::from(ramp_interval(clock, accel_steps, end));
        let last = end == ramp_steps;
        interval = push_ramp_partition(
            &mut runs,
            clock,
            accel_steps,
            start,
            count,
            interval,
            target,
            last,
        )?;
    }
    // Every run ended at or above its target, and the last target
    // (`clock / sqrt(2 a ramp_steps)` with `ramp_steps` floored from
    // `v^2 / 2a`) is at or above the cruise interval, so the ramp arrives
    // from above and the join only ever walks down.
    let cruise_interval = i128::from(leg.cruise_interval_ticks);
    debug_assert!(interval >= cruise_interval);
    let join_steps =
        u64::try_from(interval - cruise_interval).map_err(|_| StrokePlanError::SegmentOverflow)?;
    let mut cruise_steps = u64::from(leg.cruise_steps);
    if join_steps > 0 {
        // The deceleration ramp mirrors these runs verbatim, including this
        // join, so the same join length is spent again on the far side of
        // the cruise; both must come out of the cruise for the leg to still
        // total `steps_per_leg`.
        let both_joins = join_steps
            .checked_mul(2)
            .ok_or(StrokePlanError::SegmentOverflow)?;
        if both_joins >= cruise_steps {
            return Err(StrokePlanError::SegmentOverflow);
        }
        interval = push_ramp_run(&mut runs, interval, join_steps, -1)?;
        cruise_steps -= both_joins;
        debug_assert_eq!(interval, cruise_interval);
    }
    Ok((runs, cruise_steps))
}

/// Push one constant-`add` run, split at the `u16` count limit with the
/// interval carried across the split.
#[allow(dead_code)]
fn push_run(
    segments: &mut heapless::Vec<StrokeSegment, MAX_STROKE_SEGMENTS>,
    forward: bool,
    mut interval: u64,
    mut count: u64,
    add: i16,
) -> Result<(), StrokePlanError> {
    while count > 0 {
        let chunk = count.min(u64::from(u16::MAX));
        let segment = StrokeSegment {
            forward,
            interval: u32::try_from(interval).map_err(|_| StrokePlanError::SegmentOverflow)?,
            count: chunk as u16,
            add,
        };
        segments
            .push(segment)
            .map_err(|_| StrokePlanError::SegmentOverflow)?;
        let delta = i128::from(add) * i128::from(chunk);
        interval = u64::try_from(i128::from(interval) + delta)
            .map_err(|_| StrokePlanError::SegmentOverflow)?;
        count -= chunk;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 400 mm/s at 8000 mm/s^2 on a 40 mm/rev axis with STEP_WIDTH 8
    /// (8192 steps/rev), 60 mm per leg, Ouroboros 260 MHz step timer.
    fn live_rig_fast() -> StrokeRequest {
        StrokeRequest {
            speed_mrev_s: 10_000,
            accel_mrev_s2: 200_000,
            excursion_mrev: 1_500,
            steps_per_rev: 8_192,
            step_timer_hz: 260_000_000,
            min_cruise_us: 50_000,
        }
    }

    #[test]
    fn plan_leg_matches_the_worked_example() {
        let plan = plan_leg(&live_rig_fast()).unwrap();
        assert_eq!(plan.speed_mrev_s, 10_000);
        assert_eq!(plan.steps_per_leg, 12_288);
        assert_eq!(plan.ramp_us, 50_000);
        // v = 81_920 steps/s, a = 1_638_400 steps/s^2, ramp = v^2 / 2a = 2_048.
        assert_eq!(plan.ramp_steps, 2_048);
        assert_eq!(plan.cruise_steps, 12_288 - 2 * 2_048);
        assert_eq!(plan.cruise_us, 99_973);
        // `plan_leg` reports the pre-join geometry; `plan_stroke`'s leg
        // moves the join's steps from the cruise into the ramp.
        assert_eq!(plan.cruise_interval_ticks, 260_000_000 / 81_920);
    }

    #[test]
    fn plan_leg_picks_the_largest_speed_that_keeps_the_minimum_cruise() {
        let request = StrokeRequest {
            excursion_mrev: 300,
            ..live_rig_fast()
        };
        let plan = plan_leg(&request).unwrap();
        assert!(plan.speed_mrev_s < 10_000);
        assert!(plan.cruise_us >= 50_000);
        assert_eq!(plan.steps_per_leg, 2 * plan.ramp_steps + plan.cruise_steps);
        assert!(leg_fits(&request, plan.speed_mrev_s));
        assert!(
            !leg_fits(&request, plan.speed_mrev_s + 1),
            "must be maximal"
        );
        // Asking for exactly the found speed returns it unchanged.
        let exact = plan_leg(&StrokeRequest {
            speed_mrev_s: plan.speed_mrev_s,
            ..request
        })
        .unwrap();
        assert_eq!(exact.speed_mrev_s, plan.speed_mrev_s);
    }

    #[test]
    fn plan_leg_rejects_an_excursion_with_no_steps() {
        // 1 mrev on 500 steps/rev is zero whole steps: nothing can move.
        let request = StrokeRequest {
            excursion_mrev: 1,
            steps_per_rev: 500,
            ..live_rig_fast()
        };
        assert_eq!(plan_leg(&request), Err(StrokePlanError::NoCruise));
    }

    #[test]
    fn a_tiny_excursion_is_feasible_at_a_low_speed_not_rejected() {
        // 2 mrev = 16 steps: no ramp to speak of, so a slow cruise fits.
        let request = StrokeRequest {
            excursion_mrev: 2,
            ..live_rig_fast()
        };
        let plan = plan_leg(&request).unwrap();
        assert!(plan.cruise_us >= 50_000);
        assert!(plan.speed_mrev_s >= 1);
    }

    #[test]
    fn minimum_cruise_rounds_the_step_count_up_using_the_emitted_interval() {
        // 19 steps/s: floor(19 * 0.05) = 0 cruise steps would pass a floor
        // check with no cruise at all; the ceiling demands 1 step, which at
        // the emitted interval lasts 52.6 ms.
        let request = StrokeRequest {
            speed_mrev_s: 95,
            accel_mrev_s2: 200_000,
            excursion_mrev: 100,
            steps_per_rev: 200,
            step_timer_hz: 1_000_000,
            min_cruise_us: 50_000,
        };
        let plan = plan_leg(&request).unwrap();
        assert!(plan.cruise_steps >= 1);
        assert!(plan.cruise_us >= 50_000);
        // 1 MHz clock: ticks are microseconds.
        let ticks = u64::from(plan.cruise_steps) * u64::from(plan.cruise_interval_ticks);
        assert!(ticks >= 50_000);
    }

    #[test]
    fn plan_leg_rejects_a_cruise_interval_too_coarse_for_two_percent() {
        let request = StrokeRequest {
            step_timer_hz: 1_000_000,
            ..live_rig_fast()
        };
        assert_eq!(plan_leg(&request), Err(StrokePlanError::IntervalTooCoarse));
    }

    #[test]
    fn plan_leg_rejects_zero_inputs() {
        for zeroed in [
            StrokeRequest {
                speed_mrev_s: 0,
                ..live_rig_fast()
            },
            StrokeRequest {
                accel_mrev_s2: 0,
                ..live_rig_fast()
            },
            StrokeRequest {
                excursion_mrev: 0,
                ..live_rig_fast()
            },
            StrokeRequest {
                steps_per_rev: 0,
                ..live_rig_fast()
            },
            StrokeRequest {
                step_timer_hz: 0,
                ..live_rig_fast()
            },
        ] {
            assert_eq!(plan_leg(&zeroed), Err(StrokePlanError::ZeroInput));
        }
    }

    #[test]
    fn plan_leg_rejects_cruise_calculation_overflow() {
        // Inputs chosen to create large cruise_steps and cruise_interval whose
        // product would overflow u64 before division by step_timer_hz, triggering
        // SegmentOverflow on the final narrow() of cruise_us.
        // speed_mrev_s=1 and accel_mrev_s2=1 produce tiny ramp_steps, leaving
        // most of the excursion for cruise. Large step_timer_hz and low velocity
        // yield large cruise_interval. Their product in microseconds exceeds u32.
        let request = StrokeRequest {
            speed_mrev_s: 1,
            accel_mrev_s2: 1,
            excursion_mrev: 3_000_000,
            steps_per_rev: 8_000,
            step_timer_hz: 4_000_000_000,
            min_cruise_us: 50_000,
        };
        assert_eq!(plan_leg(&request), Err(StrokePlanError::SegmentOverflow));
    }

    /// Walk a segment list the way the step ISR does, returning per-step
    /// (elapsed_ticks, interval_used).
    fn simulate(segments: &[StrokeSegment]) -> std::vec::Vec<(u64, i64)> {
        let mut out = std::vec::Vec::new();
        let mut elapsed = 0_u64;
        for segment in segments {
            let mut interval = i64::from(segment.interval);
            for _ in 0..segment.count {
                let used = interval.max(1);
                elapsed += used as u64;
                out.push((elapsed, used));
                interval += i64::from(segment.add);
            }
        }
        out
    }

    #[test]
    fn stroke_has_two_mirrored_legs_with_opposite_directions() {
        let profile = plan_stroke(&live_rig_fast()).unwrap();
        let (first, second): (std::vec::Vec<&StrokeSegment>, std::vec::Vec<&StrokeSegment>) =
            profile.segments.iter().partition(|s| s.forward);
        assert_eq!(first.len(), second.len());
        for (a, b) in first.iter().zip(second.iter()) {
            assert_eq!((a.interval, a.count, a.add), (b.interval, b.count, b.add));
        }
        assert!(profile.segments.iter().take(first.len()).all(|s| s.forward));
        assert!(
            profile
                .segments
                .iter()
                .skip(first.len())
                .all(|s| !s.forward)
        );
    }

    #[test]
    fn each_leg_steps_exactly_the_planned_excursion() {
        let profile = plan_stroke(&live_rig_fast()).unwrap();
        let per_leg: u32 = profile
            .segments
            .iter()
            .filter(|s| s.forward)
            .map(|s| u32::from(s.count))
            .sum();
        assert_eq!(per_leg, profile.leg.steps_per_leg);
    }

    #[test]
    fn cruise_speed_is_within_two_percent_and_ramps_are_monotone() {
        let profile = plan_stroke(&live_rig_fast()).unwrap();
        let leg: std::vec::Vec<_> = profile
            .segments
            .iter()
            .filter(|s| s.forward)
            .copied()
            .collect();
        let steps = simulate(&leg);
        let ramp = profile.leg.ramp_steps as usize;
        let cruise = profile.leg.cruise_steps as usize;
        let cruise_intervals: std::vec::Vec<i64> =
            steps[ramp..ramp + cruise].iter().map(|(_, i)| *i).collect();
        // Independent of the planner's own interval: the requested speed in
        // steps per second, checked as clock / interval within 2% by
        // cross-multiplication.
        let request = live_rig_fast();
        let expected_rate =
            i64::from(profile.leg.speed_mrev_s) * i64::from(request.steps_per_rev) / 1_000;
        let clock = i64::from(request.step_timer_hz);
        for interval in &cruise_intervals {
            let diff = (clock - interval * expected_rate).abs();
            assert!(
                diff * 50 <= interval * expected_rate,
                "cruise cadence {interval} ticks vs {expected_rate} steps/s"
            );
        }
        for pair in steps[..ramp].windows(2) {
            assert!(pair[1].1 <= pair[0].1, "acceleration must not slow down");
        }
        for pair in steps[ramp + cruise..].windows(2) {
            assert!(pair[1].1 >= pair[0].1, "deceleration must not speed up");
        }
    }

    #[test]
    fn long_runs_are_split_at_the_u16_count_limit() {
        let request = StrokeRequest {
            steps_per_rev: 65_536,
            excursion_mrev: 3_000,
            speed_mrev_s: 4_000,
            ..live_rig_fast()
        };
        let profile = plan_stroke(&request).unwrap();
        assert!(profile.segments.iter().all(|s| s.count > 0));
        let per_leg: u32 = profile
            .segments
            .iter()
            .filter(|s| s.forward)
            .map(|s| u32::from(s.count))
            .sum();
        assert_eq!(per_leg, profile.leg.steps_per_leg);
        assert!(
            profile.leg.cruise_steps > u32::from(u16::MAX),
            "test must exercise the split"
        );
        assert!(profile.cruise_run_count >= 2);
        let (_, cruise, _) = forward_slices(&profile);
        assert!(
            cruise
                .iter()
                .all(|c| c.add == 0 && c.interval == profile.leg.cruise_interval_ticks)
        );
    }

    #[test]
    fn a_split_accelerating_run_carries_its_interval_across_the_boundary() {
        let mut segments = heapless::Vec::<StrokeSegment, MAX_STROKE_SEGMENTS>::new();
        push_run(&mut segments, true, 200_000, 70_000, -1).unwrap();
        assert_eq!(segments.len(), 2);
        assert_eq!(segments[0].count, u16::MAX);
        assert_eq!(segments[1].count as u32, 70_000 - u32::from(u16::MAX));
        assert_eq!(segments[1].interval, 200_000 - u32::from(u16::MAX));
        let steps = simulate(&segments);
        let last_of_first = steps[usize::from(u16::MAX) - 1].1;
        let first_of_second = steps[usize::from(u16::MAX)].1;
        assert_eq!(
            first_of_second,
            last_of_first - 1,
            "cadence continues across the split"
        );
    }

    #[test]
    fn step_move_conversion_carries_interval_count_and_add() {
        let segment = StrokeSegment {
            forward: false,
            interval: 3_173,
            count: 100,
            add: -7,
        };
        let step_move = segment.step_move();
        assert_eq!(step_move.interval, 3_173);
        assert_eq!(step_move.count, 100);
        assert_eq!(step_move.add, -7);
    }

    #[test]
    fn run_ticks_matches_the_isr_walk() {
        let segment = StrokeSegment {
            forward: true,
            interval: 1_000,
            count: 10,
            add: -7,
        };
        let simulated: u64 = simulate(&[segment]).iter().map(|(_, i)| *i as u64).sum();
        assert_eq!(run_ticks(1_000, 10, -7), simulated);
        assert_eq!(run_ticks(3_173, 8_192, 0), 3_173 * 8_192);
    }

    /// The forward leg's runs sliced by the profile's recorded counts.
    fn forward_slices(
        profile: &StrokeProfile,
    ) -> (
        std::vec::Vec<StrokeSegment>,
        std::vec::Vec<StrokeSegment>,
        std::vec::Vec<StrokeSegment>,
    ) {
        let leg: std::vec::Vec<_> = profile
            .segments
            .iter()
            .filter(|s| s.forward)
            .copied()
            .collect();
        let a = usize::from(profile.accel_run_count);
        let c = usize::from(profile.cruise_run_count);
        (
            leg[..a].to_vec(),
            leg[a..a + c].to_vec(),
            leg[a + c..].to_vec(),
        )
    }

    fn next_interval(run: &StrokeSegment) -> i64 {
        i64::from(run.interval) + i64::from(run.add) * i64::from(run.count)
    }

    fn last_emitted(run: &StrokeSegment) -> i64 {
        i64::from(run.interval) + i64::from(run.add) * i64::from(run.count - 1)
    }

    /// A boundary is continuous when the next run's first interval follows
    /// either the previous run's slope or its own: within one run the
    /// schedule keeps its `add`; where the slope changes (join into cruise,
    /// cruise into the mirrored deceleration) the step is the new run's.
    fn continuous(prev: &StrokeSegment, next: &StrokeSegment) -> bool {
        let first = i64::from(next.interval);
        first == next_interval(prev) || first == last_emitted(prev) + i64::from(next.add)
    }

    #[test]
    fn deceleration_runs_mirror_the_emitted_acceleration_runs() {
        let profile = plan_stroke(&live_rig_fast()).unwrap();
        let (accel, _, decel) = forward_slices(&profile);
        assert_eq!(accel.len(), decel.len());
        for (a, d) in accel.iter().rev().zip(decel.iter()) {
            let last_emitted = i64::from(a.interval) + i64::from(a.add) * i64::from(a.count - 1);
            assert_eq!(i64::from(d.interval), last_emitted);
            assert_eq!(d.count, a.count);
            assert_eq!(d.add, -a.add);
        }
        let accel_ticks: u64 = accel
            .iter()
            .map(|s| run_ticks(s.interval, s.count, s.add))
            .sum();
        let decel_ticks: u64 = decel
            .iter()
            .map(|s| run_ticks(s.interval, s.count, s.add))
            .sum();
        assert_eq!(accel_ticks, decel_ticks);
        assert_eq!(profile.accel_ticks, accel_ticks);
    }

    #[test]
    fn every_run_starts_where_the_previous_schedule_lands() {
        let profile = plan_stroke(&live_rig_fast()).unwrap();
        let leg: std::vec::Vec<_> = profile
            .segments
            .iter()
            .filter(|s| s.forward)
            .copied()
            .collect();
        for pair in leg.windows(2) {
            assert!(
                continuous(&pair[0], &pair[1]),
                "cadence must be continuous across every run boundary: {pair:?}"
            );
        }
        let (accel, cruise, _) = forward_slices(&profile);
        assert_eq!(
            next_interval(accel.last().unwrap()),
            i64::from(cruise[0].interval)
        );
        assert_eq!(cruise[0].interval, profile.leg.cruise_interval_ticks);
        assert!(
            cruise
                .iter()
                .all(|c| c.add == 0 && c.interval == profile.leg.cruise_interval_ticks)
        );
    }

    #[test]
    fn the_whole_acceleration_including_the_join_never_slows_down() {
        let profile = plan_stroke(&live_rig_fast()).unwrap();
        let (accel, cruise, _) = forward_slices(&profile);
        let steps = simulate(&accel);
        for pair in steps.windows(2) {
            assert!(
                pair[1].1 <= pair[0].1,
                "acceleration interval rose: {pair:?}"
            );
        }
        let join = accel.last().unwrap();
        assert!(join.add <= 0);
        assert_eq!(next_interval(join), i64::from(cruise[0].interval));
        assert!(steps.last().unwrap().1 >= i64::from(profile.leg.cruise_interval_ticks));
    }

    #[test]
    fn push_ramp_run_splits_a_long_run_and_carries_the_interval() {
        let mut runs = RampRuns::new();
        let next = push_ramp_run(&mut runs, 200_000, 70_000, -1).unwrap();
        assert_eq!(runs.len(), 2);
        assert_eq!(runs[0].count, u16::MAX);
        assert_eq!(u32::from(runs[1].count), 70_000 - u32::from(u16::MAX));
        assert_eq!(runs[1].interval, 200_000 - u32::from(u16::MAX));
        assert_eq!(next, 200_000 - 70_000);
        assert!(continuous(&runs[0], &runs[1]));
    }

    #[test]
    fn a_ramp_partition_above_the_u16_limit_is_split_end_to_end() {
        // 65536 steps/rev at 1000 mrev/s^2: the ramp is ~550 000 steps and
        // its last velocity-spaced partition (31/256 of it) exceeds u16::MAX
        // on its own, so an acceleration run must split.
        let request = StrokeRequest {
            steps_per_rev: 65_536,
            excursion_mrev: 20_000,
            speed_mrev_s: 4_100,
            accel_mrev_s2: 1_000,
            ..live_rig_fast()
        };
        let profile = plan_stroke(&request).unwrap();
        let (accel, _, decel) = forward_slices(&profile);
        let split_at = accel
            .iter()
            .position(|s| s.count == u16::MAX)
            .expect("one acceleration partition must have been split");
        assert!(continuous(&accel[split_at], &accel[split_at + 1]));
        assert_eq!(accel[split_at].add, accel[split_at + 1].add);
        assert_eq!(accel.len(), decel.len());
        let mirrored = decel[decel.len() - 1 - split_at];
        assert_eq!(mirrored.count, u16::MAX);
        assert_eq!(mirrored.add, -accel[split_at].add);
    }

    #[test]
    fn the_cruise_left_after_the_join_still_meets_the_minimum() {
        for request in [
            live_rig_fast(),
            StrokeRequest {
                excursion_mrev: 300,
                ..live_rig_fast()
            },
            StrokeRequest {
                excursion_mrev: 2,
                ..live_rig_fast()
            },
            StrokeRequest {
                excursion_mrev: 60,
                speed_mrev_s: 2_000,
                ..live_rig_fast()
            },
        ] {
            let profile = plan_stroke(&request).unwrap();
            let min_ticks =
                u64::from(request.min_cruise_us) * u64::from(request.step_timer_hz) / 1_000_000;
            assert!(
                profile.cruise_ticks >= min_ticks,
                "{request:?}: cruise {} ticks < {min_ticks}",
                profile.cruise_ticks
            );
            assert!(stroke_fits(&request, profile.leg.speed_mrev_s));
            assert!(
                !stroke_fits(&request, profile.leg.speed_mrev_s + 1),
                "{request:?}: speed must be maximal including the join"
            );
            assert!(profile.leg.speed_mrev_s <= plan_leg(&request).unwrap().speed_mrev_s);
        }
    }

    #[test]
    fn a_steep_first_partition_falls_back_to_single_steps_and_speed_stays_maximal() {
        // 50 mm/s on the live rig: a 32-step ramp whose second partition is
        // one step with a 42 000-tick change, outside i16. The partition is
        // emitted step by step; the speed is not reduced for it, and the
        // predicate stays monotone so the search is not fooled.
        let request = StrokeRequest {
            speed_mrev_s: 1_250,
            ..live_rig_fast()
        };
        let profile = plan_stroke(&request).unwrap();
        assert_eq!(profile.leg.speed_mrev_s, 1_250);
        let (accel, _, _) = forward_slices(&profile);
        assert!(accel.iter().any(|s| s.count == 1 && s.add == 0));
        let steps = simulate(&accel);
        for pair in steps.windows(2) {
            assert!(pair[1].1 <= pair[0].1);
        }
        for lower in [1_000, 1_100, 1_200] {
            assert!(
                stroke_fits(&request, lower),
                "predicate must not dip at {lower}"
            );
        }
    }

    #[test]
    fn ramp_runs_track_the_ideal_curve_without_accumulating_error() {
        let request = live_rig_fast();
        let profile = plan_stroke(&request).unwrap();
        let (accel, _, _) = forward_slices(&profile);
        let steps = simulate(&accel);
        let accel_steps =
            u64::from(request.accel_mrev_s2) * u64::from(request.steps_per_rev) / 1_000;
        let clock = u64::from(request.step_timer_hz);
        // At every run end the emitted interval is within 10% of the ideal
        // clock / sqrt(2 a s): ceiling rounding is one-sided, so a run can
        // end above ideal by up to `count` ticks. The join run is excluded
        // (it is exact by construction); the cruise itself is held to 2%
        // by the cruise test, and the ramp's total duration is what the
        // capture windows use, measured exactly.
        let mut s_end = 0_u64;
        for run in accel.iter().take(accel.len() - 1) {
            s_end += u64::from(run.count);
            let ideal = clock / (2 * accel_steps * s_end).isqrt().max(1);
            let emitted = steps[s_end as usize - 1].1 as u64;
            assert!(
                emitted >= ideal || ideal - emitted <= 1,
                "run ends below ideal at step {s_end}"
            );
            assert!(
                emitted.abs_diff(ideal) * 10 <= ideal,
                "run end at step {s_end}: {emitted} vs {ideal}"
            );
        }
        let accounted = u32::from(profile.accel_run_count) as usize;
        assert!(accounted >= 2, "at least one ramp run plus the join");
        assert_eq!(
            profile.leg.ramp_steps * 2 + profile.leg.cruise_steps,
            profile.leg.steps_per_leg
        );
    }

    #[test]
    fn a_long_ramp_is_split_at_the_u16_limit_and_still_mirrors() {
        // 65536 steps/rev, low acceleration: the ramp alone exceeds 65535
        // steps, so at least one ramp run must split.
        let request = StrokeRequest {
            steps_per_rev: 65_536,
            excursion_mrev: 3_000,
            speed_mrev_s: 4_000,
            accel_mrev_s2: 4_000,
            ..live_rig_fast()
        };
        let profile = plan_stroke(&request).unwrap();
        assert!(profile.leg.ramp_steps > u32::from(u16::MAX));
        let (accel, _, decel) = forward_slices(&profile);
        assert!(accel.iter().all(|s| s.count > 0));
        assert_eq!(
            accel.iter().map(|s| u32::from(s.count)).sum::<u32>(),
            profile.leg.ramp_steps
        );
        assert_eq!(accel.len(), decel.len());
        let leg: std::vec::Vec<_> = profile
            .segments
            .iter()
            .filter(|s| s.forward)
            .copied()
            .collect();
        for pair in leg.windows(2) {
            assert!(continuous(&pair[0], &pair[1]));
        }
    }

    #[test]
    fn profile_timing_is_measured_from_the_generated_runs() {
        let request = live_rig_fast();
        let profile = plan_stroke(&request).unwrap();
        let leg: std::vec::Vec<_> = profile
            .segments
            .iter()
            .filter(|s| s.forward)
            .copied()
            .collect();
        let steps = simulate(&leg);
        let clock = u64::from(request.step_timer_hz);
        let ramp = profile.leg.ramp_steps as usize;
        let cruise = profile.leg.cruise_steps as usize;
        let accel_ticks = steps[ramp - 1].0;
        let cruise_end_ticks = steps[ramp + cruise - 1].0;
        let leg_ticks = steps[steps.len() - 1].0;
        assert_eq!(profile.accel_ticks, accel_ticks);
        assert_eq!(profile.accel_ticks + profile.cruise_ticks, cruise_end_ticks);
        assert_eq!(profile.leg_ticks, leg_ticks);
        assert_eq!(
            u64::from(profile.ramp_end_us()),
            accel_ticks * 1_000_000 / clock
        );
        assert_eq!(
            u64::from(profile.cruise_end_us()),
            cruise_end_ticks * 1_000_000 / clock
        );
        assert_eq!(u64::from(profile.leg_us()), leg_ticks * 1_000_000 / clock);
        assert_eq!(
            u64::from(profile.stroke_us()),
            2 * leg_ticks * 1_000_000 / clock
        );
        assert_eq!(profile.leg.ramp_us, profile.ramp_end_us());
        assert_eq!(
            profile.leg.cruise_us,
            profile.cruise_end_us() - profile.ramp_end_us()
        );
        // The analytic estimate and the emitted schedule agree closely on
        // this profile; the exact value is what the capture uses.
        assert!(profile.leg.ramp_us.abs_diff(50_000) * 20 <= 50_000);
    }
}
