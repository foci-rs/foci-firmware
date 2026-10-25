pub(crate) const RAMP_SEGMENTS: u64 = 32;

pub(crate) const MIN_CRUISE_INTERVAL_TICKS: u64 = 50;

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) struct StrokeRequest {
    /// Requested cruise speed, mrev/s.
    pub speed_mrev_s: u32,
    /// Ramp acceleration, mrev/s^2.
    pub accel_mrev_s2: u32,
    /// Distance one leg travels, mrev.
    pub excursion_mrev: u32,
    /// Logical planner steps per mechanical revolution, the unit of the step queue.
    pub steps_per_rev: u32,
    /// Step timer frequency, Hz.
    pub step_timer_hz: u32,
    /// Shortest cruise the phase accepts, us.
    pub min_cruise_us: u32,
}

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

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) enum StrokePlanError {
    ZeroInput,
    NoCruise,
    IntervalTooCoarse,
    SegmentOverflow,
}

fn steps_per_second(speed_mrev_s: u64, steps_per_rev: u64) -> u64 {
    speed_mrev_s * steps_per_rev / 1_000
}

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
        (u64::from(request.min_cruise_us) * u64::from(request.step_timer_hz)).div_ceil(1_000_000);
    let min_cruise_steps = min_cruise_ticks.div_ceil(cruise_interval).max(1);
    if 2 * ramp_steps + min_cruise_steps > steps_per_leg {
        return None;
    }
    Some((ramp_steps, steps_per_leg - 2 * ramp_steps, cruise_interval))
}

pub(crate) fn leg_fits(request: &StrokeRequest, speed_mrev_s: u32) -> bool {
    let steps_per_rev = u64::from(request.steps_per_rev);
    let steps_per_leg = u64::from(request.excursion_mrev) * steps_per_rev / 1_000;
    let accel_steps = steps_per_second(u64::from(request.accel_mrev_s2), steps_per_rev);
    accel_steps > 0
        && leg_at(request, steps_per_leg, accel_steps, u64::from(speed_mrev_s)).is_some()
}

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
    let floor = 1_000_u64.div_ceil(steps_per_rev).max(1);
    let fits = |speed: u64| leg_at(request, steps_per_leg, accel_steps, speed).is_some();
    if !fits(floor) {
        return Err(StrokePlanError::NoCruise);
    }
    if requested < floor {
        return Err(StrokePlanError::NoCruise);
    }
    let speed = if fits(requested) {
        requested
    } else {
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

fn narrow(value: u64) -> Result<u32, StrokePlanError> {
    u32::try_from(value).map_err(|_| StrokePlanError::SegmentOverflow)
}

pub(crate) const MAX_STROKE_SEGMENTS: usize = 8 * RAMP_SEGMENTS as usize + 32;

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
    pub(crate) fn step_move(&self) -> foci_klipper::stepper::StepMove {
        foci_klipper::stepper::StepMove::new(self.interval, self.count, self.add)
    }
}

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

impl StrokeProfile {
    fn ticks_to_us(&self, ticks: u64) -> u32 {
        let us = u128::from(ticks) * 1_000_000 / u128::from(self.step_timer_hz);
        u32::try_from(us).unwrap_or(u32::MAX)
    }

    pub(crate) fn ramp_end_us(&self) -> u32 {
        self.ticks_to_us(self.accel_ticks)
    }

    pub(crate) fn cruise_end_us(&self) -> u32 {
        self.ticks_to_us(self.accel_ticks + self.cruise_ticks)
    }

    pub(crate) fn leg_us(&self) -> u32 {
        self.ticks_to_us(self.leg_ticks)
    }

    pub(crate) fn stroke_us(&self) -> u32 {
        self.ticks_to_us(self.leg_ticks.saturating_mul(2))
    }
}

pub(crate) fn run_ticks(interval: u32, count: u16, add: i16) -> u64 {
    let n = i128::from(count);
    let total = n * i128::from(interval) + i128::from(add) * n * (n - 1) / 2;
    u64::try_from(total.max(0)).unwrap_or(u64::MAX)
}

type RampRuns = heapless::Vec<StrokeSegment, { 2 * RAMP_SEGMENTS as usize + 18 }>;

fn emit_leg(
    request: &StrokeRequest,
    speed_mrev_s: u32,
) -> Result<Option<(LegPlan, RampRuns, u64)>, StrokePlanError> {
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
    let min_cruise_ticks = (u64::from(request.min_cruise_us) * clock).div_ceil(1_000_000);
    if cruise_steps * u64::from(leg.cruise_interval_ticks) < min_cruise_ticks {
        return Ok(None);
    }
    Ok(Some((leg, runs, cruise_steps)))
}

pub(crate) fn stroke_fits(request: &StrokeRequest, speed_mrev_s: u32) -> bool {
    matches!(emit_leg(request, speed_mrev_s), Ok(Some(_)))
}

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

fn ramp_boundary(ramp_steps: u64, k: u64) -> u64 {
    ramp_steps * k * k / (RAMP_SEGMENTS * RAMP_SEGMENTS)
}

fn ramp_interval(clock: u64, accel_steps: u64, step: u64) -> u64 {
    let velocity = (2 * accel_steps * step.max(1)).isqrt().max(1);
    clock / velocity
}

fn ceil_div_toward_zero(numerator: i128, denominator: i128) -> i128 {
    debug_assert!(numerator <= 0 && denominator > 0);
    -((-numerator) / denominator)
}

fn push_ramp_partition(
    runs: &mut RampRuns,
    clock: u64,
    accel_steps: u64,
    start: u64,
    count: u64,
    last_emitted: i128,
    target: i128,
) -> Result<i128, StrokePlanError> {
    if count == 1 {
        let value = target.min(last_emitted);
        let interval = u32::try_from(value).map_err(|_| StrokePlanError::SegmentOverflow)?;
        runs.push(StrokeSegment {
            forward: true,
            interval,
            count: 1,
            add: 0,
        })
        .map_err(|_| StrokePlanError::SegmentOverflow)?;
        return Ok(value);
    }
    let is_first = runs.is_empty();
    let divisor = if is_first { count - 1 } else { count };
    let add = ceil_div_toward_zero((target - last_emitted).min(0), i128::from(divisor));
    if let Ok(add16) = i16::try_from(add) {
        let run_start = if is_first {
            last_emitted
        } else {
            last_emitted + add
        };
        push_ramp_run(runs, run_start, count, add16)?;
        return Ok(run_start + add * i128::from(count - 1));
    }
    let mut prev = last_emitted;
    for step in 0..count {
        let exact = i128::from(ramp_interval(clock, accel_steps, start + step + 1)).min(prev);
        runs.push(StrokeSegment {
            forward: true,
            interval: u32::try_from(exact).map_err(|_| StrokePlanError::SegmentOverflow)?,
            count: 1,
            add: 0,
        })
        .map_err(|_| StrokePlanError::SegmentOverflow)?;
        prev = exact;
    }
    Ok(prev)
}

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

fn acceleration_runs(
    leg: &LegPlan,
    accel_steps: u64,
    clock: u64,
) -> Result<(RampRuns, u64), StrokePlanError> {
    let ramp_steps = u64::from(leg.ramp_steps);
    let mut runs = RampRuns::new();
    let mut last_emitted = if ramp_steps == 0 {
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
        last_emitted = push_ramp_partition(
            &mut runs,
            clock,
            accel_steps,
            start,
            count,
            last_emitted,
            target,
        )?;
    }
    let cruise_interval = i128::from(leg.cruise_interval_ticks);
    debug_assert!(last_emitted >= cruise_interval);
    let join_steps = u64::try_from(last_emitted - cruise_interval)
        .map_err(|_| StrokePlanError::SegmentOverflow)?;
    let mut cruise_steps = u64::from(leg.cruise_steps);
    if join_steps > 0 {
        let both_joins = join_steps
            .checked_mul(2)
            .ok_or(StrokePlanError::SegmentOverflow)?;
        if both_joins >= cruise_steps {
            return Err(StrokePlanError::SegmentOverflow);
        }
        let add = -1_i128;
        let run_start = last_emitted + add;
        push_ramp_run(&mut runs, run_start, join_steps, -1)?;
        last_emitted = run_start + add * i128::from(join_steps - 1);
        cruise_steps -= both_joins;
        debug_assert_eq!(last_emitted, cruise_interval);
    }
    Ok((runs, cruise_steps))
}

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
        assert_eq!(plan.ramp_steps, 2_048);
        assert_eq!(plan.cruise_steps, 12_288 - 2 * 2_048);
        assert_eq!(plan.cruise_us, 99_973);
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
        let exact = plan_leg(&StrokeRequest {
            speed_mrev_s: plan.speed_mrev_s,
            ..request
        })
        .unwrap();
        assert_eq!(exact.speed_mrev_s, plan.speed_mrev_s);
    }

    #[test]
    fn plan_leg_rejects_a_requested_speed_below_the_floor_instead_of_underflowing() {
        let request = StrokeRequest {
            speed_mrev_s: 1,
            steps_per_rev: 500,
            ..live_rig_fast()
        };
        assert_eq!(plan_leg(&request), Err(StrokePlanError::NoCruise));
    }

    #[test]
    fn plan_leg_rejects_an_excursion_with_no_steps() {
        let request = StrokeRequest {
            excursion_mrev: 1,
            steps_per_rev: 500,
            ..live_rig_fast()
        };
        assert_eq!(plan_leg(&request), Err(StrokePlanError::NoCruise));
    }

    #[test]
    fn a_tiny_excursion_is_feasible_at_a_low_speed_not_rejected() {
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
        let ticks = u64::from(plan.cruise_steps) * u64::from(plan.cruise_interval_ticks);
        assert!(ticks >= 50_000);
    }

    #[test]
    fn minimum_cruise_holds_on_a_non_round_clock() {
        let request = StrokeRequest {
            speed_mrev_s: 20,
            accel_mrev_s2: 200,
            excursion_mrev: 3,
            steps_per_rev: 1_000,
            step_timer_hz: 1_000_001,
            min_cruise_us: 50_000,
        };
        let plan = plan_leg(&request).unwrap();
        assert!(
            plan.cruise_us >= request.min_cruise_us,
            "cruise {} us must meet the {} us minimum",
            plan.cruise_us,
            request.min_cruise_us
        );
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

    fn continuous(prev: &StrokeSegment, next: &StrokeSegment) -> bool {
        if prev.count == 1 && prev.add == 0 && next.count == 1 && next.add == 0 {
            return true;
        }
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
            last_emitted(accel.last().unwrap()),
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
        assert_eq!(last_emitted(join), i64::from(cruise[0].interval));
        assert!(steps.last().unwrap().1 >= i64::from(profile.leg.cruise_interval_ticks));
    }

    #[test]
    fn push_ramp_partition_falls_back_to_exact_per_step_values_when_add_overflows_i16() {
        let clock = 200_000_u64;
        let accel_steps = 1_u64;
        let start = 0_u64;
        let count = 2_u64;
        let last_emitted = i128::from(ramp_interval(clock, accel_steps, 1));
        let target = i128::from(ramp_interval(clock, accel_steps, start + count));
        let mut runs = RampRuns::new();
        let returned = push_ramp_partition(
            &mut runs,
            clock,
            accel_steps,
            start,
            count,
            last_emitted,
            target,
        )
        .unwrap();
        assert_eq!(runs.len(), 2, "each fallback step is its own run");
        for (index, run) in runs.iter().enumerate() {
            let step = start + index as u64 + 1;
            assert_eq!(run.count, 1);
            assert_eq!(run.add, 0);
            assert_eq!(
                i128::from(run.interval),
                i128::from(ramp_interval(clock, accel_steps, step)),
                "step {step} must use the exact ideal interval"
            );
        }
        assert_eq!(
            returned,
            i128::from(ramp_interval(clock, accel_steps, start + count))
        );
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
        let request = StrokeRequest {
            steps_per_rev: 65_536,
            excursion_mrev: 40_000,
            speed_mrev_s: 6_000,
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
    fn every_ramp_step_stays_at_or_above_the_ideal_curve() {
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
            let (accel, _, _) = forward_slices(&profile);
            let steps = simulate(&accel);
            let accel_steps =
                u64::from(request.accel_mrev_s2) * u64::from(request.steps_per_rev) / 1_000;
            let clock = u64::from(request.step_timer_hz);
            for (index, (_, interval)) in steps.iter().enumerate() {
                let step = index as u64 + 1;
                let ideal = clock / (2 * accel_steps * step).isqrt().max(1);
                assert!(
                    *interval + 1 >= ideal as i64,
                    "{request:?}: step {step}: emitted {interval} ticks is faster than ideal {ideal} ticks"
                );
            }
        }
    }

    #[test]
    fn a_long_ramp_is_split_at_the_u16_limit_and_still_mirrors() {
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
        assert!(profile.leg.ramp_us.abs_diff(50_000) * 20 <= 50_000);
    }
}
