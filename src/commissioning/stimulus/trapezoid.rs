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
    let cruise_us = cruise_steps * cruise_interval * 1_000_000 / u64::from(request.step_timer_hz);
    Ok(LegPlan {
        speed_mrev_s: narrow(speed)?,
        steps_per_leg: narrow(steps_per_leg)?,
        ramp_steps: narrow(ramp_steps)?,
        cruise_steps: narrow(cruise_steps)?,
        ramp_us: narrow(ramp_us)?,
        cruise_us: narrow(cruise_us)?,
        cruise_interval_ticks: narrow(cruise_interval)?,
    })
}

#[allow(dead_code)]
fn narrow(value: u64) -> Result<u32, StrokePlanError> {
    u32::try_from(value).map_err(|_| StrokePlanError::SegmentOverflow)
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
}
