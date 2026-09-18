use crate::autotune::PositionTuneOutcome;
use foci_tmc4671::pid::PidPGain;

/// Least-squares slope of `velocity_rpm` (raw `PID_VELOCITY_ACTUAL`, i.e.
/// mechanical RPM per `phi_m_abn`) against `elapsed_us`, in RPM/s.
///
/// Mirrors `electrical_capture::estimate::fit_current_slope`'s integer
/// accumulator pattern **and its scale-before-divide order**: that
/// function multiplies its slope numerator by `MICROS_PER_SECOND` before
/// dividing by the denominator specifically to avoid truncating a
/// sub-1-count-per-microsecond slope to zero before it's scaled up. An
/// earlier draft of this function divided first, which is exactly that
/// bug -- it silently truncated every realistic slope to zero.
///
/// Returns `None` for fewer than 3 samples, non-monotonic timestamps, or
/// checked-arithmetic overflow.
fn fit_velocity_slope_rpm_per_s(samples: &[(u32, i32)]) -> Option<i64> {
    if samples.len() < 3 {
        return None;
    }
    let first_us = samples[0].0;
    let mut sum_x: i128 = 0;
    let mut sum_x2: i128 = 0;
    let mut sum_y: i128 = 0;
    let mut sum_xy: i128 = 0;
    let mut previous_us = first_us;

    for &(t_us, rpm) in samples {
        if t_us < previous_us {
            return None;
        }
        previous_us = t_us;
        let x = i128::from(t_us.checked_sub(first_us)?);
        let y = i128::from(rpm);
        sum_x = sum_x.checked_add(x)?;
        sum_x2 = sum_x2.checked_add(x.checked_mul(x)?)?;
        sum_y = sum_y.checked_add(y)?;
        sum_xy = sum_xy.checked_add(x.checked_mul(y)?)?;
    }

    let n = i128::try_from(samples.len()).ok()?;
    let numerator = n
        .checked_mul(sum_xy)?
        .checked_sub(sum_x.checked_mul(sum_y)?)?;
    let denominator = n
        .checked_mul(sum_x2)?
        .checked_sub(sum_x.checked_mul(sum_x)?)?;
    if denominator == 0 {
        return None;
    }
    // Scale by 1_000_000 (elapsed_us -> per-second) BEFORE dividing, so a
    // fractional RPM/us slope survives instead of truncating to zero.
    let slope_rpm_per_s = numerator.checked_mul(1_000_000)?.checked_div(denominator)?;
    i64::try_from(slope_rpm_per_s).ok()
}

/// Measure the floor position-P from a fresh P-only velocity rung capture.
///
/// `samples` are `(elapsed_us, velocity_rpm)` pairs from the onset of a
/// fixed-torque step, `velocity_rpm` the raw `PID_VELOCITY_ACTUAL` reading
/// (see this module's Interfaces note — this is load-bearing, not a
/// convenience choice). `commanded_torque_count` is that fixed torque
/// command in raw `PID_TORQUE_FLUX_TARGET` register counts. `velocity_p`
/// is the outer velocity autotune's already-selected gain.
///
/// See `docs/specs/2026-09-13-foci-340-position-tune-two-bound-design.md`,
/// "Floor: inner (velocity-loop) bandwidth", for the physical derivation
/// (`accel_per_count` equals FOCI-215's combined `Kadc*Kt/J` term, `bw_inner`
/// is in rad/s not Hz). Fitting directly against the register's native RPM
/// domain rather than converting to true SI rad/s is a further
/// simplification beyond the spec: the accel-per-torque fit's RPM/s result
/// times `Kp_v` already equals `bw_inner` in rad/s with no separate `2*pi`
/// conversion anywhere in this function -- the `RPM<->rad/s` conversion
/// factors that appear in the physical derivation cancel exactly (verified
/// numerically: `Kp_v=1.367` (`velocity_p=350`) times the RPM/s-domain
/// accel-per-torque derived from FOCI-215's constants reproduces the same
/// ~266 rad/s the spec's rad/s²-domain derivation gives). Do not
/// reintroduce a `2*pi` factor into this function.
///
/// Returns both the floor and `bw_inner_rad_s` — Task 6 retains the latter
/// for Task 7's `inner_bandwidth_rad_s` report field; it is not just an
/// intermediate value to discard.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FloorResult {
    pub floor: PidPGain,
    pub bw_inner_rad_s: i64,
}

pub fn compute_position_floor(
    samples: &[(u32, i32)],
    commanded_torque_count: i32,
    velocity_p: PidPGain,
) -> Result<FloorResult, PositionTuneOutcome> {
    if commanded_torque_count == 0 {
        return Err(PositionTuneOutcome::AccelerationFitRejected);
    }
    let slope_rpm_per_s = fit_velocity_slope_rpm_per_s(samples)
        .ok_or(PositionTuneOutcome::AccelerationFitRejected)?;
    if slope_rpm_per_s <= 0 {
        return Err(PositionTuneOutcome::AccelerationFitRejected);
    }

    // accel_per_count: RPM/s per raw torque-register count.
    let accel_per_count = slope_rpm_per_s / i64::from(commanded_torque_count);
    if accel_per_count <= 0 {
        return Err(PositionTuneOutcome::AccelerationFitRejected);
    }

    // bw_inner_rad_s = Kp_v * accel_per_count = (velocity_p/256) * accel_per_count.
    // Kept as one integer expression to avoid intermediate truncation.
    let velocity_p_raw = i64::from(velocity_p.raw());
    let bw_numerator = velocity_p_raw
        .checked_mul(accel_per_count)
        .ok_or(PositionTuneOutcome::AccelerationFitRejected)?;
    let bw_inner_rad_s = bw_numerator / 256;

    // floor_raw = bw_inner * 60 * 256 / 65536, rounded UP -- the *256
    // converts from the "Q8.8 float" effective-gain domain (what bw_inner
    // and Andrew's position-loop formula both operate in) to the raw
    // register integer domain PidPGain stores.
    let floor_numerator = bw_inner_rad_s
        .checked_mul(60)
        .and_then(|v| v.checked_mul(256))
        .ok_or(PositionTuneOutcome::AccelerationFitRejected)?;
    let floor_raw =
        floor_numerator.div_euclid(65_536) + i64::from(floor_numerator.rem_euclid(65_536) != 0);

    if floor_raw <= 0 {
        return Err(PositionTuneOutcome::FloorUnrepresentableZero);
    }
    let floor_raw_u16 =
        u16::try_from(floor_raw).map_err(|_| PositionTuneOutcome::FloorUnrepresentableTooLarge)?;
    let floor = PidPGain::new(floor_raw_u16)
        .map_err(|_| PositionTuneOutcome::FloorUnrepresentableTooLarge)?;
    Ok(FloorResult {
        floor,
        bw_inner_rad_s,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fit_rejects_non_positive_slope() {
        // Flat velocity (no acceleration) -- a bad capture, not a valid fit.
        // (t_us, velocity_rpm) pairs, velocity_rpm a raw PID_VELOCITY_ACTUAL
        // reading (mechanical RPM, 1 count = 1 RPM per phi_m_abn selection).
        let samples: [(u32, i32); 4] = [(0, 0), (1_000, 0), (2_000, 0), (3_000, 0)];
        let result = compute_position_floor(&samples, 500, PidPGain::new(755).unwrap());
        assert_eq!(result, Err(PositionTuneOutcome::AccelerationFitRejected));
    }

    #[test]
    fn floor_matches_foci_215_rotor_only_worked_example() {
        // FOCI-215 archived crosscheck's implied accel-per-torque (rotor-only
        // inertia, physical Kt=0.15 N*m/A, J=8.5e-6 kg*m^2, Kadc=1.155e-3
        // A/count) is ~20.38 rad/s^2/count, i.e. ~194.6 RPM/s/count. This
        // fixture rounds that to a clean 195 RPM/s/count and a commanded
        // torque step of 1000 raw counts, giving samples with an *exact*
        // integer slope of 195 RPM per 1000us (no rounding at any sample
        // point): velocity(t_us) = 195_000 * t_us / 1_000_000.
        let commanded_torque_count = 1000_i32;
        let samples: [(u32, i32); 5] = [
            (0, 0),
            (1_000, 195),
            (2_000, 390),
            (3_000, 585),
            (4_000, 780),
        ];
        let velocity_p = PidPGain::new(350).unwrap();
        let result = compute_position_floor(&samples, commanded_torque_count, velocity_p)
            .expect("valid fit");
        // Verified: accel_per_count=195, bw_inner_rad_s = (350*195)/256 = 266,
        // floor_raw = ceil(266*60*256/65536) = 63 exactly.
        assert_eq!(result.floor.raw(), 63);
        assert_eq!(result.bw_inner_rad_s, 266);
    }

    #[test]
    fn floor_too_large_reports_distinct_outcome() {
        // A very large slope drives bw_inner, and therefore the floor,
        // past PidPGain::MAX. (Verified: this fixture's slope of 500,000
        // RPM/s over a commanded_torque_count of 1 produces a floor many
        // orders of magnitude past 0x7fff.)
        let samples: [(u32, i32); 3] = [(0, 0), (1_000, 500_000_000), (2_000, 1_000_000_000)];
        let velocity_p = PidPGain::new(755).unwrap();
        let result = compute_position_floor(&samples, 1, velocity_p);
        assert_eq!(
            result,
            Err(PositionTuneOutcome::FloorUnrepresentableTooLarge)
        );
    }
}
