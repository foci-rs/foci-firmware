//! Pure response-ramp fallback helpers for velocity validation.

use crate::autotune::VelocityValidation;
use crate::commissioning::outer::{OUTER_MAX_VELOCITY_P, response};
use foci_tmc4671::pid::PidPGain;

/// Response-ramp P gain multiplier between attempts.
const VELOCITY_RESPONSE_P_MUL_NUM: u32 = 3;
const VELOCITY_RESPONSE_P_MUL_DEN: u32 = 2;
const VELOCITY_RESPONSE_BREAKAWAY_OVERSHOOT_COUNTS: u32 = 4;
const VELOCITY_RESPONSE_MAX_STEP_PERCENT: u32 = 50;
const VELOCITY_RESPONSE_MIN_BRACKET_P: u16 = 64;
pub(crate) const VELOCITY_RESPONSE_MAX_RETRIES: u8 = 12;

pub(super) fn response_next_velocity_p(current_p: PidPGain) -> Option<PidPGain> {
    let current_raw = u32::from(current_p);
    let next_p = current_raw * VELOCITY_RESPONSE_P_MUL_NUM / VELOCITY_RESPONSE_P_MUL_DEN;
    if next_p > OUTER_MAX_VELOCITY_P as u32 || next_p <= current_raw {
        None
    } else {
        PidPGain::new(next_p as u16).ok()
    }
}

pub(super) fn response_midpoint_p(lower_p: u16, upper_p: u16) -> Option<PidPGain> {
    if lower_p == 0 || upper_p <= lower_p.saturating_add(VELOCITY_RESPONSE_MIN_BRACKET_P) {
        return None;
    }

    PidPGain::new(lower_p + (upper_p - lower_p) / 2).ok()
}

pub(super) fn response_next_gain_limited_p(current_p: PidPGain, upper_p: u16) -> Option<PidPGain> {
    if let Some(mid_p) = response_midpoint_p(current_p.raw(), upper_p) {
        Some(mid_p)
    } else if upper_p == 0 {
        response_next_velocity_p(current_p)
    } else {
        None
    }
}

pub(super) fn response_next_step_target(current_target: u32, velocity_limit: u32) -> Option<u32> {
    if current_target == 0 || velocity_limit == 0 {
        return None;
    }

    let max_target = (velocity_limit * VELOCITY_RESPONSE_MAX_STEP_PERCENT / 100).max(1);
    let next_target = current_target.saturating_mul(2).min(max_target);
    (next_target > current_target).then_some(next_target)
}

pub(super) fn response_scaled_p_for_target(
    lower_p: u16,
    old_target: u32,
    new_target: u32,
) -> Option<PidPGain> {
    if old_target == 0 || new_target == 0 {
        return PidPGain::new(response::VELOCITY_RESPONSE_INITIAL_P).ok();
    }

    let reference_p = lower_p.max(response::VELOCITY_RESPONSE_INITIAL_P);
    let scaled = (reference_p as u32 * old_target).div_ceil(new_target);
    let raw = scaled
        .max(response::VELOCITY_RESPONSE_INITIAL_P as u32)
        .min(OUTER_MAX_VELOCITY_P as u32) as u16;
    PidPGain::new(raw).ok()
}

pub(super) fn response_result_is_gain_limited(
    metrics: &VelocityValidation,
    fwd: &VelocityValidation,
    rev: &VelocityValidation,
    target: u32,
    total_samples: u16,
) -> bool {
    let saturation_permille =
        crate::autotune::saturation_permille(metrics.saturation_samples, total_samples);

    let min_peak = fwd.peak_velocity.min(rev.peak_velocity);
    let max_peak = fwd.peak_velocity.max(rev.peak_velocity);
    let breakaway_peak_cap =
        target.saturating_add(target.max(VELOCITY_RESPONSE_BREAKAWAY_OVERSHOOT_COUNTS));
    let breakaway_stiction = min_peak < target && max_peak <= breakaway_peak_cap;

    // Before the first passing response, only keep increasing P when the
    // capture looks under-driven. A one-sided breakaway hop is also
    // gain-limited: one direction moved a little, the other never overcame
    // stiction. Saturation/chatter still mean the gain is unsafe.
    (metrics.overshoot_permille <= crate::autotune::Permille::new(200) || breakaway_stiction)
        && saturation_permille.is_none_or(|saturation_permille| {
            saturation_permille <= crate::autotune::Permille::new(100)
        })
        && !metrics.zero_speed_chatter
}

#[cfg(test)]
mod tests {
    use crate::autotune::Permille;

    use super::*;

    fn gain_limited_metrics() -> VelocityValidation {
        VelocityValidation {
            rise_time_ms: 1,
            overshoot_permille: Permille::ZERO,
            peak_velocity: 10,
            settling_time_ms: 1,
            zero_speed_chatter: false,
            saturation_samples: 0,
            symmetry_permille: Permille::ZERO,
        }
    }

    #[test]
    fn gain_limited_response_rejects_fractional_saturation_above_ten_percent() {
        let mut metrics = gain_limited_metrics();
        let fwd = gain_limited_metrics();
        let rev = gain_limited_metrics();

        metrics.saturation_samples = 100;
        assert!(response_result_is_gain_limited(
            &metrics, &fwd, &rev, 10, 1000,
        ));

        metrics.saturation_samples = 101;
        assert!(!response_result_is_gain_limited(
            &metrics, &fwd, &rev, 10, 1000,
        ));
    }
}
