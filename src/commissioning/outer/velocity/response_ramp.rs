//! Pure response-ramp fallback helpers for velocity validation.

use crate::autotune::VelocityValidation;
use crate::commissioning::outer::{OUTER_MAX_VELOCITY_P, response};

/// Response-ramp P gain multiplier between attempts.
const VELOCITY_RESPONSE_P_MUL_NUM: u32 = 3;
const VELOCITY_RESPONSE_P_MUL_DEN: u32 = 2;
const VELOCITY_RESPONSE_BREAKAWAY_OVERSHOOT_COUNTS: u32 = 4;
const VELOCITY_RESPONSE_MAX_STEP_PERCENT: u32 = 50;
const VELOCITY_RESPONSE_MIN_BRACKET_P: u16 = 64;
pub(super) const VELOCITY_RESPONSE_MAX_RETRIES: u8 = 12;

pub(super) fn response_next_velocity_p(current_p: u16) -> Option<u16> {
    let next_p = current_p as u32 * VELOCITY_RESPONSE_P_MUL_NUM / VELOCITY_RESPONSE_P_MUL_DEN;
    if next_p > OUTER_MAX_VELOCITY_P as u32 || next_p <= current_p as u32 {
        None
    } else {
        Some(next_p as u16)
    }
}

pub(super) fn response_midpoint_p(lower_p: u16, upper_p: u16) -> Option<u16> {
    if lower_p == 0 || upper_p <= lower_p.saturating_add(VELOCITY_RESPONSE_MIN_BRACKET_P) {
        return None;
    }

    Some(lower_p + (upper_p - lower_p) / 2)
}

pub(super) fn response_next_gain_limited_p(current_p: u16, upper_p: u16) -> Option<u16> {
    if let Some(mid_p) = response_midpoint_p(current_p, upper_p) {
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

pub(super) fn response_scaled_p_for_target(lower_p: u16, old_target: u32, new_target: u32) -> u16 {
    if old_target == 0 || new_target == 0 {
        return response::VELOCITY_RESPONSE_INITIAL_P;
    }

    let reference_p = lower_p.max(response::VELOCITY_RESPONSE_INITIAL_P);
    let scaled = (reference_p as u32 * old_target).div_ceil(new_target);
    scaled
        .max(response::VELOCITY_RESPONSE_INITIAL_P as u32)
        .min(OUTER_MAX_VELOCITY_P as u32) as u16
}

pub(super) fn response_result_is_gain_limited(
    metrics: &VelocityValidation,
    fwd: &VelocityValidation,
    rev: &VelocityValidation,
    target: u32,
    total_samples: u16,
) -> bool {
    let sat_pct = if total_samples > 0 {
        (metrics.saturation_samples as u32 * 100) / total_samples as u32
    } else {
        0
    };

    let min_peak = fwd.peak_velocity.min(rev.peak_velocity);
    let max_peak = fwd.peak_velocity.max(rev.peak_velocity);
    let breakaway_peak_cap =
        target.saturating_add(target.max(VELOCITY_RESPONSE_BREAKAWAY_OVERSHOOT_COUNTS));
    let breakaway_stiction = min_peak < target && max_peak <= breakaway_peak_cap;

    // Before the first passing response, only keep increasing P when the
    // capture looks under-driven. A one-sided breakaway hop is also
    // gain-limited: one direction moved a little, the other never overcame
    // stiction. Saturation/chatter still mean the gain is unsafe.
    (metrics.overshoot_pct <= 200 || breakaway_stiction)
        && sat_pct <= 10
        && !metrics.zero_speed_chatter
}
