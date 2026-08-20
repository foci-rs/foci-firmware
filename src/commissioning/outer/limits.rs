//! Pure outer commissioning limits and fallback constants.

/// Reference encoder resolution used only for default values before the
/// entering outer phase's setup step reads the real ABN decoder PPR register.
pub(crate) const REFERENCE_ENCODER_COUNTS_PER_REV: u32 = 4000;

/// Default outer-safety velocity cap in encoder counts per millisecond.
/// The physical limit is 6 rev/s; each outer phase's setup step rescales this
/// once the real encoder resolution is known.
pub(crate) const MECHANICAL_ID_VELOCITY_CAP_COUNTS_PER_MS: u32 =
    mechanical_id_velocity_cap_counts_per_ms(REFERENCE_ENCODER_COUNTS_PER_REV);

/// Default position window: ± encoder counts from start position.
/// The physical limit is 2 revolutions.
pub(super) const POSITION_WINDOW_COUNTS: u32 =
    position_window_counts(REFERENCE_ENCODER_COUNTS_PER_REV);

/// Minimum useful motion budget: a peak velocity below this reads as noise
/// rather than commanded motion. Shared by every outer phase's safety
/// envelope through `OuterSafetyLimits::budget_is_usable`.
pub(crate) const MECHANICAL_ID_MIN_PEAK_VELOCITY_COUNTS_PER_MS: u32 = 4;
/// Minimum useful motion budget: travel below this encoder-count window
/// cannot carry a meaningful safety envelope. Shared by every outer phase's
/// safety envelope through `OuterSafetyLimits::budget_is_usable`.
pub(crate) const MECHANICAL_ID_MIN_TRAVEL_COUNTS: u32 = 64;

pub(crate) const OUTER_MAX_VELOCITY_I: u16 = 8_192;

const fn counts_per_ms_for_revs_per_second(counts_per_rev: u32, revs_per_second: u32) -> u32 {
    let counts_per_second = (counts_per_rev as u64).saturating_mul(revs_per_second as u64);
    let rounded = (counts_per_second + 500) / 1000;
    if rounded == 0 {
        1
    } else if rounded > u32::MAX as u64 {
        u32::MAX
    } else {
        rounded as u32
    }
}

pub(crate) const fn mechanical_id_velocity_cap_counts_per_ms(counts_per_rev: u32) -> u32 {
    counts_per_ms_for_revs_per_second(counts_per_rev, 6)
}

pub(crate) const fn position_window_counts(counts_per_rev: u32) -> u32 {
    counts_per_rev.saturating_mul(2)
}
