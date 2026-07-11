//! Pure outer commissioning limits and fallback constants.

/// Reference encoder resolution used only for default values before
/// MechanicalId reads the real ABN decoder PPR register.
pub(crate) const REFERENCE_ENCODER_COUNTS_PER_REV: u32 = 4000;

/// MechanicalId safety velocity cap in encoder counts per millisecond.
/// The physical limit is 6 rev/s; MechanicalId setup rescales this once the
/// real encoder resolution is known.
pub(crate) const MECHANICAL_ID_VELOCITY_CAP_COUNTS_PER_MS: u32 =
    mechanical_id_velocity_cap_counts_per_ms(REFERENCE_ENCODER_COUNTS_PER_REV);

/// Default position window: ± encoder counts from start position.
/// The physical limit is 2 revolutions.
pub(super) const POSITION_WINDOW_COUNTS: u32 =
    position_window_counts(REFERENCE_ENCODER_COUNTS_PER_REV);

/// Conservative velocity-loop runtime fallback used when autotuned outer
/// gains exceed the accepted safety envelope. These are the same defaults
/// used by closed-loop entry when no outer gains are configured.
pub(crate) const OUTER_FALLBACK_VELOCITY_P: u16 = 1152;
pub(crate) const OUTER_FALLBACK_VELOCITY_I: u16 = 0;

/// Conservative position-loop runtime fallback used with
/// `OUTER_FALLBACK_VELOCITY_*`.
pub(crate) const OUTER_FALLBACK_POSITION_P: u16 = 640;
pub(crate) const OUTER_FALLBACK_POSITION_I: u16 = 0;

/// Maximum accepted autotuned velocity P gain.
///
/// Keep runtime gains near the known-good default envelope even when SIMC
/// widens lambda to fit high-damping mechanical models.
pub(crate) const OUTER_MAX_VELOCITY_P: u16 = crate::autotune::VELOCITY_SIMC_MAX_P_REG;
pub(crate) const OUTER_MAX_VELOCITY_I: u16 = 512;
pub(crate) const OUTER_MAX_POSITION_P: u16 = OUTER_FALLBACK_POSITION_P * 4;
pub(crate) const OUTER_MAX_POSITION_I: u16 = 128;

pub(crate) const OUTER_WARNING_NONE: u8 = 0;
pub(crate) const OUTER_WARNING_UNSAFE_SYNTH_GAINS: u8 = 2;

/// Starting position P gain: 128 = 0.5 in Q8.8 (conservative).
pub(crate) const POS_TUNE_INITIAL_P: u16 = 128;

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

pub(crate) const fn response_fallback_velocity_limit_counts_per_ms(counts_per_rev: u32) -> u32 {
    counts_per_ms_for_revs_per_second(counts_per_rev, 4)
}

pub(crate) const fn position_window_counts(counts_per_rev: u32) -> u32 {
    counts_per_rev.saturating_mul(2)
}

/// Motor-local position step for position-loop tuning, in ABN encoder counts.
///
/// The physical step is 0.002 revolutions. The TMC position target is written
/// in electrical-position units, while response capture uses ABN counts.
pub(crate) const fn position_tune_step_counts(counts_per_rev: u32) -> u32 {
    let step = (counts_per_rev + 250) / 500;
    if step > 0 { step } else { 1 }
}
