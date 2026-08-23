//! Nomination screen and held-out family-eight confirmation for Phase 4.
//!
//! Discovery is an adaptive search: selecting a candidate proportional gain on
//! the same data that then accepts it contaminates the interval. This module
//! implements the two-part remedy from
//! `docs/specs/2026-07-31-foc-213-breakaway-seeded-p-ladder.md` section 7:
//!
//! - [`nominate_confirmation_candidate`]: a pointwise screen, computed purely
//!   from discovery data, that predicts which rung is most likely to survive a
//!   fresh family-eight confirmation and how much margin it has. It never
//!   authorizes Stage C by itself.
//! - [`nominate_confirmation_candidate_from_bounds`]: the same screen, run from
//!   the compact [`DiscoveryRungNominationBounds`] discovery retains per rung
//!   after collection ([`rung_nomination_bounds`]) instead of the raw
//!   observation atoms. Shares its margin math and selection with the
//!   raw-atom screen so the two cannot silently diverge.
//! - [`evaluate_band_confirmation`]: the held-out block. Four fresh
//!   observations per direction, eight strokes, predeclared before motion.
//!   Only this function's `Ok` result may authorize Stage C.
//!
//! The two use different half-width formulas on purpose. The screen inflates
//! its predicted half-width by [`NOMINATION_SCREEN_C_SIGMA`] because it is
//! extrapolating a fresh family-eight standard error from one discovery
//! observation's own `df = 15` estimate. The held-out block measures its own
//! family-eight standard error directly, so `c_sigma` never appears there.

use foci_tmc4671::pid::PidPGain;

use crate::autotune::velocity_measurement::{VelocityDirection, confidence_multiplier_95_df15};

use super::sweep_consensus::{
    CONSENSUS_OBSERVATIONS_PER_DIRECTION, DirectionalConsensus, DirectionalConsensusClass,
    FULL_OBSERVATION_MASK,
};
use super::sweep_evidence::{VelocityObservationClass, direction_report_code};
use super::sweep_plan::{MOVING_STROKE_US, SETTLE_ANALYSIS_BUDGET_US, ZERO_SETTLE_US};

/// Confirmation family size: four fresh observations per direction, eight total.
///
/// Declared directly per spec section 7, never derived from
/// `rung_count * observations_per_direction` (`sweep_analysis.rs:134`), which
/// would give family 4 for a single-rung confirmation plan and silently narrow
/// every interval.
pub const CONFIRMATION_FAMILY_SIZE: u16 = 8;
/// Fresh observations collected per direction during confirmation.
pub const CONFIRMATION_OBSERVATIONS_PER_DIRECTION: u16 = 4;
/// Inclusive lower edge of the confirmable proportional-response band, in percent.
pub const CONFIRMATION_BAND_LOWER_PERCENT: u8 = 70;
/// Inclusive upper edge of the confirmable proportional-response band, in percent.
pub const CONFIRMATION_BAND_UPPER_PERCENT: u8 = 80;
/// Stable wire code for the only admitted confirmation capture profile.
///
/// Spec section 7, "Noise levers": "The initial implementation permits only
/// the existing capture profile." A longer profile requires its own
/// profile-specific SE prediction rule and effective-df constants before it
/// can be admitted, so this code has exactly one valid value today.
pub const CONFIRMATION_CAPTURE_PROFILE_EXISTING: u8 = 0;
/// Stable wire code for the only admitted held-out acceptance rule: both
/// directions at complete `collected == eligible == 0b1111` masks and all eight
/// fresh intervals wholly inside the band.
///
/// The code is unchanged at 0. This rule has always been "all eight
/// contained"; briefly also demanding a full depth-three `included_mask` was a
/// defect in implementing it, not a different rule, so captures carrying
/// `acceptance_rule = 0` remain replayable.
pub const CONFIRMATION_ACCEPTANCE_RULE_ALL_EIGHT_CONTAINED: u8 = 0;

/// Fixed execution order of the eight held-out confirmation strokes: four
/// forward observations followed by four reverse observations.
pub const CONFIRMATION_SLOT_ORDER: [VelocityDirection; 8] = [
    VelocityDirection::Forward,
    VelocityDirection::Forward,
    VelocityDirection::Forward,
    VelocityDirection::Forward,
    VelocityDirection::Reverse,
    VelocityDirection::Reverse,
    VelocityDirection::Reverse,
    VelocityDirection::Reverse,
];

/// Pointwise 95% scale-inflation heuristic the nomination screen applies on
/// top of the family-eight multiplier: `c_sigma = sqrt(15 / chi2_0.05,15)`.
///
/// This is **not** a coverage guarantee for confirmation, and it is **not**
/// 95% after nomination selects `max_i M_i` across every discovery rung:
/// taking that maximum is exactly the adaptive step that inflates apparent
/// margin, because the rung that looks most confirmable is disproportionately
/// the one whose discovery standard error happened low. No multiplicity
/// correction is applied on top of it, and none is claimed. It only bounds
/// `sigma / S_d` for one observation's standard error considered in
/// isolation.
///
/// It is **not tunable**: it is derived from `df = 15` (the same degrees of
/// freedom baked into `CONFIDENCE_MULTIPLIERS_95_DF15`) and the existing
/// 95% level, nothing else. The acceptance guarantee comes entirely from the
/// held-out confirmation block in [`evaluate_band_confirmation`], whose
/// family and rule are fixed before its first stroke and are unaffected by
/// how the candidate was chosen.
pub const NOMINATION_SCREEN_C_SIGMA: f32 = 1.4373055;

/// Largest half-width fraction for which containment inside a 10-point band is
/// even achievable by any response location: `1 / 15`.
///
/// Derived in spec section 7, "The confirmable window is narrower than the
/// band": `r_min = 70 / (1 - h)`, `r_max = 80 / (1 + h)`, and `r_min <= r_max`
/// only when `h <= 1/15`.
const MAXIMUM_CONTAINABLE_HALF_WIDTH_FRACTION: f32 = 1.0 / 15.0;

/// Fractional-percentage-point granularity at which interval endpoints are
/// conservatively widened before containment is judged.
///
/// Defends the exact 70/80 boundary comparisons against floating-point
/// rounding noise: the lower endpoint always rounds down and the upper
/// endpoint always rounds up, so no interval can appear narrower, and no
/// margin can appear larger, than its true value.
const INTERVAL_ENDPOINT_ROUNDING_SCALE: f32 = 1.0e4;

const FULL_DIRECTIONAL_OBSERVATION_MASK: u8 = FULL_OBSERVATION_MASK;

/// One raw discovery or confirmation observation's response statistics.
///
/// The denominator (`target_rate_q`) is kept explicit rather than folded into
/// a precomputed ratio, so callers cannot accidentally divide out a previous
/// family multiplier or lose the distinction between the measured mean and
/// the commanded target. `variance_of_mean_q2` is stored family-free
/// (`decision.rs:113-133`): no confidence multiplier has been applied to it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct DiscoveryObservationAtom {
    /// Raw signed settled-rate mean in fixed-point rate units.
    pub mean_rate_q: i64,
    /// Family-free variance of the mean, in squared fixed-point rate units.
    pub variance_of_mean_q2: u128,
    /// Positive commanded-magnitude target rate shared by both directions.
    pub target_rate_q: i64,
    /// Commanded mechanical direction for this observation.
    pub direction: VelocityDirection,
}

/// Failure classifying one observation's percent response and relative SE.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(super) enum ObservationStatisticError {
    /// `target_rate_q` was not a positive magnitude.
    InvalidTarget,
    /// A zero or wrong-sign mean is not representable as a candidate response.
    NotRepresentable,
    /// A computed quantity was not finite.
    Arithmetic,
}

/// One observation's percent response and relative standard error.
#[derive(Debug, Clone, Copy)]
pub(super) struct ObservationStatistic {
    /// `r = 100 * commanded_side(mean_rate_q) / target_rate_q`, in percent.
    pub(super) response_percent: f32,
    /// `SE = sqrt(variance_of_mean_q2) / abs(mean_rate_q)`, as a fraction.
    pub(super) relative_standard_error: f32,
}

/// Sign-normalize one raw mean rate onto its commanded-direction magnitude.
///
/// Mirrors the established `commanded_side` convention: `commanded_side_lower_rate_q`
/// in `recovery.rs` and the match in `sweep_evidence.rs:847`. A forward
/// observation must show a strictly positive mean rate and a reverse
/// observation a strictly negative one. A zero or wrong-sign mean is not
/// representable as a candidate response and returns `None`.
const fn commanded_side_mean_rate_q(direction: VelocityDirection, mean_rate_q: i64) -> Option<u64> {
    match direction {
        VelocityDirection::Forward if mean_rate_q > 0 => Some(mean_rate_q as u64),
        VelocityDirection::Reverse if mean_rate_q < 0 => Some(mean_rate_q.unsigned_abs()),
        VelocityDirection::Forward | VelocityDirection::Reverse => None,
    }
}

/// Compute one observation's percent response and relative standard error.
///
/// `pub(super)` (rather than private) so the additive discovery driver's own
/// stop-rule containment check (`breakaway_campaign.rs`) can share this exact
/// computation instead of forking it. See
/// [`rung_response_bounds_percent`]'s doc comment for how the two callers
/// diverge afterward.
pub(super) fn observation_statistic(
    atom: DiscoveryObservationAtom,
) -> Result<ObservationStatistic, ObservationStatisticError> {
    if atom.target_rate_q <= 0 {
        return Err(ObservationStatisticError::InvalidTarget);
    }
    let commanded_magnitude_q = commanded_side_mean_rate_q(atom.direction, atom.mean_rate_q)
        .ok_or(ObservationStatisticError::NotRepresentable)?;
    let variance = atom.variance_of_mean_q2 as f32;
    if !variance.is_finite() {
        return Err(ObservationStatisticError::Arithmetic);
    }
    let commanded_magnitude = commanded_magnitude_q as f32;
    let target_magnitude = atom.target_rate_q as f32;
    let response_percent = 100.0 * commanded_magnitude / target_magnitude;
    let relative_standard_error = libm::sqrtf(variance) / commanded_magnitude;
    if !response_percent.is_finite() || !relative_standard_error.is_finite() {
        return Err(ObservationStatisticError::Arithmetic);
    }
    Ok(ObservationStatistic {
        response_percent,
        relative_standard_error,
    })
}

/// Round one interval's lower endpoint outward (down), never inward.
fn round_outward_lower(value: f32) -> f32 {
    libm::floorf(value * INTERVAL_ENDPOINT_ROUNDING_SCALE) / INTERVAL_ENDPOINT_ROUNDING_SCALE
}

/// Round one interval's upper endpoint outward (up), never inward.
fn round_outward_upper(value: f32) -> f32 {
    libm::ceilf(value * INTERVAL_ENDPOINT_ROUNDING_SCALE) / INTERVAL_ENDPOINT_ROUNDING_SCALE
}

/// Build one observation's percent-response interval: `h = multiplier * SE *
/// c_sigma`, then `[r * (1 - h), r * (1 + h)]`, optionally rounded outward.
///
/// `pub(super)` so both callers of [`rung_response_bounds_percent`] build the
/// SAME per-observation interval, differing only in the two parameters that
/// carry a real, documented statistical meaning: `c_sigma` (an inflation
/// factor predicting a FUTURE held-out interval from this one discovery
/// observation) and `round_outward` (whether the 70/80 boundary comparison
/// defends against floating-point rounding noise). See
/// [`rung_response_bounds_percent`]'s doc comment for how confirmation and
/// discovery set these two parameters differently, and why.
pub(super) fn observation_interval_percent(
    atom: DiscoveryObservationAtom,
    multiplier: f32,
    c_sigma: f32,
    round_outward: bool,
) -> Option<(f32, f32)> {
    let statistic = observation_statistic(atom).ok()?;
    let half_width_fraction = multiplier * statistic.relative_standard_error * c_sigma;
    if !half_width_fraction.is_finite() {
        return None;
    }
    let low_raw = statistic.response_percent * (1.0 - half_width_fraction);
    let high_raw = statistic.response_percent * (1.0 + half_width_fraction);
    let (low, high) = if round_outward {
        (round_outward_lower(low_raw), round_outward_upper(high_raw))
    } else {
        (low_raw, high_raw)
    };
    Some((low, high))
}

/// Frozen discovery evidence for one gain rung's nomination screen.
///
/// Only observations selected by each direction's depth-three `included_mask`
/// contribute. Both `forward_consensus` and `reverse_consensus` must report
/// [`DirectionalConsensusClass::EligibleConsensus`] for the rung to be
/// nominatable at all.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct DiscoveryRungNominationInput {
    /// Proportional gain used at this rung.
    pub p: PidPGain,
    /// Forward-direction depth-three consensus for this rung.
    pub forward_consensus: DirectionalConsensus,
    /// Forward observations in declared schedule order; `None` where uncollected.
    pub forward_observations:
        [Option<DiscoveryObservationAtom>; CONSENSUS_OBSERVATIONS_PER_DIRECTION],
    /// Reverse-direction depth-three consensus for this rung.
    pub reverse_consensus: DirectionalConsensus,
    /// Reverse observations in declared schedule order; `None` where uncollected.
    pub reverse_observations:
        [Option<DiscoveryObservationAtom>; CONSENSUS_OBSERVATIONS_PER_DIRECTION],
}

/// One in-band rung's nomination interval and eligibility, computed once at
/// discovery collection time and retained in place of its raw observation
/// atoms.
///
/// Nomination consumes only the candidate `p`, the enveloped nomination
/// interval `(lower_percent, upper_percent)`, and whether the rung is
/// nominatable at all -- nothing else about the raw atoms is needed once
/// [`rung_nomination_bounds`] has folded them. `lower_percent`/`upper_percent`
/// are computed with the NOMINATION parameters (the family-eight multiplier,
/// [`NOMINATION_SCREEN_C_SIGMA`], outward rounding), the same parameters
/// [`nominate_confirmation_candidate`]'s own [`rung_confirmable_margin_percent`]
/// uses -- see [`rung_nomination_bounds`] for the equivalence this depends on.
/// When `nominatable` is `false`, `lower_percent`/`upper_percent` carry no
/// meaning and must not be read.
#[derive(Debug, Clone, Copy, PartialEq, defmt::Format)]
pub struct DiscoveryRungNominationBounds {
    /// Proportional gain used at this rung.
    pub p: PidPGain,
    /// Enveloped nomination-interval lower bound `L_i`, in percent.
    pub lower_percent: f32,
    /// Enveloped nomination-interval upper bound `U_i`, in percent.
    pub upper_percent: f32,
    /// Whether this rung is nominatable at all: both directions reported
    /// [`DirectionalConsensusClass::EligibleConsensus`] and every included
    /// observation was present with a representable (nonzero, correctly
    /// signed) mean.
    pub nominatable: bool,
}

/// Fold one rung's raw discovery evidence into its compact nomination bounds,
/// using the NOMINATION parameters: the family-eight multiplier,
/// [`NOMINATION_SCREEN_C_SIGMA`], and outward rounding -- the same three
/// parameters [`rung_confirmable_margin_percent`] uses internally. Intended to
/// run once per rung at discovery collection time, so the raw atoms can be
/// dropped afterward: see `breakaway_campaign.rs`'s `CollectedInBandRungs`.
///
/// When the rung is not nominatable (either direction lacks
/// `EligibleConsensus`, an included observation is missing, or an included
/// observation's mean is zero or wrong-signed), `nominatable` is `false` and
/// `lower_percent`/`upper_percent` are both `0.0` placeholders that must not
/// be read.
pub fn rung_nomination_bounds(
    rung: &DiscoveryRungNominationInput,
) -> DiscoveryRungNominationBounds {
    let multiplier = confidence_multiplier_95_df15(CONFIRMATION_FAMILY_SIZE)
        .expect("confirmation family size 8 is within the audited multiplier table");
    match rung_response_bounds_percent(rung, multiplier, NOMINATION_SCREEN_C_SIGMA, true) {
        Some((lower_percent, upper_percent)) => DiscoveryRungNominationBounds {
            p: rung.p,
            lower_percent,
            upper_percent,
            nominatable: true,
        },
        None => DiscoveryRungNominationBounds {
            p: rung.p,
            lower_percent: 0.0,
            upper_percent: 0.0,
            nominatable: false,
        },
    }
}

/// The largest discovery rung nominated to spend the eight held-out strokes on.
#[derive(Debug, Clone, Copy, PartialEq, defmt::Format)]
pub struct NominatedConfirmationCandidate {
    /// Nominated proportional gain.
    pub p: PidPGain,
    /// `M_i = min(L_i - 70, 80 - U_i)` for the nominated rung, in percent.
    pub margin_percent: f32,
}

/// Reported when no discovery rung's pointwise margin `M_i` reaches zero.
///
/// Zero confirmation strokes are spent: nothing downstream may construct a
/// [`BandConfirmationPlan`] without a nominated candidate gain.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct TargetBandUnconfirmableFromDiscovery;

/// Fold one rung's included discovery observations (both directions) into a
/// single combined percent-response interval.
///
/// Returns `None` when the rung is not judgeable at all: either direction
/// lacks `EligibleConsensus`, an included observation is missing, or an
/// included observation's mean is zero or wrong-signed and therefore not
/// representable as a candidate response.
///
/// `pub(super)`, parameterized by `c_sigma` and `round_outward`, so this one
/// aggregation is shared rather than forked:
///
/// - Confirmation's own [`rung_confirmable_margin_percent`] calls this with
///   `c_sigma = `[`NOMINATION_SCREEN_C_SIGMA`]` and `round_outward = true`: it
///   is the conservative gate that actually authorizes Stage C, so it inflates
///   for a FUTURE held-out family-eight test and defends the exact 70/80
///   boundary against floating-point rounding noise.
/// - The additive discovery driver's own stop-rule containment check
///   (`breakaway_campaign.rs::classify_rung_band`) calls this with `c_sigma =
///   1.0` and `round_outward = false`: discovery only decides which rungs to
///   collect and hand to nomination, it never authorizes Stage C itself, and
///   under-collecting a borderline-viable rung here is worse than collecting
///   one that nomination's own (conservative) screen later rejects. The two
///   callers' numeric results are therefore deliberately different at the band
///   edge -- this parameterization is what keeps that difference intentional
///   and in one place instead of silent, undocumented drift between two copies
///   of the same math.
pub(super) fn rung_response_bounds_percent(
    rung: &DiscoveryRungNominationInput,
    multiplier: f32,
    c_sigma: f32,
    round_outward: bool,
) -> Option<(f32, f32)> {
    if rung.forward_consensus.classification != DirectionalConsensusClass::EligibleConsensus
        || rung.reverse_consensus.classification != DirectionalConsensusClass::EligibleConsensus
    {
        return None;
    }

    let mut lower: Option<f32> = None;
    let mut upper: Option<f32> = None;
    for index in 0..CONSENSUS_OBSERVATIONS_PER_DIRECTION {
        if rung.forward_consensus.included_mask & (1 << index) != 0 {
            let atom = rung.forward_observations[index]?;
            let (low, high) =
                observation_interval_percent(atom, multiplier, c_sigma, round_outward)?;
            lower = Some(lower.map_or(low, |current| current.min(low)));
            upper = Some(upper.map_or(high, |current| current.max(high)));
        }
        if rung.reverse_consensus.included_mask & (1 << index) != 0 {
            let atom = rung.reverse_observations[index]?;
            let (low, high) =
                observation_interval_percent(atom, multiplier, c_sigma, round_outward)?;
            lower = Some(lower.map_or(low, |current| current.min(low)));
            upper = Some(upper.map_or(high, |current| current.max(high)));
        }
    }

    Some((lower?, upper?))
}

/// Fold one rung's included discovery observations into per-direction
/// inverse-variance-pooled percent-response intervals.
///
/// Unlike [`rung_response_bounds_percent`] (a min/max union, kept unchanged
/// for nomination and confirmation-margin math), this pools each direction's
/// included observations by inverse variance, so more evidence narrows the
/// interval instead of widening it. Used only by discovery's own collection
/// stop-rule (`breakaway_campaign.rs::classify_rung_band`); must not be
/// reused for nomination or confirmation.
///
/// Returns `None` under the same unjudgeable conditions as
/// [`rung_response_bounds_percent`]: ineligible consensus, a missing included
/// observation, or a zero/wrong-signed mean.
pub(super) fn rung_directional_pooled_bounds_percent(
    rung: &DiscoveryRungNominationInput,
    multiplier: f32,
    c_sigma: f32,
) -> Option<[(f32, f32); 2]> {
    let forward = pooled_direction_interval_percent(
        rung.forward_consensus,
        &rung.forward_observations,
        multiplier,
        c_sigma,
    )?;
    let reverse = pooled_direction_interval_percent(
        rung.reverse_consensus,
        &rung.reverse_observations,
        multiplier,
        c_sigma,
    )?;
    Some([forward, reverse])
}

/// Inverse-variance pool one direction's included observations into one
/// percent-response interval.
///
/// A zero-variance observation is exact and dominates any nonzero-variance
/// ones included alongside it. Conflicting exact observations return `None`.
fn pooled_direction_interval_percent(
    consensus: DirectionalConsensus,
    observations: &[Option<DiscoveryObservationAtom>; CONSENSUS_OBSERVATIONS_PER_DIRECTION],
    multiplier: f32,
    c_sigma: f32,
) -> Option<(f32, f32)> {
    if consensus.classification != DirectionalConsensusClass::EligibleConsensus {
        return None;
    }

    let mut weighted_magnitude_sum = 0.0_f32;
    let mut weight_sum = 0.0_f32;
    let mut exact: Option<(i64, f32)> = None;
    let mut target_magnitude: Option<f32> = None;
    for (index, observation) in observations.iter().enumerate() {
        if consensus.included_mask & (1 << index) == 0 {
            continue;
        }
        let atom = (*observation)?;
        if atom.target_rate_q <= 0 {
            return None;
        }
        let magnitude = commanded_side_mean_rate_q(atom.direction, atom.mean_rate_q)? as f32;
        let variance = atom.variance_of_mean_q2 as f32;
        if !magnitude.is_finite() || !variance.is_finite() {
            return None;
        }
        if target_magnitude.is_none() {
            target_magnitude = Some(atom.target_rate_q as f32);
        }
        if variance == 0.0 {
            match exact {
                Some((existing_mean, _)) if existing_mean != atom.mean_rate_q => return None,
                _ => exact = Some((atom.mean_rate_q, magnitude)),
            }
        } else {
            let weight = 1.0 / variance;
            if !weight.is_finite() {
                return None;
            }
            weighted_magnitude_sum += magnitude * weight;
            weight_sum += weight;
        }
    }
    let target_magnitude = target_magnitude?;

    let (pooled_magnitude, pooled_relative_se) = if let Some((_, magnitude)) = exact {
        (magnitude, 0.0)
    } else {
        if weight_sum <= 0.0 || !weight_sum.is_finite() {
            return None;
        }
        let pooled_magnitude = weighted_magnitude_sum / weight_sum;
        let pooled_variance = 1.0 / weight_sum;
        if !pooled_magnitude.is_finite() || !pooled_variance.is_finite() || pooled_magnitude <= 0.0
        {
            return None;
        }
        (
            pooled_magnitude,
            libm::sqrtf(pooled_variance) / pooled_magnitude,
        )
    };

    let response_percent = 100.0 * pooled_magnitude / target_magnitude;
    let half_width_fraction = multiplier * pooled_relative_se * c_sigma;
    if !response_percent.is_finite() || !half_width_fraction.is_finite() {
        return None;
    }
    let low = response_percent * (1.0 - half_width_fraction);
    let high = response_percent * (1.0 + half_width_fraction);
    if !low.is_finite() || !high.is_finite() {
        return None;
    }
    Some((low, high))
}

/// Fold one rung's discovery evidence into its pointwise confirmable margin.
///
/// Returns `None` when the rung is not nominatable at all: either direction
/// lacks `EligibleConsensus`, an included observation is missing, or an
/// included observation's mean is zero or wrong-signed and therefore not
/// representable as a candidate response. Unchanged in behavior from before
/// [`rung_response_bounds_percent`] was extracted -- same `c_sigma`, same
/// outward rounding, same math.
fn rung_confirmable_margin_percent(
    rung: &DiscoveryRungNominationInput,
    multiplier: f32,
) -> Option<f32> {
    let (lower, upper) =
        rung_response_bounds_percent(rung, multiplier, NOMINATION_SCREEN_C_SIGMA, true)?;
    Some(margin_percent_from_bounds(lower, upper))
}

/// `M_i = min(L_i - 70, 80 - U_i)` for one rung's enveloped nomination
/// interval, in percent.
///
/// Shared by [`rung_confirmable_margin_percent`] (raw-atom path) and
/// [`nominate_confirmation_candidate_from_bounds`] (the reduced
/// [`DiscoveryRungNominationBounds`] path) so the margin formula cannot drift
/// between the two: both compute `(lower, upper)` from the same NOMINATION
/// parameters -- see [`rung_nomination_bounds`] -- then fold them through this
/// one function.
fn margin_percent_from_bounds(lower_percent: f32, upper_percent: f32) -> f32 {
    let lower_margin = lower_percent - f32::from(CONFIRMATION_BAND_LOWER_PERCENT);
    let upper_margin = f32::from(CONFIRMATION_BAND_UPPER_PERCENT) - upper_percent;
    lower_margin.min(upper_margin)
}

/// Select the max-margin candidate from `(p, margin_percent)` pairs, breaking
/// exact ties toward the higher gain and excluding negative margins.
///
/// Shared by [`nominate_confirmation_candidate`] (raw-atom evidence) and
/// [`nominate_confirmation_candidate_from_bounds`] (the reduced
/// [`DiscoveryRungNominationBounds`] representation) so the two nomination
/// paths cannot silently diverge: both feed this one selection loop, differing
/// only in how their `(p, margin_percent)` pairs are produced.
fn select_nominated_candidate<I>(
    candidates: I,
) -> Result<NominatedConfirmationCandidate, TargetBandUnconfirmableFromDiscovery>
where
    I: Iterator<Item = (PidPGain, f32)>,
{
    let mut best: Option<NominatedConfirmationCandidate> = None;
    for (p, margin_percent) in candidates {
        if margin_percent < 0.0 {
            continue;
        }
        let replace = match best {
            None => true,
            Some(current) => {
                margin_percent > current.margin_percent
                    || (margin_percent == current.margin_percent && p.raw() > current.p.raw())
            }
        };
        if replace {
            best = Some(NominatedConfirmationCandidate { p, margin_percent });
        }
    }
    best.ok_or(TargetBandUnconfirmableFromDiscovery)
}

/// Screen every discovery rung and nominate the confirmable candidate with the
/// largest pointwise margin `M_i`, breaking exact ties toward the higher gain.
///
/// See spec section 7, "Nomination screen, computed from discovery data only".
/// A rung is confirmable from discovery when `M_i >= 0`. When no rung
/// qualifies, this returns [`TargetBandUnconfirmableFromDiscovery`] and no
/// confirmation strokes may be spent.
pub fn nominate_confirmation_candidate(
    rungs: &[DiscoveryRungNominationInput],
) -> Result<NominatedConfirmationCandidate, TargetBandUnconfirmableFromDiscovery> {
    let multiplier = confidence_multiplier_95_df15(CONFIRMATION_FAMILY_SIZE)
        .expect("confirmation family size 8 is within the audited multiplier table");
    select_nominated_candidate(rungs.iter().filter_map(|rung| {
        rung_confirmable_margin_percent(rung, multiplier)
            .map(|margin_percent| (rung.p, margin_percent))
    }))
}

/// Screen every collected [`DiscoveryRungNominationBounds`] and nominate the
/// confirmable candidate with the largest pointwise margin `M_i`, breaking
/// exact ties toward the higher gain.
///
/// Runs the SAME selection as [`nominate_confirmation_candidate`] --
/// [`select_nominated_candidate`] -- but from the compact bounds discovery
/// retains after collection instead of recomputing from raw observation
/// atoms. A non-nominatable rung (`nominatable == false`) is excluded, exactly
/// as a rung that fails [`rung_response_bounds_percent`] is excluded from the
/// raw-atom path. When no rung qualifies, this returns
/// [`TargetBandUnconfirmableFromDiscovery`] and no confirmation strokes may be
/// spent.
pub fn nominate_confirmation_candidate_from_bounds(
    bounds: &[DiscoveryRungNominationBounds],
) -> Result<NominatedConfirmationCandidate, TargetBandUnconfirmableFromDiscovery> {
    select_nominated_candidate(
        bounds
            .iter()
            .filter(|entry| entry.nominatable)
            .map(|entry| {
                (
                    entry.p,
                    margin_percent_from_bounds(entry.lower_percent, entry.upper_percent),
                )
            }),
    )
}

/// Frozen fresh evidence for one direction of the held-out confirmation block.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct ConfirmationDirectionalInput {
    /// Depth-three consensus computed over the four fresh confirmation strokes.
    ///
    /// Consulted only for its masks; confirmation acceptance does not reuse
    /// the depth-three inclusion rule the way discovery nomination does.
    pub consensus: DirectionalConsensus,
    /// Per-slot observation classification, in declared schedule order.
    pub classes: [VelocityObservationClass; CONSENSUS_OBSERVATIONS_PER_DIRECTION],
    /// Fresh observations in declared schedule order; `None` where uncollected.
    pub observations: [Option<DiscoveryObservationAtom>; CONSENSUS_OBSERVATIONS_PER_DIRECTION],
}

/// Diagnostic cause reported alongside [`TargetBandConfirmationInconclusive`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum BandConfirmationFailureCause {
    /// A fully formed fresh interval was not wholly inside 70-80%.
    ResponseLocation,
    /// A fresh interval's half-width fraction alone exceeds `1 / 15`, so no
    /// achievable response location could ever satisfy containment.
    ExcessiveUncertainty,
    /// Either direction failed to report complete
    /// `collected_mask == eligible_mask == 0b1111`, or an expected observation
    /// atom was absent, for a reason other than current-headroom exclusion.
    ///
    /// The name predates the acceptance rule dropping `included_mask`; the
    /// wire code ([`super::breakaway_reporting::BreakawayTerminalCause::ConfirmationLossOfConsensus`],
    /// 17) is stable, so it is retained. A split depth-three consensus alone no
    /// longer reaches here.
    LossOfConsensus,
    /// Either direction's incomplete mask traces to at least one observation
    /// classified [`VelocityObservationClass::CurrentHeadroom`].
    CurrentAuthority,
}

/// Reported when the held-out family-eight block fails to authorize Stage C.
///
/// No new P is produced. Callers must retain the previously commissioned P
/// unchanged and must not retry with a second candidate (spec section 7,
/// "Transactional failure behaviour").
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct TargetBandConfirmationInconclusive(pub BandConfirmationFailureCause);

/// Held-out confirmation success: the only value that may authorize Stage C.
///
/// This is an unforgeable capability. Its `candidate_p` field is **private**
/// and this module exposes no public constructor, no `Default`, and no
/// `From`: the only code that may build a `BandConfirmationAcceptance` is
/// [`evaluate_band_confirmation`] in this module, on the path where all eight
/// fresh intervals passed. A module outside `band_confirmation` therefore
/// cannot write `BandConfirmationAcceptance { candidate_p, .. }` or otherwise
/// mint one, so an `Ok(acceptance)` handed to
/// [`super::integral_plan::bind_breakaway_stage_c_authority`] can only have
/// come from a genuine all-eight-contained held-out pass -- it can never be
/// synthesized to bypass the confirmation gate. Read the confirmed gain with
/// [`Self::candidate_p`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct BandConfirmationAcceptance {
    /// Confirmed proportional gain. Private so no sibling module can construct
    /// an acceptance; see the type's own documentation.
    candidate_p: PidPGain,
    /// Per-direction held-out response interval, forward then reverse, in
    /// the SAME signed settled-rate-Q domain `directional_pooled_intervals`
    /// already carries on the combined Stage-C path
    /// (see [`super::combined_plan::CombinedSelectedResponse::response_intervals_q`]):
    /// forward's `[low_q, high_q]` is a strictly
    /// positive commanded-magnitude bound (`low_q > 0`); reverse's is
    /// strictly negative (`high_q < 0`), formed as `[-high_magnitude_q,
    /// -low_magnitude_q]` so `low_q <= high_q` still holds. This is what
    /// [`super::recovery::commanded_side_lower_rate_q`] requires of both
    /// directions, and what the acceptance-matrix installer downstream
    /// ultimately reads this value as.
    ///
    /// Derived from the same per-atom commanded-side magnitude and
    /// `half_width_fraction` [`evaluate_band_confirmation`]'s containment
    /// loop already computes for the percent gate -- applied to the atom's
    /// signed `mean_rate_q` magnitude instead of its percent-of-target, then
    /// pooled per direction (`low` = minimum accepted-atom magnitude bound,
    /// `high` = maximum) exactly the way `rung_response_bounds_percent`
    /// above pools a rung's included atoms into one interval. NOT a
    /// percent-domain value: the 70-80% band gate stays in percent (that
    /// math is unchanged), only the retained, returned interval is in the
    /// rate domain.
    ///
    /// `pub(crate)`, not private: unlike `candidate_p`, this field carries no
    /// unforgeability requirement of its own (the struct as a whole is still
    /// unforgeable because `candidate_p` stays private), and both
    /// [`super::integral_plan::StageCAuthority::new_breakaway`]'s test and the
    /// breakaway campaign need to read it directly from a sibling,
    /// non-descendant module, so a `candidate_p`-style
    /// private-field-plus-accessor pair would only add ceremony here.
    pub(crate) held_out_response_intervals_q: [[i32; 2]; 2],
}

impl BandConfirmationAcceptance {
    /// The confirmed proportional gain this held-out block accepted.
    pub(crate) fn candidate_p(&self) -> PidPGain {
        self.candidate_p
    }
}

/// Whether one direction contributed its complete four-observation schedule
/// with every observation intrinsically eligible.
///
/// `included_mask` is deliberately **not** consulted. It is the depth-three
/// consensus over this direction's `EquivalentTorqueTargetInterval`s -- the
/// disturbance torque -- which is a different measured quantity than the
/// proportional response the confirmation block is testing. Requiring it full
/// would impose unanimous four-way disturbance agreement on top of the stated
/// rule, rejecting blocks whose eight response intervals all satisfy the 70-80%
/// claim. Both hazards spec section 7 names for demanding complete masks are
/// covered without it: an ineligible observation by `eligible_mask`, and an
/// out-of-band one by [`evaluate_band_confirmation`]'s containment loop, which
/// judges all eight observations unconditionally rather than a mask-selected
/// subset. Consensus stays on the wire as diagnostic evidence.
fn directional_input_is_complete(input: &ConfirmationDirectionalInput) -> bool {
    input.consensus.collected_mask == FULL_DIRECTIONAL_OBSERVATION_MASK
        && input.consensus.eligible_mask == FULL_DIRECTIONAL_OBSERVATION_MASK
}

/// Judge the held-out family-eight confirmation block.
///
/// Acceptance requires both directions to report complete
/// `collected_mask == eligible_mask == 0b1111`, every observation atom to be
/// present, and all eight fresh intervals to lie wholly inside 70-80%. The
/// containment loop below judges all eight unconditionally, so no observation
/// can be dropped from the accepted set: any exclusion — stationarity,
/// eligibility, headroom, or otherwise — fails confirmation rather than
/// shrinking it (spec section 7, "Confirmation block").
///
/// The depth-three `included_mask` is not part of the gate; see
/// [`directional_input_is_complete`] for why. `c_sigma` does not appear here
/// either: this is real held-out data, not a discovery-derived prediction, so
/// the interval uses `h = m(8) * SE` directly.
pub fn evaluate_band_confirmation(
    candidate_p: PidPGain,
    forward: ConfirmationDirectionalInput,
    reverse: ConfirmationDirectionalInput,
) -> Result<BandConfirmationAcceptance, TargetBandConfirmationInconclusive> {
    if !directional_input_is_complete(&forward) || !directional_input_is_complete(&reverse) {
        let any_current_headroom = forward
            .classes
            .iter()
            .chain(reverse.classes.iter())
            .any(|class| *class == VelocityObservationClass::CurrentHeadroom);
        let cause = if any_current_headroom {
            BandConfirmationFailureCause::CurrentAuthority
        } else {
            BandConfirmationFailureCause::LossOfConsensus
        };
        return Err(TargetBandConfirmationInconclusive(cause));
    }

    let multiplier = confidence_multiplier_95_df15(CONFIRMATION_FAMILY_SIZE)
        .expect("confirmation family size 8 is within the audited multiplier table");

    let mut excessive_uncertainty = false;
    let mut response_location = false;
    // Per-direction pooled `(low_magnitude_q, high_magnitude_q)`, forward at
    // index 0 and reverse at index 1: the same `half_width_fraction` envelope
    // the percent containment check below uses, applied to the atom's
    // *commanded-side magnitude* (`|mean_rate_q|`) instead of its
    // percent-of-target, then widened by `min`/`max` exactly the way
    // `rung_response_bounds_percent` above pools a rung's included atoms into
    // one interval. Always a positive magnitude regardless of direction --
    // [`commanded_side_mean_rate_q`] already normalizes reverse's negative
    // mean to its magnitude -- so the sign each direction actually needs
    // (forward positive, reverse negative) is applied once, after pooling,
    // below. `None` until this direction's first atom contributes; `Some` by
    // the time the acceptance path reads it, for the same reason
    // `BandConfirmationAcceptance::held_out_response_intervals_q`'s field doc
    // gives: every atom in `[forward, reverse]` is visited, and any atom that
    // fails to contribute a bound (missing, or a statistic error) sets
    // `response_location` and this function returns `Err` before ever
    // constructing `Ok`.
    let mut pooled_magnitude_q: [Option<(f32, f32)>; 2] = [None, None];
    for (direction_index, directional) in [forward, reverse].iter().enumerate() {
        for atom in directional.observations.iter() {
            let Some(atom) = atom else {
                return Err(TargetBandConfirmationInconclusive(
                    BandConfirmationFailureCause::LossOfConsensus,
                ));
            };
            let statistic = match observation_statistic(*atom) {
                Ok(statistic) => statistic,
                Err(_) => {
                    // A zero, wrong-sign, or otherwise malformed mean is not a
                    // representable response: the response location is the
                    // problem, not the uncertainty around it.
                    response_location = true;
                    continue;
                }
            };
            let half_width_fraction = multiplier * statistic.relative_standard_error;
            if half_width_fraction > MAXIMUM_CONTAINABLE_HALF_WIDTH_FRACTION {
                excessive_uncertainty = true;
            }
            let low = round_outward_lower(statistic.response_percent * (1.0 - half_width_fraction));
            let high =
                round_outward_upper(statistic.response_percent * (1.0 + half_width_fraction));
            if low < f32::from(CONFIRMATION_BAND_LOWER_PERCENT)
                || high > f32::from(CONFIRMATION_BAND_UPPER_PERCENT)
            {
                response_location = true;
            }
            // `observation_statistic` above already proved this atom's mean
            // representable under the same direction/mean-rate rule, so this
            // is guaranteed `Some`.
            let commanded_magnitude_q = commanded_side_mean_rate_q(atom.direction, atom.mean_rate_q)
                .expect("observation_statistic already proved this atom's mean is representable")
                as f32;
            let magnitude_low = commanded_magnitude_q * (1.0 - half_width_fraction);
            let magnitude_high = commanded_magnitude_q * (1.0 + half_width_fraction);
            pooled_magnitude_q[direction_index] = Some(match pooled_magnitude_q[direction_index] {
                None => (magnitude_low, magnitude_high),
                Some((pooled_low, pooled_high)) => (
                    pooled_low.min(magnitude_low),
                    pooled_high.max(magnitude_high),
                ),
            });
        }
    }

    if excessive_uncertainty {
        return Err(TargetBandConfirmationInconclusive(
            BandConfirmationFailureCause::ExcessiveUncertainty,
        ));
    }
    if response_location {
        return Err(TargetBandConfirmationInconclusive(
            BandConfirmationFailureCause::ResponseLocation,
        ));
    }

    let held_out_response_intervals_q: [[i32; 2]; 2] = core::array::from_fn(|direction_index| {
        let (magnitude_low, magnitude_high) = pooled_magnitude_q[direction_index]
            .expect("every atom in both directions contributed a bound on the accepted path above");
        let low_magnitude_q = libm::roundf(magnitude_low) as i32;
        let high_magnitude_q = libm::roundf(magnitude_high) as i32;
        if direction_index == 0 {
            // Forward: positive commanded magnitude, low_q > 0.
            [low_magnitude_q, high_magnitude_q]
        } else {
            // Reverse: negate and swap so low_q <= high_q still holds, with
            // high_q < 0 -- the sign `commanded_side_lower_rate_q` requires.
            [-high_magnitude_q, -low_magnitude_q]
        }
    });

    Ok(BandConfirmationAcceptance {
        candidate_p,
        held_out_response_intervals_q,
    })
}

/// Immutable, pre-motion plan for the held-out family-eight confirmation block.
///
/// Freezes every input the acceptance rule depends on before its first
/// stroke: the discovery plan that produced the nominated candidate, the
/// candidate gain itself, the capture profile, the family size, the fixed
/// eight-slot order, the response band, and the acceptance rule. See spec
/// section 7, "Confirmation block".
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct BandConfirmationPlan {
    /// Digest of the discovery plan whose nomination screen selected `candidate_p`.
    pub discovery_plan_digest: u64,
    /// Nominated proportional gain under held-out test.
    pub candidate_p: PidPGain,
    /// Stable wire code for the confirmation capture profile.
    pub capture_profile: u8,
    /// Predeclared confirmation family size. Always [`CONFIRMATION_FAMILY_SIZE`],
    /// never derived from rung count or observation count.
    pub family_size: u16,
    /// Fresh observations collected per direction.
    pub observations_per_direction: u16,
    /// Fixed moving-stroke duration, matching the existing capture profile.
    pub moving_stroke_us: u32,
    /// Fixed zero-target settle duration, matching the existing capture profile.
    pub zero_settle_us: u32,
    /// Cooperative analysis budget, matching the existing capture profile.
    pub settle_analysis_budget_us: u32,
    /// Inclusive lower edge of the confirmable response band, in percent.
    pub band_lower_percent: u8,
    /// Inclusive upper edge of the confirmable response band, in percent.
    pub band_upper_percent: u8,
    /// Stable wire code for the acceptance rule.
    pub acceptance_rule: u8,
    /// Digest of every frozen field above and the fixed eight-slot order.
    pub plan_digest: u64,
}

/// Failure to construct a held-out confirmation plan.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum BandConfirmationPlanError {
    /// The nominated candidate gain was zero or otherwise invalid.
    InvalidRequest,
}

impl BandConfirmationPlan {
    /// Hash every frozen field and the fixed eight-slot order exactly once.
    pub fn digest(self) -> u64 {
        let mut hash = 0xcbf2_9ce4_8422_2325_u64;
        macro_rules! hash_value {
            ($value:expr) => {
                for byte in $value.to_le_bytes() {
                    hash = (hash ^ u64::from(byte)).wrapping_mul(0x0000_0100_0000_01b3);
                }
            };
        }
        hash_value!(self.discovery_plan_digest);
        hash_value!(self.candidate_p.raw());
        hash_value!(self.capture_profile);
        hash_value!(self.family_size);
        hash_value!(self.observations_per_direction);
        hash_value!(self.moving_stroke_us);
        hash_value!(self.zero_settle_us);
        hash_value!(self.settle_analysis_budget_us);
        hash_value!(self.band_lower_percent);
        hash_value!(self.band_upper_percent);
        hash_value!(self.acceptance_rule);
        for direction in CONFIRMATION_SLOT_ORDER {
            hash_value!(direction_report_code(direction));
        }
        hash
    }
}

/// Build the immutable held-out confirmation plan for one nominated candidate.
///
/// Binds `discovery_plan_digest`, the digest of the discovery plan whose
/// nomination screen produced `candidate_p`, so a later Stage-C plan can trace
/// authority back through confirmation to the discovery evidence that
/// selected its candidate.
pub fn plan_band_confirmation(
    discovery_plan_digest: u64,
    candidate_p: PidPGain,
) -> Result<BandConfirmationPlan, BandConfirmationPlanError> {
    if candidate_p == PidPGain::ZERO {
        return Err(BandConfirmationPlanError::InvalidRequest);
    }
    let mut plan = BandConfirmationPlan {
        discovery_plan_digest,
        candidate_p,
        capture_profile: CONFIRMATION_CAPTURE_PROFILE_EXISTING,
        family_size: CONFIRMATION_FAMILY_SIZE,
        observations_per_direction: CONFIRMATION_OBSERVATIONS_PER_DIRECTION,
        moving_stroke_us: MOVING_STROKE_US,
        zero_settle_us: ZERO_SETTLE_US,
        settle_analysis_budget_us: SETTLE_ANALYSIS_BUDGET_US,
        band_lower_percent: CONFIRMATION_BAND_LOWER_PERCENT,
        band_upper_percent: CONFIRMATION_BAND_UPPER_PERCENT,
        acceptance_rule: CONFIRMATION_ACCEPTANCE_RULE_ALL_EIGHT_CONTAINED,
        plan_digest: 0,
    };
    plan.plan_digest = plan.digest();
    Ok(plan)
}

/// The fixed contract field that failed [`validate_fixed_contract`].
///
/// Every variant names a value that is frozen for the only admitted
/// confirmation profile; a plan carrying anything else never drove motion and
/// never authorizes Stage C.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) enum BandConfirmationContractError {
    /// `family_size` was not [`CONFIRMATION_FAMILY_SIZE`].
    FamilySize,
    /// `observations_per_direction` was not
    /// [`CONFIRMATION_OBSERVATIONS_PER_DIRECTION`].
    ObservationsPerDirection,
    /// `capture_profile` was not [`CONFIRMATION_CAPTURE_PROFILE_EXISTING`].
    CaptureProfile,
    /// A timing field (`moving_stroke_us`, `zero_settle_us`, or
    /// `settle_analysis_budget_us`) did not equal the existing capture
    /// profile's value.
    Timing,
    /// `band_lower_percent` was not [`CONFIRMATION_BAND_LOWER_PERCENT`] or
    /// `band_upper_percent` was not [`CONFIRMATION_BAND_UPPER_PERCENT`].
    BandEdge,
    /// `acceptance_rule` was not
    /// [`CONFIRMATION_ACCEPTANCE_RULE_ALL_EIGHT_CONTAINED`].
    AcceptanceRule,
    /// The module's fixed [`CONFIRMATION_SLOT_ORDER`] was not the canonical
    /// four-forward-then-four-reverse schedule the digest binds.
    SlotOrder,
    /// The recomputed [`BandConfirmationPlan::digest`] did not match the stored
    /// `plan_digest`. Because `digest()` folds every frozen field, the
    /// candidate gain, the bound discovery digest, and the fixed slot order,
    /// any tamper the field checks above do not name (a mutated `candidate_p`,
    /// `discovery_plan_digest`, or slot-order contribution) is caught here.
    DigestMismatch,
}

/// The canonical eight-slot confirmation schedule the fixed contract binds:
/// four forward strokes followed by four reverse strokes. Compared against the
/// module's [`CONFIRMATION_SLOT_ORDER`] so an edit to that constant cannot
/// silently redefine the contract the digest is taken over.
const CANONICAL_CONFIRMATION_SLOT_ORDER: [VelocityDirection; 8] = [
    VelocityDirection::Forward,
    VelocityDirection::Forward,
    VelocityDirection::Forward,
    VelocityDirection::Forward,
    VelocityDirection::Reverse,
    VelocityDirection::Reverse,
    VelocityDirection::Reverse,
    VelocityDirection::Reverse,
];

/// Validate every fixed field of a held-out confirmation plan.
///
/// This is the single canonical contract check, called at both sites that
/// consume a [`BandConfirmationPlan`]: before the confirmation phase drives any
/// motion (in `breakaway_campaign.rs`, at plan freeze) and at Stage-C
/// construction in
/// [`super::integral_plan::plan_breakaway_velocity_integral_response`]. Because
/// there is exactly one implementation, the timing, band, family, slot-order,
/// observations-per-direction, and acceptance-rule contract cannot drift
/// between the two checks.
///
/// Every field frozen for the only admitted confirmation profile is verified:
/// `family_size`, `observations_per_direction`, `capture_profile`, all three
/// timing fields, both band edges, the acceptance rule, the fixed slot order,
/// and -- last -- digest self-consistency, which binds `candidate_p`, the
/// discovery digest, and the slot-order contribution the earlier scalar checks
/// do not name individually.
pub(crate) fn validate_fixed_contract(
    plan: &BandConfirmationPlan,
) -> Result<(), BandConfirmationContractError> {
    if plan.family_size != CONFIRMATION_FAMILY_SIZE {
        return Err(BandConfirmationContractError::FamilySize);
    }
    if plan.observations_per_direction != CONFIRMATION_OBSERVATIONS_PER_DIRECTION {
        return Err(BandConfirmationContractError::ObservationsPerDirection);
    }
    if plan.capture_profile != CONFIRMATION_CAPTURE_PROFILE_EXISTING {
        return Err(BandConfirmationContractError::CaptureProfile);
    }
    if plan.moving_stroke_us != MOVING_STROKE_US
        || plan.zero_settle_us != ZERO_SETTLE_US
        || plan.settle_analysis_budget_us != SETTLE_ANALYSIS_BUDGET_US
    {
        return Err(BandConfirmationContractError::Timing);
    }
    if plan.band_lower_percent != CONFIRMATION_BAND_LOWER_PERCENT
        || plan.band_upper_percent != CONFIRMATION_BAND_UPPER_PERCENT
    {
        return Err(BandConfirmationContractError::BandEdge);
    }
    if plan.acceptance_rule != CONFIRMATION_ACCEPTANCE_RULE_ALL_EIGHT_CONTAINED {
        return Err(BandConfirmationContractError::AcceptanceRule);
    }
    if CONFIRMATION_SLOT_ORDER != CANONICAL_CONFIRMATION_SLOT_ORDER {
        return Err(BandConfirmationContractError::SlotOrder);
    }
    if plan.digest() != plan.plan_digest {
        return Err(BandConfirmationContractError::DigestMismatch);
    }
    Ok(())
}

#[cfg(test)]
#[path = "band_confirmation_tests.rs"]
mod tests;
