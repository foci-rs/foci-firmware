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
/// directions at complete `collected == eligible == included == 0b1111` masks
/// and all eight fresh intervals wholly inside the band.
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
    /// `collected_mask == eligible_mask == included_mask == 0b1111`, for a
    /// reason other than current-headroom exclusion.
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
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct BandConfirmationAcceptance {
    /// Confirmed proportional gain.
    pub candidate_p: PidPGain,
}

fn directional_input_is_complete(input: &ConfirmationDirectionalInput) -> bool {
    input.consensus.collected_mask == FULL_DIRECTIONAL_OBSERVATION_MASK
        && input.consensus.eligible_mask == FULL_DIRECTIONAL_OBSERVATION_MASK
        && input.consensus.included_mask == FULL_DIRECTIONAL_OBSERVATION_MASK
}

/// Judge the held-out family-eight confirmation block.
///
/// Acceptance requires both directions to report complete
/// `collected_mask == eligible_mask == included_mask == 0b1111` and all eight
/// fresh intervals to lie wholly inside 70-80%. Any exclusion — stationarity,
/// consensus, headroom, or otherwise — fails confirmation rather than
/// shrinking the set (spec section 7, "Confirmation block"). `c_sigma` does
/// not appear here: this is real held-out data, not a discovery-derived
/// prediction, so the interval uses `h = m(8) * SE` directly.
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
    for atom in forward
        .observations
        .iter()
        .chain(reverse.observations.iter())
    {
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
        let high = round_outward_upper(statistic.response_percent * (1.0 + half_width_fraction));
        if low < f32::from(CONFIRMATION_BAND_LOWER_PERCENT)
            || high > f32::from(CONFIRMATION_BAND_UPPER_PERCENT)
        {
            response_location = true;
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

    Ok(BandConfirmationAcceptance { candidate_p })
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

#[cfg(test)]
#[path = "band_confirmation_tests.rs"]
mod tests;
