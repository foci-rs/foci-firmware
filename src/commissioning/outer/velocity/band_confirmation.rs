//! Nomination screen and held-out family-eight confirmation for Phase 4.
//!
//! Only the `Ok` result of [`evaluate_band_confirmation`] may authorize Integral; the nomination
//! screens never do.

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
/// Declared directly, never derived from rung count times observations per direction.
pub const CONFIRMATION_FAMILY_SIZE: u16 = 8;
/// Fresh observations collected per direction during confirmation.
pub const CONFIRMATION_OBSERVATIONS_PER_DIRECTION: u16 = 4;
/// Inclusive lower edge of the confirmable proportional-response band, in percent.
pub const CONFIRMATION_BAND_LOWER_PERCENT: u8 = 70;
/// Inclusive upper edge of the confirmable proportional-response band, in percent.
pub const CONFIRMATION_BAND_UPPER_PERCENT: u8 = 80;
/// Stable wire code for the only admitted confirmation capture profile.
pub const CONFIRMATION_CAPTURE_PROFILE_EXISTING: u8 = 0;
/// Stable wire code for the only admitted held-out acceptance rule: both directions at complete
/// `collected == eligible == 0b1111` masks and all eight fresh intervals wholly inside the band.
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

/// A discovery step whose velocity peak exceeds this fraction of its commanded target, in
/// permille, is over-gained: a peak more than 40% over target.
pub const STEP_PEAK_ADMISSION_PERMILLE: u16 = 1400;

/// Pointwise 95% scale-inflation heuristic the nomination screen applies on top of the
/// family-eight multiplier: `c_sigma = sqrt(15 / chi2_0.05,15)`.
///
/// Not a coverage guarantee and not tunable; acceptance comes only from
/// [`evaluate_band_confirmation`].
pub const NOMINATION_SCREEN_C_SIGMA: f32 = 1.4373055;

const MAXIMUM_CONTAINABLE_HALF_WIDTH_FRACTION: f32 = 1.0 / 15.0;

const INTERVAL_ENDPOINT_ROUNDING_SCALE: f32 = 1.0e4;

const FULL_DIRECTIONAL_OBSERVATION_MASK: u8 = FULL_OBSERVATION_MASK;

/// One raw discovery or confirmation observation's response statistics.
///
/// `variance_of_mean_q2` is family-free: no confidence multiplier has been applied.
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

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(super) enum ObservationStatisticError {
    InvalidTarget,
    NotRepresentable,
    Arithmetic,
}

#[derive(Debug, Clone, Copy)]
pub(super) struct ObservationStatistic {
    pub(super) response_percent: f32,
    pub(super) relative_standard_error: f32,
}

const fn commanded_side_mean_rate_q(direction: VelocityDirection, mean_rate_q: i64) -> Option<u64> {
    match direction {
        VelocityDirection::Forward if mean_rate_q > 0 => Some(mean_rate_q as u64),
        VelocityDirection::Reverse if mean_rate_q < 0 => Some(mean_rate_q.unsigned_abs()),
        VelocityDirection::Forward | VelocityDirection::Reverse => None,
    }
}

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

fn round_outward_lower(value: f32) -> f32 {
    libm::floorf(value * INTERVAL_ENDPOINT_ROUNDING_SCALE) / INTERVAL_ENDPOINT_ROUNDING_SCALE
}

fn round_outward_upper(value: f32) -> f32 {
    libm::ceilf(value * INTERVAL_ENDPOINT_ROUNDING_SCALE) / INTERVAL_ENDPOINT_ROUNDING_SCALE
}

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

/// One in-band rung's nomination interval and eligibility, retained in place of its raw
/// observation atoms and computed by [`rung_nomination_bounds`].
///
/// When `nominatable` is `false`, `lower_percent`/`upper_percent` carry no meaning and must not
/// be read.
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
    /// Largest step velocity peak over the observations the rung's consensus included, in
    /// permille of the commanded target. `0` when no included step was scored.
    pub step_peak_permille: u16,
}

/// Fold one rung's raw discovery evidence into its compact nomination bounds.
///
/// When the rung is not nominatable, `nominatable` is `false` and both percents are `0.0`
/// placeholders that must not be read.
pub fn rung_nomination_bounds(
    rung: &DiscoveryRungNominationInput,
    step_peak_permille: u16,
) -> DiscoveryRungNominationBounds {
    let multiplier = confidence_multiplier_95_df15(CONFIRMATION_FAMILY_SIZE)
        .expect("confirmation family size 8 is within the audited multiplier table");
    match rung_response_bounds_percent(rung, multiplier, NOMINATION_SCREEN_C_SIGMA, true) {
        Some((lower_percent, upper_percent)) => DiscoveryRungNominationBounds {
            p: rung.p,
            lower_percent,
            upper_percent,
            nominatable: true,
            step_peak_permille,
        },
        None => DiscoveryRungNominationBounds {
            p: rung.p,
            lower_percent: 0.0,
            upper_percent: 0.0,
            nominatable: false,
            step_peak_permille: 0,
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
    /// Largest step velocity peak of the nominated rung, in permille of target; `0` when unscored.
    pub step_peak_permille: u16,
    /// Rungs that were band-admissible and transition-capable but exceeded
    /// [`STEP_PEAK_ADMISSION_PERMILLE`].
    pub overshoot_excluded_rungs: u8,
}

/// Reported when no discovery rung's pointwise margin `M_i` reaches zero.
///
/// Zero confirmation strokes are spent: nothing downstream may construct a
/// [`BandConfirmationPlan`] without a nominated candidate gain.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct TargetBandUnconfirmableFromDiscovery;

/// Largest step velocity peak, in permille of target, over the slots each direction's consensus
/// included. Returns `0` when no included slot has a nonzero peak.
pub fn rung_step_peak_permille(
    forward_included_mask: u8,
    forward_peaks: &[u16; CONSENSUS_OBSERVATIONS_PER_DIRECTION],
    reverse_included_mask: u8,
    reverse_peaks: &[u16; CONSENSUS_OBSERVATIONS_PER_DIRECTION],
) -> u16 {
    let included = |mask: u8, peaks: &[u16; CONSENSUS_OBSERVATIONS_PER_DIRECTION]| {
        peaks
            .iter()
            .enumerate()
            .filter(move |(index, _)| mask & (1 << index) != 0)
            .map(|(_, peak)| *peak)
            .max()
            .unwrap_or(0)
    };
    included(forward_included_mask, forward_peaks)
        .max(included(reverse_included_mask, reverse_peaks))
}

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

fn rung_confirmable_margin_percent(
    rung: &DiscoveryRungNominationInput,
    multiplier: f32,
) -> Option<f32> {
    let (lower, upper) =
        rung_response_bounds_percent(rung, multiplier, NOMINATION_SCREEN_C_SIGMA, true)?;
    Some(margin_percent_from_bounds(lower, upper))
}

fn margin_percent_from_bounds(lower_percent: f32, upper_percent: f32) -> f32 {
    let lower_margin = lower_percent - f32::from(CONFIRMATION_BAND_LOWER_PERCENT);
    let upper_margin = f32::from(CONFIRMATION_BAND_UPPER_PERCENT) - upper_percent;
    lower_margin.min(upper_margin)
}

fn select_nominated_candidate<I>(
    candidates: I,
) -> Result<NominatedConfirmationCandidate, TargetBandUnconfirmableFromDiscovery>
where
    I: Iterator<Item = (PidPGain, f32, u16)>,
{
    let mut best: Option<NominatedConfirmationCandidate> = None;
    for (p, margin_percent, step_peak_permille) in candidates {
        defmt::info!(
            "nomination screen: p_raw={} margin_percent={} step_peak_permille={}",
            p.raw(),
            margin_percent,
            step_peak_permille,
        );
        if margin_percent < 0.0 {
            continue;
        }
        let replace = match best {
            None => true,
            Some(current) => p.raw() > current.p.raw(),
        };
        if replace {
            best = Some(NominatedConfirmationCandidate {
                p,
                margin_percent,
                step_peak_permille,
                overshoot_excluded_rungs: 0,
            });
        }
    }
    best.ok_or(TargetBandUnconfirmableFromDiscovery)
}

/// Screen every discovery rung and nominate the highest band-admissible rung (`M_i >= 0`).
///
/// Returns [`TargetBandUnconfirmableFromDiscovery`] when no rung qualifies.
pub fn nominate_confirmation_candidate(
    rungs: &[DiscoveryRungNominationInput],
) -> Result<NominatedConfirmationCandidate, TargetBandUnconfirmableFromDiscovery> {
    let multiplier = confidence_multiplier_95_df15(CONFIRMATION_FAMILY_SIZE)
        .expect("confirmation family size 8 is within the audited multiplier table");
    select_nominated_candidate(rungs.iter().filter_map(|rung| {
        rung_confirmable_margin_percent(rung, multiplier)
            .map(|margin_percent| (rung.p, margin_percent, 0))
    }))
}

/// Screen every [`DiscoveryRungNominationBounds`] and nominate the highest band-admissible rung
/// (`M_i >= 0`); non-nominatable rungs are excluded.
///
/// Returns [`TargetBandUnconfirmableFromDiscovery`] when no rung qualifies.
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
                    entry.step_peak_permille,
                )
            }),
    )
}

/// Why nomination spent zero confirmation strokes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum NominationRejection {
    /// No discovery rung's response was band-admissible (`M_i >= 0`).
    Unconfirmable,
    /// Band-admissible rungs existed, but every one's proportional gain exceeds
    /// the Integral transition-capability ceiling, so admitting any would clip the
    /// current clamp during Integral. Refused before motion.
    NoTransitionCapableCandidate,
    /// Band-admissible, transition-capable rungs existed, but every one's step peak exceeded
    /// [`STEP_PEAK_ADMISSION_PERMILLE`].
    NoDampedCandidate {
        /// Number of rungs excluded by the step peak bound.
        excluded: u8,
    },
    /// Band-admissible, transition-capable, damped rungs existed, but none had a proportional
    /// gain strictly below the nomination ceiling.
    NoRungBelowCeiling {
        /// The nomination ceiling that excluded every remaining rung.
        ceiling: u16,
        /// Number of rungs excluded by the step peak bound.
        excluded: u8,
    },
}

/// Nominate the highest band-admissible rung whose proportional gain is transition-capable
/// (`p <= transition_ceiling`, where `transition_ceiling` is
/// `super::integral_plan::max_transition_capable_p` for the live acquisition context) and whose
/// step peak is at or under [`STEP_PEAK_ADMISSION_PERMILLE`] (`0`, unscored, passes), and whose
/// proportional gain is strictly below `nomination_p_below` (`0` is unconstrained).
///
/// The ceiling filters only the nominated set; `overshoot_excluded_rungs` counts damping
/// exclusions as if it were absent. The rejection names the first predicate that emptied the
/// candidate set.
pub fn nominate_transition_capable_candidate_from_bounds(
    bounds: &[DiscoveryRungNominationBounds],
    transition_ceiling: u16,
    nomination_p_below: u16,
) -> Result<NominatedConfirmationCandidate, NominationRejection> {
    let band_admissible = |entry: &&DiscoveryRungNominationBounds| {
        entry.nominatable
            && margin_percent_from_bounds(entry.lower_percent, entry.upper_percent) >= 0.0
    };
    let capable = |entry: &&DiscoveryRungNominationBounds| entry.p.raw() <= transition_ceiling;
    let damped = |entry: &&DiscoveryRungNominationBounds| {
        entry.step_peak_permille <= STEP_PEAK_ADMISSION_PERMILLE
    };
    let below = |entry: &&DiscoveryRungNominationBounds| {
        nomination_p_below == 0 || entry.p.raw() < nomination_p_below
    };

    if !bounds.iter().any(|entry| band_admissible(&entry)) {
        return Err(NominationRejection::Unconfirmable);
    }
    let capable_count = bounds
        .iter()
        .filter(band_admissible)
        .filter(capable)
        .count();
    if capable_count == 0 {
        return Err(NominationRejection::NoTransitionCapableCandidate);
    }
    let excluded = bounds
        .iter()
        .filter(band_admissible)
        .filter(capable)
        .filter(|entry| !damped(entry))
        .count();
    let damped_count = capable_count - excluded;
    let excluded = u8::try_from(excluded).unwrap_or(u8::MAX);
    defmt::info!(
        "nomination damping screen: overshoot_excluded_rungs={}",
        excluded
    );
    let nominated = select_nominated_candidate(
        bounds
            .iter()
            .filter(band_admissible)
            .filter(capable)
            .filter(damped)
            .filter(below)
            .map(|entry| {
                (
                    entry.p,
                    margin_percent_from_bounds(entry.lower_percent, entry.upper_percent),
                    entry.step_peak_permille,
                )
            }),
    );
    match nominated {
        Ok(candidate) => Ok(NominatedConfirmationCandidate {
            overshoot_excluded_rungs: excluded,
            ..candidate
        }),
        Err(TargetBandUnconfirmableFromDiscovery) if damped_count == 0 => {
            Err(NominationRejection::NoDampedCandidate { excluded })
        }
        Err(TargetBandUnconfirmableFromDiscovery) => Err(NominationRejection::NoRungBelowCeiling {
            ceiling: nomination_p_below,
            excluded,
        }),
    }
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
    /// Either direction failed to report complete `collected_mask == eligible_mask == 0b1111`, or an
    /// expected observation atom was absent, for a reason other than current-headroom exclusion.
    IncompleteObservations,
    /// Either direction's incomplete mask traces to at least one observation
    /// classified [`VelocityObservationClass::CurrentHeadroom`].
    CurrentAuthority,
}

/// Reported when the held-out family-eight block fails to authorize Integral.
///
/// No new P is produced. Callers must retain the previously commissioned P
/// unchanged and must not retry with a second candidate (spec section 7,
/// "Transactional failure behaviour").
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct TargetBandConfirmationInconclusive(pub BandConfirmationFailureCause);

/// Held-out confirmation success: the only value that may authorize Integral.
///
/// Only [`evaluate_band_confirmation`] can build one. Read the confirmed gain with
/// `Self::nominated_p`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct BandConfirmationAcceptance {
    nominated_p: PidPGain,
    pub(crate) held_out_response_intervals_q: [[i32; 2]; 2],
    pub(crate) confidence_q: u16,
    pub(crate) band_position_q: u16,
}

impl BandConfirmationAcceptance {
    pub(crate) fn nominated_p(&self) -> PidPGain {
        self.nominated_p
    }
}

fn directional_input_is_complete(input: &ConfirmationDirectionalInput) -> bool {
    input.consensus.collected_mask == FULL_DIRECTIONAL_OBSERVATION_MASK
        && input.consensus.eligible_mask == FULL_DIRECTIONAL_OBSERVATION_MASK
}

/// Judge the held-out family-eight confirmation block.
///
/// Acceptance requires both directions to report complete `collected_mask == eligible_mask ==
/// 0b1111`, every observation atom present, and all eight fresh intervals wholly inside 70-80%.
/// Any excluded observation fails confirmation rather than shrinking the set.
pub fn evaluate_band_confirmation(
    nominated_p: PidPGain,
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
            BandConfirmationFailureCause::IncompleteObservations
        };
        return Err(TargetBandConfirmationInconclusive(cause));
    }

    let multiplier = confidence_multiplier_95_df15(CONFIRMATION_FAMILY_SIZE)
        .expect("confirmation family size 8 is within the audited multiplier table");

    let mut excessive_uncertainty = false;
    let mut response_location = false;
    let mut pooled_magnitude_q: [Option<(f32, f32)>; 2] = [None, None];
    let mut worst_half_width_fraction = 0.0_f32;
    let mut response_percent_sum = 0.0_f32;
    let mut response_percent_count = 0_u32;
    for (direction_index, directional) in [forward, reverse].iter().enumerate() {
        for atom in directional.observations.iter() {
            let Some(atom) = atom else {
                return Err(TargetBandConfirmationInconclusive(
                    BandConfirmationFailureCause::IncompleteObservations,
                ));
            };
            let statistic = match observation_statistic(*atom) {
                Ok(statistic) => statistic,
                Err(_) => {
                    response_location = true;
                    continue;
                }
            };
            let half_width_fraction = multiplier * statistic.relative_standard_error;
            if half_width_fraction > MAXIMUM_CONTAINABLE_HALF_WIDTH_FRACTION {
                excessive_uncertainty = true;
            }
            worst_half_width_fraction = worst_half_width_fraction.max(half_width_fraction);
            response_percent_sum += statistic.response_percent;
            response_percent_count += 1;
            let low = round_outward_lower(statistic.response_percent * (1.0 - half_width_fraction));
            let high =
                round_outward_upper(statistic.response_percent * (1.0 + half_width_fraction));
            if low < f32::from(CONFIRMATION_BAND_LOWER_PERCENT)
                || high > f32::from(CONFIRMATION_BAND_UPPER_PERCENT)
            {
                response_location = true;
            }
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
            [low_magnitude_q, high_magnitude_q]
        } else {
            [-high_magnitude_q, -low_magnitude_q]
        }
    });

    let confidence =
        (1.0 - worst_half_width_fraction / MAXIMUM_CONTAINABLE_HALF_WIDTH_FRACTION).clamp(0.0, 1.0);
    let confidence_q = (confidence * 65535.0) as u16;
    let response_percent_count = core::num::NonZeroU32::new(response_percent_count).expect(
        "every atom in both directions contributed a response_percent on the accepted path above",
    );
    let mean_r = response_percent_sum / response_percent_count.get() as f32;
    let span = f32::from(CONFIRMATION_BAND_UPPER_PERCENT - CONFIRMATION_BAND_LOWER_PERCENT);
    let pos = ((mean_r - f32::from(CONFIRMATION_BAND_LOWER_PERCENT)) / span).clamp(0.0, 1.0);
    let band_position_q = (pos * 65535.0) as u16;

    Ok(BandConfirmationAcceptance {
        nominated_p,
        held_out_response_intervals_q,
        confidence_q,
        band_position_q,
    })
}

/// Immutable, pre-motion plan for the held-out family-eight confirmation block.
///
/// Freezes every input the acceptance rule depends on before the first stroke.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct BandConfirmationPlan {
    /// Digest of the discovery plan whose nomination screen selected `nominated_p`.
    pub discovery_plan_digest: u64,
    /// Nominated proportional gain under held-out test.
    pub nominated_p: PidPGain,
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
        hash_value!(self.nominated_p.raw());
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

/// Build the immutable held-out confirmation plan for one nominated candidate, binding the
/// digest of the discovery plan whose nomination screen produced `nominated_p`.
pub fn plan_band_confirmation(
    discovery_plan_digest: u64,
    nominated_p: PidPGain,
) -> Result<BandConfirmationPlan, BandConfirmationPlanError> {
    if nominated_p == PidPGain::ZERO {
        return Err(BandConfirmationPlanError::InvalidRequest);
    }
    let mut plan = BandConfirmationPlan {
        discovery_plan_digest,
        nominated_p,
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

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) enum BandConfirmationContractError {
    FamilySize,
    ObservationsPerDirection,
    CaptureProfile,
    Timing,
    BandEdge,
    AcceptanceRule,
    SlotOrder,
    DigestMismatch,
}

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
