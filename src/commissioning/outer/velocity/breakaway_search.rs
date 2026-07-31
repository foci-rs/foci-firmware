//! Pure, host-testable breakaway probe search and boundary/motion classification.
//!
//! Nothing here moves the axis. This module classifies already-executed probe
//! observations and drives the binary-search index arithmetic that decides which
//! post-dedupe [`super::sweep_plan::gain_at`] grid index to test next. Motion is
//! driven by a later phase, which supplies the classified outcome of each requested
//! probe back through [`BreakawaySearchCursor::advance`].
//!
//! The motion criterion reuses the sweep's own sign-correct interval test
//! ([`classify_velocity_observation`]) rather than a bare encoder-delta comparison.
//! `delta == 0` is exactly the false-`ENCODER_FAULT` shape; a settled-rate
//! confidence interval that excludes zero on the commanded side is not.

use super::sweep_evidence::{
    VelocityObservationClass, VelocityObservationInput, classify_velocity_observation,
};

/// Sign-correct motion verdict for one probe observation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum BreakawayMotionClass {
    /// The sign-correct settled-rate confidence interval still contains zero.
    Inert,
    /// The sign-correct settled-rate confidence interval excludes zero.
    Moving,
}

/// Resolved inert/moving bracket for one signed probe direction.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct DirectionalBreakaway {
    /// Post-dedupe grid index of the confirmed inert bracket endpoint, or `None`
    /// when breakaway is already the grid's smallest representable rung and no
    /// lower diagnostic index exists.
    pub inert_index: Option<u8>,
    /// Post-dedupe grid index of the confirmed moving bracket endpoint: this
    /// direction's resolved breakaway.
    pub moving_index: u8,
    /// Total probe observations spent resolving this direction, including the
    /// mandatory bracket-repeat confirmation(s).
    pub observations: u8,
}

/// Failure to resolve a directional or campaign breakaway before motion continues.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum BreakawaySearchError {
    /// A required search input was zero or otherwise unusable.
    InvalidRequest,
    /// Checked search-cursor arithmetic exceeded its representation.
    Arithmetic,
    /// The observation settled significantly opposite the command, or reached the
    /// configured hard controller-output clip. The existing sweep-evidence
    /// classification is returned unchanged.
    SafetyFault(VelocityObservationClass),
    /// Target-independent stationarity evidence, or another controller-domain
    /// evidence field, was unusable for classification.
    ProbeEvidenceExcluded,
    /// The observation reached the inclusive usable-current boundary before a
    /// motion or inert verdict was reached.
    AuthorityFailure,
    /// The bisection search reached the top of the search grid without ever
    /// confirming commanded-side motion.
    NoMotionAtAuthority,
    /// The mandatory bracket-repeat observation contradicted the search's own read
    /// at that grid index.
    UnstableBreakaway,
    /// The two directional breakaways differed by more than one grid step.
    DirectionalBreakawayDisagreement,
}

/// Classify one already-executed probe observation into a motion verdict or a
/// typed probe outcome.
///
/// Delegates to [`classify_velocity_observation`] for the sign-correct interval
/// test, the current-headroom boundary, and every safety and evidence-quality
/// predicate, rather than reimplementing any of them. Reverse-direction
/// observations are sign-normalized onto the commanded side by that same
/// delegation, because `VelocityObservationInput::direction` drives its
/// `wrong_direction` and `moving` predicates.
pub fn classify_breakaway_observation(
    input: VelocityObservationInput,
) -> Result<BreakawayMotionClass, BreakawaySearchError> {
    let summary = classify_velocity_observation(input);
    match summary.classification {
        VelocityObservationClass::Eligible | VelocityObservationClass::DeficitUnresolved => {
            Ok(BreakawayMotionClass::Moving)
        }
        VelocityObservationClass::Inert => Ok(BreakawayMotionClass::Inert),
        VelocityObservationClass::CurrentHeadroom => Err(BreakawaySearchError::AuthorityFailure),
        VelocityObservationClass::StationarityExcluded(_)
        | VelocityObservationClass::InvalidEvidenceFault => {
            Err(BreakawaySearchError::ProbeEvidenceExcluded)
        }
        VelocityObservationClass::WrongDirectionFault | VelocityObservationClass::HardClipFault => {
            Err(BreakawaySearchError::SafetyFault(summary.classification))
        }
    }
}

/// Next action the caller of [`BreakawaySearchCursor`] must take.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum BreakawaySearchStep {
    /// Test this post-dedupe grid index next and fold its classified outcome
    /// back through [`BreakawaySearchCursor::advance`].
    Probe(u8),
    /// The directional search resolved to this confirmed bracket.
    Resolved(DirectionalBreakaway),
}

/// Search stage of one directional [`BreakawaySearchCursor`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
enum SearchStage {
    /// Narrowing the `[lo, hi)` first-moving-index bisection window.
    Bisecting,
    /// Confirming the bisected inert endpoint a second time.
    ConfirmInert,
    /// Confirming the bisected moving endpoint a second time.
    ConfirmMoving,
}

/// Pure directional binary-search cursor over the probe's post-dedupe half-octave
/// search grid.
///
/// This type never drives motion. It only decides which grid index to test next
/// and folds the caller-supplied classified outcome of the most recently requested
/// index. The search finds the smallest grid index whose commanded-side settled
/// rate excludes zero (the "first moving index"), assuming monotonicity: motion at
/// index `i` implies motion at every index above `i`. A non-monotone axis returns a
/// wrong answer without complaint; this is a documented assumption, not something
/// this cursor guards against.
///
/// After the bisection window collapses, the cursor repeats the confirmed inert and
/// moving bracket endpoints once each (the "mandatory bracket repeat") before
/// resolving. A contradictory repeat yields
/// [`BreakawaySearchError::UnstableBreakaway`]. When breakaway is already the
/// grid's smallest index, there is no representable inert endpoint below it; the
/// cursor repeats only the moving endpoint and reports `inert_index: None`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct BreakawaySearchCursor {
    rung_count: u8,
    lo: u8,
    hi: u8,
    observations: u8,
    stage: SearchStage,
    pending_index: u8,
    boundary_inert: Option<u8>,
    boundary_moving: Option<u8>,
}

impl BreakawaySearchCursor {
    /// Construct a fresh cursor and the first grid index the caller must probe.
    ///
    /// `rung_count` may be any value up to [`super::sweep_plan::MAX_SWEEP_RUNGS`]:
    /// [`super::breakaway_plan::PROBE_MAX_OBSERVATIONS_PER_DIRECTION`] budgets a
    /// worst case of 6 binary-search bisections plus the 2 mandatory
    /// bracket-confirmation repeats, which resolves the full grid, so no rung count
    /// needs refusing before motion on observation-budget grounds alone.
    pub fn start(rung_count: u8) -> Result<(Self, u8), BreakawaySearchError> {
        if rung_count == 0 {
            return Err(BreakawaySearchError::InvalidRequest);
        }
        let lo = 0;
        let hi = rung_count;
        let pending_index = bisect_mid(lo, hi);
        Ok((
            Self {
                rung_count,
                lo,
                hi,
                observations: 0,
                stage: SearchStage::Bisecting,
                pending_index,
                boundary_inert: None,
                boundary_moving: None,
            },
            pending_index,
        ))
    }

    /// Fold the classified outcome of the grid index most recently returned by
    /// [`Self::start`] or a prior call to this method.
    ///
    /// A safety fault, evidence exclusion, or authority failure from the
    /// underlying observation short-circuits the search and is returned
    /// unchanged.
    pub fn advance(
        &mut self,
        outcome: Result<BreakawayMotionClass, BreakawaySearchError>,
    ) -> Result<BreakawaySearchStep, BreakawaySearchError> {
        let class = outcome?;
        self.observations = self
            .observations
            .checked_add(1)
            .ok_or(BreakawaySearchError::Arithmetic)?;
        match self.stage {
            SearchStage::Bisecting => self.advance_bisecting(class),
            SearchStage::ConfirmInert => self.advance_confirm_inert(class),
            SearchStage::ConfirmMoving => self.advance_confirm_moving(class),
        }
    }

    fn advance_bisecting(
        &mut self,
        class: BreakawayMotionClass,
    ) -> Result<BreakawaySearchStep, BreakawaySearchError> {
        let tested = self.pending_index;
        match class {
            BreakawayMotionClass::Moving => self.hi = tested,
            BreakawayMotionClass::Inert => {
                self.lo = tested
                    .checked_add(1)
                    .ok_or(BreakawaySearchError::Arithmetic)?;
            }
        }
        if self.lo < self.hi {
            self.pending_index = bisect_mid(self.lo, self.hi);
            return Ok(BreakawaySearchStep::Probe(self.pending_index));
        }

        // Bisection window collapsed: `self.lo == self.hi` is the first confirmed
        // moving index, or `rung_count` when no index ever tested moving.
        if self.lo >= self.rung_count {
            return Err(BreakawaySearchError::NoMotionAtAuthority);
        }
        self.boundary_moving = Some(self.lo);
        self.boundary_inert = self.lo.checked_sub(1);

        if let Some(inert_index) = self.boundary_inert {
            self.stage = SearchStage::ConfirmInert;
            self.pending_index = inert_index;
        } else {
            self.stage = SearchStage::ConfirmMoving;
            self.pending_index = self.lo;
        }
        Ok(BreakawaySearchStep::Probe(self.pending_index))
    }

    fn advance_confirm_inert(
        &mut self,
        class: BreakawayMotionClass,
    ) -> Result<BreakawaySearchStep, BreakawaySearchError> {
        if class != BreakawayMotionClass::Inert {
            return Err(BreakawaySearchError::UnstableBreakaway);
        }
        let moving_index = self
            .boundary_moving
            .ok_or(BreakawaySearchError::Arithmetic)?;
        self.stage = SearchStage::ConfirmMoving;
        self.pending_index = moving_index;
        Ok(BreakawaySearchStep::Probe(moving_index))
    }

    fn advance_confirm_moving(
        &mut self,
        class: BreakawayMotionClass,
    ) -> Result<BreakawaySearchStep, BreakawaySearchError> {
        if class != BreakawayMotionClass::Moving {
            return Err(BreakawaySearchError::UnstableBreakaway);
        }
        let moving_index = self
            .boundary_moving
            .ok_or(BreakawaySearchError::Arithmetic)?;
        Ok(BreakawaySearchStep::Resolved(DirectionalBreakaway {
            inert_index: self.boundary_inert,
            moving_index,
            observations: self.observations,
        }))
    }
}

/// Midpoint of the half-open bisection window `[lo, hi)`.
const fn bisect_mid(lo: u8, hi: u8) -> u8 {
    lo + (hi - lo) / 2
}

/// Resolved campaign breakaway: the lower of the two directional moving endpoints,
/// with both retained directional brackets for reporting.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct ResolvedCampaignBreakaway {
    /// Post-dedupe grid index of the resolved campaign breakaway: `min` of the two
    /// directional moving endpoints.
    pub breakaway_index: u8,
    /// Retained forward-direction directional breakaway bracket.
    pub forward: DirectionalBreakaway,
    /// Retained reverse-direction directional breakaway bracket.
    pub reverse: DirectionalBreakaway,
}

/// Resolve the campaign breakaway from both completed directional searches.
///
/// The two directional breakaways may differ by at most one grid step; erring low
/// keeps a few extra inert rungs in the additive ladder, while erring high would
/// silently discard real ones. A larger disagreement yields
/// [`BreakawaySearchError::DirectionalBreakawayDisagreement`] rather than picking a
/// side.
pub fn resolve_campaign_breakaway(
    forward: DirectionalBreakaway,
    reverse: DirectionalBreakaway,
) -> Result<ResolvedCampaignBreakaway, BreakawaySearchError> {
    let lower = forward.moving_index.min(reverse.moving_index);
    let higher = forward.moving_index.max(reverse.moving_index);
    if higher - lower > 1 {
        return Err(BreakawaySearchError::DirectionalBreakawayDisagreement);
    }
    Ok(ResolvedCampaignBreakaway {
        breakaway_index: lower,
        forward,
        reverse,
    })
}

#[cfg(test)]
#[path = "breakaway_search_tests.rs"]
mod tests;
