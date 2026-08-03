//! Pure, host-testable breakaway probe cursors and boundary/motion classification.
//!
//! Nothing here moves the axis. [`BreakawaySearchCursor`] retains the live v14
//! binary-search path while [`UpwardBreakawayCursor`] provides the deterministic
//! pair-and-repeat replacement. A later phase drives each requested stroke and folds
//! its classified result back into the owning cursor.
//!
//! The motion criterion reuses the sweep's own sign-correct interval test
//! ([`classify_velocity_observation`]) rather than a bare encoder-delta comparison.
//! `delta == 0` is exactly the false-`ENCODER_FAULT` shape; a settled-rate
//! confidence interval that excludes zero on the commanded side is not.

use super::breakaway_plan::PROBE_MAX_OBSERVATIONS;
use super::sweep_evidence::{
    VelocityObservationClass, VelocityObservationInput, classify_velocity_observation,
};
use super::sweep_plan::MAX_SWEEP_RUNGS;
use crate::autotune::velocity_measurement::{
    MAX_MOVING_DRIFT_PPM, StationarityExclusion, StationarityResult, VelocityDirection,
};

/// Role of one stroke in a rung's initial or repeated directional pair.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum ProbeStrokeRole {
    /// First forward stroke at this rung.
    InitialForward,
    /// First reverse stroke at this rung.
    InitialReverse,
    /// Repeated forward stroke after both initial strokes moved.
    RepeatForward,
    /// Repeated reverse stroke after both initial strokes moved.
    RepeatReverse,
}

impl ProbeStrokeRole {
    /// Commanded direction for this stroke.
    pub const fn direction(self) -> VelocityDirection {
        match self {
            Self::InitialForward | Self::RepeatForward => VelocityDirection::Forward,
            Self::InitialReverse | Self::RepeatReverse => VelocityDirection::Reverse,
        }
    }

    /// Direction-local slot index: zero for initial strokes and one for repeats.
    pub const fn slot_index(self) -> u8 {
        match self {
            Self::InitialForward | Self::InitialReverse => 0,
            Self::RepeatForward | Self::RepeatReverse => 1,
        }
    }
}

/// Next action in the deterministic upward breakaway walk.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum UpwardProbeStep {
    /// Execute the stated stroke at the zero-based realized-grid rung.
    Probe {
        /// Zero-based rung in the realized probe grid.
        rung_index: u8,
        /// Initial or repeated directional role.
        role: ProbeStrokeRole,
    },
    /// The first rung whose initial and repeated pairs all moved.
    Resolved {
        /// Zero-based resolved rung in the realized probe grid.
        rung_index: u8,
        /// Total observations consumed across the upward walk.
        observations: u8,
    },
    /// The authority rung was exhausted without a repeatable four-stroke pass.
    NoRepeatableMotionAtAuthority {
        /// Total observations consumed across the upward walk.
        observations: u8,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
enum UpwardProbeStage {
    InitialForward,
    InitialReverse,
    RepeatForward,
    RepeatReverse,
    Terminal,
}

/// Pure cursor for the deterministic pair-and-repeat upward probe.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct UpwardBreakawayCursor {
    rung_count: u8,
    rung_index: u8,
    observations: u8,
    stage: UpwardProbeStage,
    initial_forward_moved: bool,
    repeat_forward_moved: bool,
}

impl UpwardBreakawayCursor {
    /// Construct a cursor and return the first forward stroke at rung zero.
    pub fn start(rung_count: u8) -> Result<(Self, UpwardProbeStep), BreakawaySearchError> {
        if rung_count == 0 || usize::from(rung_count) > MAX_SWEEP_RUNGS {
            return Err(BreakawaySearchError::InvalidRequest);
        }
        Ok((
            Self {
                rung_count,
                rung_index: 0,
                observations: 0,
                stage: UpwardProbeStage::InitialForward,
                initial_forward_moved: false,
                repeat_forward_moved: false,
            },
            UpwardProbeStep::Probe {
                rung_index: 0,
                role: ProbeStrokeRole::InitialForward,
            },
        ))
    }

    /// Number of observations already folded into this cursor.
    pub const fn observations(&self) -> u8 {
        self.observations
    }

    /// Fold the physical-motion verdict for the most recently requested stroke.
    pub fn advance(&mut self, moved: bool) -> Result<UpwardProbeStep, BreakawaySearchError> {
        if self.stage == UpwardProbeStage::Terminal {
            return Err(BreakawaySearchError::InvalidRequest);
        }
        self.observations = self
            .observations
            .checked_add(1)
            .ok_or(BreakawaySearchError::Arithmetic)?;
        if self.observations > PROBE_MAX_OBSERVATIONS {
            return Err(BreakawaySearchError::Arithmetic);
        }

        match self.stage {
            UpwardProbeStage::InitialForward => {
                self.initial_forward_moved = moved;
                self.stage = UpwardProbeStage::InitialReverse;
                Ok(self.probe(ProbeStrokeRole::InitialReverse))
            }
            UpwardProbeStage::InitialReverse => {
                if self.initial_forward_moved && moved {
                    self.stage = UpwardProbeStage::RepeatForward;
                    Ok(self.probe(ProbeStrokeRole::RepeatForward))
                } else {
                    self.advance_rung()
                }
            }
            UpwardProbeStage::RepeatForward => {
                self.repeat_forward_moved = moved;
                self.stage = UpwardProbeStage::RepeatReverse;
                Ok(self.probe(ProbeStrokeRole::RepeatReverse))
            }
            UpwardProbeStage::RepeatReverse => {
                if self.repeat_forward_moved && moved {
                    self.stage = UpwardProbeStage::Terminal;
                    Ok(UpwardProbeStep::Resolved {
                        rung_index: self.rung_index,
                        observations: self.observations,
                    })
                } else {
                    self.advance_rung()
                }
            }
            UpwardProbeStage::Terminal => Err(BreakawaySearchError::InvalidRequest),
        }
    }

    const fn probe(&self, role: ProbeStrokeRole) -> UpwardProbeStep {
        UpwardProbeStep::Probe {
            rung_index: self.rung_index,
            role,
        }
    }

    fn advance_rung(&mut self) -> Result<UpwardProbeStep, BreakawaySearchError> {
        let next = self
            .rung_index
            .checked_add(1)
            .ok_or(BreakawaySearchError::Arithmetic)?;
        if next >= self.rung_count {
            self.stage = UpwardProbeStage::Terminal;
            return Ok(UpwardProbeStep::NoRepeatableMotionAtAuthority {
                observations: self.observations,
            });
        }
        self.rung_index = next;
        self.stage = UpwardProbeStage::InitialForward;
        self.initial_forward_moved = false;
        self.repeat_forward_moved = false;
        Ok(self.probe(ProbeStrokeRole::InitialForward))
    }
}

/// Sign-correct motion verdict for one probe observation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum BreakawayMotionClass {
    /// The sign-correct settled-rate confidence interval still contains zero.
    Inert,
    /// The sign-correct settled-rate confidence interval excludes zero.
    Moving,
}

/// Observable reason one probe response remains below the productive boundary.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum BelowBoundaryDisposition {
    /// The family-16 settled-rate interval contains zero.
    Inert,
    /// The axis responded but excessive drift prevented a stationary result.
    Transient,
}

impl BelowBoundaryDisposition {
    const fn mask(self) -> u8 {
        match self {
            Self::Inert => 1,
            Self::Transient => 2,
        }
    }
}

/// Probe-local productive-boundary verdict for one completed observation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum ProbeProductivityClass {
    /// The observation is below the productive boundary for the stated reason.
    BelowProductive(BelowBoundaryDisposition),
    /// A usable stationary response excludes zero on the commanded side.
    Productive,
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
    /// Dispositions observed at the confirmed lower endpoint.
    ///
    /// Bit 0 records inert and bit 1 records transient. Zero means the resolved
    /// productive endpoint was the grid floor and no lower endpoint existed.
    pub lower_disposition_mask: u8,
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
    /// Target-independent stationarity evidence was unusable for classification.
    ProbeEvidenceExcluded(StationarityExclusion),
    /// Another controller-domain evidence field was unusable for classification.
    InvalidEvidence,
    /// The observation reached the inclusive usable-current boundary before a
    /// motion or inert verdict was reached.
    AuthorityFailure,
    /// The bisection search reached the top of the search grid without ever
    /// confirming commanded-side motion, and the mandatory repeat of the sole
    /// representable boundary endpoint (the top rung) confirmed it inert too.
    NoProductiveResponseAtAuthority,
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
) -> Result<ProbeProductivityClass, BreakawaySearchError> {
    if matches!(
        input.stationarity,
        StationarityResult::Excluded(StationarityExclusion::ExcessiveDrift)
    ) {
        let valid_intervals = input.settled_rate_low_q <= input.settled_rate_high_q
            && input.deficit_low_q <= input.deficit_high_q
            && input.usable_torque_limit <= input.hard_torque_limit;
        if !valid_intervals
            || input.drift_ratio_ppm == u32::MAX
            || input.drift_ratio_ppm <= MAX_MOVING_DRIFT_PPM
        {
            return Err(BreakawaySearchError::InvalidEvidence);
        }
        if input.predicted_torque_target_abs >= input.hard_torque_limit {
            return Err(BreakawaySearchError::SafetyFault(
                VelocityObservationClass::HardClipFault,
            ));
        }
        let wrong_direction = match input.direction {
            crate::autotune::velocity_measurement::VelocityDirection::Forward => {
                input.settled_rate_high_q < 0
            }
            crate::autotune::velocity_measurement::VelocityDirection::Reverse => {
                input.settled_rate_low_q > 0
            }
        };
        if wrong_direction {
            return Err(BreakawaySearchError::SafetyFault(
                VelocityObservationClass::WrongDirectionFault,
            ));
        }
        if input.predicted_torque_target_abs >= input.usable_torque_limit {
            return Err(BreakawaySearchError::AuthorityFailure);
        }
        match input.post_fit_lag_one_state {
            0 => return Err(BreakawaySearchError::InvalidEvidence),
            1 => {
                return Err(BreakawaySearchError::ProbeEvidenceExcluded(
                    StationarityExclusion::StructuredResidual,
                ));
            }
            2 => {}
            _ => return Err(BreakawaySearchError::InvalidEvidence),
        }
        let contains_zero = input.settled_rate_low_q <= 0 && input.settled_rate_high_q >= 0;
        return Ok(ProbeProductivityClass::BelowProductive(if contains_zero {
            BelowBoundaryDisposition::Inert
        } else {
            BelowBoundaryDisposition::Transient
        }));
    }

    let summary = classify_velocity_observation(input);
    match summary.classification {
        VelocityObservationClass::Eligible | VelocityObservationClass::DeficitUnresolved => {
            Ok(ProbeProductivityClass::Productive)
        }
        VelocityObservationClass::Inert => Ok(ProbeProductivityClass::BelowProductive(
            BelowBoundaryDisposition::Inert,
        )),
        VelocityObservationClass::CurrentHeadroom => Err(BreakawaySearchError::AuthorityFailure),
        VelocityObservationClass::StationarityExcluded(reason) => {
            Err(BreakawaySearchError::ProbeEvidenceExcluded(reason))
        }
        VelocityObservationClass::InvalidEvidenceFault => {
            Err(BreakawaySearchError::InvalidEvidence)
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
    /// Confirming the top-rung authority boundary a second time, after bisection
    /// found every grid rung inert and there is no moving endpoint to repeat.
    ConfirmAllInertBoundary,
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
///
/// When bisection concludes every grid rung is inert (no moving index was ever
/// confirmed), there is no moving endpoint to repeat either. The cursor still
/// applies the mandatory-repeat discipline: it re-probes the sole representable
/// boundary endpoint, the top rung (`rung_count - 1`), once. A confirming repeat
/// yields [`BreakawaySearchError::NoProductiveResponseAtAuthority`]; a contradictory repeat
/// (the top rung now reads moving) yields
/// [`BreakawaySearchError::UnstableBreakaway`] instead of silently accepting the
/// single unconfirmed observation.
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
    lower_disposition_mask: u8,
}

impl BreakawaySearchCursor {
    /// Number of observations already consumed in this directional search.
    pub const fn observations(&self) -> u8 {
        self.observations
    }

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
                lower_disposition_mask: 0,
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
        outcome: Result<ProbeProductivityClass, BreakawaySearchError>,
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
            SearchStage::ConfirmAllInertBoundary => self.advance_confirm_all_inert_boundary(class),
        }
    }

    fn advance_bisecting(
        &mut self,
        class: ProbeProductivityClass,
    ) -> Result<BreakawaySearchStep, BreakawaySearchError> {
        let tested = self.pending_index;
        match class {
            ProbeProductivityClass::Productive => self.hi = tested,
            ProbeProductivityClass::BelowProductive(disposition) => {
                self.lo = tested
                    .checked_add(1)
                    .ok_or(BreakawaySearchError::Arithmetic)?;
                self.lower_disposition_mask = disposition.mask();
            }
        }
        if self.lo < self.hi {
            self.pending_index = bisect_mid(self.lo, self.hi);
            return Ok(BreakawaySearchStep::Probe(self.pending_index));
        }

        // Bisection window collapsed: `self.lo == self.hi` is the first confirmed
        // moving index, or `rung_count` when no index ever tested moving.
        if self.lo >= self.rung_count {
            // Every grid rung tested inert; there is no moving endpoint to repeat.
            // Mirror the mandatory-repeat discipline anyway by re-probing the sole
            // representable boundary, the top rung, before conceding no motion.
            let authority_index = self.rung_count - 1;
            self.stage = SearchStage::ConfirmAllInertBoundary;
            self.pending_index = authority_index;
            return Ok(BreakawaySearchStep::Probe(authority_index));
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
        class: ProbeProductivityClass,
    ) -> Result<BreakawaySearchStep, BreakawaySearchError> {
        let ProbeProductivityClass::BelowProductive(disposition) = class else {
            return Err(BreakawaySearchError::UnstableBreakaway);
        };
        self.lower_disposition_mask |= disposition.mask();
        let moving_index = self
            .boundary_moving
            .ok_or(BreakawaySearchError::Arithmetic)?;
        self.stage = SearchStage::ConfirmMoving;
        self.pending_index = moving_index;
        Ok(BreakawaySearchStep::Probe(moving_index))
    }

    fn advance_confirm_moving(
        &mut self,
        class: ProbeProductivityClass,
    ) -> Result<BreakawaySearchStep, BreakawaySearchError> {
        if class != ProbeProductivityClass::Productive {
            return Err(BreakawaySearchError::UnstableBreakaway);
        }
        let moving_index = self
            .boundary_moving
            .ok_or(BreakawaySearchError::Arithmetic)?;
        Ok(BreakawaySearchStep::Resolved(DirectionalBreakaway {
            inert_index: self.boundary_inert,
            moving_index,
            observations: self.observations,
            lower_disposition_mask: self.lower_disposition_mask,
        }))
    }

    fn advance_confirm_all_inert_boundary(
        &mut self,
        class: ProbeProductivityClass,
    ) -> Result<BreakawaySearchStep, BreakawaySearchError> {
        match class {
            ProbeProductivityClass::BelowProductive(_) => {
                Err(BreakawaySearchError::NoProductiveResponseAtAuthority)
            }
            ProbeProductivityClass::Productive => Err(BreakawaySearchError::UnstableBreakaway),
        }
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
