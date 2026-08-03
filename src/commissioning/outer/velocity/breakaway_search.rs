//! Pure, host-testable breakaway probe cursor and shared motion vocabulary.
//!
//! Nothing here moves the axis. A later phase drives each requested stroke and folds
//! its physical-excursion result back into [`UpwardBreakawayCursor`].

use super::breakaway_plan::PROBE_MAX_OBSERVATIONS;
use super::sweep_plan::MAX_SWEEP_RUNGS;
use crate::autotune::velocity_measurement::VelocityDirection;

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

/// Failure to advance the deterministic upward probe cursor.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum BreakawaySearchError {
    /// A required search input was zero or otherwise unusable.
    InvalidRequest,
    /// Checked search-cursor arithmetic exceeded its representation.
    Arithmetic,
}

#[cfg(test)]
#[path = "breakaway_search_tests.rs"]
mod tests;
