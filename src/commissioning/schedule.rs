//! Explicit scheduling contract for the cooperative commissioning engine.

use crate::autotune::{CommissionError, PhaseId};
use crate::commissioning::types::{
    AdcCalResult, CommissionResult, EngineKind, OuterResult, StepResult,
};

/// Nonterminal commissioning evidence emitted independently of scheduling.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum CommissionEvent {
    /// The engine entered a new commissioning phase.
    PhaseChanged {
        /// Object id of the commissioning channel.
        oid: u8,
        /// Newly active phase.
        phase_id: PhaseId,
        /// Engine kind used to route the progress reply.
        engine_kind: EngineKind,
    },
}

/// Terminal result from one commissioning engine run.
#[derive(Debug, defmt::Format)]
#[allow(clippy::large_enum_variant)]
pub enum CommissionTerminal {
    /// Inner commissioning failed.
    InnerFailed(CommissionError),
    /// Outer commissioning completed.
    OuterDone(OuterResult),
    /// Outer commissioning failed.
    OuterFailed(CommissionError),
    /// ADC calibration completed.
    CalibrateDone(AdcCalResult),
    /// Self-test completed.
    SelftestDone,
    /// Stage 1 commissioning completed.
    CommissionDone(CommissionResult),
}

impl CommissionTerminal {
    pub(crate) fn into_step_result(self) -> StepResult {
        match self {
            Self::InnerFailed(error) => StepResult::InnerFailed(error),
            Self::OuterDone(result) => StepResult::OuterDone(result),
            Self::OuterFailed(error) => StepResult::OuterFailed(error),
            Self::CalibrateDone(result) => StepResult::CalibrateDone(result),
            Self::SelftestDone => StepResult::SelftestDone,
            Self::CommissionDone(result) => StepResult::CommissionDone(result),
        }
    }
}

/// Scheduling and terminal contract returned by the commissioning engine.
#[derive(Debug, defmt::Format)]
#[allow(clippy::large_enum_variant)]
pub enum CommissionPoll {
    /// Poll again without inserting an artificial delay.
    ReadyNow {
        /// Optional nonterminal event produced by this poll.
        event: Option<CommissionEvent>,
    },
    /// Poll again at a strictly future absolute microsecond timestamp.
    At {
        /// Absolute timestamp in the [`crate::commissioning::CommissionTiming`] domain.
        timestamp_us: u64,
        /// Optional nonterminal event produced by this poll.
        event: Option<CommissionEvent>,
    },
    /// The engine reached a terminal result.
    Terminal {
        /// Final result for dispatch and reply generation.
        result: CommissionTerminal,
    },
}

/// Adapt a step result while retaining one-millisecond legacy pacing.
///
/// # Errors
/// Returns [`CommissionError::InvalidSchedule`] for legacy-wait overflow or
/// missing phase-change context.
pub fn adapt_step_result(
    result: StepResult,
    now_us: u64,
    event_context: Option<(u8, EngineKind)>,
) -> Result<CommissionPoll, CommissionError> {
    match result {
        StepResult::ReadyNow => Ok(CommissionPoll::ReadyNow { event: None }),
        StepResult::At(timestamp_us) if timestamp_us <= now_us => {
            Ok(CommissionPoll::ReadyNow { event: None })
        }
        StepResult::At(timestamp_us) => Ok(CommissionPoll::At {
            timestamp_us,
            event: None,
        }),
        StepResult::Continue => legacy_wait(now_us, None),
        StepResult::PhaseChanged(phase_id) => {
            let (oid, engine_kind) = event_context.ok_or(CommissionError::InvalidSchedule)?;
            legacy_wait(
                now_us,
                Some(CommissionEvent::PhaseChanged {
                    oid,
                    phase_id,
                    engine_kind,
                }),
            )
        }
        StepResult::InnerFailed(error) => Ok(CommissionPoll::Terminal {
            result: CommissionTerminal::InnerFailed(error),
        }),
        StepResult::OuterDone(result) => Ok(CommissionPoll::Terminal {
            result: CommissionTerminal::OuterDone(result),
        }),
        StepResult::OuterFailed(error) => Ok(CommissionPoll::Terminal {
            result: CommissionTerminal::OuterFailed(error),
        }),
        StepResult::CalibrateDone(result) => Ok(CommissionPoll::Terminal {
            result: CommissionTerminal::CalibrateDone(result),
        }),
        StepResult::SelftestDone => Ok(CommissionPoll::Terminal {
            result: CommissionTerminal::SelftestDone,
        }),
        StepResult::CommissionDone(result) => Ok(CommissionPoll::Terminal {
            result: CommissionTerminal::CommissionDone(result),
        }),
    }
}

fn legacy_wait(
    now_us: u64,
    event: Option<CommissionEvent>,
) -> Result<CommissionPoll, CommissionError> {
    let timestamp_us = now_us
        .checked_add(crate::tmc_control::commission_poll::COMMISSION_WAIT_SLICE_US)
        .ok_or(CommissionError::InvalidSchedule)?;
    Ok(CommissionPoll::At {
        timestamp_us,
        event,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn legacy_continue_maps_to_one_millisecond_deadline() {
        assert!(matches!(
            adapt_step_result(StepResult::Continue, 42_000, None).unwrap(),
            CommissionPoll::At {
                timestamp_us: 43_000,
                event: None,
            }
        ));
    }

    #[test]
    fn explicit_future_deadline_is_preserved() {
        let at = adapt_step_result(StepResult::At(42_160), 42_000, None).unwrap();
        assert!(matches!(
            at,
            CommissionPoll::At {
                timestamp_us: 42_160,
                event: None,
            }
        ));
    }

    #[test]
    fn elapsed_explicit_deadline_maps_to_ready_now() {
        assert!(matches!(
            adapt_step_result(StepResult::At(42_000), 42_000, None).unwrap(),
            CommissionPoll::ReadyNow { event: None }
        ));
    }

    #[test]
    fn overflowed_legacy_schedule_is_rejected() {
        assert!(matches!(
            adapt_step_result(StepResult::Continue, u64::MAX, None),
            Err(CommissionError::InvalidSchedule)
        ));
        assert_eq!(CommissionError::InvalidSchedule.status_code(), 47);
    }
}
