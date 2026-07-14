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

/// Advance a due periodic deadline to the first strictly future grid point.
///
/// Callers must establish `now_us >= previous_deadline_us` before invoking
/// this helper. An early invocation is rejected.
///
/// # Errors
///
/// Returns [`CommissionError::InvalidSchedule`] for a zero period, an early
/// invocation, or arithmetic overflow.
pub(crate) fn next_grid_deadline(
    previous_deadline_us: u64,
    period_us: u64,
    now_us: u64,
) -> Result<u64, CommissionError> {
    if period_us == 0 {
        return Err(CommissionError::InvalidSchedule);
    }
    let elapsed_us = now_us
        .checked_sub(previous_deadline_us)
        .ok_or(CommissionError::InvalidSchedule)?;
    let slots = elapsed_us
        .checked_div(period_us)
        .and_then(|elapsed_slots| elapsed_slots.checked_add(1))
        .ok_or(CommissionError::InvalidSchedule)?;
    previous_deadline_us
        .checked_add(
            slots
                .checked_mul(period_us)
                .ok_or(CommissionError::InvalidSchedule)?,
        )
        .ok_or(CommissionError::InvalidSchedule)
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
    fn exactly_due_grid_deadline_advances_one_period() {
        assert_eq!(next_grid_deadline(1_000, 200, 1_000), Ok(1_200));
    }

    #[test]
    fn late_grid_deadline_skips_to_first_future_slot() {
        assert_eq!(next_grid_deadline(1_000, 200, 1_450), Ok(1_600));
    }

    #[test]
    fn grid_deadline_rejects_early_invocation() {
        assert!(matches!(
            next_grid_deadline(1_000, 200, 999),
            Err(CommissionError::InvalidSchedule)
        ));
    }

    #[test]
    fn grid_deadline_rejects_zero_period() {
        assert!(matches!(
            next_grid_deadline(1_000, 0, 1_000),
            Err(CommissionError::InvalidSchedule)
        ));
    }

    #[test]
    fn grid_deadline_rejects_slot_multiplication_overflow() {
        let period_us = u64::MAX / 2 + 1;
        assert!(matches!(
            next_grid_deadline(0, period_us, u64::MAX),
            Err(CommissionError::InvalidSchedule)
        ));
    }

    #[test]
    fn grid_deadline_rejects_deadline_addition_overflow() {
        assert!(matches!(
            next_grid_deadline(u64::MAX - 5, 10, u64::MAX - 5),
            Err(CommissionError::InvalidSchedule)
        ));
    }

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
