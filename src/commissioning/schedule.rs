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

/// Adapt an engine step result to the explicit scheduling contract.
///
/// # Errors
/// Returns [`CommissionError::InvalidSchedule`] for missing phase-change context.
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
        StepResult::PhaseChanged(phase_id) => {
            let (oid, engine_kind) = event_context.ok_or(CommissionError::InvalidSchedule)?;
            Ok(CommissionPoll::ReadyNow {
                event: Some(CommissionEvent::PhaseChanged {
                    oid,
                    phase_id,
                    engine_kind,
                }),
            })
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
    fn phase_change_maps_to_ready_now_with_its_event() {
        let poll = adapt_step_result(
            StepResult::PhaseChanged(PhaseId::ClosedLoopEntry),
            42_000,
            Some((7, EngineKind::Commission)),
        )
        .unwrap();

        assert!(matches!(
            poll,
            CommissionPoll::ReadyNow {
                event: Some(CommissionEvent::PhaseChanged {
                    oid: 7,
                    phase_id: PhaseId::ClosedLoopEntry,
                    engine_kind: EngineKind::Commission,
                })
            }
        ));
    }

    fn maximum_position_rungs() -> u64 {
        let mut rungs = 1_u64;
        let mut position_p = u32::from(crate::commissioning::outer::POS_TUNE_INITIAL_P);
        let maximum_p = u32::from(crate::commissioning::outer::position::POSITION_TUNE_MAX_P);
        while position_p < maximum_p {
            let next_p = (position_p
                * crate::commissioning::outer::position::POSITION_TUNE_P_MUL_NUM
                / crate::commissioning::outer::position::POSITION_TUNE_P_MUL_DEN)
                .min(maximum_p);
            if next_p <= position_p {
                break;
            }
            position_p = next_p;
            rungs += 1;
        }
        rungs
    }

    #[test]
    fn outer_worst_path_clock_model_stays_inside_the_derived_bound() {
        const DERIVED_SCHEDULED_SAMPLE_BOUND: u64 = 13_238;
        const DERIVED_REQUESTED_WAIT_BOUND_US: u64 = 23_396_000;

        #[derive(Default)]
        struct StructuralClock {
            now_us: u64,
            scheduled_samples: u64,
        }

        impl StructuralClock {
            fn wait(&mut self, duration_us: u64) {
                self.now_us = self
                    .now_us
                    .checked_add(duration_us)
                    .expect("the reviewed worst path fits in the commissioning clock");
                self.scheduled_samples += 1;
            }
        }

        let transient_samples = crate::autotune::TRANSIENT_LEN as u64;
        let mechanical_attempts =
            u64::from(crate::commissioning::outer::response::MECHANICAL_ID_MAX_ATTEMPTS);
        let velocity_attempts =
            u64::from(crate::commissioning::outer::velocity::VELOCITY_RESPONSE_MAX_RETRIES) + 1;
        let position_rungs = maximum_position_rungs();

        let mut clock = StructuralClock::default();
        let post_switch_samples =
            crate::commissioning::outer::mechanical::MECHANICAL_ID_POST_SWITCH_TIMEOUT_US
                / crate::commissioning::outer::response::MECHANICAL_ID_SAMPLE_PERIOD_US;
        for _ in 0..post_switch_samples {
            clock.wait(crate::commissioning::outer::response::MECHANICAL_ID_SAMPLE_PERIOD_US);
        }
        for _ in 0..mechanical_attempts {
            for _ in 0..2 {
                for _ in 0..transient_samples {
                    clock.wait(
                        crate::commissioning::outer::response::MECHANICAL_ID_SAMPLE_PERIOD_US,
                    );
                }
                clock.wait(crate::commissioning::outer::mechanical::MECHANICAL_ID_SETTLE_US);
            }
        }
        for _ in 0..velocity_attempts {
            for _ in 0..2 {
                for _ in 0..transient_samples {
                    clock.wait(
                        crate::commissioning::outer::velocity::VELOCITY_VALIDATE_SAMPLE_PERIOD_US,
                    );
                }
                clock.wait(crate::commissioning::outer::velocity::VELOCITY_VALIDATE_SETTLE_US);
            }
            for _ in 0..crate::commissioning::outer::velocity::VELOCITY_VALIDATE_CHATTER_WINDOW {
                clock.wait(
                    crate::commissioning::outer::velocity::VELOCITY_VALIDATE_SAMPLE_PERIOD_US,
                );
            }
        }
        clock.wait(crate::commissioning::outer::velocity::VELOCITY_VALIDATE_SETTLE_US);
        for _ in 0..position_rungs {
            for _ in 0..transient_samples {
                clock.wait(crate::commissioning::outer::position::POSITION_TUNE_SAMPLE_PERIOD_US);
            }
            clock.wait(crate::commissioning::outer::position::POSITION_TUNE_SETTLE_US);
        }

        assert_eq!(position_rungs, 9);
        assert_eq!(clock.scheduled_samples, DERIVED_SCHEDULED_SAMPLE_BOUND);
        assert_eq!(clock.now_us, DERIVED_REQUESTED_WAIT_BOUND_US);
        assert!(clock.now_us < 30_000_000);
    }

    #[test]
    fn nominal_outer_primitives_fit_the_unchanged_duration_budget() {
        let samples = crate::autotune::TRANSIENT_LEN as u64;
        let primitive_durations_us = [
            samples * crate::commissioning::outer::response::MECHANICAL_ID_SAMPLE_PERIOD_US,
            samples * crate::commissioning::outer::velocity::VELOCITY_VALIDATE_SAMPLE_PERIOD_US,
            samples * crate::commissioning::outer::position::POSITION_TUNE_SAMPLE_PERIOD_US,
        ];
        assert!(
            primitive_durations_us
                .iter()
                .all(|duration_us| *duration_us < 3_000_000)
        );
    }
}
