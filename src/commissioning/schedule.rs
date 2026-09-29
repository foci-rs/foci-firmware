//! Explicit scheduling contract for the cooperative commissioning engine.

use crate::autotune::{CommissionError, CommissionPhase};
use crate::commissioning::types::{
    AdcCalResult, CommissionResult, EngineKind, FixedGainAmplitudeTerminal, OuterResult,
    RobustnessReversalTerminal, StepResult, VelocityIntegralEvent, VelocityIntegralTerminal,
};

/// Nonterminal commissioning evidence emitted independently of scheduling.
#[allow(clippy::large_enum_variant)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum CommissionEvent {
    /// The engine entered a new commissioning phase.
    PhaseChanged {
        oid: u8,
        phase_id: CommissionPhase,
        /// Engine kind used to route the progress reply.
        engine_kind: EngineKind,
    },
    /// Streamed report-only Integral velocity-integral evidence.
    VelocityIntegral {
        oid: u8,
        event: VelocityIntegralEvent,
    },
    /// Streamed breakaway-campaign evidence.
    BreakawayCampaign {
        oid: u8,
        event: crate::commissioning::types::BreakawayCampaignEvent,
    },
    /// The exact Integral plan an accepted breakaway campaign handed off to.
    VelocityIntegralPlan {
        oid: u8,
        /// Campaign run sequence the Integral evidence chains to.
        run_sequence: u32,
        /// Exact Integral plan installed by the hand-off.
        plan: crate::commissioning::outer::velocity::integral_plan::VelocityIntegralPlan,
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
    /// Current-loop commissioning completed.
    CommissionDone(CommissionResult),
    /// Report-only Integral response sweep completed, remained inconclusive, or faulted.
    VelocityIntegral(VelocityIntegralTerminal),
    /// One fixed-I fixed-gain amplitude validation completed, remained inconclusive, or faulted.
    FixedGainAmplitude(FixedGainAmplitudeTerminal),
    /// One reversal-standstill robustness gate completed, remained inconclusive, or faulted.
    RobustnessReversal(RobustnessReversalTerminal),
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
            Self::VelocityIntegral(result) => StepResult::VelocityIntegralTerminal(result),
            Self::FixedGainAmplitude(result) => StepResult::FixedGainAmplitudeTerminal(result),
            Self::RobustnessReversal(result) => StepResult::RobustnessReversalTerminal(result),
        }
    }
}

/// Scheduling and terminal contract returned by the commissioning engine.
#[derive(Debug, defmt::Format)]
#[allow(clippy::large_enum_variant)]
pub enum CommissionPoll {
    /// Poll again without inserting an artificial delay.
    ReadyNow { event: Option<CommissionEvent> },
    /// Poll again at a strictly future absolute microsecond timestamp.
    At {
        /// Absolute timestamp in the [`crate::commissioning::CommissionTiming`] domain.
        timestamp_us: u64,
        event: Option<CommissionEvent>,
    },
    /// The engine reached a terminal result.
    Terminal { result: CommissionTerminal },
}

/// Advance a due periodic deadline to the first strictly future grid point.
///
/// # Errors
///
/// Returns [`CommissionError::SchedulePeriodZero`], [`CommissionError::ScheduleEarlyInvocation`]
/// (`now_us < previous_deadline_us`), or [`CommissionError::ScheduleDeadlineOverflow`].
pub(crate) fn next_grid_deadline(
    previous_deadline_us: u64,
    period_us: u64,
    now_us: u64,
) -> Result<u64, CommissionError> {
    if period_us == 0 {
        return Err(CommissionError::SchedulePeriodZero);
    }
    let elapsed_us = now_us
        .checked_sub(previous_deadline_us)
        .ok_or(CommissionError::ScheduleEarlyInvocation)?;
    let slots = elapsed_us
        .checked_div(period_us)
        .and_then(|elapsed_slots| elapsed_slots.checked_add(1))
        .ok_or(CommissionError::ScheduleDeadlineOverflow)?;
    previous_deadline_us
        .checked_add(
            slots
                .checked_mul(period_us)
                .ok_or(CommissionError::ScheduleDeadlineOverflow)?,
        )
        .ok_or(CommissionError::ScheduleDeadlineOverflow)
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
        StepResult::VelocityIntegralEvent(event) => {
            let (oid, _) = event_context.ok_or(CommissionError::InvalidSchedule)?;
            Ok(CommissionPoll::ReadyNow {
                event: Some(CommissionEvent::VelocityIntegral { oid, event }),
            })
        }
        StepResult::VelocityIntegralTerminal(result) => Ok(CommissionPoll::Terminal {
            result: CommissionTerminal::VelocityIntegral(result),
        }),
        StepResult::FixedGainAmplitudeEvent(_) => Ok(CommissionPoll::ReadyNow { event: None }),
        StepResult::BreakawayCampaignEvent(event) => {
            let (oid, _) = event_context.ok_or(CommissionError::InvalidSchedule)?;
            Ok(CommissionPoll::ReadyNow {
                event: Some(CommissionEvent::BreakawayCampaign { oid, event }),
            })
        }
        StepResult::BreakawayIntegralPlanReady { run_sequence, plan } => {
            let (oid, _) = event_context.ok_or(CommissionError::InvalidSchedule)?;
            Ok(CommissionPoll::ReadyNow {
                event: Some(CommissionEvent::VelocityIntegralPlan {
                    oid,
                    run_sequence,
                    plan,
                }),
            })
        }
        StepResult::FixedGainAmplitudeTerminal(result) => Ok(CommissionPoll::Terminal {
            result: CommissionTerminal::FixedGainAmplitude(result),
        }),
        StepResult::RobustnessReversalTerminal(result) => Ok(CommissionPoll::Terminal {
            result: CommissionTerminal::RobustnessReversal(result),
        }),
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
    fn breakaway_campaign_evidence_reaches_the_host() {
        use crate::commissioning::types::{BreakawayCampaignEvent, BreakawayProbeTerminal};

        let event = BreakawayCampaignEvent::ProbeTerminal(BreakawayProbeTerminal {
            run_sequence: 1,
            evidence_sequence: 2,
            plan_digest: 3,
            terminal_cause: 4,
            has_safety_fault: 0,
            fault_checkpoint: 0,
        });
        let poll = adapt_step_result(
            StepResult::BreakawayCampaignEvent(event),
            0,
            Some((3, EngineKind::OuterTune)),
        )
        .expect("campaign evidence adapts");

        assert!(
            matches!(
                poll,
                CommissionPoll::ReadyNow {
                    event: Some(CommissionEvent::BreakawayCampaign { oid: 3, .. })
                }
            ),
            "campaign evidence must be carried to the host, got {poll:?}"
        );
    }

    #[test]
    fn late_grid_deadline_skips_to_first_future_slot() {
        assert_eq!(next_grid_deadline(1_000, 200, 1_450), Ok(1_600));
    }

    #[test]
    fn grid_deadline_rejects_early_invocation() {
        assert!(matches!(
            next_grid_deadline(1_000, 200, 999),
            Err(CommissionError::ScheduleEarlyInvocation)
        ));
    }

    #[test]
    fn grid_deadline_rejects_zero_period() {
        assert!(matches!(
            next_grid_deadline(1_000, 0, 1_000),
            Err(CommissionError::SchedulePeriodZero)
        ));
    }

    #[test]
    fn grid_deadline_rejects_slot_multiplication_overflow() {
        let period_us = u64::MAX / 2 + 1;
        assert!(matches!(
            next_grid_deadline(0, period_us, u64::MAX),
            Err(CommissionError::ScheduleDeadlineOverflow)
        ));
    }

    #[test]
    fn grid_deadline_rejects_deadline_addition_overflow() {
        assert!(matches!(
            next_grid_deadline(u64::MAX - 5, 10, u64::MAX - 5),
            Err(CommissionError::ScheduleDeadlineOverflow)
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
    fn fixed_gain_amplitude_evidence_stays_out_of_scheduler_events() {
        let report =
            crate::commissioning::outer::velocity::fixed_gain_amplitude_evidence::FixedGainAmplitudeReport {
                amplitude_index: 0,
                target_rpm: 16,
                directions: core::array::from_fn(|_| {
                    crate::commissioning::outer::velocity::fixed_gain_amplitude_evidence::
                        FixedGainAmplitudeDirectionalEvidence {
                            availability: crate::commissioning::outer::velocity::
                                fixed_gain_amplitude_evidence::FixedGainAmplitudeAvailability::Unattempted,
                            consensus: crate::commissioning::outer::velocity::sweep_consensus::
                                DirectionalConsensus {
                                    classification: crate::commissioning::outer::velocity::
                                        sweep_consensus::DirectionalConsensusClass::
                                        InsufficientConsensus,
                                    components: [[0; 2]; 2],
                                    component_count: 0,
                                    collected_mask: 0,
                                    eligible_mask: 0,
                                    included_mask: 0,
                                },
                            signed_target_rate_q: 0,
                            response_interval_q: None,
                            robust_location_q: None,
                            deficit_interval_q: None,
                            excluded_mask: 0,
                            blocking_levels: [0; 4],
                        }
                }),
            };
        let event = crate::commissioning::types::FixedGainAmplitudeEvent::Amplitude(
            crate::commissioning::types::FixedGainAmplitudeEvidence {
                run_sequence: 1,
                evidence_sequence: 2,
                plan_digest: 3,
                report,
            },
        );

        assert!(matches!(
            adapt_step_result(StepResult::FixedGainAmplitudeEvent(event), 42_000, None).unwrap(),
            CommissionPoll::ReadyNow { event: None }
        ));
    }

    #[test]
    fn phase_change_maps_to_ready_now_with_its_event() {
        let poll = adapt_step_result(
            StepResult::PhaseChanged(CommissionPhase::ClosedLoopActivation),
            42_000,
            Some((7, EngineKind::Commission)),
        )
        .unwrap();

        assert!(matches!(
            poll,
            CommissionPoll::ReadyNow {
                event: Some(CommissionEvent::PhaseChanged {
                    oid: 7,
                    phase_id: CommissionPhase::ClosedLoopActivation,
                    engine_kind: EngineKind::Commission,
                })
            }
        ));
    }
}
