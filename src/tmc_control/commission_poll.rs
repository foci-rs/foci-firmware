//! Shared board policy for consuming explicit commissioning polls.

use crate::autotune::CommissionError;
use crate::commissioning::schedule::CommissionPoll;
use crate::tmc::TmcRequestKind;

/// Maximum duration between request and cancellation checks while waiting.
pub const COMMISSION_WAIT_SLICE_US: u64 = 1_000;
/// Consecutive immediate polls allowed before forcing one scheduler yield.
pub const MAX_CONSECUTIVE_READY_POLLS: u8 = 8;
/// Duration of the forced scheduler yield after the immediate-poll limit.
pub const FORCED_READY_YIELD_US: u64 = 1;

/// Board action selected while waiting for an absolute commissioning deadline.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum AbsoluteWaitAction {
    /// Poll the commissioning engine immediately.
    PollNow,
    /// Sleep until the absolute timestamp.
    SleepUntil(u64),
}

/// Board action selected by the shared commissioning poll driver.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum PollDriverAction {
    /// Poll the engine again immediately.
    PollNow,
    /// Perform one guaranteed scheduler yield before any further processing.
    YieldNow,
    /// Wait until the absolute timestamp.
    WaitUntil(u64),
    /// Apply terminal dispatch and leave the polling loop.
    Terminal,
}

/// Request disposition while commissioning exclusively owns the motor.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum CommissionWaitRequestDisposition {
    /// Cancel commissioning and disarm the motor.
    Cancel,
    /// Emit the existing deterministic busy response.
    RejectBusy,
    /// Drop an asynchronous configuration write.
    DropBusy,
}

/// State shared by both board poll loops.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct CommissionPollDriver {
    consecutive_ready: u8,
}

impl CommissionPollDriver {
    /// Create a driver with no immediate polls accumulated.
    pub const fn new() -> Self {
        Self {
            consecutive_ready: 0,
        }
    }

    /// Classify one complete poll result.
    pub fn classify(
        &mut self,
        poll: &CommissionPoll,
        now_us: u64,
    ) -> Result<PollDriverAction, CommissionError> {
        match poll {
            CommissionPoll::ReadyNow { .. } => self.classify_ready(now_us),
            CommissionPoll::At { timestamp_us, .. } => {
                self.consecutive_ready = 0;
                if *timestamp_us <= now_us {
                    return Ok(PollDriverAction::PollNow);
                }
                Ok(PollDriverAction::WaitUntil(*timestamp_us))
            }
            CommissionPoll::Terminal { .. } => {
                self.consecutive_ready = 0;
                Ok(PollDriverAction::Terminal)
            }
        }
    }

    /// Classify one immediate poll while enforcing the bounded ready chain.
    ///
    /// # Errors
    /// Reserved for consistency with complete-poll classification.
    pub fn classify_ready(&mut self, now_us: u64) -> Result<PollDriverAction, CommissionError> {
        if self.consecutive_ready < MAX_CONSECUTIVE_READY_POLLS {
            self.consecutive_ready += 1;
            return Ok(PollDriverAction::PollNow);
        }

        self.consecutive_ready = 0;
        let _ = now_us;
        Ok(PollDriverAction::YieldNow)
    }
}

/// Classify the next action for an absolute commissioning deadline.
///
/// # Errors
/// Returns [`CommissionError::InvalidSchedule`] if the bounded sleep timestamp
/// cannot be represented.
pub fn classify_absolute_wait(
    now_us: u64,
    deadline_us: u64,
) -> Result<AbsoluteWaitAction, CommissionError> {
    if now_us >= deadline_us {
        return Ok(AbsoluteWaitAction::PollNow);
    }

    let slice_end_us = now_us
        .checked_add(COMMISSION_WAIT_SLICE_US)
        .ok_or(CommissionError::InvalidSchedule)?;
    Ok(AbsoluteWaitAction::SleepUntil(
        slice_end_us.min(deadline_us),
    ))
}

/// Resolve the polling-loop continuation for a nonterminal scheduled action.
///
/// The board classifies each poll before dispatching it, so a `Terminal`
/// classification normally ends the polling loop. The staged terminal dispatch
/// can instead promote a completed Stage-C run straight into the inline
/// robustness gate and hand back a nonterminal `ReadyNow`/`At` action. That
/// promotion keeps the live engine stepping, so a stale `Terminal`
/// classification collapses to an immediate re-poll; every other classification
/// passes through unchanged.
#[must_use]
pub fn nonterminal_continuation(driver_action: PollDriverAction) -> PollDriverAction {
    match driver_action {
        PollDriverAction::Terminal => PollDriverAction::PollNow,
        action @ (PollDriverAction::PollNow
        | PollDriverAction::YieldNow
        | PollDriverAction::WaitUntil(_)) => action,
    }
}

/// Classify a queued request while commissioning owns the motor.
pub fn classify_wait_request(kind: &TmcRequestKind) -> CommissionWaitRequestDisposition {
    match kind {
        TmcRequestKind::SetEnable(false) => CommissionWaitRequestDisposition::Cancel,
        TmcRequestKind::SetEnable(true)
        | TmcRequestKind::Calibrate { .. }
        | TmcRequestKind::SelfTest { .. }
        | TmcRequestKind::DumpRegisters { .. }
        | TmcRequestKind::Commission { .. }
        | TmcRequestKind::Tune { .. }
        | TmcRequestKind::CurrentStepTest { .. }
        | TmcRequestKind::CurrentVectorStepTest { .. }
        | TmcRequestKind::CurrentTorqueSampleTest { .. }
        | TmcRequestKind::PositionTorqueOffsetSampleTest { .. }
        | TmcRequestKind::VoltageStepTest { .. }
        | TmcRequestKind::ResistanceTest { .. } => CommissionWaitRequestDisposition::RejectBusy,
        #[cfg(feature = "trace-velocity-limit-latch-test")]
        TmcRequestKind::VelocityLimitLatchTest { .. } => {
            CommissionWaitRequestDisposition::RejectBusy
        }
        TmcRequestKind::SetCurrent { .. }
        | TmcRequestKind::SetMotionScale { .. }
        | TmcRequestKind::SetEncoderDir { .. }
        | TmcRequestKind::SetPidGains { .. }
        | TmcRequestKind::SetVelocityFilter { .. }
        | TmcRequestKind::SetTorqueFilter { .. }
        | TmcRequestKind::SetPositionFilter { .. }
        | TmcRequestKind::SetFluxFilter { .. }
        | TmcRequestKind::SetPositionGains { .. }
        | TmcRequestKind::SetVelocityFeedforward { .. }
        | TmcRequestKind::SetVelocityTransientFeedforward { .. }
        | TmcRequestKind::SetAccelFeedforward { .. }
        | TmcRequestKind::SetDecouplingFeedforward { .. }
        | TmcRequestKind::SetPositionLead { .. }
        | TmcRequestKind::SetPhaseAdvance { .. }
        | TmcRequestKind::SetVoltageLimit { .. } => CommissionWaitRequestDisposition::DropBusy,
        #[cfg(feature = "dev")]
        TmcRequestKind::Write { .. } => CommissionWaitRequestDisposition::DropBusy,
        #[cfg(feature = "dev")]
        TmcRequestKind::Read { .. } | TmcRequestKind::Init | TmcRequestKind::TestSpin { .. } => {
            CommissionWaitRequestDisposition::RejectBusy
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ninth_ready_poll_forces_non_expiring_yield() {
        let mut driver = CommissionPollDriver::new();
        for _ in 0..8 {
            assert_eq!(
                driver.classify_ready(10_000).unwrap(),
                PollDriverAction::PollNow
            );
        }
        assert_eq!(
            driver.classify_ready(10_000).unwrap(),
            PollDriverAction::YieldNow
        );

        let processing_finished_us = 20_000;
        assert!(processing_finished_us > 10_001);
        assert_eq!(
            driver.classify_ready(processing_finished_us).unwrap(),
            PollDriverAction::PollNow,
            "processing time cannot consume the already-issued yield action"
        );
    }

    #[test]
    fn deadline_expiring_during_handoff_repolls_and_resets_ready_chain() {
        let poll = crate::commissioning::schedule::adapt_step_result(
            crate::commissioning::types::StepResult::At(10_100),
            10_099,
            None,
        )
        .unwrap();
        let mut driver = CommissionPollDriver::new();
        for _ in 0..MAX_CONSECUTIVE_READY_POLLS {
            assert_eq!(
                driver.classify_ready(10_099).unwrap(),
                PollDriverAction::PollNow
            );
        }

        assert_eq!(
            driver.classify(&poll, 10_100).unwrap(),
            PollDriverAction::PollNow
        );
        assert_eq!(
            driver.classify_ready(10_100).unwrap(),
            PollDriverAction::PollNow,
            "an absolute deadline resets the bounded ready chain"
        );
    }

    #[test]
    fn absolute_wait_slices_directly_toward_the_deadline() {
        assert_eq!(
            classify_absolute_wait(5_000, 20_000).unwrap(),
            AbsoluteWaitAction::SleepUntil(6_000),
        );
        assert_eq!(
            classify_absolute_wait(19_000, 20_000).unwrap(),
            AbsoluteWaitAction::SleepUntil(20_000),
        );
        assert_eq!(
            classify_absolute_wait(19_968, 20_000).unwrap(),
            AbsoluteWaitAction::SleepUntil(20_000),
        );
        assert_eq!(
            classify_absolute_wait(19_999, 20_000).unwrap(),
            AbsoluteWaitAction::SleepUntil(20_000),
        );
    }

    #[test]
    fn elapsed_absolute_wait_repolls_without_spinning() {
        assert_eq!(
            classify_absolute_wait(20_000, 20_000).unwrap(),
            AbsoluteWaitAction::PollNow,
        );
        assert_eq!(
            classify_absolute_wait(20_017, 20_000).unwrap(),
            AbsoluteWaitAction::PollNow,
        );
    }

    #[test]
    fn early_epoch_deadline_sleeps_to_the_deadline() {
        assert_eq!(
            classify_absolute_wait(0, 31).unwrap(),
            AbsoluteWaitAction::SleepUntil(31),
        );
    }

    #[test]
    fn promoted_phase_change_reclassifies_terminal_as_immediate_repoll() {
        // A promoted Stage-C completion reaches the terminal poll but dispatches
        // to a nonterminal action; the board must keep polling instead of ending
        // the loop, so a `Terminal` pre-classification collapses to `PollNow`.
        assert_eq!(
            nonterminal_continuation(PollDriverAction::Terminal),
            PollDriverAction::PollNow
        );
        assert_eq!(
            nonterminal_continuation(PollDriverAction::PollNow),
            PollDriverAction::PollNow
        );
        assert_eq!(
            nonterminal_continuation(PollDriverAction::YieldNow),
            PollDriverAction::YieldNow
        );
        assert_eq!(
            nonterminal_continuation(PollDriverAction::WaitUntil(42)),
            PollDriverAction::WaitUntil(42)
        );
    }

    #[test]
    fn wait_request_policy_separates_cancel_reject_and_drop() {
        assert_eq!(
            classify_wait_request(&TmcRequestKind::SetEnable(false)),
            CommissionWaitRequestDisposition::Cancel
        );
        assert_eq!(
            classify_wait_request(&TmcRequestKind::Commission { oid: 2, profile: 1 }),
            CommissionWaitRequestDisposition::RejectBusy
        );
        assert_eq!(
            classify_wait_request(&TmcRequestKind::SelfTest { oid: 2 }),
            CommissionWaitRequestDisposition::RejectBusy
        );
        assert_eq!(
            classify_wait_request(&TmcRequestKind::SetCurrent { run_ma: 800 }),
            CommissionWaitRequestDisposition::DropBusy
        );
    }
}
