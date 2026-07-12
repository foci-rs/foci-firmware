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
    ///
    /// # Errors
    /// Returns [`CommissionError::InvalidSchedule`] for an invalid deadline.
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
                    return Err(CommissionError::InvalidSchedule);
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

/// Return the next bounded wake, or `InvalidSchedule` on timestamp overflow.
pub fn next_wait_slice(now_us: u64, deadline_us: u64) -> Result<u64, CommissionError> {
    if now_us >= deadline_us {
        return Ok(deadline_us);
    }
    let slice_end = now_us
        .checked_add(COMMISSION_WAIT_SLICE_US)
        .ok_or(CommissionError::InvalidSchedule)?;
    Ok(slice_end.min(deadline_us))
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
    fn long_wait_is_sliced() {
        assert_eq!(next_wait_slice(5_000, 20_000).unwrap(), 6_000);
        assert_eq!(next_wait_slice(19_500, 20_000).unwrap(), 20_000);
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
