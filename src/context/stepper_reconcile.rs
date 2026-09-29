//! Lossless pending-state model for deferred stepper reconciliation.

use core::sync::atomic::{AtomicU8, Ordering};

use super::stepper::{StepperArmCheckBoard, StepperStopCheckBoard, arm_check, stop_check};
use super::trsync::TriggerDomainState;

/// Invalid board-local channel supplied to the reconciliation publisher.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct InvalidStepperReconcileChannel {
    channel: u8,
    channel_count: u8,
}

impl InvalidStepperReconcileChannel {
    /// Return the rejected board-local channel.
    pub const fn channel(self) -> u8 {
        self.channel
    }

    /// Return the channel count accepted by the pending-state instance.
    pub const fn channel_count(self) -> u8 {
        self.channel_count
    }
}

/// One ownership snapshot of pending stop and arm reconciliation work.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct PendingStepperReconciliation<const CHANNELS: usize> {
    stop_mask: u8,
    arm_mask: u8,
}

impl<const CHANNELS: usize> PendingStepperReconciliation<CHANNELS> {
    /// Return whether this snapshot contains no pending work.
    pub const fn is_empty(self) -> bool {
        self.stop_mask == 0 && self.arm_mask == 0
    }

    /// Return the captured stop-channel mask.
    pub const fn stop_mask(self) -> u8 {
        self.stop_mask
    }

    /// Return the captured arm-channel mask.
    pub const fn arm_mask(self) -> u8 {
        self.arm_mask
    }
}

/// Coalesced per-channel stop and arm reconciliation state.
pub struct StepperReconcilePending<const CHANNELS: usize> {
    stop_mask: AtomicU8,
    arm_mask: AtomicU8,
}

impl<const CHANNELS: usize> StepperReconcilePending<CHANNELS> {
    /// Construct empty pending state for `1..=8` board-local channels.
    pub const fn new() -> Self {
        assert!(CHANNELS > 0 && CHANNELS <= u8::BITS as usize);
        Self {
            stop_mask: AtomicU8::new(0),
            arm_mask: AtomicU8::new(0),
        }
    }

    /// Publish stop reconciliation for `channel`.
    pub fn request_stop(&self, channel: u8) -> Result<(), InvalidStepperReconcileChannel> {
        let bit = channel_bit::<CHANNELS>(channel)?;
        self.stop_mask.fetch_or(bit, Ordering::Release);
        Ok(())
    }

    /// Publish arm reconciliation for `channel`.
    pub fn request_arm(&self, channel: u8) -> Result<(), InvalidStepperReconcileChannel> {
        let bit = channel_bit::<CHANNELS>(channel)?;
        self.arm_mask.fetch_or(bit, Ordering::Release);
        Ok(())
    }

    /// Transfer currently published work into one local snapshot.
    ///
    /// The two masks transfer independently; a stop published between the swaps can follow an arm
    /// captured earlier, which is safe because stop producers clear `motor_armed` before the stop
    /// bit.
    pub fn take_pending(&self) -> PendingStepperReconciliation<CHANNELS> {
        PendingStepperReconciliation {
            stop_mask: self.stop_mask.swap(0, Ordering::Acquire),
            arm_mask: self.arm_mask.swap(0, Ordering::Acquire),
        }
    }
}

impl<const CHANNELS: usize> Default for StepperReconcilePending<CHANNELS> {
    fn default() -> Self {
        Self::new()
    }
}

fn channel_bit<const CHANNELS: usize>(channel: u8) -> Result<u8, InvalidStepperReconcileChannel> {
    if usize::from(channel) >= CHANNELS {
        return Err(InvalidStepperReconcileChannel {
            channel,
            channel_count: CHANNELS as u8,
        });
    }
    Ok(1_u8 << channel)
}

/// Drain one captured reconciliation snapshot in deterministic safety order.
pub fn drain_stepper_reconciliation<B, const CHANNELS: usize>(
    state: &mut TriggerDomainState,
    board: &mut B,
    pending: PendingStepperReconciliation<CHANNELS>,
) where
    B: StepperStopCheckBoard + StepperArmCheckBoard + ?Sized,
{
    for channel in 0..CHANNELS {
        let channel = channel as u8;
        let bit = 1_u8 << channel;
        if pending.stop_mask() & bit != 0 {
            stop_check(state, board, channel);
        }
    }
    for channel in 0..CHANNELS {
        let channel = channel as u8;
        let bit = 1_u8 << channel;
        if pending.arm_mask() & bit != 0 {
            arm_check(state, board, channel);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::context::stepper::{
        StepperArmCheckBoard, StepperStopCheckBoard, StepperTimerStateBoard,
    };
    use crate::context::trsync::TriggerDomainState;

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    enum Event {
        Stop(u8),
        Arm(u8),
    }

    struct MockBoard {
        events: heapless::Vec<Event, 8>,
        motor_armed: [bool; 2],
        queue_has_moves: [bool; 2],
        timer_active_mirror: [bool; 2],
    }

    impl MockBoard {
        fn ready_on_both_channels() -> Self {
            Self {
                events: heapless::Vec::new(),
                motor_armed: [true; 2],
                queue_has_moves: [true; 2],
                timer_active_mirror: [false; 2],
            }
        }
    }

    impl StepperTimerStateBoard for MockBoard {
        fn set_timer_active(
            &mut self,
            state: &mut TriggerDomainState,
            stepper_idx: usize,
            active: bool,
        ) {
            state.stepper_stop[stepper_idx].timer_active = active;
            self.timer_active_mirror[stepper_idx] = active;
        }
    }

    impl StepperStopCheckBoard for MockBoard {
        fn stop_check_index(&self, channel: u8) -> Option<usize> {
            (usize::from(channel) < self.motor_armed.len()).then_some(usize::from(channel))
        }

        fn clear_step_timer(
            &mut self,
            state: &mut TriggerDomainState,
            stepper_idx: usize,
            channel: u8,
        ) {
            self.events.push(Event::Stop(channel)).unwrap();
            self.set_timer_active(state, stepper_idx, false);
        }
    }

    impl StepperArmCheckBoard for MockBoard {
        fn arm_check_index(&self, channel: u8) -> Option<usize> {
            (usize::from(channel) < self.motor_armed.len()).then_some(usize::from(channel))
        }

        fn motor_armed(&self, channel: u8) -> bool {
            self.motor_armed[usize::from(channel)]
        }

        fn queue_has_moves(&self, channel: u8) -> bool {
            self.queue_has_moves[usize::from(channel)]
        }

        fn arm_check_timer(
            &mut self,
            state: &mut TriggerDomainState,
            stepper_idx: usize,
            channel: u8,
        ) {
            self.events.push(Event::Arm(channel)).unwrap();
            self.set_timer_active(state, stepper_idx, true);
        }
    }

    #[test]
    fn duplicate_requests_for_one_channel_coalesce() {
        let pending = StepperReconcilePending::<2>::new();

        pending.request_stop(0).unwrap();
        pending.request_stop(0).unwrap();
        pending.request_arm(0).unwrap();
        pending.request_arm(0).unwrap();

        let snapshot = pending.take_pending();
        assert_eq!(snapshot.stop_mask(), 0b01);
        assert_eq!(snapshot.arm_mask(), 0b01);
    }

    #[test]
    fn both_channels_remain_pending_simultaneously() {
        let pending = StepperReconcilePending::<2>::new();

        pending.request_arm(0).unwrap();
        pending.request_arm(1).unwrap();

        assert_eq!(pending.take_pending().arm_mask(), 0b11);
    }

    #[test]
    fn arm_and_stop_masks_remain_independent() {
        let pending = StepperReconcilePending::<2>::new();

        pending.request_stop(0).unwrap();
        pending.request_arm(1).unwrap();

        let snapshot = pending.take_pending();
        assert_eq!(snapshot.stop_mask(), 0b01);
        assert_eq!(snapshot.arm_mask(), 0b10);
    }

    #[test]
    fn request_after_snapshot_remains_for_the_next_drain() {
        let pending = StepperReconcilePending::<2>::new();
        pending.request_arm(0).unwrap();
        let first = pending.take_pending();

        pending.request_stop(1).unwrap();

        assert_eq!(first.arm_mask(), 0b01);
        assert_eq!(pending.take_pending().stop_mask(), 0b10);
        assert!(pending.take_pending().is_empty());
    }

    #[test]
    fn invalid_channel_changes_neither_mask() {
        let pending = StepperReconcilePending::<2>::new();

        let arm_error = pending.request_arm(2).unwrap_err();
        let stop_error = pending.request_stop(u8::MAX).unwrap_err();

        assert_eq!(arm_error.channel(), 2);
        assert_eq!(arm_error.channel_count(), 2);
        assert_eq!(stop_error.channel(), u8::MAX);
        assert_eq!(stop_error.channel_count(), 2);
        assert!(pending.take_pending().is_empty());
    }

    #[test]
    fn drain_processes_all_stops_before_any_arms() {
        let pending = StepperReconcilePending::<2>::new();
        pending.request_arm(1).unwrap();
        pending.request_stop(1).unwrap();
        pending.request_arm(0).unwrap();
        pending.request_stop(0).unwrap();
        let mut state = TriggerDomainState::new();
        let mut board = MockBoard::ready_on_both_channels();

        drain_stepper_reconciliation(&mut state, &mut board, pending.take_pending());

        assert_eq!(
            board.events.as_slice(),
            &[Event::Stop(0), Event::Stop(1), Event::Arm(0), Event::Arm(1)],
        );
    }

    #[test]
    fn captured_arm_noops_after_newer_stop_publishes_motor_unarmed() {
        let pending = StepperReconcilePending::<1>::new();
        pending.request_arm(0).unwrap();
        let earlier_arm = pending.take_pending();
        let mut state = TriggerDomainState::new();
        let mut board = MockBoard::ready_on_both_channels();

        board.motor_armed[0] = false;
        pending.request_stop(0).unwrap();
        drain_stepper_reconciliation(&mut state, &mut board, earlier_arm);
        assert!(board.events.is_empty());

        drain_stepper_reconciliation(&mut state, &mut board, pending.take_pending());
        assert_eq!(board.events.as_slice(), &[Event::Stop(0)]);
    }
}
