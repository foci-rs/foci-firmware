//! Shared stepper context boundary and policy helpers.
//!
//! `StepperHost` is the handler-facing trait. The remaining items are
//! board-hook traits and shared policy helpers used by board crates.

use foci_klipper::commands::CommandError;
use foci_klipper::commands::stepper::{QueueStepResult, queue_step_hw, reset_step_clock_hw};
use foci_klipper::stepper::StepMove;

use super::trsync::TriggerDomainState;

/// Stepper subsystem: direction/queue/clock operations exposed at the
/// granularity the handler bodies actually need.
///
/// The step-timer / stop-check helpers exercised only by the RTIC task graph
/// are intentionally separate from the handler-facing trait.
pub trait StepperHost: super::StateAccess {
    /// Capture the direction bit for the next enqueued move on `channel`.
    fn set_next_step_dir(&mut self, channel: u8, forward: bool);

    /// Enqueue a trapezoidal move onto the hardware step queue for `channel`.
    fn queue_step_move(&mut self, channel: u8, move_: StepMove) -> Result<(), StepperEnqueueError>;

    /// Reset the step-clock reference for `channel`.
    fn reset_step_clock(&mut self, channel: u8, clock: u32) -> Result<(), CommandError>;
}

impl<T: ?Sized + StepperHost> StepperHost for &mut T {
    fn set_next_step_dir(&mut self, channel: u8, forward: bool) {
        (**self).set_next_step_dir(channel, forward)
    }

    fn queue_step_move(&mut self, channel: u8, move_: StepMove) -> Result<(), StepperEnqueueError> {
        (**self).queue_step_move(channel, move_)
    }

    fn reset_step_clock(&mut self, channel: u8, clock: u32) -> Result<(), CommandError> {
        (**self).reset_step_clock(channel, clock)
    }
}

/// Queue drain accounting returned by board-local reset hooks.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct StepperDrainStats {
    /// Number of stale move segments drained from the board queue.
    pub segments: u32,
    /// Number of stale steps represented by drained move segments.
    pub steps: u32,
}

impl StepperDrainStats {
    /// No drain counters were recorded.
    pub const ZERO: Self = Self {
        segments: 0,
        steps: 0,
    };
}

/// Distinct command-path failures while validating or enqueuing a move.
#[derive(Debug, Clone, PartialEq, Eq, defmt::Format)]
pub enum StepperEnqueueError {
    /// A prerequisite board lookup failed before enqueue.
    Command(CommandError),
    /// The physical move queue could not accept the segment.
    QueueFull {
        /// Board-local channel that rejected the segment.
        channel: u8,
        /// Queue length observed at the failed insertion.
        queue_len: usize,
    },
    /// A move with no logical steps was rejected.
    ZeroCount {
        /// Board-local channel that rejected the segment.
        channel: u8,
    },
    /// Motion was requested before the logical pulse mapper was installed.
    MissingMotionScale {
        /// Board-local channel that rejected the segment.
        channel: u8,
    },
}

/// Board hook that keeps TriggerDomain timer state and the P1 mirror in sync.
pub trait StepperTimerStateBoard {
    /// Set shared timer-active state and update the board's P1-readable mirror.
    fn set_timer_active(
        &mut self,
        state: &mut TriggerDomainState,
        stepper_idx: usize,
        active: bool,
    );
}

/// Board hooks needed by shared `queue_step_move` policy.
pub trait StepperQueueBoard: StepperTimerStateBoard {
    /// Map an already validated command channel to a trigger-domain stepper.
    fn queue_stepper_index(&self, channel: u8) -> Result<usize, CommandError>;

    /// Return whether the board queue is empty before enqueue.
    fn queue_was_empty(&self, channel: u8) -> Result<bool, CommandError>;

    /// Return whether the channel's logical-to-physical mapper is installed.
    fn motion_scale_configured(&self, channel: u8) -> Result<bool, CommandError>;

    /// Prepare the move with board-local P2 stepper state and enqueue it.
    fn prepare_and_enqueue(&mut self, channel: u8, move_: StepMove) -> Result<usize, usize>;

    /// Return whether the motor channel is electrically armed.
    fn motor_armed(&self, channel: u8) -> bool;

    /// Record a discarded segment.
    fn record_discarded_segment(&mut self, _channel: u8, _steps: u32) {}

    /// Record queue-full diagnostics.
    fn record_queue_full(&mut self, _channel: u8, _queue_len: usize) {}

    /// Record an accepted segment and trace queue depth.
    fn record_queued_segment(&mut self, _channel: u8, _steps: u32, _queue_len: usize) {}

    /// Arm the board timer after shared policy has accepted a queued move.
    fn arm_queue_step_timer(
        &mut self,
        state: &mut TriggerDomainState,
        stepper_idx: usize,
        channel: u8,
    );
}

/// Board hooks needed by shared `reset_step_clock` policy.
pub trait StepperResetBoard {
    /// Map an already validated command channel to a trigger-domain stepper.
    fn reset_stepper_index(&self, channel: u8) -> Result<usize, CommandError>;

    /// Drain stale queued moves after `reset_step_clock_hw` accepts the reset.
    fn drain_step_queue(&mut self, channel: u8) -> Result<StepperDrainStats, CommandError>;

    /// Reset board-local P2 and ISR stepper clocks.
    fn reset_stepper_clocks(&mut self, channel: u8, clock: u32) -> Result<(), CommandError>;

    /// Record board-local reset diagnostics.
    fn record_reset_step_clock(&mut self, _channel: u8, _clock: u32, _stats: StepperDrainStats) {}
}

/// Board hooks needed by shared P2 stop-check policy.
pub trait StepperStopCheckBoard: StepperTimerStateBoard {
    /// Map a P2 stop-check channel to a trigger-domain stepper.
    fn stop_check_index(&self, channel: u8) -> Option<usize>;

    /// Perform board-local stop-check side effects and clear timer-active.
    fn clear_step_timer(&mut self, state: &mut TriggerDomainState, stepper_idx: usize, channel: u8);
}

/// Board hooks needed by shared P2 arm-check policy.
pub trait StepperArmCheckBoard: StepperTimerStateBoard {
    /// Map a P2 arm-check channel to a trigger-domain stepper.
    fn arm_check_index(&self, channel: u8) -> Option<usize>;

    /// Return whether the motor channel is electrically armed.
    fn motor_armed(&self, channel: u8) -> bool;

    /// Return whether queued moves are waiting for this channel.
    fn queue_has_moves(&self, channel: u8) -> bool;

    /// Perform board-local arm-check side effects and set timer-active.
    fn arm_check_timer(&mut self, state: &mut TriggerDomainState, stepper_idx: usize, channel: u8);
}

/// Shared command-path queue-step policy.
pub fn queue_step_move<B: StepperQueueBoard + ?Sized>(
    state: &mut TriggerDomainState,
    board: &mut B,
    channel: u8,
    move_: StepMove,
) -> Result<(), StepperEnqueueError> {
    let stepper_idx = board
        .queue_stepper_index(channel)
        .map_err(StepperEnqueueError::Command)?;
    let need_reset = state.stepper_stop[stepper_idx].need_reset;

    // Resolve reset discard and zero count before consulting any fallible board
    // state. The placeholder values admit every positive, non-reset move.
    match queue_step_hw(need_reset, move_.count, false, true) {
        Ok(QueueStepResult::Discarded) => {
            board.record_discarded_segment(channel, u32::from(move_.count));
            if note_host_segment_discarded(state, stepper_idx, board.motor_armed(channel)) {
                defmt::warn!(
                    "stepper {}: discarding host steps until reset_step_clock (motor armed)",
                    channel
                );
            }
            return Ok(());
        }
        Err(CommandError::ZeroCount) => {
            return Err(StepperEnqueueError::ZeroCount { channel });
        }
        Err(err) => return Err(StepperEnqueueError::Command(err)),
        Ok(QueueStepResult::Queued | QueueStepResult::QueuedFirstMove) => {}
    }

    let queue_was_empty = board
        .queue_was_empty(channel)
        .map_err(StepperEnqueueError::Command)?;
    let motion_scale_configured = board
        .motion_scale_configured(channel)
        .map_err(StepperEnqueueError::Command)?;

    match queue_step_hw(false, move_.count, queue_was_empty, motion_scale_configured) {
        Ok(QueueStepResult::Discarded) | Err(CommandError::ZeroCount) => unreachable!(),
        Err(CommandError::MotionScaleUnconfigured) => {
            Err(StepperEnqueueError::MissingMotionScale { channel })
        }
        Err(err) => Err(StepperEnqueueError::Command(err)),
        Ok(QueueStepResult::Queued | QueueStepResult::QueuedFirstMove) => {
            let queue_len = board
                .prepare_and_enqueue(channel, move_)
                .map_err(|queue_len| {
                    board.record_queue_full(channel, queue_len);
                    StepperEnqueueError::QueueFull { channel, queue_len }
                })?;

            board.record_queued_segment(channel, u32::from(move_.count), queue_len);

            if !state.stepper_stop[stepper_idx].timer_active && board.motor_armed(channel) {
                board.arm_queue_step_timer(state, stepper_idx, channel);
            }
            Ok(())
        }
    }
}

/// Shared command-path reset-step-clock policy.
pub fn reset_step_clock<B: StepperResetBoard + ?Sized>(
    state: &mut TriggerDomainState,
    board: &mut B,
    channel: u8,
    clock: u32,
) -> Result<(), CommandError> {
    let stepper_idx = board.reset_stepper_index(channel)?;
    reset_step_clock_hw(state.stepper_stop[stepper_idx].timer_active)?;
    let stats = board.drain_step_queue(channel)?;
    board.reset_stepper_clocks(channel, clock)?;
    board.record_reset_step_clock(channel, clock, stats);

    if !state.stepper_stop[stepper_idx].blocked_by_trigger {
        state.stepper_stop[stepper_idx].need_reset = false;
        state.discard_silenced[stepper_idx] = false;
    }
    Ok(())
}

/// Record a host segment discarded under `need_reset` and return whether it
/// is the first worth logging in this episode.
///
/// A trigger stop silences its own episode when it fires, since the host
/// keeps streaming until it has processed the trigger, and a disarmed motor
/// is not being driven. Anything else means the host is stepping a channel
/// the firmware stopped without telling it.
pub fn note_host_segment_discarded(
    state: &mut TriggerDomainState,
    stepper_idx: usize,
    motor_armed: bool,
) -> bool {
    if !motor_armed || state.discard_silenced[stepper_idx] {
        return false;
    }
    state.discard_silenced[stepper_idx] = true;
    true
}

/// Shared P2 stop-check policy.
pub fn stop_check<B: StepperStopCheckBoard + ?Sized>(
    state: &mut TriggerDomainState,
    board: &mut B,
    channel: u8,
) {
    if let Some(stepper_idx) = board.stop_check_index(channel) {
        board.clear_step_timer(state, stepper_idx, channel);
    }
}

/// Shared P2 arm-check policy.
pub fn arm_check<B: StepperArmCheckBoard + ?Sized>(
    state: &mut TriggerDomainState,
    board: &mut B,
    channel: u8,
) {
    let Some(stepper_idx) = board.arm_check_index(channel) else {
        return;
    };
    if !board.motor_armed(channel) || state.stepper_stop[stepper_idx].timer_active {
        return;
    }
    if board.queue_has_moves(channel) {
        board.arm_check_timer(state, stepper_idx, channel);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    enum Event {
        Discarded(u8, u32),
        Queued(u8, u32, usize),
        QueueFull(u8, usize),
        QueueArm(usize, u8),
        Reset(u8, u32, StepperDrainStats),
        StopClear(usize, u8),
        ArmCheck(usize, u8),
    }

    struct MockStepperBoard {
        index: Option<usize>,
        queue_empty: bool,
        queue_empty_error: Option<CommandError>,
        enqueue_full: bool,
        motion_scale_configured: bool,
        motion_scale_error: Option<CommandError>,
        enqueue_calls: u8,
        queue_len: usize,
        motor_armed: bool,
        queue_has_moves: bool,
        drain_stats: StepperDrainStats,
        mirror: [bool; foci_klipper::state::NUM_STEPPERS],
        events: heapless::Vec<Event, 16>,
    }

    impl Default for MockStepperBoard {
        fn default() -> Self {
            Self {
                index: Some(0),
                queue_empty: true,
                queue_empty_error: None,
                enqueue_full: false,
                motion_scale_configured: true,
                motion_scale_error: None,
                enqueue_calls: 0,
                queue_len: 1,
                motor_armed: false,
                queue_has_moves: false,
                drain_stats: StepperDrainStats::ZERO,
                mirror: [false; foci_klipper::state::NUM_STEPPERS],
                events: heapless::Vec::new(),
            }
        }
    }

    impl StepperTimerStateBoard for MockStepperBoard {
        fn set_timer_active(
            &mut self,
            state: &mut TriggerDomainState,
            stepper_idx: usize,
            active: bool,
        ) {
            state.stepper_stop[stepper_idx].timer_active = active;
            self.mirror[stepper_idx] = active;
        }
    }

    impl StepperQueueBoard for MockStepperBoard {
        fn queue_stepper_index(&self, channel: u8) -> Result<usize, CommandError> {
            self.index.ok_or(CommandError::InvalidChannel(channel))
        }

        fn queue_was_empty(&self, _channel: u8) -> Result<bool, CommandError> {
            self.queue_empty_error
                .clone()
                .map_or(Ok(self.queue_empty), Err)
        }

        fn motion_scale_configured(&self, _channel: u8) -> Result<bool, CommandError> {
            self.motion_scale_error
                .clone()
                .map_or(Ok(self.motion_scale_configured), Err)
        }

        fn prepare_and_enqueue(&mut self, _channel: u8, _move_: StepMove) -> Result<usize, usize> {
            self.enqueue_calls = self.enqueue_calls.saturating_add(1);
            if self.enqueue_full {
                Err(self.queue_len)
            } else {
                Ok(self.queue_len)
            }
        }

        fn motor_armed(&self, _channel: u8) -> bool {
            self.motor_armed
        }

        fn record_discarded_segment(&mut self, channel: u8, steps: u32) {
            self.events.push(Event::Discarded(channel, steps)).unwrap();
        }

        fn record_queue_full(&mut self, channel: u8, queue_len: usize) {
            self.events
                .push(Event::QueueFull(channel, queue_len))
                .unwrap();
        }

        fn record_queued_segment(&mut self, channel: u8, steps: u32, queue_len: usize) {
            self.events
                .push(Event::Queued(channel, steps, queue_len))
                .unwrap();
        }

        fn arm_queue_step_timer(
            &mut self,
            state: &mut TriggerDomainState,
            stepper_idx: usize,
            channel: u8,
        ) {
            self.events
                .push(Event::QueueArm(stepper_idx, channel))
                .unwrap();
            self.set_timer_active(state, stepper_idx, true);
        }
    }

    impl StepperResetBoard for MockStepperBoard {
        fn reset_stepper_index(&self, channel: u8) -> Result<usize, CommandError> {
            self.index.ok_or(CommandError::InvalidChannel(channel))
        }

        fn drain_step_queue(&mut self, _channel: u8) -> Result<StepperDrainStats, CommandError> {
            Ok(self.drain_stats)
        }

        fn reset_stepper_clocks(&mut self, _channel: u8, _clock: u32) -> Result<(), CommandError> {
            Ok(())
        }

        fn record_reset_step_clock(&mut self, channel: u8, clock: u32, stats: StepperDrainStats) {
            self.events
                .push(Event::Reset(channel, clock, stats))
                .unwrap();
        }
    }

    impl StepperStopCheckBoard for MockStepperBoard {
        fn stop_check_index(&self, _channel: u8) -> Option<usize> {
            self.index
        }

        fn clear_step_timer(
            &mut self,
            state: &mut TriggerDomainState,
            stepper_idx: usize,
            channel: u8,
        ) {
            self.events
                .push(Event::StopClear(stepper_idx, channel))
                .unwrap();
            self.set_timer_active(state, stepper_idx, false);
        }
    }

    impl StepperArmCheckBoard for MockStepperBoard {
        fn arm_check_index(&self, _channel: u8) -> Option<usize> {
            self.index
        }

        fn motor_armed(&self, _channel: u8) -> bool {
            self.motor_armed
        }

        fn queue_has_moves(&self, _channel: u8) -> bool {
            self.queue_has_moves
        }

        fn arm_check_timer(
            &mut self,
            state: &mut TriggerDomainState,
            stepper_idx: usize,
            channel: u8,
        ) {
            self.events
                .push(Event::ArmCheck(stepper_idx, channel))
                .unwrap();
            self.set_timer_active(state, stepper_idx, true);
        }
    }

    #[test]
    fn discarded_host_segment_is_reported_once_per_needs_reset_episode() {
        let mut state = TriggerDomainState::new();
        let mut board = MockStepperBoard::default();
        state.stepper_stop[0].need_reset = true;

        assert!(note_host_segment_discarded(&mut state, 0, true));
        assert!(!note_host_segment_discarded(&mut state, 0, true));

        assert_eq!(reset_step_clock(&mut state, &mut board, 0, 5000), Ok(()));
        state.stepper_stop[0].need_reset = true;
        assert!(note_host_segment_discarded(&mut state, 0, true));
    }

    #[test]
    fn discarded_host_segment_with_motor_disarmed_is_not_reported() {
        let mut state = TriggerDomainState::new();
        state.stepper_stop[0].need_reset = true;

        assert!(!note_host_segment_discarded(&mut state, 0, false));
    }

    #[test]
    fn queue_step_move_discards_when_need_reset() {
        let mut state = TriggerDomainState::new();
        state.stepper_stop[0].need_reset = true;
        let mut board = MockStepperBoard::default();

        assert_eq!(
            queue_step_move(&mut state, &mut board, 0, StepMove::new(100, 12, 0)),
            Ok(())
        );

        assert_eq!(board.events.as_slice(), &[Event::Discarded(0, 12)]);
        assert_eq!(board.enqueue_calls, 0);
        assert!(!state.stepper_stop[0].timer_active);
        assert!(!board.mirror[0]);
    }

    #[test]
    fn queue_step_move_reset_discard_precedes_fallible_ancillary_hooks() {
        let mut state = TriggerDomainState::new();
        state.stepper_stop[0].need_reset = true;
        let mut board = MockStepperBoard {
            queue_empty_error: Some(CommandError::QueueFull),
            motion_scale_error: Some(CommandError::MotionScaleUnconfigured),
            ..MockStepperBoard::default()
        };

        assert_eq!(
            queue_step_move(&mut state, &mut board, 0, StepMove::new(100, 12, 0)),
            Ok(())
        );
        assert_eq!(board.events.as_slice(), &[Event::Discarded(0, 12)]);
        assert_eq!(board.enqueue_calls, 0);
        assert!(!state.stepper_stop[0].timer_active);
        assert!(!board.mirror[0]);
    }

    #[test]
    fn queue_step_move_propagates_zero_count_rejection() {
        let mut state = TriggerDomainState::new();
        let mut board = MockStepperBoard::default();

        assert_eq!(
            queue_step_move(&mut state, &mut board, 0, StepMove::new(100, 0, 0)),
            Err(StepperEnqueueError::ZeroCount { channel: 0 })
        );

        assert!(board.events.is_empty());
        assert_eq!(board.enqueue_calls, 0);
    }

    #[test]
    fn queue_step_move_zero_count_precedes_fallible_ancillary_hooks() {
        let mut state = TriggerDomainState::new();
        let mut board = MockStepperBoard {
            queue_empty_error: Some(CommandError::QueueFull),
            motion_scale_error: Some(CommandError::MotionScaleUnconfigured),
            ..MockStepperBoard::default()
        };

        assert_eq!(
            queue_step_move(&mut state, &mut board, 0, StepMove::new(100, 0, 0)),
            Err(StepperEnqueueError::ZeroCount { channel: 0 })
        );
        assert!(board.events.is_empty());
        assert_eq!(board.enqueue_calls, 0);
        assert!(!state.stepper_stop[0].timer_active);
        assert!(!board.mirror[0]);
    }

    #[test]
    fn queue_step_move_rejects_missing_scale_without_enqueue() {
        let mut state = TriggerDomainState::new();
        let mut board = MockStepperBoard {
            motion_scale_configured: false,
            ..MockStepperBoard::default()
        };

        assert_eq!(
            queue_step_move(&mut state, &mut board, 0, StepMove::new(100, 1, 0)),
            Err(StepperEnqueueError::MissingMotionScale { channel: 0 })
        );
        assert!(board.events.is_empty());
    }

    #[test]
    fn queue_step_move_accepts_interval_shorter_than_waveform_cost() {
        let mut state = TriggerDomainState::new();
        let mut board = MockStepperBoard::default();

        assert_eq!(
            queue_step_move(&mut state, &mut board, 0, StepMove::new(1, 3, 0)),
            Ok(())
        );
        assert_eq!(board.events.as_slice(), &[Event::Queued(0, 3, 1)]);
    }

    #[test]
    fn queue_step_move_records_enqueue_before_timer_arm() {
        let mut state = TriggerDomainState::new();
        let mut board = MockStepperBoard {
            motor_armed: true,
            queue_len: 3,
            ..MockStepperBoard::default()
        };

        assert_eq!(
            queue_step_move(&mut state, &mut board, 0, StepMove::new(100, 8, 0)),
            Ok(())
        );

        assert_eq!(
            board.events.as_slice(),
            &[Event::Queued(0, 8, 3), Event::QueueArm(0, 0)]
        );
        assert!(state.stepper_stop[0].timer_active);
        assert!(board.mirror[0]);
    }

    #[test]
    fn queue_step_move_reports_queue_full() {
        let mut state = TriggerDomainState::new();
        let mut board = MockStepperBoard {
            enqueue_full: true,
            queue_len: 7,
            ..MockStepperBoard::default()
        };

        assert_eq!(
            queue_step_move(&mut state, &mut board, 0, StepMove::new(100, 4, 0)),
            Err(StepperEnqueueError::QueueFull {
                channel: 0,
                queue_len: 7,
            })
        );
        assert_eq!(board.events.as_slice(), &[Event::QueueFull(0, 7)]);
    }

    #[test]
    fn reset_step_clock_rejects_active_stepper() {
        let mut state = TriggerDomainState::new();
        state.stepper_stop[0].timer_active = true;
        let mut board = MockStepperBoard::default();

        assert_eq!(
            reset_step_clock(&mut state, &mut board, 0, 1234),
            Err(CommandError::StepperActive)
        );
        assert!(board.events.is_empty());
    }

    #[test]
    fn reset_step_clock_drains_records_and_clears_need_reset_when_unblocked() {
        let mut state = TriggerDomainState::new();
        state.stepper_stop[0].need_reset = true;
        let mut board = MockStepperBoard {
            drain_stats: StepperDrainStats {
                segments: 2,
                steps: 17,
            },
            ..MockStepperBoard::default()
        };

        assert_eq!(reset_step_clock(&mut state, &mut board, 0, 1234), Ok(()));

        assert!(!state.stepper_stop[0].need_reset);
        assert_eq!(
            board.events.as_slice(),
            &[Event::Reset(
                0,
                1234,
                StepperDrainStats {
                    segments: 2,
                    steps: 17,
                },
            )]
        );
    }

    #[test]
    fn reset_step_clock_preserves_need_reset_when_blocked_by_trigger() {
        let mut state = TriggerDomainState::new();
        state.stepper_stop[0].need_reset = true;
        state.stepper_stop[0].blocked_by_trigger = true;
        let mut board = MockStepperBoard::default();

        assert_eq!(reset_step_clock(&mut state, &mut board, 0, 1234), Ok(()));

        assert!(state.stepper_stop[0].need_reset);
    }

    #[test]
    fn stop_check_clears_timer_active_and_mirror() {
        let mut state = TriggerDomainState::new();
        state.stepper_stop[0].timer_active = true;
        let mut board = MockStepperBoard::default();
        board.mirror[0] = true;

        stop_check(&mut state, &mut board, 0);

        assert_eq!(board.events.as_slice(), &[Event::StopClear(0, 0)]);
        assert!(!state.stepper_stop[0].timer_active);
        assert!(!board.mirror[0]);
    }

    #[test]
    fn arm_check_returns_when_motor_unarmed_timer_active_or_queue_empty() {
        let mut state = TriggerDomainState::new();
        let mut board = MockStepperBoard::default();

        arm_check(&mut state, &mut board, 0);
        assert!(board.events.is_empty());

        board.motor_armed = true;
        state.stepper_stop[0].timer_active = true;
        arm_check(&mut state, &mut board, 0);
        assert!(board.events.is_empty());

        state.stepper_stop[0].timer_active = false;
        board.queue_has_moves = false;
        arm_check(&mut state, &mut board, 0);
        assert!(board.events.is_empty());
    }

    #[test]
    fn arm_check_arms_timer_and_sets_mirror_when_all_gates_pass() {
        let mut state = TriggerDomainState::new();
        let mut board = MockStepperBoard {
            motor_armed: true,
            queue_has_moves: true,
            ..MockStepperBoard::default()
        };

        arm_check(&mut state, &mut board, 0);

        assert_eq!(board.events.as_slice(), &[Event::ArmCheck(0, 0)]);
        assert!(state.stepper_stop[0].timer_active);
        assert!(board.mirror[0]);
    }
}
