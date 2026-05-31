//! Shared trsync/endstop context boundary and policy helpers.
//!
//! `TrsyncHost` is the handler-facing trait. The remaining items in this
//! module are the shared trigger-domain policy surface used by board crates.

use core::sync::atomic::Ordering;

use foci_klipper::scheduler::{
    ScheduleResult, SchedulerEvent, SchedulerEventKind, TriggerScheduler,
};
use foci_klipper::state::{NUM_ENDSTOPS, NUM_STEPPERS, NUM_TRSYNC};
use foci_klipper::trigger::{StopSignal, TriggerResult};

use crate::diagnostic::StepperEventReason;
use crate::state::trsync::HOMING_TRIGGERED;

/// Trsync + endstop-trigger subsystem bridge. Each method corresponds
/// one-to-one to a single handler's `with_trigger_domain(|td| ...)`
/// closure body: the trait exposes handler-specific coarse methods.
///
/// `TriggerDomain` itself stays in the board crate; this trait is the
/// dyn-safe bridge used from the lifted handlers.
#[allow(clippy::result_unit_err)]
pub trait TrsyncHost: crate::context::StateAccess {
    /// Full trigger-domain reset used by `config_reset` to bring
    /// steppers, trsync, endstops, and scheduler back to power-on
    /// defaults.
    fn reset_all(&mut self);

    /// Allocate a trsync slot for `oid`.
    ///
    /// Returns `Err` if the OID is already registered or no free slot exists.
    fn register_trsync(&mut self, oid: u8) -> Result<(), ()>;

    /// Arm a trsync for synchronized triggering.
    fn trsync_arm(
        &mut self,
        oid: u8,
        report_clock: u32,
        report_ticks: u32,
        expire_reason: u8,
    ) -> Result<(), ()>;

    /// Update the expiry deadline for a trsync and schedule its expire event.
    fn trsync_set_timeout(&mut self, oid: u8, clock: u32) -> Result<(), ()>;

    /// Manually fire a trsync and return the effective trigger reason.
    fn trsync_trigger(&mut self, oid: u8, reason: u8) -> Result<u8, ()>;

    /// Allocate an endstop slot for `oid`.
    ///
    /// Returns the allocated index on success.
    fn register_endstop(&mut self, oid: u8) -> Result<usize, ()>;

    /// Configure an endstop runtime for homing and schedule its first sample.
    ///
    /// `sample_count == 0` acts as a disarm.
    #[allow(clippy::too_many_arguments)]
    fn endstop_start_homing(
        &mut self,
        oid: u8,
        clock: u32,
        sample_ticks: u32,
        sample_count: u8,
        rest_ticks: u32,
        pin_value_expected: bool,
        trsync_oid: u8,
        trigger_reason: u8,
    ) -> Result<(), ()>;

    /// Read the endstop runtime state tuple `(is_homing, next_clock, pin_id)`.
    fn endstop_query(&mut self, oid: u8) -> Result<(bool, u32, u8), ()>;

    /// Link a physical stepper to a trsync stop signal.
    fn link_stepper_stop(&mut self, stepper_idx: usize, trsync_oid: u8) -> Result<(), ()>;
}

#[allow(clippy::result_unit_err)]
impl<T: ?Sized + TrsyncHost> TrsyncHost for &mut T {
    fn reset_all(&mut self) {
        (**self).reset_all()
    }

    fn register_trsync(&mut self, oid: u8) -> Result<(), ()> {
        (**self).register_trsync(oid)
    }

    fn trsync_arm(
        &mut self,
        oid: u8,
        report_clock: u32,
        report_ticks: u32,
        expire_reason: u8,
    ) -> Result<(), ()> {
        (**self).trsync_arm(oid, report_clock, report_ticks, expire_reason)
    }

    fn trsync_set_timeout(&mut self, oid: u8, clock: u32) -> Result<(), ()> {
        (**self).trsync_set_timeout(oid, clock)
    }

    fn trsync_trigger(&mut self, oid: u8, reason: u8) -> Result<u8, ()> {
        (**self).trsync_trigger(oid, reason)
    }

    fn register_endstop(&mut self, oid: u8) -> Result<usize, ()> {
        (**self).register_endstop(oid)
    }

    fn endstop_start_homing(
        &mut self,
        oid: u8,
        clock: u32,
        sample_ticks: u32,
        sample_count: u8,
        rest_ticks: u32,
        pin_value_expected: bool,
        trsync_oid: u8,
        trigger_reason: u8,
    ) -> Result<(), ()> {
        (**self).endstop_start_homing(
            oid,
            clock,
            sample_ticks,
            sample_count,
            rest_ticks,
            pin_value_expected,
            trsync_oid,
            trigger_reason,
        )
    }

    fn endstop_query(&mut self, oid: u8) -> Result<(bool, u32, u8), ()> {
        (**self).endstop_query(oid)
    }

    fn link_stepper_stop(&mut self, stepper_idx: usize, trsync_oid: u8) -> Result<(), ()> {
        (**self).link_stepper_stop(stepper_idx, trsync_oid)
    }
}

/// Maximum number of stepper stop links tracked by a trigger domain.
pub const STOP_SIGNAL_CAPACITY: usize = 8;

/// Shared trsync, endstop, scheduler, and stepper-stop data.
///
/// The fields are public because
/// board-local shutdown and stepper code still reads this state directly.
pub struct TriggerDomainState {
    /// Runtime state for configured trsync objects.
    pub trsync: [foci_klipper::trigger::TrSyncRuntime; NUM_TRSYNC],
    /// OID-to-runtime-slot map for trsync objects.
    pub trsync_by_oid: heapless::LinearMap<u8, usize, { NUM_TRSYNC }>,
    /// Stop links consumed on first trsync trigger.
    pub stop_signals: [Option<StopSignal>; STOP_SIGNAL_CAPACITY],
    /// Per-stepper state used by triggered stops and reset checks.
    pub stepper_stop: [foci_klipper::trigger::StepperStopRuntime; NUM_STEPPERS],
    /// Runtime state for configured endstops.
    pub endstops: [foci_klipper::endstop::EndstopRuntime; NUM_ENDSTOPS],
    /// OID-to-runtime-slot map for endstops.
    pub endstop_by_oid: heapless::LinearMap<u8, usize, { NUM_ENDSTOPS }>,
    /// Unified trigger scheduler.
    pub scheduler: TriggerScheduler,
}

impl TriggerDomainState {
    /// Return a power-on trigger-domain state.
    pub const fn new() -> Self {
        Self {
            trsync: [const { foci_klipper::trigger::TrSyncRuntime::new() }; NUM_TRSYNC],
            trsync_by_oid: heapless::LinearMap::new(),
            stop_signals: [None; STOP_SIGNAL_CAPACITY],
            stepper_stop: [const { foci_klipper::trigger::StepperStopRuntime::new() };
                NUM_STEPPERS],
            endstops: [const { foci_klipper::endstop::EndstopRuntime::new() }; NUM_ENDSTOPS],
            endstop_by_oid: heapless::LinearMap::new(),
            scheduler: TriggerScheduler::new(),
        }
    }
}

impl Default for TriggerDomainState {
    fn default() -> Self {
        Self::new()
    }
}

/// Exact internal failure reason for shared trsync/endstop policy helpers.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum TrsyncPolicyError {
    /// A trsync OID was registered more than once.
    DuplicateTrsyncOid,
    /// No trsync runtime slot was available.
    TrsyncCapacity,
    /// A trsync OID was not registered.
    UnknownTrsyncOid,
    /// An endstop OID was registered more than once.
    DuplicateEndstopOid,
    /// No endstop runtime slot was available.
    EndstopCapacity,
    /// An endstop OID was not registered.
    UnknownEndstopOid,
    /// Active homing referenced a trsync OID that was not registered.
    MissingTrsyncForHoming,
    /// A Klipper stepper index did not map to a physical stepper.
    InvalidStepper,
    /// A physical stepper already had a stop link.
    DuplicateStopSignal,
    /// No stop-signal slot was available.
    StopSignalCapacity,
}

/// Board effects required by trigger dispatch.
///
/// This hook is intentionally smaller than `TrsyncBoard`: TIM5 ISR callers can
/// construct it without borrowing `State`.
pub trait TriggerDispatchBoard {
    /// Stop a physical stepper before shared code marks it blocked.
    fn stop_physical_stepper(
        &mut self,
        state: &TriggerDomainState,
        stepper_idx: usize,
        reason: StepperEventReason,
    );

    /// Record board-local diagnostics for a trsync stop.
    fn record_trsync_stop(&mut self, stepper_idx: usize, clock: u32, trsync_oid: u8, reason: u8);

    /// Set shared timer-active state and update the board's P1-readable mirror.
    fn set_timer_active(
        &mut self,
        state: &mut TriggerDomainState,
        stepper_idx: usize,
        active: bool,
    );
}

/// Command-path board hooks used by shared `TrsyncHost` policy helpers.
pub trait TrsyncBoard: TriggerDispatchBoard {
    /// Physical stepper indexes controlled by this board.
    fn physical_steppers(&self) -> &'static [usize];

    /// Map a Klipper stepper index to a physical stepper index.
    fn map_physical_stepper(&self, klipper_stepper_idx: usize) -> Option<usize>;

    /// Return the configured physical pin for an endstop runtime slot.
    fn endstop_pin_id(&self, endstop_idx: usize) -> u8;

    /// Record the result of a scheduler insertion.
    fn record_schedule_result(&mut self, result: ScheduleResult);

    /// Reprogram the board trigger timer from the shared scheduler.
    fn reprogram_trigger_timer(&mut self, state: &mut TriggerDomainState);

    /// Disarm the board trigger timer using board-correct compare-channel semantics.
    fn disarm_trigger_timer(&mut self, state: &mut TriggerDomainState);
}

/// Reset config-time trsync, endstop, scheduler, and stepper-stop state.
pub fn reset_all<B: TrsyncBoard + ?Sized>(state: &mut TriggerDomainState, board: &mut B) {
    state.scheduler.clear();
    board.disarm_trigger_timer(state);
    for ts in state.trsync.iter_mut() {
        ts.reset();
    }
    for es in state.endstops.iter_mut() {
        es.reset();
    }
    for sig in state.stop_signals.iter_mut() {
        *sig = None;
    }
    state.trsync_by_oid.clear();
    state.endstop_by_oid.clear();
    for &idx in board.physical_steppers() {
        board.stop_physical_stepper(state, idx, StepperEventReason::TmcDisableSignal);
        board.set_timer_active(state, idx, false);
    }
    for ss in state.stepper_stop.iter_mut() {
        *ss = foci_klipper::trigger::StepperStopRuntime::new();
    }
}

/// Register a trsync OID.
pub fn register_trsync(state: &mut TriggerDomainState, oid: u8) -> Result<(), TrsyncPolicyError> {
    if state.trsync_by_oid.contains_key(&oid) {
        return Err(TrsyncPolicyError::DuplicateTrsyncOid);
    }
    let idx = state.trsync_by_oid.len();
    if idx >= NUM_TRSYNC {
        return Err(TrsyncPolicyError::TrsyncCapacity);
    }
    state
        .trsync_by_oid
        .insert(oid, idx)
        .map_err(|_| TrsyncPolicyError::TrsyncCapacity)?;
    Ok(())
}

/// Register an endstop OID and return its runtime slot.
pub fn register_endstop(
    state: &mut TriggerDomainState,
    oid: u8,
) -> Result<usize, TrsyncPolicyError> {
    if state.endstop_by_oid.contains_key(&oid) {
        return Err(TrsyncPolicyError::DuplicateEndstopOid);
    }
    let idx = state.endstop_by_oid.len();
    if idx >= NUM_ENDSTOPS {
        return Err(TrsyncPolicyError::EndstopCapacity);
    }
    state
        .endstop_by_oid
        .insert(oid, idx)
        .map_err(|_| TrsyncPolicyError::EndstopCapacity)?;
    Ok(idx)
}

/// Dispatch a trsync trigger and stop linked steppers on first trigger.
pub fn dispatch_trigger<B: TriggerDispatchBoard + ?Sized>(
    state: &mut TriggerDomainState,
    board: &mut B,
    trsync_oid: u8,
    reason: u8,
    clock: u32,
) -> TriggerResult {
    let idx = match state.trsync_by_oid.get(&trsync_oid) {
        Some(&idx) => idx,
        None => return TriggerResult::UnknownOid,
    };
    let result = state.trsync[idx].trigger(trsync_oid, reason, clock);
    if matches!(result, TriggerResult::First { .. }) {
        let mut stopped_indices: heapless::Vec<usize, STOP_SIGNAL_CAPACITY> = heapless::Vec::new();
        for slot in state.stop_signals.iter_mut() {
            if let Some(signal) = slot
                && signal.trsync_oid == trsync_oid
            {
                let _ = stopped_indices.push(signal.stepper_idx);
                *slot = None;
            }
        }
        for &stepper_idx in &stopped_indices {
            state
                .scheduler
                .cancel(|kind| matches!(kind, SchedulerEventKind::Stepper(i) if *i == stepper_idx));
            board.stop_physical_stepper(state, stepper_idx, StepperEventReason::TrsyncStop);
            board.record_trsync_stop(stepper_idx, clock, trsync_oid, reason);
            state.stepper_stop[stepper_idx].need_reset = true;
            state.stepper_stop[stepper_idx].blocked_by_trigger = true;
            state.stepper_stop[stepper_idx].blocked_by_trsync = Some(trsync_oid);
            board.set_timer_active(state, stepper_idx, false);
        }
        if !stopped_indices.is_empty() {
            HOMING_TRIGGERED.store(trsync_oid, Ordering::Relaxed);
        }
    }
    result
}

/// Arm a trsync for synchronized triggering.
pub fn trsync_arm<B: TrsyncBoard + ?Sized>(
    state: &mut TriggerDomainState,
    board: &mut B,
    oid: u8,
    report_clock: u32,
    report_ticks: u32,
    expire_reason: u8,
) -> Result<(), TrsyncPolicyError> {
    let idx = trsync_idx(state, oid)?;
    clear_stop_signals_for_oid(state, oid);
    unblock_steppers_for_oid(state, oid);
    state.scheduler.cancel(|kind| {
        matches!(
            kind,
            SchedulerEventKind::TrSyncExpire(i) | SchedulerEventKind::TrSyncReport(i)
                if *i == idx
        )
    });
    state.trsync[idx].arm(report_clock, report_ticks, expire_reason);
    if report_ticks > 0 {
        let result = state.scheduler.try_schedule(SchedulerEvent {
            wake_clock: report_clock,
            kind: SchedulerEventKind::TrSyncReport(idx),
        });
        board.record_schedule_result(result);
    }
    board.reprogram_trigger_timer(state);
    Ok(())
}

/// Update the expiry deadline for a trsync.
pub fn trsync_set_timeout<B: TrsyncBoard + ?Sized>(
    state: &mut TriggerDomainState,
    board: &mut B,
    oid: u8,
    clock: u32,
) -> Result<(), TrsyncPolicyError> {
    let idx = trsync_idx(state, oid)?;
    if state.trsync[idx].can_trigger() {
        state.trsync[idx].set_timeout(clock);
        state
            .scheduler
            .cancel(|kind| matches!(kind, SchedulerEventKind::TrSyncExpire(i) if *i == idx));
        let result = state.scheduler.try_schedule(SchedulerEvent {
            wake_clock: clock,
            kind: SchedulerEventKind::TrSyncExpire(idx),
        });
        board.record_schedule_result(result);
        board.reprogram_trigger_timer(state);
    }
    Ok(())
}

/// Manually fire a trsync and return the effective trigger reason.
pub fn trsync_trigger<B: TrsyncBoard + ?Sized>(
    state: &mut TriggerDomainState,
    board: &mut B,
    oid: u8,
    reason: u8,
) -> Result<u8, TrsyncPolicyError> {
    let idx = trsync_idx(state, oid)?;
    let _ = dispatch_trigger(state, board, oid, reason, 0);
    let trigger_reason = state.trsync[idx].trigger_reason();

    state.scheduler.cancel(|kind| {
        matches!(
            kind,
            SchedulerEventKind::TrSyncExpire(i) | SchedulerEventKind::TrSyncReport(i)
                if *i == idx
        )
    });
    state.trsync[idx].clear_pending_report();
    state.trsync[idx].cancel_timers();
    clear_stop_signals_for_oid(state, oid);
    unblock_steppers_for_oid(state, oid);
    board.reprogram_trigger_timer(state);
    Ok(trigger_reason)
}

/// Configure an endstop runtime for homing.
#[allow(clippy::too_many_arguments)]
pub fn endstop_start_homing<B: TrsyncBoard + ?Sized>(
    state: &mut TriggerDomainState,
    board: &mut B,
    oid: u8,
    clock: u32,
    sample_ticks: u32,
    sample_count: u8,
    rest_ticks: u32,
    pin_value_expected: bool,
    trsync_oid: u8,
    trigger_reason: u8,
) -> Result<(), TrsyncPolicyError> {
    let idx = endstop_idx(state, oid)?;
    if sample_count != 0 && !state.trsync_by_oid.contains_key(&trsync_oid) {
        return Err(TrsyncPolicyError::MissingTrsyncForHoming);
    }
    let pin_id = board.endstop_pin_id(idx);
    state
        .scheduler
        .cancel(|kind| matches!(kind, SchedulerEventKind::EndstopSample(i) if *i == idx));
    state.endstops[idx].start_homing(
        clock,
        sample_ticks,
        sample_count,
        rest_ticks,
        pin_value_expected,
        trsync_oid,
        trigger_reason,
        pin_id,
    );
    if sample_count > 0 {
        let result = state.scheduler.try_schedule(SchedulerEvent {
            wake_clock: clock,
            kind: SchedulerEventKind::EndstopSample(idx),
        });
        board.record_schedule_result(result);
    }
    board.reprogram_trigger_timer(state);
    Ok(())
}

/// Read the endstop runtime state tuple `(is_homing, next_clock, pin_id)`.
pub fn endstop_query<B: TrsyncBoard + ?Sized>(
    state: &TriggerDomainState,
    board: &B,
    oid: u8,
) -> Result<(bool, u32, u8), TrsyncPolicyError> {
    let idx = endstop_idx(state, oid)?;
    let (homing, next_clock) = state.endstops[idx].query_state();
    Ok((homing, next_clock, board.endstop_pin_id(idx)))
}

/// Link a physical stepper to a trsync stop signal.
pub fn link_stepper_stop<B: TrsyncBoard + ?Sized>(
    state: &mut TriggerDomainState,
    board: &B,
    stepper_idx: usize,
    trsync_oid: u8,
) -> Result<(), TrsyncPolicyError> {
    if !state.trsync_by_oid.contains_key(&trsync_oid) {
        return Err(TrsyncPolicyError::UnknownTrsyncOid);
    }
    let physical_idx = board
        .map_physical_stepper(stepper_idx)
        .ok_or(TrsyncPolicyError::InvalidStepper)?;
    for signal in state.stop_signals.iter().flatten() {
        if signal.stepper_idx == physical_idx {
            return Err(TrsyncPolicyError::DuplicateStopSignal);
        }
    }
    for slot in state.stop_signals.iter_mut() {
        if slot.is_none() {
            *slot = Some(StopSignal {
                trsync_oid,
                stepper_idx: physical_idx,
            });
            return Ok(());
        }
    }
    Err(TrsyncPolicyError::StopSignalCapacity)
}

fn trsync_idx(state: &TriggerDomainState, oid: u8) -> Result<usize, TrsyncPolicyError> {
    state
        .trsync_by_oid
        .get(&oid)
        .copied()
        .ok_or(TrsyncPolicyError::UnknownTrsyncOid)
}

fn endstop_idx(state: &TriggerDomainState, oid: u8) -> Result<usize, TrsyncPolicyError> {
    state
        .endstop_by_oid
        .get(&oid)
        .copied()
        .ok_or(TrsyncPolicyError::UnknownEndstopOid)
}

fn clear_stop_signals_for_oid(state: &mut TriggerDomainState, oid: u8) {
    for slot in state.stop_signals.iter_mut() {
        if let Some(signal) = slot
            && signal.trsync_oid == oid
        {
            *slot = None;
        }
    }
}

fn unblock_steppers_for_oid(state: &mut TriggerDomainState, oid: u8) {
    for stepper in state.stepper_stop.iter_mut() {
        if stepper.blocked_by_trsync == Some(oid) {
            stepper.blocked_by_trigger = false;
            stepper.blocked_by_trsync = None;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use foci_klipper::scheduler::{SchedulerEvent, SchedulerEventKind};
    use std::sync::{Mutex, MutexGuard};
    use std::vec::Vec;

    static HOMING_TEST_LOCK: Mutex<()> = Mutex::new(());

    #[derive(Default)]
    struct MockBoard {
        physical_steppers: &'static [usize],
        valid_mapping: bool,
        pin_ids: [u8; NUM_ENDSTOPS],
        schedule_results: usize,
        disarm_calls: usize,
        stopped: Vec<(usize, StepperEventReason, bool)>,
        trsync_stops: Vec<(usize, u32, u8, u8)>,
        timer_active_sets: Vec<(usize, bool)>,
        reprogram_calls: usize,
    }

    impl MockBoard {
        fn new() -> Self {
            Self {
                physical_steppers: &[0],
                valid_mapping: true,
                pin_ids: [0xFF; NUM_ENDSTOPS],
                schedule_results: 0,
                disarm_calls: 0,
                stopped: Vec::new(),
                trsync_stops: Vec::new(),
                timer_active_sets: Vec::new(),
                reprogram_calls: 0,
            }
        }
    }

    impl TriggerDispatchBoard for MockBoard {
        fn stop_physical_stepper(
            &mut self,
            state: &TriggerDomainState,
            stepper_idx: usize,
            reason: StepperEventReason,
        ) {
            self.stopped.push((
                stepper_idx,
                reason,
                state.stepper_stop[stepper_idx].blocked_by_trigger,
            ));
        }

        fn record_trsync_stop(
            &mut self,
            stepper_idx: usize,
            clock: u32,
            trsync_oid: u8,
            reason: u8,
        ) {
            self.trsync_stops
                .push((stepper_idx, clock, trsync_oid, reason));
        }

        fn set_timer_active(
            &mut self,
            state: &mut TriggerDomainState,
            stepper_idx: usize,
            active: bool,
        ) {
            state.stepper_stop[stepper_idx].timer_active = active;
            self.timer_active_sets.push((stepper_idx, active));
        }
    }

    impl TrsyncBoard for MockBoard {
        fn physical_steppers(&self) -> &'static [usize] {
            self.physical_steppers
        }

        fn map_physical_stepper(&self, klipper_stepper_idx: usize) -> Option<usize> {
            self.valid_mapping.then_some(klipper_stepper_idx)
        }

        fn endstop_pin_id(&self, endstop_idx: usize) -> u8 {
            self.pin_ids[endstop_idx]
        }

        fn record_schedule_result(&mut self, _result: ScheduleResult) {
            self.schedule_results += 1;
        }

        fn reprogram_trigger_timer(&mut self, _state: &mut TriggerDomainState) {
            self.reprogram_calls += 1;
        }

        fn disarm_trigger_timer(&mut self, state: &mut TriggerDomainState) {
            state.scheduler.set_cc2_active(false);
            self.disarm_calls += 1;
        }
    }

    fn homing_test_guard() -> MutexGuard<'static, ()> {
        HOMING_TEST_LOCK
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner())
    }

    fn reset_homing_triggered() {
        HOMING_TRIGGERED.store(0xFF, Ordering::Relaxed);
    }

    // Keep global HOMING_TRIGGERED resets out of fresh_state(); tests run in
    // parallel and only global-touching tests may reset or assert it while
    // holding HOMING_TEST_LOCK.
    fn fresh_state() -> TriggerDomainState {
        TriggerDomainState::new()
    }

    #[test]
    fn duplicate_trsync_oid_returns_exact_error() {
        let mut state = fresh_state();
        assert_eq!(register_trsync(&mut state, 7), Ok(()));
        assert_eq!(
            register_trsync(&mut state, 7),
            Err(TrsyncPolicyError::DuplicateTrsyncOid)
        );
    }

    #[test]
    fn full_trsync_map_returns_capacity() {
        let mut state = fresh_state();
        for oid in 0..NUM_TRSYNC as u8 {
            assert_eq!(register_trsync(&mut state, oid), Ok(()));
        }
        assert_eq!(
            register_trsync(&mut state, 99),
            Err(TrsyncPolicyError::TrsyncCapacity)
        );
    }

    #[test]
    fn unknown_trsync_oids_fail_for_arm_timeout_and_trigger() {
        let mut state = fresh_state();
        let mut board = MockBoard::new();
        assert_eq!(
            trsync_arm(&mut state, &mut board, 4, 100, 10, 5),
            Err(TrsyncPolicyError::UnknownTrsyncOid)
        );
        assert_eq!(
            trsync_set_timeout(&mut state, &mut board, 4, 200),
            Err(TrsyncPolicyError::UnknownTrsyncOid)
        );
        assert_eq!(
            trsync_trigger(&mut state, &mut board, 4, 6),
            Err(TrsyncPolicyError::UnknownTrsyncOid)
        );
    }

    #[test]
    fn duplicate_and_full_endstop_maps_return_exact_errors() {
        let mut state = fresh_state();
        assert_eq!(register_endstop(&mut state, 9), Ok(0));
        assert_eq!(
            register_endstop(&mut state, 9),
            Err(TrsyncPolicyError::DuplicateEndstopOid)
        );

        let mut state = fresh_state();
        for oid in 0..NUM_ENDSTOPS as u8 {
            assert_eq!(register_endstop(&mut state, oid), Ok(oid as usize));
        }
        assert_eq!(
            register_endstop(&mut state, 99),
            Err(TrsyncPolicyError::EndstopCapacity)
        );
    }

    #[test]
    fn unknown_endstop_oids_fail_for_homing_and_query() {
        let mut state = fresh_state();
        let mut board = MockBoard::new();
        assert_eq!(
            endstop_start_homing(&mut state, &mut board, 12, 100, 10, 1, 30, true, 1, 8),
            Err(TrsyncPolicyError::UnknownEndstopOid)
        );
        assert_eq!(
            endstop_query(&state, &board, 12),
            Err(TrsyncPolicyError::UnknownEndstopOid)
        );
    }

    #[test]
    fn zero_sample_homing_does_not_require_trsync() {
        let mut state = fresh_state();
        let mut board = MockBoard::new();
        assert_eq!(register_endstop(&mut state, 3), Ok(0));
        assert_eq!(
            endstop_start_homing(&mut state, &mut board, 3, 100, 10, 0, 30, true, 99, 8),
            Ok(())
        );
    }

    #[test]
    fn active_homing_requires_registered_trsync() {
        let mut state = fresh_state();
        let mut board = MockBoard::new();
        assert_eq!(register_endstop(&mut state, 3), Ok(0));
        assert_eq!(
            endstop_start_homing(&mut state, &mut board, 3, 100, 10, 1, 30, true, 99, 8),
            Err(TrsyncPolicyError::MissingTrsyncForHoming)
        );
    }

    #[test]
    fn trsync_arm_clears_old_stop_links_and_unblocks_matching_steppers() {
        let mut state = fresh_state();
        let mut board = MockBoard::new();
        register_trsync(&mut state, 2).unwrap();
        state.stop_signals[0] = Some(StopSignal {
            trsync_oid: 2,
            stepper_idx: 0,
        });
        state.stepper_stop[0].blocked_by_trigger = true;
        state.stepper_stop[0].blocked_by_trsync = Some(2);

        assert_eq!(trsync_arm(&mut state, &mut board, 2, 100, 10, 7), Ok(()));

        assert_eq!(state.stop_signals[0], None);
        assert!(!state.stepper_stop[0].blocked_by_trigger);
        assert_eq!(state.stepper_stop[0].blocked_by_trsync, None);
    }

    #[test]
    fn trsync_set_timeout_noops_after_trigger() {
        let mut state = fresh_state();
        let mut board = MockBoard::new();
        register_trsync(&mut state, 2).unwrap();
        trsync_arm(&mut state, &mut board, 2, 100, 0, 7).unwrap();
        assert_eq!(
            dispatch_trigger(&mut state, &mut board, 2, 42, 1234),
            TriggerResult::First {
                snapshot: foci_klipper::trigger::ReportSnapshot {
                    oid: 2,
                    can_trigger: 0,
                    trigger_reason: 42,
                    clock: 1234,
                }
            }
        );

        assert_eq!(trsync_set_timeout(&mut state, &mut board, 2, 999), Ok(()));
        assert_eq!(state.scheduler.earliest_deadline(), None);
    }

    #[test]
    fn trsync_trigger_sets_homing_only_when_stop_signal_fires() {
        let _guard = homing_test_guard();
        reset_homing_triggered();
        let mut state = fresh_state();
        let mut board = MockBoard::new();
        register_trsync(&mut state, 2).unwrap();
        trsync_arm(&mut state, &mut board, 2, 100, 0, 7).unwrap();
        assert_eq!(trsync_trigger(&mut state, &mut board, 2, 9), Ok(9));
        assert_eq!(HOMING_TRIGGERED.load(Ordering::Relaxed), 0xFF);

        register_trsync(&mut state, 3).unwrap();
        trsync_arm(&mut state, &mut board, 3, 100, 0, 7).unwrap();
        link_stepper_stop(&mut state, &board, 0, 3).unwrap();
        assert_eq!(trsync_trigger(&mut state, &mut board, 3, 10), Ok(10));
        assert_eq!(HOMING_TRIGGERED.load(Ordering::Relaxed), 3);
    }

    #[test]
    fn dispatch_stops_stepper_before_marking_it_blocked() {
        let _guard = homing_test_guard();
        reset_homing_triggered();
        let mut state = fresh_state();
        let mut board = MockBoard::new();
        register_trsync(&mut state, 2).unwrap();
        trsync_arm(&mut state, &mut board, 2, 100, 0, 7).unwrap();
        link_stepper_stop(&mut state, &board, 0, 2).unwrap();

        let result = dispatch_trigger(&mut state, &mut board, 2, 9, 123);

        assert!(matches!(result, TriggerResult::First { .. }));
        assert_eq!(
            board.stopped.as_slice(),
            &[(0, StepperEventReason::TrsyncStop, false)]
        );
        assert!(state.stepper_stop[0].blocked_by_trigger);
        assert_eq!(state.stepper_stop[0].blocked_by_trsync, Some(2));
    }

    #[test]
    fn link_stepper_stop_reports_mapping_duplicate_and_capacity_errors() {
        let mut state = fresh_state();
        let mut board = MockBoard::new();
        register_trsync(&mut state, 1).unwrap();

        board.valid_mapping = false;
        assert_eq!(
            link_stepper_stop(&mut state, &board, 0, 1),
            Err(TrsyncPolicyError::InvalidStepper)
        );

        board.valid_mapping = true;
        assert_eq!(link_stepper_stop(&mut state, &board, 0, 1), Ok(()));
        assert_eq!(
            link_stepper_stop(&mut state, &board, 0, 1),
            Err(TrsyncPolicyError::DuplicateStopSignal)
        );

        let mut state = fresh_state();
        register_trsync(&mut state, 1).unwrap();
        for idx in 0..STOP_SIGNAL_CAPACITY {
            assert_eq!(link_stepper_stop(&mut state, &board, idx, 1), Ok(()));
        }
        assert_eq!(
            link_stepper_stop(&mut state, &board, STOP_SIGNAL_CAPACITY, 1),
            Err(TrsyncPolicyError::StopSignalCapacity)
        );
    }

    #[test]
    fn endstop_query_returns_live_tuple() {
        let mut state = fresh_state();
        let mut board = MockBoard::new();
        board.pin_ids[0] = 0x31;
        register_trsync(&mut state, 2).unwrap();
        assert_eq!(register_endstop(&mut state, 4), Ok(0));
        endstop_start_homing(&mut state, &mut board, 4, 100, 10, 2, 30, true, 2, 8).unwrap();

        assert_eq!(endstop_query(&state, &board, 4), Ok((true, 100, 0x31)));
    }

    #[test]
    fn reset_all_clears_state_and_disarms_timer() {
        let mut state = fresh_state();
        let mut board = MockBoard::new();
        register_trsync(&mut state, 2).unwrap();
        register_endstop(&mut state, 4).unwrap();
        state.stop_signals[0] = Some(StopSignal {
            trsync_oid: 2,
            stepper_idx: 0,
        });
        state.stepper_stop[0].need_reset = true;
        state.stepper_stop[0].blocked_by_trigger = true;
        state.scheduler.try_schedule(SchedulerEvent {
            wake_clock: 10,
            kind: SchedulerEventKind::Stepper(0),
        });

        reset_all(&mut state, &mut board);

        assert_eq!(state.trsync_by_oid.len(), 0);
        assert_eq!(state.endstop_by_oid.len(), 0);
        assert!(state.stop_signals.iter().all(Option::is_none));
        assert_eq!(
            state.stepper_stop[0],
            foci_klipper::trigger::StepperStopRuntime::new()
        );
        assert_eq!(board.disarm_calls, 1);
        assert_eq!(
            board.stopped.as_slice(),
            &[(0, StepperEventReason::TmcDisableSignal, true)]
        );
    }
}
