//! Shared trsync/endstop context boundary and policy helpers.
//!
//! `TrsyncHost` is the handler-facing trait. The remaining items in this
//! module are the shared trigger-domain policy surface used by board crates.

use core::sync::atomic::Ordering;

use foci_klipper::scheduler::{ScheduleResult, SchedulerEventKind, TriggerScheduler};
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
    _state: &mut TriggerDomainState,
    _board: &mut B,
    _oid: u8,
    _report_clock: u32,
    _report_ticks: u32,
    _expire_reason: u8,
) -> Result<(), TrsyncPolicyError> {
    unimplemented!("Task 3 implements trsync_arm")
}

/// Update the expiry deadline for a trsync.
pub fn trsync_set_timeout<B: TrsyncBoard + ?Sized>(
    _state: &mut TriggerDomainState,
    _board: &mut B,
    _oid: u8,
    _clock: u32,
) -> Result<(), TrsyncPolicyError> {
    unimplemented!("Task 3 implements trsync_set_timeout")
}

/// Manually fire a trsync and return the effective trigger reason.
pub fn trsync_trigger<B: TrsyncBoard + ?Sized>(
    _state: &mut TriggerDomainState,
    _board: &mut B,
    _oid: u8,
    _reason: u8,
) -> Result<u8, TrsyncPolicyError> {
    unimplemented!("Task 3 implements trsync_trigger")
}

/// Configure an endstop runtime for homing.
#[allow(clippy::too_many_arguments)]
pub fn endstop_start_homing<B: TrsyncBoard + ?Sized>(
    _state: &mut TriggerDomainState,
    _board: &mut B,
    _oid: u8,
    _clock: u32,
    _sample_ticks: u32,
    _sample_count: u8,
    _rest_ticks: u32,
    _pin_value_expected: bool,
    _trsync_oid: u8,
    _trigger_reason: u8,
) -> Result<(), TrsyncPolicyError> {
    unimplemented!("Task 3 implements endstop_start_homing")
}

/// Read the endstop runtime state tuple `(is_homing, next_clock, pin_id)`.
pub fn endstop_query<B: TrsyncBoard + ?Sized>(
    _state: &TriggerDomainState,
    _board: &B,
    _oid: u8,
) -> Result<(bool, u32, u8), TrsyncPolicyError> {
    unimplemented!("Task 3 implements endstop_query")
}

/// Link a physical stepper to a trsync stop signal.
pub fn link_stepper_stop<B: TrsyncBoard + ?Sized>(
    _state: &mut TriggerDomainState,
    _board: &B,
    _stepper_idx: usize,
    _trsync_oid: u8,
) -> Result<(), TrsyncPolicyError> {
    unimplemented!("Task 3 implements link_stepper_stop")
}
