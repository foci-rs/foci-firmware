//! Persistent state for the shared TMC control runtime.

use core::sync::atomic::Ordering::Relaxed;

use crate::autotune::CommissionError;
use crate::commission_dispatch::{DispatchState, TerminalRestoreMachine};
use crate::commissioning::CommissioningEngine;
use crate::commissioning::schedule::CommissionEvent;
use crate::commissioning::schedule::{CommissionPoll, CommissionTerminal, adapt_step_result};
use crate::commissioning::types::{CommissioningBackend, EngineKind};
use crate::pid::InnerLoopGains;
use crate::state::stepper::StepperChannelState;
use crate::state::tmc::TmcChannelState;
use crate::tmc::{TmcRequest, TmcRequestKind};
use crate::tmc_control::actions::{
    BoardAction, BoardActions, DelegatedDiagnostic, ScheduledCommissionTickAction, ShutdownReason,
    StepperAction, push_action,
};
use crate::tmc_control::config::{TmcControlConfig, TmcControlHomingRefs, TmcControlStateRefs};
use crate::tmc_control::signals::TmcControlSignals;
use foci_tmc4671::pid::{PidGainPair, PidIGain};

/// Consecutive failed state-1 homing attempts tolerated before failing closed.
pub(crate) const HOMING_CLAMP_MAX_RETRIES: u8 = 3;

/// Consecutive run-current restore attempts tolerated before failing closed.
pub(crate) const HOMING_RESTORE_MAX_RETRIES: u8 = 2;

/// Trace-only cached state. Core safety state stays in `TmcControlRuntime`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct TmcControlTraceState {
    /// Trace control tick counter.
    pub control_tick: u32,
    /// Last command rate in TMC position counts per PWM cycle.
    pub cached_velocity_reference: i32,
    /// Last velocity feedforward offset emitted to trace sampling.
    pub cached_velocity_offset: i32,
    /// Last transient velocity feedforward offset emitted to trace sampling.
    pub cached_velocity_transient_offset: i32,
    /// Velocity feedforward offset was clamped on the last tick.
    pub cached_velocity_clamped: bool,
    /// Last acceleration/decoupling torque offset emitted to trace sampling.
    pub cached_accel_torque_offset: i32,
    /// Last acceleration gain emitted to trace sampling.
    pub cached_accel_gain_permille: u16,
    /// Last acceleration/decoupling trace flags.
    pub cached_accel_flags: u8,
    /// Last phase-advance counts emitted to trace sampling.
    pub phase_advance_last_counts: i16,
    /// Last phase-advance trace flags.
    pub phase_advance_last_flags: u8,
    /// Force one phase-advance trace sample on the next trace tick.
    pub phase_advance_emit_once: bool,
    /// Last phase-advance register write failed.
    pub phase_advance_write_failed: bool,
}

impl TmcControlTraceState {
    /// Create an empty trace state.
    pub const fn new() -> Self {
        Self {
            control_tick: 0,
            cached_velocity_reference: 0,
            cached_velocity_offset: 0,
            cached_velocity_transient_offset: 0,
            cached_velocity_clamped: false,
            cached_accel_torque_offset: 0,
            cached_accel_gain_permille: 0,
            cached_accel_flags: 0,
            phase_advance_last_counts: 0,
            phase_advance_last_flags: 0,
            phase_advance_emit_once: false,
            phase_advance_write_failed: false,
        }
    }
}

impl Default for TmcControlTraceState {
    fn default() -> Self {
        Self::new()
    }
}

/// Trace timing decisions for one USB/FTRC slow-control emission pass.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct TmcControlTraceTick {
    /// Tick value to pass to `sample_tick`.
    pub sample_tick: u32,
    /// Incremented trace control tick for status frames and metadata cadence.
    pub control_tick: u32,
    /// Whether a phase-advance frame should be emitted this tick.
    pub phase_advance_due: bool,
    /// Phase-advance counts cached by the runtime tick.
    pub phase_advance_counts: i16,
    /// Phase-advance flags cached by the runtime tick.
    pub phase_advance_flags: u8,
    /// Whether detailed status frames are due.
    pub status_detail_due: bool,
    /// Whether trace metadata is due.
    pub metadata_due: bool,
}

/// Result of applying one channel's homing maintenance for a frozen state.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum HomingMaintenanceResult {
    /// This channel completed or did not need work for the frozen state.
    ReadyToAdvance,
    /// This channel could not complete work; keep the global homing state.
    RetryCurrentState,
    /// Homing cannot proceed safely; the board must disable the motor.
    FailClosed,
    /// The clamp window has not opened yet; keep the global homing state and
    /// retry on the next maintenance group without consuming the retry budget.
    Deferred,
}

/// Board clock context for the homing preparation window.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct HomingClockWindow {
    /// Current Klipper clock.
    pub now: u32,
    /// Clock at which the homing move's steps begin (`HOMING_START_CLOCK`).
    pub start: u32,
}

/// Lead, in Klipper clock ticks, by which homing preparation precedes the
/// homing move's start clock: 4 ms.
///
/// Wide enough for one maintenance period plus the clamp's SPI writes on
/// either board, and for the report-clock offset Klipper adds on a later MCU
/// of a multi-MCU home, while short enough that only the decelerating tail
/// of the previous move runs under the clamp.
pub const fn homing_clamp_lead_ticks(clock_freq: u32) -> u32 {
    clock_freq / 250
}

/// Whether the homing preparation window has opened for `window`.
///
/// Uses wrapping clock arithmetic; Klipper never schedules a homing start
/// more than a fraction of the clock period ahead of the command that
/// announces it.
pub fn homing_clamp_due(window: HomingClockWindow, clock_freq: u32) -> bool {
    let opens = window
        .start
        .wrapping_sub(homing_clamp_lead_ticks(clock_freq));
    (window.now.wrapping_sub(opens) as i32) >= 0
}

/// Long-lived board-agnostic TMC control state.
pub struct TmcControlRuntime {
    channel: u8,
    motor_enabled: bool,
    calibrated: bool,
    stored_gains: Option<InnerLoopGains>,
    commission_engine: Option<CommissioningEngine>,
    restore_machine: Option<TerminalRestoreMachine>,
    restore_cancel_requested: bool,
    shutdown_handled: bool,
    planner_velocity_scale: Option<crate::motion_scale::PlannerVelocityScale>,
    planner_rpm_scale: Option<crate::motion_scale::PlannerRpmScale>,
    feedforward_was_active: bool,
    torque_flux_offset_was_active: bool,
    applied_velocity_offset: Option<u32>,
    applied_torque_flux_offset: Option<u32>,
    position_lead_applied_units: i32,
    phase_advance_was_active: bool,
    phase_advance_base_register: u32,
    phase_advance_last_register: u32,
    pid_error_clear_countdown_ms: u8,
    stall_detector: Option<crate::tmc_control::stall_detector::StallDetector>,
    homing_state_1_retries: u8,
    homing_state_3_retries: u8,
    #[cfg(feature = "trace")]
    trace: TmcControlTraceState,
}

impl TmcControlRuntime {
    fn retain_integral_authority_from_event(
        channel: u8,
        event: &Option<CommissionEvent>,
        retention: &mut crate::commission_dispatch::CommissionRetention<'_>,
    ) {
        if let Some(CommissionEvent::VelocityIntegralPlan { plan, .. }) = event {
            crate::commission_dispatch::integral::install_authority_from_plan(
                channel,
                retention.integral,
                *plan,
            );
        }
    }

    /// Create a runtime for one board-local TMC channel.
    pub const fn new_for_channel(channel: u8) -> Self {
        Self {
            channel,
            motor_enabled: false,
            calibrated: false,
            stored_gains: None,
            commission_engine: None,
            restore_machine: None,
            restore_cancel_requested: false,
            shutdown_handled: false,
            planner_velocity_scale: None,
            planner_rpm_scale: None,
            feedforward_was_active: false,
            torque_flux_offset_was_active: false,
            applied_velocity_offset: None,
            applied_torque_flux_offset: None,
            position_lead_applied_units: 0,
            phase_advance_was_active: false,
            phase_advance_base_register: 0,
            phase_advance_last_register: 0,
            pid_error_clear_countdown_ms: 0,
            stall_detector: None,
            homing_state_1_retries: 0,
            homing_state_3_retries: 0,
            #[cfg(feature = "trace")]
            trace: TmcControlTraceState::new(),
        }
    }

    /// Return this runtime's board-local TMC channel.
    pub const fn channel(&self) -> u8 {
        self.channel
    }

    pub(super) const fn maintenance_obligations_clear(&self) -> bool {
        !self.motor_enabled
            && self.commission_engine.is_none()
            && self.restore_machine.is_none()
            && !self.feedforward_was_active
            && !self.torque_flux_offset_was_active
            && self.position_lead_applied_units == 0
            && !self.phase_advance_was_active
            && self.stall_detector.is_none()
            && self.homing_state_1_retries == 0
            && self.homing_state_3_retries == 0
    }

    /// Return whether the motor is currently electrically enabled.
    pub const fn is_motor_enabled(&self) -> bool {
        self.motor_enabled
    }

    /// Return whether calibration has completed.
    pub const fn is_calibrated(&self) -> bool {
        self.calibrated
    }

    /// Return stored inner-loop gains.
    pub const fn stored_gains(&self) -> Option<InnerLoopGains> {
        self.stored_gains
    }

    /// Store inner-loop gains for future calibration/commissioning.
    pub fn set_stored_gains(&mut self, gains: InnerLoopGains) {
        self.stored_gains = Some(gains);
    }

    /// Mutably borrow the stored inner-loop gains slot.
    pub fn stored_gains_mut(&mut self) -> &mut Option<InnerLoopGains> {
        &mut self.stored_gains
    }

    /// Return the active commissioning kind when an engine exists.
    pub fn pending_commission_kind(&self) -> Option<crate::commissioning::types::EngineKind> {
        self.commission_engine
            .as_ref()
            .map(CommissioningEngine::kind)
    }

    /// Return trace-only cached state when trace support is enabled.
    #[cfg(feature = "trace")]
    pub const fn trace_state(&self) -> &TmcControlTraceState {
        &self.trace
    }

    /// Mutably borrow trace-only cached state when trace support is enabled.
    #[cfg(feature = "trace")]
    pub fn trace_state_mut(&mut self) -> &mut TmcControlTraceState {
        &mut self.trace
    }

    /// Build a borrowed dispatch view over runtime-owned state.
    ///
    /// The `tmc` and `stepper` args must be `'static` because `DispatchState`
    /// borrows them as `&'static`; the three flag fields are borrowed mutably
    /// for the lifetime of the returned view.
    pub fn dispatch_state<'a>(
        &'a mut self,
        tmc: &'static TmcChannelState,
        stepper: &'static StepperChannelState,
    ) -> DispatchState<'a> {
        DispatchState::new(
            self.channel,
            tmc,
            stepper,
            &mut self.calibrated,
            &mut self.motor_enabled,
            &mut self.stored_gains,
        )
    }

    /// Reset runtime-local state after a config reset or shutdown clear.
    ///
    /// Channel atomics (`TmcChannelState`, `StepperChannelState`) are separate;
    /// call `reset_channel_state` for those.
    pub fn reset_runtime_state(&mut self) {
        self.motor_enabled = false;
        self.calibrated = false;
        self.stored_gains = None;
        self.clear_or_cancel_commission();
        self.shutdown_handled = false;
        self.planner_velocity_scale = None;
        self.planner_rpm_scale = None;
        self.feedforward_was_active = false;
        self.torque_flux_offset_was_active = false;
        self.applied_velocity_offset = None;
        self.applied_torque_flux_offset = None;
        self.position_lead_applied_units = 0;
        self.phase_advance_was_active = false;
        self.phase_advance_base_register = 0;
        self.phase_advance_last_register = 0;
        self.pid_error_clear_countdown_ms = 0;
        self.clear_homing_runtime();
        #[cfg(feature = "trace")]
        {
            self.trace = TmcControlTraceState::new();
        }
    }

    /// Drop runtime-owned homing state without touching channel atomics.
    ///
    /// The clamped torque limit is restored by construction on every path that
    /// calls this: shutdown de-energizes and every re-energize path rewrites
    /// the limit from stored channel state, and a config reset makes the host
    /// re-send the run current.
    pub fn clear_homing_runtime(&mut self) {
        self.stall_detector = None;
        self.homing_state_1_retries = 0;
        self.homing_state_3_retries = 0;
    }

    /// Reset channel atomics to firmware defaults after a config reset.
    pub fn reset_channel_state(tmc: &TmcChannelState, stepper: &StepperChannelState) {
        tmc.reset();
        stepper.reset();
    }

    /// Consume a config-reset queue flush request and return board actions.
    ///
    /// Returns an empty action list when no queue flush was requested. When a
    /// flush is pending, this consumes the signal, resets runtime-local and
    /// channel-local state, and emits `StopNow` so the board shell can stop
    /// physical stepping.
    ///
    /// The fault latch is retired only when neither containment leg reports a
    /// fault: `tmc.status_pin_asserted`, which the board republishes from its
    /// polled STATUS level, and a pending emergency disable. A host config
    /// reset that coincides with a live fault would otherwise hand the board a
    /// clean slate before its emergency path has read `STATUS_FLAGS`, losing
    /// the diagnosis of which condition tripped. A caller that never publishes
    /// `status_pin_asserted` leaves that leg reading clear.
    pub fn poll_queue_flush(
        &mut self,
        signals: &TmcControlSignals,
        tmc: &TmcChannelState,
        stepper: &StepperChannelState,
    ) -> BoardActions {
        let mut actions = BoardActions::new();
        if !signals.queue_flush.swap(false, Relaxed) {
            return actions;
        }

        // Sample before the channel reset, which clears the cached pin level.
        let contained = tmc.status_pin_asserted.load(Relaxed) || signals.disable_signal.signaled();

        self.reset_runtime_state();
        Self::reset_channel_state(tmc, stepper);
        if !contained {
            signals.fault_active.store(false, Relaxed);
        }
        push_action(&mut actions, BoardAction::Stepper(StepperAction::StopNow));
        actions
    }

    /// Poll host request to clear PID error sums after hold settles.
    pub fn poll_pid_error_clear_request(&mut self, tmc: &TmcChannelState) {
        if tmc.pid_error_sums_clear.swap(false, Relaxed) {
            self.pid_error_clear_countdown_ms = crate::tmc_control::HOLD_SETTLE_DELAY_MS;
        }
    }

    /// Apply host-requested PID error-sum settle and clear maintenance.
    ///
    /// Consumes the request latch, waits for the hold timer to become inactive,
    /// then snaps the hold target and clears PID error accumulators when the
    /// runtime countdown expires. SPI failures are best-effort and logged.
    pub async fn apply_pid_error_settle_tick<B>(
        &mut self,
        tmc_spi: &mut foci_tmc4671::spi::Tmc4671Spi<B>,
        refs: TmcControlStateRefs<'_>,
    ) where
        B: embedded_hal_async::spi::SpiDevice<u8>,
    {
        self.poll_pid_error_clear_request(refs.tmc);
        let timer_active = refs.stepper.timer_active_mirror.load(Relaxed);
        if self.motor_enabled
            && self.commission_engine.is_none()
            && !timer_active
            && self.pid_error_clear_countdown_ms > 0
        {
            self.pid_error_clear_countdown_ms -= 1;
            if self.pid_error_clear_countdown_ms == 0 {
                if crate::tmc_control::settle::settle_hold_and_clear_pid_errors(tmc_spi, refs.tmc)
                    .await
                    .is_err()
                {
                    defmt::warn!("TMC4671 PID settle/clear failed");
                } else {
                    defmt::info!("TMC4671 PID settled and cleared after motion");
                }
            }
        }
    }

    /// Prepare immediate shutdown/fault quiesce for one channel.
    ///
    /// Clears runtime-owned energized and transient maintenance state without
    /// resetting configuration such as stored gains or velocity scale. The
    /// board shell must apply the returned stepper action and lower enable
    /// pins before any awaited SPI safe-state cleanup.
    pub fn prepare_shutdown_quiesce(&mut self) -> BoardActions {
        let mut actions = BoardActions::new();
        self.clear_or_cancel_commission();
        self.motor_enabled = false;
        self.calibrated = false;
        self.feedforward_was_active = false;
        self.torque_flux_offset_was_active = false;
        self.applied_velocity_offset = None;
        self.applied_torque_flux_offset = None;
        self.pid_error_clear_countdown_ms = 0;
        self.clear_homing_runtime();
        push_action(&mut actions, BoardAction::Stepper(StepperAction::StopNow));
        actions
    }

    /// Poll shutdown state and emit one-shot safe-state actions.
    ///
    /// Returns a `StopNow` stepper action on the first call while shutdown is
    /// active, clearing `motor_enabled`, `calibrated`, and the commission
    /// engine. Subsequent calls while shutdown remains active return an empty
    /// action list. Clears `shutdown_handled` when shutdown is no longer active
    /// so the next shutdown cycle fires again.
    pub fn poll_shutdown(&mut self, signals: &TmcControlSignals) -> BoardActions {
        let mut actions = BoardActions::new();
        if signals.shutdown_active.load(Relaxed) {
            if !self.shutdown_handled {
                self.clear_or_cancel_commission();
                self.applied_velocity_offset = None;
                self.applied_torque_flux_offset = None;
                push_action(&mut actions, BoardAction::Stepper(StepperAction::StopNow));
                self.motor_enabled = false;
                self.calibrated = false;
                self.shutdown_handled = true;
            }
            return actions;
        }

        self.shutdown_handled = false;
        actions
    }

    /// Consume an emergency-disable signal and emit shutdown actions.
    ///
    /// Returns nothing when the signal has not been raised. When the signal is
    /// present, clears `fault_active`, disarms the motor, and returns a
    /// `StopNow` action followed by a `Shutdown` action whose reason is
    /// `AdcCurrentClipped` when `status_flags` is a successful read with bit 26
    /// set, or `TmcFault` otherwise -- including when the register read itself
    /// failed. A failed read is never coerced into a clean status value first:
    /// it carries no evidence of clipping, so it falls through to the generic
    /// fault reason exactly like a genuine non-clipped read.
    pub fn poll_emergency_disable(
        &mut self,
        signals: &TmcControlSignals,
        status_flags: Result<u32, ()>,
    ) -> BoardActions {
        let mut actions = BoardActions::new();
        if signals.disable_signal.try_take().is_none() {
            return actions;
        }

        signals.fault_active.store(false, Relaxed);
        self.motor_enabled = false;
        self.calibrated = false;
        push_action(&mut actions, BoardAction::Stepper(StepperAction::StopNow));
        let reason = if is_adc_current_clipped(status_flags) {
            ShutdownReason::AdcCurrentClipped
        } else {
            ShutdownReason::TmcFault
        };
        push_action(&mut actions, BoardAction::Shutdown(reason));
        actions
    }

    /// Classify a [`TmcRequest`] before SPI-heavy handling.
    ///
    /// Returns shutdown actions for wrong-channel requests and delegates active
    /// diagnostic requests to the board shell via [`BoardAction::DelegatedDiagnostic`].
    /// Non-diagnostic, correct-channel requests return an empty action list and
    /// are processed by the caller's SPI path.
    pub fn classify_request(&mut self, req: TmcRequest) -> BoardActions {
        let mut actions = BoardActions::new();
        if req.channel != self.channel {
            push_action(
                &mut actions,
                BoardAction::Shutdown(ShutdownReason::InvalidTmcChannel),
            );
            return actions;
        }

        match req.kind {
            TmcRequestKind::CurrentStepTest { .. }
            | TmcRequestKind::CurrentVectorStepTest { .. }
            | TmcRequestKind::CurrentTorqueSampleTest { .. }
            | TmcRequestKind::PositionTorqueOffsetSampleTest { .. }
            | TmcRequestKind::VoltageStepTest { .. }
            | TmcRequestKind::ResistanceTest { .. } => {
                push_action(
                    &mut actions,
                    BoardAction::DelegatedDiagnostic(DelegatedDiagnostic { request: req }),
                );
            }
            #[cfg(feature = "trace-velocity-limit-latch-test")]
            TmcRequestKind::VelocityLimitLatchTest { .. } => {
                push_action(
                    &mut actions,
                    BoardAction::DelegatedDiagnostic(DelegatedDiagnostic { request: req }),
                );
            }
            _ => {}
        }
        actions
    }

    /// Returns true when a commissioning engine is currently active.
    pub fn is_commission_active(&self) -> bool {
        self.commission_engine.is_some()
    }

    /// Whether the planner-referenced control paths (feedforward, phase
    /// advance, fast control period) may run: either no commissioning
    /// engine is installed, or the installed one is driving a step-queue
    /// stimulus and has opted those paths back on.
    pub(crate) fn planner_paths_allowed(&self, refs: TmcControlStateRefs<'_>) -> bool {
        self.commission_engine.is_none() || refs.tmc.commission_stimulus_active.load(Relaxed)
    }

    /// True while any feedforward or phase-advance surface is still applied
    /// on the chip and needs a tick to restore it.
    pub(crate) fn feedforward_restoration_pending(&self) -> bool {
        self.feedforward_was_active
            || self.torque_flux_offset_was_active
            || self.phase_advance_was_active
    }

    fn publish_feedforward_surfaces_idle(&self, refs: TmcControlStateRefs<'_>) {
        refs.tmc
            .feedforward_surfaces_idle
            .store(!self.feedforward_restoration_pending(), Relaxed);
    }

    /// Force the pending terminal restore onto its safe-disarm path with
    /// the safe-state-incomplete outcome. Used when a stimulus's
    /// feedforward surfaces could not be restored within the driver's
    /// retry budget. Reuses the operator-cancel machinery: the cancel
    /// latch plus `drive_restore_cancel`, which runs the machine's
    /// `force_safe_terminal` (safe-state register sequence, disarm, stop
    /// post-action) and retires the engine in one step.
    pub(crate) async fn force_restore_safe_state_incomplete<B>(
        &mut self,
        backend: &mut B,
        tmc: &'static TmcChannelState,
        stepper: &'static StepperChannelState,
    ) -> Option<crate::commission_dispatch::CommissionOutcome>
    where
        B: CommissioningBackend,
    {
        self.restore_machine.as_ref()?;
        self.request_restore_cancel();
        match self
            .drive_restore_cancel(backend, tmc, stepper, CommissionError::SafeStateIncomplete)
            .await
        {
            crate::commission_dispatch::RestoreStep::Done(outcome) => Some(outcome),
            crate::commission_dispatch::RestoreStep::Pending => {
                unreachable!("force_safe_terminal completes in one step")
            }
        }
    }

    #[cfg(test)]
    pub(crate) fn restore_status_error_for_test(&self) -> Option<CommissionError> {
        self.restore_machine
            .as_ref()
            .map(|machine| machine.status_error())
    }

    /// Install a new commissioning engine, replacing any existing one.
    ///
    /// Invalidates the feedforward-offset write cache: commissioning writes
    /// `PID_VELOCITY_OFFSET`/`PID_TORQUE_FLUX_OFFSET` directly while a
    /// commission engine is installed, bypassing the control tick's cache.
    pub fn set_commission_engine(&mut self, engine: CommissioningEngine) {
        self.commission_engine = Some(engine);
        self.applied_velocity_offset = None;
        self.applied_torque_flux_offset = None;
    }

    /// Remove and return the active commissioning engine without completing a
    /// breakaway-campaign breadcrumb.
    ///
    /// Invalidates the feedforward-offset write cache; see
    /// [`Self::set_commission_engine`].
    pub fn take_commission_engine(&mut self) -> Option<CommissioningEngine> {
        self.applied_velocity_offset = None;
        self.applied_torque_flux_offset = None;
        self.commission_engine.take()
    }

    /// Complete a retained active breakaway cancellation, then remove its engine.
    ///
    /// Callers must first restore their path-specific safe state. Non-breakaway
    /// engines are removed without changing persistent breakaway evidence.
    /// Invalidates the feedforward-offset write cache; see
    /// [`Self::set_commission_engine`].
    pub fn take_commission_engine_after_handled_stop(&mut self) -> Option<CommissioningEngine> {
        self.applied_velocity_offset = None;
        self.applied_torque_flux_offset = None;
        CommissioningEngine::take_after_handled_stop(&mut self.commission_engine)
    }

    /// Clear the calibrated flag before launching a (re)calibration run.
    pub fn clear_calibrated(&mut self) {
        self.calibrated = false;
    }

    /// Advance the engine through explicit scheduling, or `None` when idle.
    /// Legacy results are adapted to absolute scheduling at this boundary.
    pub async fn step_commissioning_scheduled<B>(
        &mut self,
        backend: &mut B,
    ) -> Option<CommissionPoll>
    where
        B: CommissioningBackend,
    {
        let (oid, engine_kind) = self
            .commission_engine
            .as_ref()
            .map(|engine| (engine.oid(), engine.kind()))?;
        let result = self
            .commission_engine
            .as_mut()
            .expect("commission engine disappeared during scheduled step")
            .step_scheduled(backend)
            .await;
        let now_us = crate::commissioning::CommissionTiming::elapsed_us(backend);

        Some(
            adapt_step_result(result, now_us, Some((oid, engine_kind))).unwrap_or_else(|error| {
                CommissionPoll::Terminal {
                    result: if matches!(engine_kind, EngineKind::OuterTune) {
                        CommissionTerminal::OuterFailed(error)
                    } else {
                        CommissionTerminal::InnerFailed(error)
                    },
                }
            }),
        )
    }

    /// Convert a nonterminal explicit poll into the board-facing action.
    #[must_use]
    pub fn classify_commission_poll(
        &self,
        poll: CommissionPoll,
    ) -> Option<ScheduledCommissionTickAction> {
        self.commission_engine.as_ref()?;
        match poll {
            CommissionPoll::ReadyNow { event } => {
                Some(ScheduledCommissionTickAction::ReadyNow { event })
            }
            CommissionPoll::At {
                timestamp_us,
                event,
            } => Some(ScheduledCommissionTickAction::At {
                timestamp_us,
                event,
            }),
            CommissionPoll::Terminal { .. } => None,
        }
    }

    /// Clear the commissioning engine, unless a terminal restore is mid-flight.
    ///
    /// A pending restore owns the motor: dropping its bookkeeping here would
    /// strand a half-armed motor with no record that a restore was running.
    /// Latch a cancel request instead and let `drive_restore_cancel` retire
    /// both slots once the safe-state sequence has actually run.
    fn clear_or_cancel_commission(&mut self) {
        if self.restore_machine.is_some() {
            self.restore_cancel_requested = true;
        } else {
            self.commission_engine = None;
        }
    }

    /// Begin a poll-driven terminal restore for one commissioning terminal.
    ///
    /// The restore is advanced by [`Self::poll_commissioning_terminal`], one
    /// backend operation per call, until it reports its terminal action.
    pub fn begin_commissioning_terminal(
        &mut self,
        result: CommissionTerminal,
        config: &TmcControlConfig,
    ) {
        debug_assert!(
            self.restore_machine.is_none(),
            "a terminal restore is already in flight on this channel"
        );
        // Seed the pre-force status with the terminal's own cause, so a
        // safe-state-incomplete escape that reads it before forcing sees the
        // fault that put the channel on the restore path, not the machine's
        // Shutdown default. `drive_restore_cancel` still overwrites this with
        // `SafeStateIncomplete` immediately before the forced terminal.
        let status_error = match &result {
            CommissionTerminal::InnerFailed(error) | CommissionTerminal::OuterFailed(error) => {
                Some(*error)
            }
            _ => None,
        };
        let mut machine =
            TerminalRestoreMachine::start_staged(result, self.channel, config.tmc.pwm_maxcnt);
        if let Some(error) = status_error {
            machine.set_status_error(error);
        }
        self.restore_machine = Some(machine);
    }

    /// Returns true while a terminal restore still owns this channel.
    pub const fn is_restore_pending(&self) -> bool {
        self.restore_machine.is_some()
    }

    /// Latch a cancel request for a pending terminal restore.
    ///
    /// Synchronous, so any exit path — including the sync quiesce helpers —
    /// can mark the restore doomed. The slots stay populated until
    /// [`Self::drive_restore_cancel`] has run the safe-state sequence.
    pub fn request_restore_cancel(&mut self) {
        if self.restore_machine.is_some() {
            self.restore_cancel_requested = true;
        }
    }

    /// Returns true while a latched cancel still needs draining.
    pub const fn restore_cancel_pending(&self) -> bool {
        self.restore_cancel_requested
    }

    /// Advance a pending terminal restore by one backend operation.
    ///
    /// Returns `None` while the restore still has work left, and the outcome
    /// once it reaches its terminal.
    async fn poll_restore_outcome<B>(
        &mut self,
        backend: &mut B,
        retention: &mut crate::commission_dispatch::CommissionRetention<'_>,
        tmc: &'static TmcChannelState,
        stepper: &'static StepperChannelState,
    ) -> Option<crate::commission_dispatch::CommissionOutcome>
    where
        B: CommissioningBackend,
    {
        // The machine is advanced in place, never moved into a local across the
        // await: the slot must read `Some` for every suspension point of the
        // poll, so a torn-down poll cannot lose the machine that owns the motor.
        let step = {
            let machine = self.restore_machine.as_mut()?;
            let mut dispatch = crate::commission_dispatch::DispatchState::new(
                self.channel,
                tmc,
                stepper,
                &mut self.calibrated,
                &mut self.motor_enabled,
                &mut self.stored_gains,
            );
            machine
                .poll_step(
                    &mut self.commission_engine,
                    backend,
                    &mut dispatch,
                    Some(retention),
                )
                .await
        };
        match step {
            crate::commission_dispatch::RestoreStep::Pending => None,
            crate::commission_dispatch::RestoreStep::Done(outcome) => {
                self.restore_machine = None;
                self.applied_velocity_offset = None;
                self.applied_torque_flux_offset = None;
                Some(outcome)
            }
        }
    }

    /// Advance a pending terminal restore and map a completed one to the
    /// board-facing action.
    ///
    /// Returns `None` while the restore is still in flight; the caller must
    /// poll again rather than stepping the engine.
    pub async fn poll_commissioning_terminal<B>(
        &mut self,
        backend: &mut B,
        retention: &mut crate::commission_dispatch::CommissionRetention<'_>,
        tmc: &'static TmcChannelState,
        stepper: &'static StepperChannelState,
    ) -> Option<ScheduledCommissionTickAction>
    where
        B: CommissioningBackend,
    {
        let outcome = self
            .poll_restore_outcome(backend, retention, tmc, stepper)
            .await?;
        match outcome {
            crate::commission_dispatch::CommissionOutcome::Terminal { reply, actions } => {
                Some(ScheduledCommissionTickAction::Terminal { reply, actions })
            }
            crate::commission_dispatch::CommissionOutcome::Continue
            | crate::commission_dispatch::CommissionOutcome::Evidence { .. }
            | crate::commission_dispatch::CommissionOutcome::PhaseChanged { .. } => {
                unreachable!("terminal restore produced a nonterminal dispatch action")
            }
        }
    }

    /// Drive a latched cancel to safe state, then retire both slots.
    ///
    /// Runs the mandatory disarm/safe-state sequence with the caller's backend
    /// and only then clears `restore_machine`, consumes the engine, and drops
    /// the latch, so an interrupted teardown never leaves cleared bookkeeping
    /// behind a half-restored motor.
    pub async fn drive_restore_cancel<B>(
        &mut self,
        backend: &mut B,
        tmc: &'static TmcChannelState,
        stepper: &'static StepperChannelState,
        status_error: crate::autotune::CommissionError,
    ) -> crate::commission_dispatch::RestoreStep
    where
        B: CommissioningBackend,
    {
        if !self.restore_cancel_requested {
            return crate::commission_dispatch::RestoreStep::Done(
                crate::commission_dispatch::CommissionOutcome::Continue,
            );
        }
        if self.restore_machine.is_none() {
            self.commission_engine = None;
            self.restore_cancel_requested = false;
            return crate::commission_dispatch::RestoreStep::Done(
                crate::commission_dispatch::CommissionOutcome::Continue,
            );
        }
        // Driven in place, never moved into a local across the await: both the
        // machine slot and the latch must still read "restore in progress" at
        // every suspension point of the safe-state sequence, and are retired
        // only once it has actually run.
        let outcome = {
            let machine = self
                .restore_machine
                .as_mut()
                .expect("restore machine checked present above");
            let mut dispatch = crate::commission_dispatch::DispatchState::new(
                self.channel,
                tmc,
                stepper,
                &mut self.calibrated,
                &mut self.motor_enabled,
                &mut self.stored_gains,
            );
            machine.set_status_error(status_error);
            machine
                .force_safe_terminal(&mut self.commission_engine, backend, &mut dispatch)
                .await
        };
        self.restore_machine = None;
        self.applied_velocity_offset = None;
        self.applied_torque_flux_offset = None;
        self.restore_cancel_requested = false;
        crate::commission_dispatch::RestoreStep::Done(outcome)
    }

    /// Cancel this channel's in-flight commissioning run on operator
    /// request. Returns `None` when `tmc.cancel_requested` was not set, or
    /// when it was set but nothing on this channel needs cancelling (the
    /// flag is still consumed in that case). The caller must emit the
    /// returned reply itself, exactly as it already does for
    /// `poll_commissioning_terminal`'s `Terminal` result.
    pub async fn drive_operator_cancel<B>(
        &mut self,
        backend: &mut B,
        tmc: &'static TmcChannelState,
        stepper: &'static StepperChannelState,
        pwm_maxcnt: u16,
    ) -> Option<crate::commission_dispatch::CommissionOutcome>
    where
        B: CommissioningBackend,
    {
        use core::sync::atomic::Ordering::Relaxed;
        if !tmc.cancel_requested.swap(false, Relaxed) {
            return None;
        }
        // Halt the step-queue stimulus before either cancel branch below
        // drives its own disarm. Both branches eventually clear
        // `motor_armed` (via `disarm_motor`), which stops the step-timer
        // ISR from emitting further steps, but does not itself drain the
        // queue, cancel the scheduler's stepper entry, or close the
        // stimulus's motor-armed capture session (see
        // `TmcChannelState::stimulus_restore_motor_armed`) -- a queue left
        // undrained and a capture session left open corrupt the next
        // stimulus session's own restore. Unconditional, matching the
        // engine-level shutdown abort in `CommissioningEngine::step`:
        // `stimulus_stop`/`set_stimulus_feedforward(false)` are documented
        // idempotent, a safe no-op when no stimulus session is open.
        crate::commissioning::outer::position_tune::abort_stimulus(backend);
        if self.restore_machine.is_some() {
            self.request_restore_cancel();
            match self
                .drive_restore_cancel(
                    backend,
                    tmc,
                    stepper,
                    crate::autotune::CommissionError::Cancelled,
                )
                .await
            {
                crate::commission_dispatch::RestoreStep::Done(outcome) => Some(outcome),
                crate::commission_dispatch::RestoreStep::Pending => {
                    unreachable!("force_safe_terminal completes in one step")
                }
            }
        } else if let Some(engine) = self.commission_engine.as_ref() {
            let kind = engine.kind();
            let mut machine = crate::commission_dispatch::TerminalRestoreMachine::start_cancelled(
                kind,
                self.channel,
                pwm_maxcnt,
            );
            let mut dispatch = crate::commission_dispatch::DispatchState::new(
                self.channel,
                tmc,
                stepper,
                &mut self.calibrated,
                &mut self.motor_enabled,
                &mut self.stored_gains,
            );
            match machine
                .poll_step(&mut self.commission_engine, backend, &mut dispatch, None)
                .await
            {
                crate::commission_dispatch::RestoreStep::Done(outcome) => Some(outcome),
                crate::commission_dispatch::RestoreStep::Pending => {
                    unreachable!("ForceSafeDisarm completes in one step")
                }
            }
        } else {
            None
        }
    }

    /// Dispatch a terminal commissioning result into the board-facing action.
    ///
    /// Drives the restore machine to completion in one call. Boards poll
    /// [`Self::poll_commissioning_terminal`] instead so a shutdown or lease
    /// expiry can interrupt the restore between backend operations.
    pub async fn drive_commissioning_terminal_with_stages<B, E>(
        &mut self,
        result: CommissionTerminal,
        backend: &mut B,
        context: crate::commission_dispatch::CommissionTerminalContext<'_, E>,
        tmc: &'static TmcChannelState,
        stepper: &'static StepperChannelState,
        config: &TmcControlConfig,
    ) -> Option<ScheduledCommissionTickAction>
    where
        B: CommissioningBackend,
        E: FnOnce(crate::commission_dispatch::ReplyPayload),
    {
        self.commission_engine.as_ref()?;
        let crate::commission_dispatch::CommissionTerminalContext {
            mut retention,
            emit_evidence,
        } = context;
        self.begin_commissioning_terminal(result, config);
        let outcome = loop {
            if let Some(outcome) = self
                .poll_restore_outcome(backend, &mut retention, tmc, stepper)
                .await
            {
                break outcome;
            }
        };
        match outcome {
            crate::commission_dispatch::CommissionOutcome::Terminal { reply, actions } => {
                Some(ScheduledCommissionTickAction::Terminal { reply, actions })
            }
            crate::commission_dispatch::CommissionOutcome::Evidence { reply } => {
                emit_evidence(reply);
                Some(ScheduledCommissionTickAction::WorkflowContinuation)
            }
            crate::commission_dispatch::CommissionOutcome::PhaseChanged {
                oid,
                phase_id,
                engine_kind,
            } => Some(ScheduledCommissionTickAction::ReadyNow {
                event: Some(
                    crate::commissioning::schedule::CommissionEvent::PhaseChanged {
                        oid,
                        phase_id,
                        engine_kind,
                    },
                ),
            }),
            crate::commission_dispatch::CommissionOutcome::Continue => {
                unreachable!("terminal poll produced a nonterminal dispatch action")
            }
        }
    }

    /// Classify one explicit poll, starting a terminal restore when the poll is
    /// terminal.
    ///
    /// Returns `None` when a restore was started (or when no engine is
    /// installed); the caller then polls [`Self::poll_commissioning_terminal`]
    /// until it reports the terminal action.
    pub fn begin_commissioning_poll_with_stages(
        &mut self,
        poll: CommissionPoll,
        retention: &mut crate::commission_dispatch::CommissionRetention<'_>,
        config: &TmcControlConfig,
    ) -> Option<ScheduledCommissionTickAction> {
        match poll {
            CommissionPoll::ReadyNow { event } => {
                Self::retain_integral_authority_from_event(self.channel, &event, retention);
                self.classify_commission_poll(CommissionPoll::ReadyNow { event })
            }
            CommissionPoll::At {
                timestamp_us,
                event,
            } => {
                Self::retain_integral_authority_from_event(self.channel, &event, retention);
                self.classify_commission_poll(CommissionPoll::At {
                    timestamp_us,
                    event,
                })
            }
            CommissionPoll::Terminal { result } => {
                self.commission_engine.as_ref()?;
                self.begin_commissioning_terminal(result, config);
                None
            }
        }
    }

    /// Convert an explicit poll into the board-facing scheduled action.
    pub async fn drive_commissioning_poll_with_stages<B, E>(
        &mut self,
        poll: CommissionPoll,
        backend: &mut B,
        mut context: crate::commission_dispatch::CommissionTerminalContext<'_, E>,
        tmc: &'static TmcChannelState,
        stepper: &'static StepperChannelState,
        config: &TmcControlConfig,
    ) -> Option<ScheduledCommissionTickAction>
    where
        B: CommissioningBackend,
        E: FnOnce(crate::commission_dispatch::ReplyPayload),
    {
        match poll {
            CommissionPoll::ReadyNow { event } => {
                Self::retain_integral_authority_from_event(
                    self.channel,
                    &event,
                    &mut context.retention,
                );
                self.classify_commission_poll(CommissionPoll::ReadyNow { event })
            }
            CommissionPoll::At {
                timestamp_us,
                event,
            } => {
                Self::retain_integral_authority_from_event(
                    self.channel,
                    &event,
                    &mut context.retention,
                );
                self.classify_commission_poll(CommissionPoll::At {
                    timestamp_us,
                    event,
                })
            }
            CommissionPoll::Terminal { result } => {
                self.drive_commissioning_terminal_with_stages(
                    result, backend, context, tmc, stepper, config,
                )
                .await
            }
        }
    }

    /// Dispatch a terminal with temporary empty Stage-C retention.
    ///
    /// Delegates to [`Self::drive_commissioning_terminal_with_stages`] with a
    /// throwaway Stage-C candidate slot, for callers that do not carry a
    /// persistent cross-call retention context.
    pub async fn drive_commissioning_terminal<B>(
        &mut self,
        result: CommissionTerminal,
        backend: &mut B,
        tmc: &'static TmcChannelState,
        stepper: &'static StepperChannelState,
        config: &TmcControlConfig,
    ) -> Option<ScheduledCommissionTickAction>
    where
        B: CommissioningBackend,
    {
        let mut integral_candidate =
            crate::commissioning::outer::velocity::integral_reproduction::IntegralRetentionSlot::new(
            );
        self.drive_commissioning_terminal_with_stages(
            result,
            backend,
            crate::commission_dispatch::CommissionTerminalContext::new(
                &mut integral_candidate,
                |_| unreachable!("temporary Stage-C retention cannot continue a workflow"),
            ),
            tmc,
            stepper,
            config,
        )
        .await
    }

    /// Dispatch one poll with temporary empty Stage-C retention.
    ///
    /// Delegates to [`Self::drive_commissioning_poll_with_stages`] with a
    /// throwaway Stage-C candidate slot, for callers that do not carry a
    /// persistent cross-call retention context.
    pub async fn drive_commissioning_poll<B>(
        &mut self,
        poll: CommissionPoll,
        backend: &mut B,
        tmc: &'static TmcChannelState,
        stepper: &'static StepperChannelState,
        config: &TmcControlConfig,
    ) -> Option<ScheduledCommissionTickAction>
    where
        B: CommissioningBackend,
    {
        let mut integral_candidate =
            crate::commissioning::outer::velocity::integral_reproduction::IntegralRetentionSlot::new(
            );
        self.drive_commissioning_poll_with_stages(
            poll,
            backend,
            crate::commission_dispatch::CommissionTerminalContext::new(
                &mut integral_candidate,
                |_| unreachable!("temporary Stage-C retention cannot continue a workflow"),
            ),
            tmc,
            stepper,
            config,
        )
        .await
    }

    /// Prepare an enable request up to the point where SPI or backend work begins.
    ///
    /// Returns no action when the motor is already enabled, or when
    /// the motor is uncalibrated and `auto_calibrate_on_enable_allowed` is not
    /// set. Returns the required stepper post-action when the motor is calibrated,
    /// so the board shell can drive the electrical arm verification and step/dir
    /// arming without carrying the general reply-capable action buffer.
    ///
    /// This helper intentionally does not energize the motor; the caller's SPI
    /// path and the auto-calibrate commissioning construction remain in the
    /// request handler.
    pub fn prepare_enable_request(
        &mut self,
        tmc: &TmcChannelState,
    ) -> Option<crate::commission_dispatch::StepperPostAction> {
        if self.motor_enabled {
            return None;
        }
        if !self.calibrated {
            if !tmc
                .auto_calibrate_on_enable_allowed
                .load(core::sync::atomic::Ordering::Relaxed)
            {
                return None;
            }
            // Auto-calibrate construction stays in the request handler.
            // This preflight helper must not energize the motor.
            return None;
        }
        Some(crate::commission_dispatch::StepperPostAction::ArmPreservePosition)
    }

    /// Prepare a disable request, emitting an immediate board stop action.
    ///
    /// Clears `motor_enabled` and `calibrated` so the next enable request
    /// requires a new calibration pass before it can arm. The returned
    /// [`StepperAction::StopNow`] action must be processed by the board shell
    /// before issuing any further SPI commands.
    pub fn prepare_disable_request(&mut self) -> BoardActions {
        let mut actions = BoardActions::new();
        push_action(&mut actions, BoardAction::Stepper(StepperAction::StopNow));
        self.motor_enabled = false;
        self.calibrated = false;
        actions
    }

    /// Clear runtime and shared step-acceptance state after a diagnostic trip.
    ///
    /// The board must lower the physical enable GPIO and stop its step timer
    /// before calling this synchronous bookkeeping method. It performs no SPI,
    /// delay, allocation, or asynchronous work.
    pub fn contain_diagnostic_trip(&mut self) {
        self.motor_enabled = false;
        self.calibrated = false;
        self.applied_velocity_offset = None;
        self.applied_torque_flux_offset = None;
        if let Some(tmc) = crate::state::tmc::tmc_channel(self.channel) {
            tmc.motor_armed.store(false, Relaxed);
        }
        if let Some(stepper) = crate::state::stepper::stepper_channel(self.channel) {
            stepper.timer_active_mirror.store(false, Relaxed);
        }
    }

    /// Return the exact planner-timing velocity ratio cached for later consumers.
    pub const fn planner_velocity_scale(
        &self,
    ) -> Option<crate::motion_scale::PlannerVelocityScale> {
        self.planner_velocity_scale
    }

    /// Return the exact planner-timing mechanical-RPM ratio cached for feedforward.
    pub const fn planner_rpm_scale(&self) -> Option<crate::motion_scale::PlannerRpmScale> {
        self.planner_rpm_scale
    }

    /// Cache both exact planner-timing ratios derived from one planner resolution.
    pub fn set_planner_scales(
        &mut self,
        velocity: Option<crate::motion_scale::PlannerVelocityScale>,
        rpm: Option<crate::motion_scale::PlannerRpmScale>,
    ) {
        self.planner_velocity_scale = velocity;
        self.planner_rpm_scale = rpm;
    }

    /// Return whether velocity feedforward was active on the previous control tick.
    pub const fn feedforward_was_active(&self) -> bool {
        self.feedforward_was_active
    }

    /// Store whether velocity feedforward is active for the current control tick.
    pub fn set_feedforward_was_active(&mut self, active: bool) {
        self.feedforward_was_active = active;
    }

    /// Invalidate the cached applied `PID_TORQUE_FLUX_OFFSET` value.
    ///
    /// Call this after any write to that register outside the control tick's
    /// own feedforward path (for example a diagnostic that writes a test
    /// value and restores the prior one), so the next tick re-verifies rather
    /// than trusting a value it never wrote.
    pub fn invalidate_torque_flux_offset_cache(&mut self) {
        self.applied_torque_flux_offset = None;
    }

    /// Return the remaining PID error-clear settle countdown in milliseconds.
    pub const fn pid_error_clear_countdown_ms(&self) -> u8 {
        self.pid_error_clear_countdown_ms
    }

    /// Store the remaining PID error-clear settle countdown in milliseconds.
    pub fn set_pid_error_clear_countdown_ms(&mut self, value: u8) {
        self.pid_error_clear_countdown_ms = value;
    }

    /// Compute current command velocity reference for feedforward surfaces.
    ///
    /// Returns zero while the motor is disabled, velocity scaling has not been
    /// configured, commissioning is active, homing is active, or the caller has
    /// consumed a one-tick feedforward reset latch.
    pub fn command_velocity_reference(
        &mut self,
        refs: TmcControlStateRefs<'_>,
        homing_active: bool,
        reset: bool,
    ) -> i32 {
        let reference = match self.planner_velocity_scale {
            Some(scale)
                if self.motor_enabled
                    && self.planner_paths_allowed(refs)
                    && !homing_active
                    && !reset =>
            {
                let interval = refs.stepper.interval.load(Relaxed);
                let forward = refs.stepper.direction.load(Relaxed);
                crate::tmc::compute_planner_command_velocity_reference(scale, interval, forward)
            }
            Some(_) | None => 0,
        };
        #[cfg(feature = "trace")]
        {
            self.trace_state_mut().cached_velocity_reference = reference;
        }
        reference
    }

    /// Apply velocity, transient, acceleration, and decoupling feedforward.
    ///
    /// Velocity and transient offsets run on every control tick. Acceleration
    /// and decoupling offsets run only when `slow_control_tick` is true, matching
    /// the board-local decimation used by the surrounding maintenance path.
    ///
    /// Active feedforward SPI operations are best-effort to preserve existing
    /// board behavior. Failed inactive velocity- and torque-offset clears are
    /// propagated so the runtime retains its active latch and retries the clear.
    pub async fn apply_feedforward_tick<B>(
        &mut self,
        tmc_spi: &mut foci_tmc4671::spi::Tmc4671Spi<B>,
        refs: TmcControlStateRefs<'_>,
        config: &TmcControlConfig,
        homing_active: bool,
        reset: bool,
        slow_control_tick: bool,
    ) -> Result<(), ()>
    where
        B: embedded_hal_async::spi::SpiDevice<u8>,
    {
        let ff_active = self.motor_enabled
            && self.planner_rpm_scale.is_some()
            && refs.tmc.velocity_feedforward.load(Relaxed)
            && self.planner_paths_allowed(refs);
        let velocity_transient_active = self.motor_enabled
            && self.planner_rpm_scale.is_some()
            && refs.tmc.velocity_transient_feedforward.load(Relaxed)
            && self.planner_paths_allowed(refs);
        let accel_ff_active = self.motor_enabled
            && self.planner_velocity_scale.is_some()
            && refs.tmc.accel_feedforward.load(Relaxed)
            && self.planner_paths_allowed(refs);
        let decoupling_ff_active = self.motor_enabled
            && self.planner_velocity_scale.is_some()
            && refs.tmc.decoupling_feedforward.load(Relaxed)
            && self.planner_paths_allowed(refs);

        let velocity_offset_active = ff_active || velocity_transient_active;
        if velocity_offset_active {
            let interval = refs.stepper.interval.load(Relaxed);
            let interval_add = refs
                .stepper
                .interval_add
                .load(Relaxed)
                .clamp(i16::MIN as i32, i16::MAX as i32) as i16;
            let forward = refs.stepper.direction.load(Relaxed);
            let rpm_scale = self
                .planner_rpm_scale
                .expect("active feedforward has planner rpm scale");
            let base_offset = if ff_active && !homing_active && !reset {
                let gain_permille = refs.tmc.velocity_feedforward_gain_permille.load(Relaxed);
                crate::tmc::compute_planner_velocity_offset_rpm(
                    rpm_scale,
                    interval,
                    forward,
                    gain_permille,
                )
            } else {
                0
            };
            let transient_offset = if velocity_transient_active && !homing_active && !reset {
                crate::tmc::compute_planner_velocity_transient_offset(
                    crate::tmc::PlannerVelocityTransientFeedforwardInput {
                        scale: rpm_scale,
                        clock_freq: config.clock_freq,
                        step_interval: interval,
                        interval_add,
                        dir_high: forward,
                    },
                    crate::tmc::VelocityTransientFeedforwardConfig {
                        lead_time_us: refs.tmc.velocity_transient_lead_time_us.load(Relaxed),
                        gain_permille: refs.tmc.velocity_transient_gain_permille.load(Relaxed),
                        max_offset: refs.tmc.velocity_transient_max_offset.load(Relaxed),
                    },
                )
            } else {
                0
            };
            let command_rpm =
                crate::tmc::compute_planner_command_rpm_reference(rpm_scale, interval, forward);
            let (offset, clamp_engaged) = crate::tmc::clamp_velocity_offset_rpm(
                base_offset.saturating_add(transient_offset),
                command_rpm,
                crate::tmc::VELOCITY_FF_CLAMP_K,
            );
            if clamp_engaged {
                refs.tmc.record_velocity_clamp_engagement();
            }
            let desired = offset as u32;

            if self.applied_velocity_offset != Some(desired) {
                let _ = tmc_spi
                    .write_register(foci_tmc4671::registers::PID_VELOCITY_OFFSET, desired)
                    .await
                    .inspect(|_| self.applied_velocity_offset = Some(desired));
            }
            #[cfg(feature = "trace")]
            {
                let trace = self.trace_state_mut();
                trace.cached_velocity_offset = offset;
                trace.cached_velocity_transient_offset = transient_offset;
                trace.cached_velocity_clamped = clamp_engaged;
            }
        } else if self.feedforward_was_active {
            tmc_spi
                .write_register(foci_tmc4671::registers::PID_VELOCITY_OFFSET, 0)
                .await
                .map_err(|_| ())?;
            self.applied_velocity_offset = Some(0);
            #[cfg(feature = "trace")]
            {
                let trace = self.trace_state_mut();
                trace.cached_velocity_offset = 0;
                trace.cached_velocity_transient_offset = 0;
                trace.cached_velocity_clamped = false;
            }
        }
        self.feedforward_was_active = velocity_offset_active;

        if !slow_control_tick {
            self.publish_feedforward_surfaces_idle(refs);
            return Ok(());
        }

        if decoupling_ff_active {
            let max_offset = refs.tmc.decoupling_max_offset.load(Relaxed);
            let (torque_offset, flux_offset) = if homing_active || reset {
                (0, 0)
            } else {
                let interval = refs.stepper.interval.load(Relaxed);
                let forward = refs.stepper.direction.load(Relaxed);
                let target_velocity = crate::tmc::compute_planner_command_velocity_reference(
                    self.planner_velocity_scale
                        .expect("active decoupling has planner velocity scale"),
                    interval,
                    forward,
                );
                let iq_id = if target_velocity == 0 {
                    None
                } else {
                    tmc_spi
                        .read_sub_register(foci_tmc4671::registers::INTERIM_FOC_IQ_ID)
                        .await
                        .ok()
                };
                if let Some(iq_id) = iq_id {
                    let foc_iq = crate::tmc::decode_signed_upper_half(iq_id);
                    let foc_id = crate::tmc::decode_signed_lower_half(iq_id);
                    crate::tmc::compute_decoupling_current_offsets(
                        crate::tmc::DecouplingFeedforwardInput {
                            target_velocity,
                            foc_iq,
                            foc_id,
                        },
                        crate::tmc::DecouplingFeedforwardConfig {
                            r_int: refs.tmc.decoupling_r_int.load(Relaxed),
                            l_int: refs.tmc.decoupling_l_int.load(Relaxed),
                            pole_pairs: refs.tmc.decoupling_pole_pairs.load(Relaxed),
                            position_units_per_rev: refs
                                .tmc
                                .decoupling_position_units_per_rev
                                .load(Relaxed),
                            f_pwm_hz: refs.tmc.decoupling_f_pwm_hz.load(Relaxed),
                            max_offset,
                        },
                    )
                } else {
                    (0, 0)
                }
            };
            #[cfg(feature = "trace")]
            {
                let mut flags = if homing_active || reset {
                    0
                } else {
                    crate::trace::stream::ACCEL_FF_FLAG_ENABLED
                };
                if torque_offset != 0 || flux_offset != 0 {
                    flags |= crate::trace::stream::ACCEL_FF_FLAG_NONZERO;
                }
                if max_offset != 0
                    && (torque_offset.unsigned_abs() >= max_offset
                        || flux_offset.unsigned_abs() >= max_offset)
                {
                    flags |= crate::trace::stream::ACCEL_FF_FLAG_CLAMPED;
                }
                let trace = self.trace_state_mut();
                trace.cached_accel_torque_offset = torque_offset as i32;
                trace.cached_accel_gain_permille = 0;
                trace.cached_accel_flags = flags;
            }
            let offset_reg = crate::tmc::pack_torque_flux_target(torque_offset, flux_offset);
            if self.applied_torque_flux_offset != Some(offset_reg) {
                let _ = tmc_spi
                    .write_register(foci_tmc4671::registers::PID_TORQUE_FLUX_OFFSET, offset_reg)
                    .await
                    .inspect(|_| self.applied_torque_flux_offset = Some(offset_reg));
            }
        } else if accel_ff_active {
            let accel_gain_permille = refs.tmc.accel_feedforward_accel_gain_permille.load(Relaxed);
            let decel_gain_permille = refs.tmc.accel_feedforward_decel_gain_permille.load(Relaxed);
            let torque_limit = refs.tmc.load_current_limit(Relaxed).register_value();
            let interval = refs.stepper.interval.load(Relaxed);
            let interval_add = refs
                .stepper
                .interval_add
                .load(Relaxed)
                .clamp(i16::MIN as i32, i16::MAX as i32) as i16;
            #[cfg(feature = "trace")]
            let selected_gain_permille = if interval_add < 0 {
                accel_gain_permille
            } else if interval_add > 0 {
                decel_gain_permille
            } else {
                0
            };
            let offset = if homing_active || reset {
                0i16
            } else {
                let forward = refs.stepper.direction.load(Relaxed);
                crate::tmc::compute_planner_accel_torque_offset_split(
                    self.planner_velocity_scale
                        .expect("active acceleration feedforward has planner velocity scale"),
                    config.clock_freq,
                    interval,
                    interval_add,
                    forward,
                    crate::tmc::AccelFeedforwardGains {
                        accel_permille: accel_gain_permille,
                        decel_permille: decel_gain_permille,
                    },
                    torque_limit,
                )
            };
            #[cfg(feature = "trace")]
            {
                let mut flags = if homing_active || reset {
                    0
                } else {
                    crate::trace::stream::ACCEL_FF_FLAG_ENABLED
                };
                if offset != 0 {
                    flags |= crate::trace::stream::ACCEL_FF_FLAG_NONZERO;
                }
                if offset != 0 && offset.unsigned_abs() as u32 >= torque_limit.min(i16::MAX as u32)
                {
                    flags |= crate::trace::stream::ACCEL_FF_FLAG_CLAMPED;
                }
                let trace = self.trace_state_mut();
                trace.cached_accel_torque_offset = offset as i32;
                trace.cached_accel_gain_permille = if homing_active || reset {
                    0
                } else {
                    selected_gain_permille
                };
                trace.cached_accel_flags = flags;
            }
            let offset_reg = u32::from(offset as u16) << 16;
            if self.applied_torque_flux_offset != Some(offset_reg) {
                let _ = tmc_spi
                    .write_register(foci_tmc4671::registers::PID_TORQUE_FLUX_OFFSET, offset_reg)
                    .await
                    .inspect(|_| self.applied_torque_flux_offset = Some(offset_reg));
            }
        } else if self.torque_flux_offset_was_active {
            tmc_spi
                .write_register(foci_tmc4671::registers::PID_TORQUE_FLUX_OFFSET, 0)
                .await
                .map_err(|_| ())?;
            self.applied_torque_flux_offset = Some(0);
            #[cfg(feature = "trace")]
            {
                let trace = self.trace_state_mut();
                trace.cached_accel_torque_offset = 0;
                trace.cached_accel_gain_permille = 0;
                trace.cached_accel_flags = 0;
            }
        }
        self.torque_flux_offset_was_active = accel_ff_active || decoupling_ff_active;

        self.publish_feedforward_surfaces_idle(refs);
        Ok(())
    }

    /// Restore the ABN phase-offset register to the saved base value.
    ///
    /// Returns `true` when no phase advance is active or when the restore write
    /// succeeds. Returns `false` on SPI write failure and leaves the active
    /// state intact so the caller can retry before arming, disarming, or
    /// entering diagnostic/tuning modes.
    ///
    /// # Parameters
    /// - `tmc_spi`: TMC4671 SPI transport used for the register write.
    pub async fn restore_phase_advance_now<B>(
        &mut self,
        tmc_spi: &mut foci_tmc4671::spi::Tmc4671Spi<B>,
    ) -> bool
    where
        B: embedded_hal_async::spi::SpiDevice<u8>,
    {
        if !self.phase_advance_was_active {
            return true;
        }

        match tmc_spi
            .write_register(
                foci_tmc4671::registers::ABN_DECODER_PHI_E_PHI_M_OFFSET,
                self.phase_advance_base_register,
            )
            .await
        {
            Ok(()) => {
                self.mark_phase_advance_restored();
                true
            }
            Err(_) => {
                #[cfg(feature = "trace")]
                {
                    let trace = self.trace_state_mut();
                    trace.phase_advance_write_failed = true;
                    trace.phase_advance_emit_once = true;
                }
                false
            }
        }
    }

    /// Restore the ABN phase-offset register through a commissioning backend.
    ///
    /// This has the same retry-state semantics as [`Self::restore_phase_advance_now`],
    /// but writes through [`crate::commissioning::CommissionRegisterIo`] so
    /// shared lifecycle helpers can restore phase advance while holding a
    /// board backend.
    pub async fn restore_phase_advance_via_backend<B>(&mut self, backend: &mut B) -> bool
    where
        B: crate::commissioning::CommissionRegisterIo,
    {
        if !self.phase_advance_was_active {
            return true;
        }

        match backend
            .write_register(
                foci_tmc4671::registers::ABN_DECODER_PHI_E_PHI_M_OFFSET,
                self.phase_advance_base_register,
            )
            .await
        {
            Ok(()) => {
                self.mark_phase_advance_restored();
                true
            }
            Err(_) => {
                #[cfg(feature = "trace")]
                {
                    let trace = self.trace_state_mut();
                    trace.phase_advance_write_failed = true;
                    trace.phase_advance_emit_once = true;
                }
                false
            }
        }
    }

    /// Apply slow phase-advance and position-lead tick work.
    ///
    /// The caller must pass the single consumed feedforward-reset latch used by
    /// velocity feedforward for this same control tick. SPI failures are
    /// best-effort except for warnings that match the former board-local path.
    ///
    /// # Parameters
    /// - `tmc_spi`: TMC4671 SPI transport for register reads and writes.
    /// - `refs`: Borrowed channel atomics for this motor.
    /// - `command_velocity_reference`: The command velocity reference already
    ///   computed for this tick using the same reset latch.
    /// - `homing_active`: Whether homing is currently suppressing lead terms.
    /// - `reset`: The single feedforward-reset latch consumed by the board tick.
    pub async fn apply_phase_and_position_tick<B>(
        &mut self,
        tmc_spi: &mut foci_tmc4671::spi::Tmc4671Spi<B>,
        refs: TmcControlStateRefs<'_>,
        command_velocity_reference: i32,
        homing_active: bool,
        reset: bool,
    ) where
        B: embedded_hal_async::spi::SpiDevice<u8>,
    {
        let phase_advance_requested = self.motor_enabled
            && self.planner_velocity_scale.is_some()
            && refs.tmc.phase_advance.load(Relaxed)
            && self.planner_paths_allowed(refs);
        let phase_advance_active = phase_advance_requested && !homing_active && !reset;

        if phase_advance_active && !self.phase_advance_was_active {
            match tmc_spi
                .read_register(foci_tmc4671::registers::ABN_DECODER_PHI_E_PHI_M_OFFSET)
                .await
            {
                Ok(base) => {
                    self.phase_advance_base_register = base;
                    self.phase_advance_last_register = base;
                    self.phase_advance_was_active = true;
                    #[cfg(feature = "trace")]
                    {
                        self.trace_state_mut().phase_advance_emit_once = true;
                    }
                }
                Err(_) => {
                    defmt::warn!("phase advance: failed to snapshot ABN offset");
                    self.phase_advance_was_active = false;
                }
            }
        }

        #[cfg(feature = "trace")]
        let mut trace_phase_output = crate::tmc::PhaseAdvanceOutput {
            counts: 0,
            enabled: phase_advance_requested,
            deadbanded: false,
            clamped: false,
        };

        if phase_advance_active && self.phase_advance_was_active {
            let phase_output = crate::tmc::compute_phase_advance_counts(
                command_velocity_reference,
                crate::tmc::PhaseAdvanceConfig {
                    enable: true,
                    gain_ppm: refs.tmc.phase_advance_gain_ppm.load(Relaxed),
                    max_counts: refs.tmc.phase_advance_max_counts.load(Relaxed),
                    deadband_velocity: refs.tmc.phase_advance_deadband.load(Relaxed),
                },
            );
            #[cfg(feature = "trace")]
            {
                trace_phase_output = phase_output;
            }
            let next_register = crate::tmc::pack_abn_phi_offset(
                self.phase_advance_base_register,
                phase_output.counts,
            );
            if next_register != self.phase_advance_last_register {
                if tmc_spi
                    .write_register(
                        foci_tmc4671::registers::ABN_DECODER_PHI_E_PHI_M_OFFSET,
                        next_register,
                    )
                    .await
                    .is_ok()
                {
                    #[cfg(feature = "trace")]
                    {
                        let trace = self.trace_state_mut();
                        if phase_output.counts != trace.phase_advance_last_counts {
                            trace.phase_advance_emit_once = true;
                        }
                        trace.phase_advance_write_failed = false;
                    }
                    self.phase_advance_last_register = next_register;
                } else {
                    #[cfg(feature = "trace")]
                    {
                        let trace = self.trace_state_mut();
                        trace.phase_advance_write_failed = true;
                        trace.phase_advance_emit_once = true;
                    }
                }
            }
        } else if self.phase_advance_was_active && !self.restore_phase_advance_now(tmc_spi).await {
            defmt::warn!("phase advance: failed to restore ABN offset; retrying");
        }

        #[cfg(feature = "trace")]
        {
            let write_failed = self.trace_state().phase_advance_write_failed;
            let mut flags = if phase_advance_active && self.phase_advance_was_active {
                crate::trace::stream::PHASE_ADVANCE_FLAG_ENABLED
            } else if phase_advance_requested {
                crate::trace::stream::PHASE_ADVANCE_FLAG_GATED
            } else {
                0
            };
            if trace_phase_output.counts != 0 {
                flags |= crate::trace::stream::PHASE_ADVANCE_FLAG_NONZERO;
            }
            if trace_phase_output.clamped {
                flags |= crate::trace::stream::PHASE_ADVANCE_FLAG_CLAMPED;
            }
            if trace_phase_output.deadbanded {
                flags |= crate::trace::stream::PHASE_ADVANCE_FLAG_DEADBAND;
            }
            if write_failed {
                flags |= crate::trace::stream::PHASE_ADVANCE_FLAG_WRITE_FAILED;
            }

            let trace = self.trace_state_mut();
            if flags != trace.phase_advance_last_flags {
                trace.phase_advance_emit_once = true;
            }
            trace.phase_advance_last_counts = trace_phase_output.counts;
            trace.phase_advance_last_flags = flags;
        }

        let position_lead_active = self.motor_enabled
            && self.planner_velocity_scale.is_some()
            && refs.tmc.position_lead.load(Relaxed)
            && self.commission_engine.is_none()
            && !homing_active;
        if position_lead_active || self.position_lead_applied_units != 0 {
            let interval = refs.stepper.interval.load(Relaxed);
            let forward = refs.stepper.direction.load(Relaxed);
            let gain_permille = refs.tmc.position_lead_gain_permille.load(Relaxed);
            let max_counts = refs.tmc.position_lead_max_counts.load(Relaxed);
            let lead_counts = if !position_lead_active || reset || interval == 0 {
                0
            } else {
                crate::tmc::compute_position_lead_counts_from_reference(
                    command_velocity_reference,
                    forward,
                    gain_permille,
                    max_counts,
                )
            };
            if let Some(planner_steps_per_rev) = refs.stepper.planner_steps_per_rev(refs.tmc) {
                match tmc_spi
                    .read_register(foci_tmc4671::registers::PID_POSITION_TARGET)
                    .await
                {
                    Ok(current_target) => {
                        let (target, applied_units) = crate::tmc::compute_position_lead_target(
                            current_target,
                            self.position_lead_applied_units,
                            lead_counts,
                            planner_steps_per_rev,
                        );
                        if applied_units != self.position_lead_applied_units
                            && tmc_spi
                                .write_register(
                                    foci_tmc4671::registers::PID_POSITION_TARGET,
                                    target,
                                )
                                .await
                                .is_ok()
                        {
                            self.position_lead_applied_units = applied_units;
                        }
                    }
                    Err(_) => {
                        defmt::warn!("position lead: PID_POSITION_TARGET read failed");
                    }
                }
            }
        }

        self.publish_feedforward_surfaces_idle(refs);
    }

    fn mark_phase_advance_restored(&mut self) {
        self.phase_advance_was_active = false;
        self.phase_advance_last_register = self.phase_advance_base_register;
        #[cfg(feature = "trace")]
        {
            let trace = self.trace_state_mut();
            trace.phase_advance_last_counts = 0;
            trace.phase_advance_write_failed = false;
            trace.phase_advance_emit_once = true;
        }
    }

    /// Apply one channel's homing maintenance for a frozen homing state.
    ///
    /// The caller owns global homing state advancement. This method only
    /// performs per-channel TMC register work for the provided snapshot.
    ///
    /// State 1 suppresses the position integrator, clamps the torque/flux
    /// current limit to the configured homing current, and arms the stall
    /// detector. State 2 feeds the detector one target/actual position sample.
    /// State 3 restores the stored run current and publishes the detector
    /// result. A missing precondition or an exhausted write-retry budget
    /// returns [`HomingMaintenanceResult::FailClosed`], which the board must
    /// translate into an emergency disable.
    ///
    /// The clamp writes `PID_TORQUE_FLUX_LIMITS` directly and never mutates the
    /// stored run-current limit in `TmcChannelState`.
    pub async fn apply_homing_channel_tick<B>(
        &mut self,
        tmc_spi: &mut foci_tmc4671::spi::Tmc4671Spi<B>,
        refs: TmcControlStateRefs<'_>,
        config: &TmcControlConfig,
        homing_state: u8,
        window: HomingClockWindow,
    ) -> HomingMaintenanceResult
    where
        B: embedded_hal_async::spi::SpiDevice<u8>,
    {
        match homing_state {
            1 => {
                if !homing_clamp_due(window, config.clock_freq) {
                    return HomingMaintenanceResult::Deferred;
                }
                if !self.restore_phase_advance_now(tmc_spi).await {
                    defmt::warn!("phase advance: failed to restore ABN offset before homing");
                    return self.retry_homing_state_1();
                }

                let gains = match refs.tmc.load_outer_gains(Relaxed) {
                    Ok(gains) => gains,
                    Err(_) => return self.retry_homing_state_1(),
                };
                let reg = PidGainPair::new(gains.position_p, PidIGain::ZERO).register_value();
                let _ = tmc_spi
                    .write_register(foci_tmc4671::registers::PID_POSITION_P_POSITION_I, reg)
                    .await;
                defmt::info!("homing: position I suppressed");

                match self.apply_homing_current_clamp(tmc_spi, refs, config).await {
                    HomingMaintenanceResult::ReadyToAdvance => {
                        self.homing_state_1_retries = 0;
                        HomingMaintenanceResult::ReadyToAdvance
                    }
                    HomingMaintenanceResult::RetryCurrentState => self.retry_homing_state_1(),
                    HomingMaintenanceResult::FailClosed => HomingMaintenanceResult::FailClosed,
                    HomingMaintenanceResult::Deferred => HomingMaintenanceResult::Deferred,
                }
            }
            2 => {
                if let Some(detector) = self.stall_detector.as_mut() {
                    let target = tmc_spi
                        .read_register(foci_tmc4671::registers::PID_POSITION_TARGET)
                        .await;
                    let actual = tmc_spi
                        .read_register(foci_tmc4671::registers::PID_POSITION_ACTUAL)
                        .await;
                    if let (Ok(target), Ok(actual)) = (target, actual)
                        && detector.observe(target, actual)
                    {
                        refs.tmc.stall_latched.store(true, Relaxed);
                    }
                }
                HomingMaintenanceResult::ReadyToAdvance
            }
            3 => {
                let gains = match refs.tmc.load_outer_gains(Relaxed) {
                    Ok(gains) => gains,
                    Err(_) => return HomingMaintenanceResult::RetryCurrentState,
                };
                let reg = PidGainPair::new(gains.position_p, gains.position_i).register_value();
                let _ = tmc_spi
                    .write_register(foci_tmc4671::registers::PID_POSITION_P_POSITION_I, reg)
                    .await;
                let _ = tmc_spi
                    .write_sub_register(
                        foci_tmc4671::registers::PID_ERROR_PID_POSITION_ERROR_SUM,
                        0,
                    )
                    .await;
                defmt::info!("homing: position I restored, integrator cleared");

                self.restore_run_current_after_homing(tmc_spi, refs).await
            }
            _ => {
                self.homing_state_1_retries = 0;
                self.homing_state_3_retries = 0;
                if self.stall_detector.take().is_some()
                    || refs.tmc.homing_clamp_active.load(Relaxed)
                {
                    refs.tmc.homing_clamp_active.store(false, Relaxed);
                    refs.tmc.stall_latched.store(false, Relaxed);
                    defmt::warn!("homing: state forced idle outside the tick, detector dropped");
                }
                HomingMaintenanceResult::ReadyToAdvance
            }
        }
    }

    fn retry_homing_state_1(&mut self) -> HomingMaintenanceResult {
        self.homing_state_1_retries = self.homing_state_1_retries.saturating_add(1);
        if self.homing_state_1_retries >= HOMING_CLAMP_MAX_RETRIES {
            defmt::error!(
                "homing: state 1 failed {} times",
                self.homing_state_1_retries
            );
            self.homing_state_1_retries = 0;
            return HomingMaintenanceResult::FailClosed;
        }
        HomingMaintenanceResult::RetryCurrentState
    }

    /// Clamp the torque/flux limit to the homing current and arm the detector.
    async fn apply_homing_current_clamp<B>(
        &mut self,
        tmc_spi: &mut foci_tmc4671::spi::Tmc4671Spi<B>,
        refs: TmcControlStateRefs<'_>,
        config: &TmcControlConfig,
    ) -> HomingMaintenanceResult
    where
        B: embedded_hal_async::spi::SpiDevice<u8>,
    {
        let homing_ma = refs.tmc.homing_current_ma.load(Relaxed);
        if homing_ma == 0 {
            if refs.tmc.stall_endstop_configured.load(Relaxed) {
                defmt::error!("homing: stall endstop configured without homing current");
                return HomingMaintenanceResult::FailClosed;
            }
            return HomingMaintenanceResult::ReadyToAdvance;
        }
        if !(self.motor_enabled && self.calibrated) || self.is_commission_active() {
            defmt::error!("homing: channel not armed for steps, refusing clamp");
            return HomingMaintenanceResult::FailClosed;
        }
        let limit = match crate::tmc_control::current_limit::current_limit_from_run_ma(
            homing_ma,
            config.current_scale_ma_per_lsb,
        ) {
            Ok(limit) => limit,
            Err(_) => {
                defmt::error!("homing: invalid homing current {} mA", homing_ma);
                return HomingMaintenanceResult::FailClosed;
            }
        };
        if limit.raw() >= refs.tmc.load_current_limit(Relaxed).raw() {
            defmt::error!(
                "homing: homing current {} mA is not below run current",
                homing_ma
            );
            return HomingMaintenanceResult::FailClosed;
        }
        if tmc_spi
            .write_register(
                foci_tmc4671::registers::PID_TORQUE_FLUX_LIMITS,
                limit.register_value(),
            )
            .await
            .is_err()
        {
            return HomingMaintenanceResult::RetryCurrentState;
        }
        refs.tmc.stall_latched.store(false, Relaxed);
        refs.tmc.homing_clamp_active.store(true, Relaxed);
        self.stall_detector = Some(crate::tmc_control::stall_detector::StallDetector::new(
            refs.tmc.stall_threshold_units.load(Relaxed),
            refs.tmc.stall_persistence.load(Relaxed),
        ));
        defmt::info!("homing: clamp {} mA, detector armed", homing_ma);
        HomingMaintenanceResult::ReadyToAdvance
    }

    /// Rewrite the stored run current and publish the stall-detector result.
    async fn restore_run_current_after_homing<B>(
        &mut self,
        tmc_spi: &mut foci_tmc4671::spi::Tmc4671Spi<B>,
        refs: TmcControlStateRefs<'_>,
    ) -> HomingMaintenanceResult
    where
        B: embedded_hal_async::spi::SpiDevice<u8>,
    {
        if refs.tmc.homing_clamp_active.load(Relaxed) {
            let run_limit = refs.tmc.load_current_limit(Relaxed).register_value();
            if tmc_spi
                .write_register(foci_tmc4671::registers::PID_TORQUE_FLUX_LIMITS, run_limit)
                .await
                .is_err()
            {
                self.homing_state_3_retries = self.homing_state_3_retries.saturating_add(1);
                if self.homing_state_3_retries >= HOMING_RESTORE_MAX_RETRIES {
                    defmt::error!(
                        "homing: run-current restore failed {} times",
                        self.homing_state_3_retries
                    );
                    self.homing_state_3_retries = 0;
                    return HomingMaintenanceResult::FailClosed;
                }
                return HomingMaintenanceResult::RetryCurrentState;
            }
            self.homing_state_3_retries = 0;
            refs.tmc.homing_clamp_active.store(false, Relaxed);
        }
        if let Some(detector) = self.stall_detector.take() {
            refs.tmc
                .stall_result_latched
                .store(detector.latched(), Relaxed);
            refs.tmc
                .stall_peak_error_units
                .store(detector.peak_error_units(), Relaxed);
            refs.tmc
                .stall_trigger_tick
                .store(detector.trigger_tick(), Relaxed);
        }
        refs.tmc.stall_latched.store(false, Relaxed);
        defmt::info!("homing: run current restored");
        HomingMaintenanceResult::ReadyToAdvance
    }

    /// Advance the global homing state after every channel observed a snapshot.
    pub fn advance_homing_state_after_channels(
        homing: TmcControlHomingRefs<'_>,
        snapshot: u8,
        all_channels_ready: bool,
    ) {
        if !all_channels_ready {
            return;
        }

        match snapshot {
            1 => homing.state.store(2, Relaxed),
            2 if homing.triggered.load(Relaxed) != 0xFF => homing.state.store(3, Relaxed),
            3 => Self::leave_restore_pending(&homing),
            _ => {}
        }
    }

    /// Leave state 3, then promote a pass that registered during the restore.
    ///
    /// The order matters: the registration handler preempts this task and
    /// runs to completion, so it lands either before the exchange (it saw 3,
    /// set the flag, and the swap picks it up) or after it (it saw 0 and took
    /// the idle path itself). Consuming the flag before leaving 3 would let a
    /// registration in between set a flag nothing ever reads.
    fn leave_restore_pending(homing: &TmcControlHomingRefs<'_>) {
        if homing
            .state
            .compare_exchange(3, 0, Relaxed, Relaxed)
            .is_ok()
            && homing.reenter.swap(false, Relaxed)
        {
            let _ = homing.state.compare_exchange(0, 1, Relaxed, Relaxed);
        }
    }

    /// Apply the shared per-channel slow PID and homing policy.
    ///
    /// Board shells with multi-channel loops can use this helper before their
    /// own status polling and trace transport logic. A non-ready homing result
    /// means the caller should skip later slow-maintenance work for this
    /// channel until the next slow tick.
    pub async fn apply_slow_channel_maintenance_tick<B>(
        &mut self,
        tmc_spi: &mut foci_tmc4671::spi::Tmc4671Spi<B>,
        refs: TmcControlStateRefs<'_>,
        config: &TmcControlConfig,
        homing_snapshot: u8,
        window: HomingClockWindow,
    ) -> HomingMaintenanceResult
    where
        B: embedded_hal_async::spi::SpiDevice<u8>,
    {
        self.apply_pid_error_settle_tick(tmc_spi, refs).await;
        self.apply_homing_channel_tick(tmc_spi, refs, config, homing_snapshot, window)
            .await
    }

    /// Advance the trace control tick for a board-local trace sample.
    #[cfg(feature = "trace")]
    pub fn advance_trace_sample_tick(&mut self) -> u32 {
        let sample_tick = self.trace.control_tick;
        self.trace.control_tick = self.trace.control_tick.wrapping_add(1);
        sample_tick
    }

    /// Advance trace timing and return transport emission decisions.
    #[cfg(feature = "trace")]
    pub fn advance_trace_tick(&mut self) -> TmcControlTraceTick {
        let sample_tick = self.advance_trace_sample_tick();
        let control_tick = self.trace.control_tick;
        let phase_advance_counts = self.trace.phase_advance_last_counts;
        let phase_advance_flags = self.trace.phase_advance_last_flags;
        let phase_advance_periodic = control_tick.is_multiple_of(10)
            && (phase_advance_flags != 0 || phase_advance_counts != 0);
        let phase_advance_due = self.trace.phase_advance_emit_once || phase_advance_periodic;
        if phase_advance_due {
            self.trace.phase_advance_emit_once = false;
        }

        TmcControlTraceTick {
            sample_tick,
            control_tick,
            phase_advance_due,
            phase_advance_counts,
            phase_advance_flags,
            status_detail_due: control_tick.is_multiple_of(100),
            metadata_due: control_tick.is_multiple_of(1000),
        }
    }

    /// Compute the requested control period in microseconds from current state.
    ///
    /// Returns [`crate::tmc::VELOCITY_TRANSIENT_IDLE_PERIOD_US`] whenever the
    /// motor is disabled or [`Self::planner_paths_allowed`] is false.
    /// Otherwise, returns the fast transient period when the planner
    /// velocity scale is configured and the `velocity_transient_feedforward`
    /// atomic in channel state is `true`; otherwise
    /// `velocity_transient_period_us(VELOCITY_TRANSIENT_DEFAULT_RATE_HZ)`.
    ///
    /// # Parameters
    /// - `refs`: Borrowed channel atomics for this motor.
    pub fn control_period_us(&self, refs: TmcControlStateRefs<'_>) -> u32 {
        if !self.motor_enabled || !self.planner_paths_allowed(refs) {
            return crate::tmc::VELOCITY_TRANSIENT_IDLE_PERIOD_US;
        }
        let fast_active = self.planner_velocity_scale.is_some()
            && refs
                .tmc
                .velocity_transient_feedforward
                .load(core::sync::atomic::Ordering::Relaxed);
        let rate_hz = if fast_active {
            refs.tmc
                .velocity_transient_rate_hz
                .load(core::sync::atomic::Ordering::Relaxed)
        } else {
            crate::tmc::VELOCITY_TRANSIENT_DEFAULT_RATE_HZ
        };
        crate::tmc::velocity_transient_period_us(rate_hz)
    }

    #[cfg(test)]
    pub(crate) fn set_calibrated_for_test(&mut self, value: bool) {
        self.calibrated = value;
    }

    #[cfg(test)]
    pub(crate) fn set_motor_enabled_for_test(&mut self, value: bool) {
        self.motor_enabled = value;
    }

    #[cfg(test)]
    pub(crate) fn set_torque_flux_offset_was_active_for_test(&mut self, value: bool) {
        self.torque_flux_offset_was_active = value;
    }

    #[cfg(test)]
    pub(crate) fn set_position_lead_applied_units_for_test(&mut self, value: i32) {
        self.position_lead_applied_units = value;
    }

    #[cfg(test)]
    pub(crate) fn set_homing_state_1_retries_for_test(&mut self, value: u8) {
        self.homing_state_1_retries = value;
    }

    #[cfg(test)]
    pub(crate) fn set_homing_state_3_retries_for_test(&mut self, value: u8) {
        self.homing_state_3_retries = value;
    }

    #[cfg(test)]
    pub(crate) const fn homing_retry_counts_for_test(&self) -> (u8, u8) {
        (self.homing_state_1_retries, self.homing_state_3_retries)
    }

    #[cfg(test)]
    pub(crate) fn set_applied_velocity_offset_for_test(&mut self, value: Option<u32>) {
        self.applied_velocity_offset = value;
    }

    #[cfg(test)]
    pub(crate) fn set_applied_torque_flux_offset_for_test(&mut self, value: Option<u32>) {
        self.applied_torque_flux_offset = value;
    }

    #[cfg(test)]
    pub(crate) fn stall_detector_armed_for_test(&self) -> bool {
        self.stall_detector.is_some()
    }

    #[cfg(test)]
    pub(crate) fn homing_state_1_retries_for_test(&self) -> u8 {
        self.homing_state_1_retries
    }

    #[cfg(test)]
    pub(crate) fn arm_stall_detector_for_test(&mut self, threshold: u32, persistence: u8) {
        self.stall_detector = Some(crate::tmc_control::stall_detector::StallDetector::new(
            threshold,
            persistence,
        ));
    }

    #[cfg(test)]
    pub(crate) fn set_planner_velocity_scale_for_test(&mut self, numerator: u64) {
        self.planner_velocity_scale = Some(
            crate::motion_scale::PlannerVelocityScale::from_reduced(numerator, 1),
        );
        self.planner_rpm_scale = Some(crate::motion_scale::PlannerRpmScale::from_reduced(
            numerator, 1,
        ));
    }

    #[cfg(test)]
    pub(crate) fn pid_error_clear_countdown_for_test(&self) -> u8 {
        self.pid_error_clear_countdown_ms
    }

    #[cfg(test)]
    pub(crate) fn set_phase_advance_for_test(
        &mut self,
        active: bool,
        base_register: u32,
        last_register: u32,
    ) {
        self.phase_advance_was_active = active;
        self.phase_advance_base_register = base_register;
        self.phase_advance_last_register = last_register;
    }

    #[cfg(test)]
    pub(crate) fn mark_phase_advance_restored_for_test(&mut self) {
        self.mark_phase_advance_restored();
    }

    #[cfg(test)]
    pub(crate) fn phase_advance_active_for_test(&self) -> bool {
        self.phase_advance_was_active
    }

    #[cfg(test)]
    pub(crate) fn phase_advance_last_register_for_test(&self) -> u32 {
        self.phase_advance_last_register
    }

    #[cfg(test)]
    pub(crate) fn pending_calibrate_inner_gains_for_test(
        &self,
    ) -> Option<crate::pid::InnerLoopGains> {
        self.commission_engine.as_ref().and_then(
            crate::commissioning::CommissioningEngine::calibrate_entry_inner_gains_for_test,
        )
    }

    #[cfg(test)]
    pub(crate) fn position_lead_applied_units_for_test(&self) -> i32 {
        self.position_lead_applied_units
    }

    #[cfg(test)]
    pub(crate) fn torque_flux_offset_was_active_for_test(&self) -> bool {
        self.torque_flux_offset_was_active
    }

    #[cfg(all(test, feature = "trace"))]
    pub(crate) fn set_trace_control_tick_for_test(&mut self, value: u32) {
        self.trace.control_tick = value;
    }

    #[cfg(all(test, feature = "trace"))]
    pub(crate) fn set_trace_phase_advance_for_test(
        &mut self,
        emit_once: bool,
        last_flags: u8,
        last_counts: i16,
    ) {
        self.trace.phase_advance_emit_once = emit_once;
        self.trace.phase_advance_last_flags = last_flags;
        self.trace.phase_advance_last_counts = last_counts;
    }
}

/// TMC4671 STATUS_FLAGS bit 26: ADC current clipped (overcurrent).
const STATUS_ADC_I_CLIPPED: u32 = 1 << 26;

/// Whether a STATUS_FLAGS read indicates ADC current clipping (overcurrent).
///
/// A failed register read is never coerced into a clean STATUS_FLAGS value:
/// it carries no evidence either way and must report `false` here, the same
/// as a genuine non-clipped read, rather than being silently indistinguishable
/// from a healthy chip at the call site.
pub fn is_adc_current_clipped(status_flags: Result<u32, ()>) -> bool {
    matches!(status_flags, Ok(flags) if flags & STATUS_ADC_I_CLIPPED != 0)
}

/// Which emergency-disable shutdown message a STATUS_FLAGS poll (or set of
/// polls, for a multi-channel board) selects.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum EmergencyDisableFault {
    /// ADC current clipped (overcurrent) — the most specific, actionable
    /// fault; wins over the other two regardless of poll order.
    AdcCurrentClipped,
    /// The STATUS_FLAGS read itself failed, carrying no fault evidence
    /// either way. Distinct from [`Self::Generic`] so a transport error is
    /// never indistinguishable from a genuine unclassified fault bit.
    StatusReadFailed,
    /// A non-ADC-clip fault bit was set, or all attempted reads succeeded
    /// with no bit set (reaching the emergency-disable path at all implies
    /// something else already triggered it).
    Generic,
}

/// Classifies one STATUS_FLAGS poll outcome.
pub fn classify_emergency_disable_fault(status_flags: Result<u32, ()>) -> EmergencyDisableFault {
    if is_adc_current_clipped(status_flags) {
        EmergencyDisableFault::AdcCurrentClipped
    } else if status_flags.is_err() {
        EmergencyDisableFault::StatusReadFailed
    } else {
        EmergencyDisableFault::Generic
    }
}

/// Combines one channel's classification into a running multi-channel
/// aggregate, in priority order: [`EmergencyDisableFault::AdcCurrentClipped`]
/// beats [`EmergencyDisableFault::StatusReadFailed`] beats
/// [`EmergencyDisableFault::Generic`], regardless of which channel produced
/// which outcome or the order channels are folded in.
pub fn combine_emergency_disable_fault(
    so_far: EmergencyDisableFault,
    channel: EmergencyDisableFault,
) -> EmergencyDisableFault {
    use EmergencyDisableFault::{AdcCurrentClipped, Generic, StatusReadFailed};
    match (so_far, channel) {
        (AdcCurrentClipped, _) | (_, AdcCurrentClipped) => AdcCurrentClipped,
        (StatusReadFailed, _) | (_, StatusReadFailed) => StatusReadFailed,
        (Generic, Generic) => Generic,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tmc::tmc_resistance_test;

    #[test]
    fn classify_request_delegates_resistance_test() {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let request = tmc_resistance_test(0, 7, 1);

        let actions = runtime.classify_request(request.clone());

        assert_eq!(actions.len(), 1);
        let BoardAction::DelegatedDiagnostic(diagnostic) = &actions[0] else {
            panic!("expected delegated diagnostic action");
        };
        assert_eq!(diagnostic.request, request);
    }

    #[test]
    fn control_period_us_returns_idle_period_when_disabled() {
        let runtime = TmcControlRuntime::new_for_channel(0);
        let tmc = crate::state::tmc::TmcChannelState::new();
        let stepper = crate::state::stepper::StepperChannelState::new();
        let refs = TmcControlStateRefs::new(&tmc, &stepper);

        // Default-constructed runtime has motor_enabled = false.
        assert_eq!(
            runtime.control_period_us(refs),
            crate::tmc::VELOCITY_TRANSIENT_IDLE_PERIOD_US
        );
    }

    #[test]
    fn control_period_us_idle_period_bypasses_rate_normalization() {
        // Regression guard: routing the idle period through velocity_transient_period_us
        // would silently clamp 100 Hz up to the 1000 Hz floor, defeating
        // the fix. This asserts the idle period independently of whatever
        // normalize_velocity_transient_rate_hz's clamp range is today.
        assert_eq!(crate::tmc::VELOCITY_TRANSIENT_IDLE_PERIOD_US, 10_000);
        assert_ne!(
            crate::tmc::VELOCITY_TRANSIENT_IDLE_PERIOD_US,
            crate::tmc::velocity_transient_period_us(100)
        );
    }
}
