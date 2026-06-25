//! Persistent state for the shared TMC control runtime.

use core::sync::atomic::Ordering::Relaxed;

use crate::commission_dispatch::DispatchState;
use crate::commissioning::CommissioningEngine;
use crate::pid::InnerLoopGains;
use crate::state::stepper::StepperChannelState;
use crate::state::tmc::TmcChannelState;
use crate::tmc::{TmcRequest, TmcRequestKind};
use crate::tmc_control::actions::{
    BoardAction, BoardActions, DelegatedDiagnostic, ShutdownReason, StepperAction, push_action,
};
use crate::tmc_control::config::{TmcControlConfig, TmcControlHomingRefs, TmcControlStateRefs};
use crate::tmc_control::signals::TmcControlSignals;

/// Trace-only cached state. Core safety state stays in `TmcControlRuntime`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct TmcControlTraceState {
    /// Trace control tick counter.
    pub control_tick: u32,
    /// Last command velocity reference emitted to trace sampling.
    pub cached_velocity_reference: i32,
    /// Last velocity feedforward offset emitted to trace sampling.
    pub cached_velocity_offset: i32,
    /// Last transient velocity feedforward offset emitted to trace sampling.
    pub cached_velocity_transient_offset: i32,
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
}

/// Long-lived board-agnostic TMC control state.
pub struct TmcControlRuntime {
    channel: u8,
    motor_enabled: bool,
    calibrated: bool,
    stored_gains: Option<InnerLoopGains>,
    commission_engine: Option<CommissioningEngine>,
    shutdown_handled: bool,
    velocity_scale: u32,
    feedforward_was_active: bool,
    torque_flux_offset_was_active: bool,
    position_lead_applied_scaled: i32,
    phase_advance_was_active: bool,
    phase_advance_base_register: u32,
    phase_advance_last_register: u32,
    pid_error_clear_countdown_ms: u8,
    status_poll_counter: u16,
    slow_control_accum_us: u16,
    #[cfg(feature = "trace")]
    trace: TmcControlTraceState,
}

impl TmcControlRuntime {
    /// Create a runtime for one board-local TMC channel.
    pub const fn new_for_channel(channel: u8) -> Self {
        Self {
            channel,
            motor_enabled: false,
            calibrated: false,
            stored_gains: None,
            commission_engine: None,
            shutdown_handled: false,
            velocity_scale: 0,
            feedforward_was_active: false,
            torque_flux_offset_was_active: false,
            position_lead_applied_scaled: 0,
            phase_advance_was_active: false,
            phase_advance_base_register: 0,
            phase_advance_last_register: 0,
            pid_error_clear_countdown_ms: 0,
            status_poll_counter: 0,
            slow_control_accum_us: 0,
            #[cfg(feature = "trace")]
            trace: TmcControlTraceState::new(),
        }
    }

    /// Return this runtime's board-local TMC channel.
    pub const fn channel(&self) -> u8 {
        self.channel
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
        self.commission_engine = None;
        self.shutdown_handled = false;
        self.velocity_scale = 0;
        self.feedforward_was_active = false;
        self.torque_flux_offset_was_active = false;
        self.position_lead_applied_scaled = 0;
        self.phase_advance_was_active = false;
        self.phase_advance_base_register = 0;
        self.phase_advance_last_register = 0;
        self.pid_error_clear_countdown_ms = 0;
        self.status_poll_counter = 0;
        self.slow_control_accum_us = 0;
        #[cfg(feature = "trace")]
        {
            self.trace = TmcControlTraceState::new();
        }
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
    /// channel-local state, clears the fault latch, and emits `StopNow` so the
    /// board shell can stop physical stepping.
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

        self.reset_runtime_state();
        Self::reset_channel_state(tmc, stepper);
        signals.fault_active.store(false, Relaxed);
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
        self.commission_engine = None;
        self.motor_enabled = false;
        self.calibrated = false;
        self.feedforward_was_active = false;
        self.torque_flux_offset_was_active = false;
        self.pid_error_clear_countdown_ms = 0;
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
                self.commission_engine = None;
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
    /// `AdcCurrentClipped` when bit 26 of `status_flags` is set, or
    /// `TmcFault` otherwise.
    pub fn poll_emergency_disable(
        &mut self,
        signals: &TmcControlSignals,
        status_flags: u32,
    ) -> BoardActions {
        let mut actions = BoardActions::new();
        if signals.disable_signal.try_take().is_none() {
            return actions;
        }

        signals.fault_active.store(false, Relaxed);
        self.motor_enabled = false;
        self.calibrated = false;
        push_action(&mut actions, BoardAction::Stepper(StepperAction::StopNow));
        let reason = if status_flags & (1 << 26) != 0 {
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
            | TmcRequestKind::ResistanceTest { .. }
            | TmcRequestKind::ImpedanceTest { .. } => {
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

    /// Install a new commissioning engine, replacing any existing one.
    pub fn set_commission_engine(&mut self, engine: CommissioningEngine) {
        self.commission_engine = Some(engine);
    }

    /// Remove and return the active commissioning engine, if any.
    pub fn take_commission_engine(&mut self) -> Option<CommissioningEngine> {
        self.commission_engine.take()
    }

    /// Clear the calibrated flag before launching a (re)calibration run.
    pub fn clear_calibrated(&mut self) {
        self.calibrated = false;
    }

    /// Advance the active commissioning engine by one `step()` call.
    ///
    /// Returns `None` immediately when no engine is installed. This helper does
    /// not dispatch the result, emit actions, restore hardware state, or clear
    /// runtime flags; callers that need those side effects must pass the
    /// returned [`crate::commissioning::types::StepResult`] into
    /// [`Self::drive_commissioning_result`].
    ///
    /// # Parameters
    /// - `backend`: SPI + GPIO backend for register access.
    pub async fn step_commissioning<B>(
        &mut self,
        backend: &mut B,
    ) -> Option<crate::commissioning::types::StepResult>
    where
        B: crate::commissioning::types::CommissioningBackend,
    {
        let engine = self.commission_engine.as_mut()?;
        Some(engine.step(backend).await)
    }

    /// Dispatch one commissioning step result through shared post-processing.
    ///
    /// Returns `None` immediately when no engine is installed. Dispatch may
    /// restore safe state, update runtime calibration/enabled flags, re-apply
    /// filters, and return board-visible actions.
    ///
    /// # Parameters
    /// - `result`: Result returned by [`Self::step_commissioning`].
    /// - `backend`: SPI + GPIO backend for register access.
    /// - `tmc`: Static reference to the TMC channel atomics for this motor.
    /// - `stepper`: Static reference to the stepper channel atomics.
    /// - `config`: Board-level TMC control configuration (provides `pwm_maxcnt`).
    pub async fn drive_commissioning_result<B>(
        &mut self,
        result: crate::commissioning::types::StepResult,
        backend: &mut B,
        tmc: &'static crate::state::tmc::TmcChannelState,
        stepper: &'static crate::state::stepper::StepperChannelState,
        config: &crate::tmc_control::config::TmcControlConfig,
    ) -> Option<crate::tmc_control::actions::CommissionTickAction>
    where
        B: crate::commissioning::types::CommissioningBackend,
    {
        self.commission_engine.as_ref()?;

        // Construct DispatchState from direct field references so that Rust can
        // see the field borrows as disjoint from &mut self.commission_engine.
        // Calling self.dispatch_state would re-borrow all of &mut self and
        // create a conflict with the engine slot below.
        let mut dispatch = crate::commission_dispatch::DispatchState::new(
            tmc,
            stepper,
            &mut self.calibrated,
            &mut self.motor_enabled,
            &mut self.stored_gains,
        );
        let outcome = crate::commission_dispatch::drive_commission_result(
            result,
            &mut self.commission_engine,
            backend,
            &mut dispatch,
            config.tmc.pwm_maxcnt,
        )
        .await;

        match outcome {
            crate::commission_dispatch::CommissionOutcome::Continue => {
                Some(crate::tmc_control::actions::CommissionTickAction::Continue)
            }
            crate::commission_dispatch::CommissionOutcome::PhaseChanged {
                oid,
                phase_id,
                engine_kind,
            } => Some(
                crate::tmc_control::actions::CommissionTickAction::PhaseChanged {
                    oid,
                    phase_id,
                    engine_kind,
                },
            ),
            crate::commission_dispatch::CommissionOutcome::Terminal { reply, actions } => {
                Some(crate::tmc_control::actions::CommissionTickAction::Terminal { reply, actions })
            }
        }
    }

    /// Tick the active commissioning engine once and return the outcome.
    ///
    /// Drives `engine.step(backend)` then feeds the result into
    /// `drive_commission_result`, which performs any required SPI restore,
    /// atomic stores, filter re-apply, and arm/disarm work. Returns `None`
    /// immediately when no engine is installed.
    ///
    /// # Parameters
    /// - `backend`: SPI + GPIO backend for register access.
    /// - `tmc`: Static reference to the TMC channel atomics for this motor.
    /// - `stepper`: Static reference to the stepper channel atomics.
    /// - `config`: Board-level TMC control configuration (provides `pwm_maxcnt`).
    pub async fn tick_commissioning<B>(
        &mut self,
        backend: &mut B,
        tmc: &'static crate::state::tmc::TmcChannelState,
        stepper: &'static crate::state::stepper::StepperChannelState,
        config: &crate::tmc_control::config::TmcControlConfig,
    ) -> Option<crate::tmc_control::actions::CommissionTickAction>
    where
        B: crate::commissioning::types::CommissioningBackend,
    {
        let result = self.step_commissioning(backend).await?;
        self.drive_commissioning_result(result, backend, tmc, stepper, config)
            .await
    }

    /// Prepare an enable request up to the point where SPI or backend work begins.
    ///
    /// Returns an empty action list when the motor is already enabled, or when
    /// the motor is uncalibrated and `auto_calibrate_on_enable_allowed` is not
    /// set. Returns an [`StepperAction::ArmCheck`] action when the motor is
    /// calibrated, so the board shell can drive the electrical arm verification
    /// and step/dir arming.
    ///
    /// This helper intentionally does not energize the motor; the caller's SPI
    /// path and the auto-calibrate commissioning construction remain in the
    /// request handler.
    pub fn prepare_enable_request(&mut self, tmc: &TmcChannelState) -> BoardActions {
        let mut actions = BoardActions::new();
        if self.motor_enabled {
            return actions;
        }
        if !self.calibrated {
            if !tmc
                .auto_calibrate_on_enable_allowed
                .load(core::sync::atomic::Ordering::Relaxed)
            {
                return actions;
            }
            // Auto-calibrate construction stays in the request handler.
            // This preflight helper must not energize the motor.
            return actions;
        }
        push_action(
            &mut actions,
            BoardAction::Stepper(StepperAction::ArmCheck {
                post_action: crate::commission_dispatch::StepperPostAction::ArmPreservePosition,
            }),
        );
        actions
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

    /// Return the velocity feedforward scale factor.
    ///
    /// Zero until encoder PPR arrives and `compute_velocity_scale_factor` runs.
    pub fn velocity_scale(&self) -> u32 {
        self.velocity_scale
    }

    /// Set the velocity feedforward scale factor.
    ///
    /// Called when encoder PPR is received to store the pre-computed constant
    /// used by the velocity transient feedforward path.
    pub fn set_velocity_scale(&mut self, value: u32) {
        self.velocity_scale = value;
    }

    /// Return whether velocity feedforward was active on the previous control tick.
    pub const fn feedforward_was_active(&self) -> bool {
        self.feedforward_was_active
    }

    /// Store whether velocity feedforward is active for the current control tick.
    pub fn set_feedforward_was_active(&mut self, active: bool) {
        self.feedforward_was_active = active;
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
        let reference = if !self.motor_enabled
            || self.velocity_scale == 0
            || self.commission_engine.is_some()
            || homing_active
            || reset
        {
            0
        } else {
            let interval = refs.stepper.interval.load(Relaxed);
            let forward = refs.stepper.direction.load(Relaxed);
            crate::tmc::compute_command_velocity_reference(self.velocity_scale, interval, forward)
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
    /// Individual SPI operations are best-effort to preserve existing board
    /// behavior; write/read failures do not abort later control work.
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
            && self.velocity_scale != 0
            && refs.tmc.velocity_feedforward.load(Relaxed)
            && self.commission_engine.is_none();
        let velocity_transient_active = self.motor_enabled
            && self.velocity_scale != 0
            && refs.tmc.velocity_transient_feedforward.load(Relaxed)
            && self.commission_engine.is_none();
        let accel_ff_active = self.motor_enabled
            && self.velocity_scale != 0
            && refs.tmc.accel_feedforward.load(Relaxed)
            && self.commission_engine.is_none();
        let decoupling_ff_active = self.motor_enabled
            && self.velocity_scale != 0
            && refs.tmc.decoupling_feedforward.load(Relaxed)
            && self.commission_engine.is_none();

        let velocity_offset_active = ff_active || velocity_transient_active;
        if velocity_offset_active {
            let interval = refs.stepper.interval.load(Relaxed);
            let interval_add = refs
                .stepper
                .interval_add
                .load(Relaxed)
                .clamp(i16::MIN as i32, i16::MAX as i32) as i16;
            let forward = refs.stepper.direction.load(Relaxed);
            let base_offset = if ff_active && !homing_active && !reset {
                let multiplier = refs.tmc.velocity_feedforward_multiplier.load(Relaxed);
                crate::tmc::compute_velocity_offset_scaled(
                    self.velocity_scale,
                    interval,
                    forward,
                    multiplier,
                )
            } else {
                0
            };
            let transient_offset = if velocity_transient_active && !homing_active && !reset {
                crate::tmc::compute_velocity_transient_offset(
                    crate::tmc::VelocityTransientFeedforwardInput {
                        scale_factor: self.velocity_scale,
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
            let offset = base_offset.saturating_add(transient_offset);

            let _ = tmc_spi
                .write_register(foci_tmc4671::registers::PID_VELOCITY_OFFSET, offset as u32)
                .await;
            #[cfg(feature = "trace")]
            {
                let trace = self.trace_state_mut();
                trace.cached_velocity_offset = offset;
                trace.cached_velocity_transient_offset = transient_offset;
            }
        } else if self.feedforward_was_active {
            let _ = tmc_spi
                .write_register(foci_tmc4671::registers::PID_VELOCITY_OFFSET, 0)
                .await;
            #[cfg(feature = "trace")]
            {
                let trace = self.trace_state_mut();
                trace.cached_velocity_offset = 0;
                trace.cached_velocity_transient_offset = 0;
            }
        }
        self.feedforward_was_active = velocity_offset_active;

        if !slow_control_tick {
            return Ok(());
        }

        if decoupling_ff_active {
            let max_offset = refs.tmc.decoupling_max_offset.load(Relaxed);
            let (torque_offset, flux_offset) = if homing_active || reset {
                (0, 0)
            } else {
                let interval = refs.stepper.interval.load(Relaxed);
                let forward = refs.stepper.direction.load(Relaxed);
                let target_velocity = crate::tmc::compute_command_velocity_reference(
                    self.velocity_scale,
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
            let _ = tmc_spi
                .write_register(foci_tmc4671::registers::PID_TORQUE_FLUX_OFFSET, offset_reg)
                .await;
        } else if accel_ff_active {
            let accel_gain_permille = refs.tmc.accel_feedforward_accel_gain_permille.load(Relaxed);
            let decel_gain_permille = refs.tmc.accel_feedforward_decel_gain_permille.load(Relaxed);
            let torque_limit = refs.tmc.current_limit_reg.load(Relaxed);
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
                crate::tmc::compute_accel_torque_offset_split(
                    self.velocity_scale,
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
            let _ = tmc_spi
                .write_register(foci_tmc4671::registers::PID_TORQUE_FLUX_OFFSET, offset_reg)
                .await;
        } else if self.torque_flux_offset_was_active {
            let _ = tmc_spi
                .write_register(foci_tmc4671::registers::PID_TORQUE_FLUX_OFFSET, 0)
                .await;
            #[cfg(feature = "trace")]
            {
                let trace = self.trace_state_mut();
                trace.cached_accel_torque_offset = 0;
                trace.cached_accel_gain_permille = 0;
                trace.cached_accel_flags = 0;
            }
        }
        self.torque_flux_offset_was_active = accel_ff_active || decoupling_ff_active;

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
            && self.velocity_scale != 0
            && refs.tmc.phase_advance.load(Relaxed)
            && self.commission_engine.is_none();
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
            && self.velocity_scale != 0
            && refs.tmc.position_lead.load(Relaxed)
            && self.commission_engine.is_none()
            && !homing_active;
        if position_lead_active || self.position_lead_applied_scaled != 0 {
            let interval = refs.stepper.interval.load(Relaxed);
            let forward = refs.stepper.direction.load(Relaxed);
            let gain_permille = refs.tmc.position_lead_gain_permille.load(Relaxed);
            let max_counts = refs.tmc.position_lead_max_counts.load(Relaxed);
            let lead_counts = if !position_lead_active || reset || interval == 0 {
                0
            } else {
                crate::tmc::compute_position_lead_counts(
                    self.velocity_scale,
                    interval,
                    forward,
                    gain_permille,
                    max_counts,
                )
            };
            let step_width = refs.tmc.step_width_value.load(Relaxed);
            if step_width > 0 {
                match tmc_spi
                    .read_register(foci_tmc4671::registers::PID_POSITION_TARGET)
                    .await
                {
                    Ok(current_target) => {
                        let (target, applied_scaled) = crate::tmc::compute_position_lead_target(
                            current_target,
                            self.position_lead_applied_scaled,
                            lead_counts,
                            step_width as u32,
                        );
                        if applied_scaled != self.position_lead_applied_scaled
                            && tmc_spi
                                .write_register(
                                    foci_tmc4671::registers::PID_POSITION_TARGET,
                                    target,
                                )
                                .await
                                .is_ok()
                        {
                            self.position_lead_applied_scaled = applied_scaled;
                        }
                    }
                    Err(_) => {
                        defmt::warn!("position lead: PID_POSITION_TARGET read failed");
                    }
                }
            }
        }
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
    pub async fn apply_homing_channel_tick<B>(
        &mut self,
        tmc_spi: &mut foci_tmc4671::spi::Tmc4671Spi<B>,
        refs: TmcControlStateRefs<'_>,
        homing_state: u8,
    ) -> HomingMaintenanceResult
    where
        B: embedded_hal_async::spi::SpiDevice<u8>,
    {
        match homing_state {
            1 => {
                if !self.restore_phase_advance_now(tmc_spi).await {
                    defmt::warn!("phase advance: failed to restore ABN offset before homing");
                    return HomingMaintenanceResult::RetryCurrentState;
                }

                let pos_p = refs.tmc.position_p_value.load(Relaxed);
                let reg = (pos_p as u32) << 16;
                let _ = tmc_spi
                    .write_register(foci_tmc4671::registers::PID_POSITION_P_POSITION_I, reg)
                    .await;
                defmt::info!("homing: position I suppressed");
                HomingMaintenanceResult::ReadyToAdvance
            }
            3 => {
                let pos_p = refs.tmc.position_p_value.load(Relaxed);
                let pos_i = refs.tmc.position_i_value.load(Relaxed);
                let reg = ((pos_p as u32) << 16) | pos_i as u32;
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
                HomingMaintenanceResult::ReadyToAdvance
            }
            _ => HomingMaintenanceResult::ReadyToAdvance,
        }
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
            3 => homing.state.store(0, Relaxed),
            _ => {}
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
        homing_snapshot: u8,
    ) -> HomingMaintenanceResult
    where
        B: embedded_hal_async::spi::SpiDevice<u8>,
    {
        self.apply_pid_error_settle_tick(tmc_spi, refs).await;
        self.apply_homing_channel_tick(tmc_spi, refs, homing_snapshot)
            .await
    }

    /// Apply slow maintenance tick work.
    ///
    /// This covers host-requested PID error clear settling, homing position-I
    /// suppression/restoration, and periodic `STATUS_FLAGS` polling. USB trace
    /// transport remains board-local.
    ///
    /// # Parameters
    /// - `tmc_spi`: TMC4671 SPI transport for register reads and writes.
    /// - `refs`: Borrowed channel atomics for this motor.
    /// - `config`: Runtime control configuration, reserved for future slow
    ///   maintenance policies.
    /// - `homing`: Board-owned homing state atomics.
    pub async fn apply_slow_maintenance_tick<B>(
        &mut self,
        tmc_spi: &mut foci_tmc4671::spi::Tmc4671Spi<B>,
        refs: TmcControlStateRefs<'_>,
        config: &TmcControlConfig,
        homing: TmcControlHomingRefs<'_>,
    ) -> BoardActions
    where
        B: embedded_hal_async::spi::SpiDevice<u8>,
    {
        let _ = config;
        let mut actions = BoardActions::new();

        let homing_snapshot = homing.state.load(Relaxed);
        let homing_ready = self
            .apply_slow_channel_maintenance_tick(tmc_spi, refs, homing_snapshot)
            .await
            == HomingMaintenanceResult::ReadyToAdvance;
        if !homing_ready {
            return actions;
        }
        Self::advance_homing_state_after_channels(homing, homing_snapshot, true);

        self.status_poll_counter += 1;
        if self.status_poll_counter >= 1000 {
            self.status_poll_counter = 0;
            if let Ok(flags) = tmc_spi
                .read_register(foci_tmc4671::registers::STATUS_FLAGS)
                .await
            {
                const PID_LIMIT_MASK: u32 = 0x0000_FFFF;
                const PWM_LIMIT_MASK: u32 = (1 << 24) | (1 << 25);
                const FAULT_MASK: u32 = 1 << 26;

                if flags & FAULT_MASK != 0 {
                    defmt::error!("TMC4671 STATUS fault: {:#010x}", flags);
                    push_action(&mut actions, BoardAction::SignalEmergencyDisable);
                } else if self.motor_enabled && flags & (PID_LIMIT_MASK | PWM_LIMIT_MASK) != 0 {
                    defmt::warn!("TMC4671 STATUS: {:#010x}", flags);
                }
            }
        }

        actions
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
    /// Returns the fast transient period when all of the following hold:
    /// - the motor is electrically enabled,
    /// - `velocity_scale` is non-zero (encoder PPR has been received),
    /// - no commissioning engine is active, and
    /// - the `velocity_transient_feedforward` atomic in channel state is `true`.
    ///
    /// Otherwise returns `velocity_transient_period_us(VELOCITY_TRANSIENT_DEFAULT_RATE_HZ)`.
    ///
    /// # Parameters
    /// - `refs`: Borrowed channel atomics for this motor.
    pub fn control_period_us(&self, refs: TmcControlStateRefs<'_>) -> u32 {
        let fast_active = self.motor_enabled
            && self.velocity_scale != 0
            && self.commission_engine.is_none()
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

    /// Return the fastest requested control period across channel runtimes.
    pub fn minimum_control_period_us(
        runtimes: &[TmcControlRuntime],
        refs: &[TmcControlStateRefs<'_>],
    ) -> u32 {
        runtimes
            .iter()
            .zip(refs.iter().copied())
            .map(|(runtime, refs)| runtime.control_period_us(refs))
            .min()
            .unwrap_or(1_000)
    }

    /// Update a caller-owned slow-control accumulator.
    ///
    /// This is the board-level form used by multi-channel boards that need one
    /// shared slow tick for all channels.
    pub fn update_slow_control_accumulator(accum_us: &mut u16, control_period_us: u32) -> bool {
        if control_period_us >= 1_000 {
            *accum_us = 0;
            return true;
        }
        *accum_us = accum_us.saturating_add(u16::try_from(control_period_us).unwrap_or(u16::MAX));
        if *accum_us >= 1_000 {
            *accum_us -= 1_000;
            true
        } else {
            false
        }
    }

    /// Update the slow-control decimation accumulator and return whether slow
    /// work is due this tick.
    ///
    /// The slow path fires at approximately 1 kHz regardless of the fast
    /// transient period. When `control_period_us` is already at or above 1 ms
    /// the accumulator is reset and the slow path fires immediately. For faster
    /// periods the method accumulates microseconds and fires once the total
    /// reaches 1 000 µs, subtracting 1 000 from the accumulator rather than
    /// clearing it so that any overshoot carries forward.
    ///
    /// # Parameters
    /// - `control_period_us`: Current control period in microseconds.
    pub fn update_slow_control(&mut self, control_period_us: u32) -> bool {
        Self::update_slow_control_accumulator(&mut self.slow_control_accum_us, control_period_us)
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
    pub(crate) fn set_velocity_scale_for_test(&mut self, value: u32) {
        self.velocity_scale = value;
    }

    #[cfg(test)]
    pub(crate) fn velocity_scale_for_test(&self) -> u32 {
        self.velocity_scale
    }

    #[cfg(test)]
    pub(crate) fn set_slow_control_accum_us_for_test(&mut self, value: u16) {
        self.slow_control_accum_us = value;
    }

    #[cfg(test)]
    pub(crate) fn slow_control_accum_us_for_test(&self) -> u16 {
        self.slow_control_accum_us
    }

    #[cfg(test)]
    pub(crate) fn set_status_poll_counter_for_test(&mut self, value: u16) {
        self.status_poll_counter = value;
    }

    #[cfg(test)]
    pub(crate) fn pid_error_clear_countdown_for_test(&self) -> u8 {
        self.pid_error_clear_countdown_ms
    }

    #[cfg(test)]
    pub(crate) fn update_slow_control_for_test(&mut self, control_period_us: u32) -> bool {
        self.update_slow_control(control_period_us)
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
    pub(crate) fn position_lead_applied_scaled_for_test(&self) -> i32 {
        self.position_lead_applied_scaled
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tmc::tmc_impedance_test;

    #[test]
    fn classify_request_delegates_impedance_test() {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let request = tmc_impedance_test(0, 7, 1);

        let actions = runtime.classify_request(request.clone());

        assert_eq!(actions.len(), 1);
        let BoardAction::DelegatedDiagnostic(diagnostic) = &actions[0] else {
            panic!("expected delegated diagnostic action");
        };
        assert_eq!(diagnostic.request, request);
    }
}
