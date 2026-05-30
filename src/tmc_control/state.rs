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
            | TmcRequestKind::VoltageStepTest { .. } => {
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
        let result = {
            let engine = self.commission_engine.as_mut()?;
            engine.step(backend).await
        };
        // Construct DispatchState from direct field references so that Rust can
        // see the field borrows as disjoint from &mut self.commission_engine.
        // Calling self.dispatch_state(...) would re-borrow all of &mut self and
        // create a conflict with the engine slot above.
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
}
