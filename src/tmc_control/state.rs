//! Persistent state for the shared TMC control runtime.

use crate::commissioning::CommissioningEngine;
use crate::pid::InnerLoopGains;

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
}
