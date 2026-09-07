//! TMC4671 protocol state primitives. The mpsc channel sender
//! (TMC_REQUEST_SENDER) stays in the board crate — its Sender type's
//! generic params bind to the board's RTIC app declaration.

use core::sync::atomic::{AtomicBool, AtomicI32, AtomicU8, AtomicU16, AtomicU32, Ordering};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use foci_tmc4671::current_limit::PidTorqueFluxLimit;
use foci_tmc4671::pid::{PidGainOutOfRange, PidIGain, PidPGain};

use crate::pid::OuterLoopGains;

/// Maximum number of board-local TMC4671 channels supported by shared firmware.
pub const MAX_TMC_CHANNELS: usize = 2;
/// Conservative operational PIDOUT_UQ_UD_LIMITS value used after closed-loop entry.
pub const DEFAULT_OPERATIONAL_PIDOUT_UQ_UD_LIMIT: u32 = 16_000;
/// Lowest accepted raw PIDOUT_UQ_UD_LIMITS value.
pub const MIN_OPERATIONAL_PIDOUT_UQ_UD_LIMIT: u32 = 0;
/// Highest accepted diagnostic PIDOUT_UQ_UD_LIMITS value.
pub const MAX_OPERATIONAL_PIDOUT_UQ_UD_LIMIT: u32 = 32_767;

const OPTIONAL_FILTER_HZ_UNSET: u32 = u32::MAX;

/// Atomic storage for an optional biquad filter cutoff in Hz.
///
/// `None` means no host override has been configured, `Some(0)` means the host
/// explicitly disabled the filter, and `Some(hz)` means the host supplied a
/// concrete cutoff.
pub struct AtomicOptionalFilterHz {
    raw: AtomicU32,
}

impl AtomicOptionalFilterHz {
    /// Create optional filter storage with the given initial value.
    pub const fn new(filter_hz: Option<u16>) -> Self {
        Self {
            raw: AtomicU32::new(Self::encode(filter_hz)),
        }
    }

    /// Load the optional cutoff value.
    pub fn load(&self, ordering: Ordering) -> Option<u16> {
        match self.raw.load(ordering) {
            OPTIONAL_FILTER_HZ_UNSET => None,
            raw => Some(raw as u16),
        }
    }

    /// Load the cutoff value, treating an unset filter as disabled.
    pub fn load_or_zero(&self, ordering: Ordering) -> u16 {
        self.load(ordering).unwrap_or(0)
    }

    /// Store the optional cutoff value.
    pub fn store(&self, filter_hz: Option<u16>, ordering: Ordering) {
        self.raw.store(Self::encode(filter_hz), ordering);
    }

    const fn encode(filter_hz: Option<u16>) -> u32 {
        match filter_hz {
            Some(hz) => hz as u32,
            None => OPTIONAL_FILTER_HZ_UNSET,
        }
    }
}

/// Runtime state for one board-local TMC4671 channel.
pub struct TmcChannelState {
    /// Validated current limit for PID_TORQUE_FLUX_LIMITS (0x5E).
    current_limit_value: AtomicU16,
    /// Set by P1 after calibration, cleared on disable/fault.
    pub motor_armed: AtomicBool,
    /// Mirrors the physical enable-pin GPIO level. Set only from
    /// `SharedCommissioningBackend::set_commission_enable_pin`, the single
    /// choke point that actually writes the pin in a production build —
    /// covers both the runtime arm/disarm path and commissioning's direct
    /// `set_enable_pin` calls.
    pub enable_pin_asserted: AtomicBool,
    /// Captured at commission time from the actual `MOTOR_TYPE_N_POLE_PAIRS`
    /// write. Zero means "no commission has happened yet — sentinel check
    /// inactive". Used by `arm_motor_electrical` to detect silent chip
    /// resets between commission and arm.
    pub motor_sentinel: AtomicU32,
    /// Set by every `restore_safe_state_via_backend` pass, whether or not it
    /// had failures — a restore alone does not confirm the chip is safe to
    /// arm, only a fresh sentinel pass does. Cleared only by
    /// `arm_motor_electrical` immediately after `verify_motor_sentinel`
    /// succeeds, before any settling write or enable-pin assertion, or by a
    /// full `TmcChannelState::reset()`. This is the compound "safe-state
    /// pass AND fresh sentinel pass" latch the re-arm gate is built on; do
    /// not clear it anywhere else.
    pub safe_state_incomplete: AtomicBool,
    /// Set when the most recent `restore_safe_state_via_backend` pass had
    /// one or more verified-write or PID-error-sum-clear failures. Checked
    /// by `arm_motor_electrical`, which refuses to arm — without even
    /// attempting the sentinel check — while this is set. Cleared by the
    /// next restore pass that completes with zero failures. Distinct from
    /// `safe_state_incomplete`: this only reflects the last restore pass
    /// itself, not whether a sentinel pass has confirmed it since.
    pub safe_state_last_pass_failed: AtomicBool,
    /// Failed-write count from the most recent safe-state restore pass.
    pub safe_state_failed_writes: AtomicU8,
    /// Address of the first register or sub-register write that failed
    /// during the most recent safe-state restore pass. `0xFF` means none.
    pub safe_state_first_failed_register: AtomicU8,
    /// If the first failure was a PID error-sum clear, the logical
    /// sub-register selector (4=torque, 5=flux, 6=velocity, 7=position).
    /// All four sums share the same physical selector/data registers, so
    /// `safe_state_first_failed_register` alone cannot distinguish them.
    /// `0xFF` means the first failure was not a PID error-sum clear, or
    /// there was no failure.
    pub safe_state_first_failed_sub_addr: AtomicU8,
    /// Board-monotonic millisecond timestamp of the most recent safe-state
    /// restore pass. Diagnostic only: wraps every ~49 days.
    pub safe_state_failure_timestamp_ms: AtomicU32,
    /// Total number of completed safe-state restore passes, whether clean
    /// or not. Saturates rather than wraps. Never reset by a clean pass —
    /// only by [`TmcChannelState::reset`].
    pub safe_state_pass_count: AtomicU16,
    /// Whether firmware established the fixed Q8.8-P/Q4.12-I representation.
    pi_representation_established: AtomicBool,
    /// Configured mechanical ABN encoder pulses per revolution.
    pub encoder_ppr: AtomicU32,
    encoder_counts_per_rev: AtomicU32,
    tmc_grid: AtomicU32,
    step_width: AtomicU32,
    motion_scale_configured: AtomicBool,
    /// Set when a config setter has recorded a value the TMC task has not yet
    /// written to the chip.
    ///
    /// Config-time setters are pure state writes: they do not need to be
    /// ordered SPI transactions, so they record their value here instead of
    /// enqueueing a request. The TMC task drains this flag on its periodic
    /// control tick and applies the recorded state.
    ///
    /// This keeps the connect-time configuration burst entirely off the
    /// bounded TMC request channel. That burst can exceed `TMC_CHANNEL_SIZE`,
    /// and a dropped `#[klipper_command]` is an MCU shutdown.
    pub config_apply_pending: AtomicBool,
    /// Optional velocity biquad filter cutoff frequency in Hz.
    pub velocity_filter_hz: AtomicOptionalFilterHz,
    /// Optional torque biquad filter cutoff frequency in Hz.
    pub torque_filter_hz: AtomicOptionalFilterHz,
    /// Optional position target biquad filter cutoff frequency in Hz.
    pub position_filter_hz: AtomicOptionalFilterHz,
    /// Optional flux biquad filter cutoff frequency in Hz.
    pub flux_filter_hz: AtomicOptionalFilterHz,
    /// Velocity feedforward enable flag.
    pub velocity_feedforward: AtomicBool,
    /// Runtime gain in permille applied to the RPM-domain velocity feedforward
    /// offset (1000 = unity).
    pub velocity_feedforward_gain_permille: AtomicU16,
    /// Velocity-surface transient feedforward enable flag.
    pub velocity_transient_feedforward: AtomicBool,
    /// Transient feedforward lead time in microseconds.
    pub velocity_transient_lead_time_us: AtomicU16,
    /// Transient feedforward gain in permille.
    pub velocity_transient_gain_permille: AtomicU16,
    /// Maximum absolute transient contribution in raw velocity-offset counts.
    pub velocity_transient_max_offset: AtomicU16,
    /// Requested transient feedforward update rate in Hz.
    pub velocity_transient_rate_hz: AtomicU16,
    /// Acceleration feedforward enable flag.
    pub accel_feedforward: AtomicBool,
    /// Runtime gain in permille while step intervals shrink.
    pub accel_feedforward_accel_gain_permille: AtomicU16,
    /// Runtime gain in permille while step intervals grow.
    pub accel_feedforward_decel_gain_permille: AtomicU16,
    /// q/d decoupling proxy feedforward enable flag.
    pub decoupling_feedforward: AtomicBool,
    /// Identified count-space resistance, scaled by 1000.
    pub decoupling_r_int: AtomicU32,
    /// Identified count-space inductance in microseconds.
    pub decoupling_l_int: AtomicU32,
    /// Motor pole-pair count.
    pub decoupling_pole_pairs: AtomicU16,
    /// Position units per mechanical revolution.
    pub decoupling_position_units_per_rev: AtomicU32,
    /// PWM/current-loop update rate in Hz.
    pub decoupling_f_pwm_hz: AtomicU32,
    /// Maximum absolute decoupling current-target offset.
    pub decoupling_max_offset: AtomicU16,
    /// One-tick feedforward reset request after direction changes.
    pub feedforward_reset: AtomicBool,
    /// Position-target lead diagnostic enable flag.
    pub position_lead: AtomicBool,
    /// Position-target lead gain in permille of the velocity reference.
    pub position_lead_gain_permille: AtomicU16,
    /// Maximum absolute position-target lead in logical step counts.
    pub position_lead_max_counts: AtomicU16,
    /// Runtime commutation phase-advance diagnostic enable flag.
    pub phase_advance: AtomicBool,
    /// Signed gain in ppm of command velocity reference.
    pub phase_advance_gain_ppm: AtomicI32,
    /// Maximum absolute ABN PHI_E advance in electrical-angle counts.
    pub phase_advance_max_counts: AtomicU16,
    /// TMC velocity-reference deadband in counts.
    pub phase_advance_deadband: AtomicU16,
    /// PID error-sum clear request after motion drains.
    pub pid_error_sums_clear: AtomicBool,
    /// Configurable PID velocity limit.
    pub velocity_limit_cfg: AtomicU32,
    /// Configurable operational PIDOUT_UQ_UD_LIMITS value.
    pub voltage_limit_cfg: AtomicU32,
    /// Cached ADC_VM mode-5 startup offset.
    adc_vm_offset_raw: AtomicU16,
    /// Number of samples folded into the cached ADC_VM offset.
    adc_vm_offset_sample_count: AtomicU8,
    /// Whether the encoder direction is reversed.
    pub encoder_reversed: AtomicBool,
    /// Guard for single-in-flight commissioning on this channel.
    pub commission_in_flight: AtomicBool,
    /// Operator-requested cancel for this channel's in-flight commissioning
    /// run. Set by `foci_commission_cancel` (P2), observed by the
    /// commissioning engine's existing bounded wait checkpoints and by the
    /// mid-phase cancel synthesis path (P1). A fresh run start always resets
    /// this to `false` before the engine begins polling, closing the race
    /// where a stale write from a just-finished run's tail could otherwise
    /// cancel the next run (see `commission_dispatch/mod.rs`'s run-admission
    /// call sites).
    pub cancel_requested: AtomicBool,
    /// Host-controlled gate for raw-enable auto-calibration.
    pub auto_calibrate_on_enable_allowed: AtomicBool,
    /// Position P gain for cross-priority access (Q8.8).
    position_p_value: AtomicU16,
    /// Position I gain for cross-priority access (Q4.12).
    position_i_value: AtomicU16,
    /// Velocity P gain for cross-priority access (Q8.8).
    velocity_p_value: AtomicU16,
    /// Velocity I gain for cross-priority access (Q4.12).
    velocity_i_value: AtomicU16,
    /// Set once the velocity feedforward RPM-domain clamp has engaged, and
    /// stays set until the next `reset()`.
    velocity_ff_clamp_latched: AtomicBool,
    /// Saturating count of velocity feedforward clamp engagements since the
    /// last `reset()`.
    velocity_ff_clamp_count: AtomicU32,
    /// Homing-move current clamp in mA RMS; 0 disables the clamp and detector.
    pub homing_current_ma: AtomicU32,
    /// Stall threshold in TMC position units (65536 per revolution).
    pub stall_threshold_units: AtomicU32,
    /// Consecutive control ticks over threshold before the latch sets.
    pub stall_persistence: AtomicU8,
    /// An endstop on this channel is configured on the virtual STALL pin.
    pub stall_endstop_configured: AtomicBool,
    /// Live stall latch read by the endstop sample slot while homing.
    pub stall_latched: AtomicBool,
    /// The homing clamp currently overrides the run-current limit.
    pub homing_clamp_active: AtomicBool,
    /// Latch outcome of the most recent completed homing mode.
    pub stall_result_latched: AtomicBool,
    /// Peak following error of the most recent completed homing mode.
    pub stall_peak_error_units: AtomicU32,
    /// Detector tick at which the most recent latch set, 0 if none.
    pub stall_trigger_tick: AtomicU32,
}

impl TmcChannelState {
    /// Create a channel state with firmware defaults.
    pub const fn new() -> Self {
        Self {
            current_limit_value: AtomicU16::new(PidTorqueFluxLimit::ZERO.raw()),
            motor_armed: AtomicBool::new(false),
            enable_pin_asserted: AtomicBool::new(false),
            motor_sentinel: AtomicU32::new(0),
            safe_state_incomplete: AtomicBool::new(false),
            safe_state_last_pass_failed: AtomicBool::new(false),
            safe_state_failed_writes: AtomicU8::new(0),
            safe_state_first_failed_register: AtomicU8::new(0xFF),
            safe_state_first_failed_sub_addr: AtomicU8::new(0xFF),
            safe_state_failure_timestamp_ms: AtomicU32::new(0),
            safe_state_pass_count: AtomicU16::new(0),
            pi_representation_established: AtomicBool::new(false),
            encoder_ppr: AtomicU32::new(0),
            encoder_counts_per_rev: AtomicU32::new(0),
            tmc_grid: AtomicU32::new(0),
            step_width: AtomicU32::new(0),
            motion_scale_configured: AtomicBool::new(false),
            config_apply_pending: AtomicBool::new(false),
            velocity_filter_hz: AtomicOptionalFilterHz::new(None),
            torque_filter_hz: AtomicOptionalFilterHz::new(None),
            position_filter_hz: AtomicOptionalFilterHz::new(None),
            flux_filter_hz: AtomicOptionalFilterHz::new(None),
            velocity_feedforward: AtomicBool::new(false),
            velocity_feedforward_gain_permille: AtomicU16::new(1_000),
            velocity_transient_feedforward: AtomicBool::new(false),
            velocity_transient_lead_time_us: AtomicU16::new(0),
            velocity_transient_gain_permille: AtomicU16::new(0),
            velocity_transient_max_offset: AtomicU16::new(0),
            velocity_transient_rate_hz: AtomicU16::new(1000),
            accel_feedforward: AtomicBool::new(false),
            accel_feedforward_accel_gain_permille: AtomicU16::new(1000),
            accel_feedforward_decel_gain_permille: AtomicU16::new(1000),
            decoupling_feedforward: AtomicBool::new(false),
            decoupling_r_int: AtomicU32::new(0),
            decoupling_l_int: AtomicU32::new(0),
            decoupling_pole_pairs: AtomicU16::new(0),
            decoupling_position_units_per_rev: AtomicU32::new(0),
            decoupling_f_pwm_hz: AtomicU32::new(0),
            decoupling_max_offset: AtomicU16::new(0),
            feedforward_reset: AtomicBool::new(false),
            position_lead: AtomicBool::new(false),
            position_lead_gain_permille: AtomicU16::new(0),
            position_lead_max_counts: AtomicU16::new(0),
            phase_advance: AtomicBool::new(false),
            phase_advance_gain_ppm: AtomicI32::new(0),
            phase_advance_max_counts: AtomicU16::new(0),
            phase_advance_deadband: AtomicU16::new(16),
            pid_error_sums_clear: AtomicBool::new(false),
            velocity_limit_cfg: AtomicU32::new(500_000),
            voltage_limit_cfg: AtomicU32::new(DEFAULT_OPERATIONAL_PIDOUT_UQ_UD_LIMIT),
            adc_vm_offset_raw: AtomicU16::new(0),
            adc_vm_offset_sample_count: AtomicU8::new(0),
            encoder_reversed: AtomicBool::new(false),
            commission_in_flight: AtomicBool::new(false),
            cancel_requested: AtomicBool::new(false),
            auto_calibrate_on_enable_allowed: AtomicBool::new(false),
            position_p_value: AtomicU16::new(crate::pid::DEFAULT_OUTER_GAINS.position_p.raw()),
            position_i_value: AtomicU16::new(crate::pid::DEFAULT_OUTER_GAINS.position_i.raw()),
            velocity_p_value: AtomicU16::new(crate::pid::DEFAULT_OUTER_GAINS.velocity_p.raw()),
            velocity_i_value: AtomicU16::new(crate::pid::DEFAULT_OUTER_GAINS.velocity_i.raw()),
            velocity_ff_clamp_latched: AtomicBool::new(false),
            velocity_ff_clamp_count: AtomicU32::new(0),
            homing_current_ma: AtomicU32::new(0),
            stall_threshold_units: AtomicU32::new(0),
            stall_persistence: AtomicU8::new(0),
            stall_endstop_configured: AtomicBool::new(false),
            stall_latched: AtomicBool::new(false),
            homing_clamp_active: AtomicBool::new(false),
            stall_result_latched: AtomicBool::new(false),
            stall_peak_error_units: AtomicU32::new(0),
            stall_trigger_tick: AtomicU32::new(0),
        }
    }

    /// Reset channel-local runtime/configuration state to firmware defaults.
    pub fn reset(&self) {
        use core::sync::atomic::Ordering::Relaxed;

        self.motion_scale_configured.store(false, Ordering::Release);
        self.store_current_limit(PidTorqueFluxLimit::ZERO, Relaxed);
        self.motor_armed.store(false, Relaxed);
        self.motor_sentinel.store(0, Relaxed);
        self.safe_state_incomplete.store(false, Relaxed);
        self.safe_state_last_pass_failed.store(false, Relaxed);
        self.safe_state_failed_writes.store(0, Relaxed);
        self.safe_state_first_failed_register.store(0xFF, Relaxed);
        self.safe_state_first_failed_sub_addr.store(0xFF, Relaxed);
        self.safe_state_failure_timestamp_ms.store(0, Relaxed);
        self.safe_state_pass_count.store(0, Relaxed);
        self.clear_pi_representation_established();
        self.encoder_ppr.store(0, Relaxed);
        self.encoder_counts_per_rev.store(0, Relaxed);
        self.tmc_grid.store(0, Relaxed);
        self.step_width.store(0, Relaxed);
        self.velocity_filter_hz.store(None, Relaxed);
        self.torque_filter_hz.store(None, Relaxed);
        self.position_filter_hz.store(None, Relaxed);
        self.flux_filter_hz.store(None, Relaxed);
        self.velocity_feedforward.store(false, Relaxed);
        self.velocity_feedforward_gain_permille
            .store(1_000, Relaxed);
        self.velocity_transient_feedforward.store(false, Relaxed);
        self.velocity_transient_lead_time_us.store(0, Relaxed);
        self.velocity_transient_gain_permille.store(0, Relaxed);
        self.velocity_transient_max_offset.store(0, Relaxed);
        self.velocity_transient_rate_hz.store(1000, Relaxed);
        self.accel_feedforward.store(false, Relaxed);
        self.accel_feedforward_accel_gain_permille
            .store(1000, Relaxed);
        self.accel_feedforward_decel_gain_permille
            .store(1000, Relaxed);
        self.decoupling_feedforward.store(false, Relaxed);
        self.decoupling_r_int.store(0, Relaxed);
        self.decoupling_l_int.store(0, Relaxed);
        self.decoupling_pole_pairs.store(0, Relaxed);
        self.decoupling_position_units_per_rev.store(0, Relaxed);
        self.decoupling_f_pwm_hz.store(0, Relaxed);
        self.decoupling_max_offset.store(0, Relaxed);
        self.feedforward_reset.store(false, Relaxed);
        self.position_lead.store(false, Relaxed);
        self.position_lead_gain_permille.store(0, Relaxed);
        self.position_lead_max_counts.store(0, Relaxed);
        self.phase_advance.store(false, Relaxed);
        self.phase_advance_gain_ppm.store(0, Relaxed);
        self.phase_advance_max_counts.store(0, Relaxed);
        self.phase_advance_deadband.store(16, Relaxed);
        self.pid_error_sums_clear.store(false, Relaxed);
        self.velocity_limit_cfg.store(500_000, Relaxed);
        self.voltage_limit_cfg
            .store(DEFAULT_OPERATIONAL_PIDOUT_UQ_UD_LIMIT, Relaxed);
        self.encoder_reversed.store(false, Relaxed);
        self.commission_in_flight.store(false, Relaxed);
        self.cancel_requested.store(false, Relaxed);
        self.auto_calibrate_on_enable_allowed.store(false, Relaxed);
        self.store_outer_gains(crate::pid::DEFAULT_OUTER_GAINS, Relaxed);
        self.clear_velocity_clamp_latch();
        self.homing_current_ma.store(0, Relaxed);
        self.stall_threshold_units.store(0, Relaxed);
        self.stall_persistence.store(0, Relaxed);
        self.stall_endstop_configured.store(false, Relaxed);
        self.stall_latched.store(false, Relaxed);
        self.homing_clamp_active.store(false, Relaxed);
        self.stall_result_latched.store(false, Relaxed);
        self.stall_peak_error_units.store(0, Relaxed);
        self.stall_trigger_tick.store(0, Relaxed);
    }

    /// Load the validated torque/flux current limit.
    pub fn load_current_limit(&self, ordering: Ordering) -> PidTorqueFluxLimit {
        let raw = u32::from(self.current_limit_value.load(ordering));
        PidTorqueFluxLimit::new(raw)
            .expect("private current-limit storage accepts only validated values")
    }

    /// Publish a validated torque/flux current limit.
    pub fn store_current_limit(&self, limit: PidTorqueFluxLimit, ordering: Ordering) {
        self.current_limit_value.store(limit.raw(), ordering);
    }

    /// Return whether startup established the fixed PI representation.
    pub fn pi_representation_established(&self) -> bool {
        self.pi_representation_established.load(Ordering::Acquire)
    }

    /// Clear the representation invariant after chip reset or state loss.
    pub(crate) fn clear_pi_representation_established(&self) {
        self.pi_representation_established
            .store(false, Ordering::Release);
    }

    /// Publish successful exact representation readback.
    pub(crate) fn mark_pi_representation_established(&self) {
        self.pi_representation_established
            .store(true, Ordering::Release);
    }

    /// Load and validate the retained outer-loop gains.
    pub fn load_outer_gains(
        &self,
        ordering: Ordering,
    ) -> Result<OuterLoopGains, PidGainOutOfRange> {
        Ok(OuterLoopGains {
            position_p: PidPGain::new(self.position_p_value.load(ordering))?,
            position_i: PidIGain::new(self.position_i_value.load(ordering))?,
            velocity_p: PidPGain::new(self.velocity_p_value.load(ordering))?,
            velocity_i: PidIGain::new(self.velocity_i_value.load(ordering))?,
        })
    }

    /// Publish one complete validated outer-loop gain set.
    pub fn store_outer_gains(&self, gains: OuterLoopGains, ordering: Ordering) {
        self.position_p_value
            .store(gains.position_p.raw(), ordering);
        self.position_i_value
            .store(gains.position_i.raw(), ordering);
        self.velocity_p_value
            .store(gains.velocity_p.raw(), ordering);
        self.velocity_i_value
            .store(gains.velocity_i.raw(), ordering);
    }

    #[cfg(test)]
    pub(crate) fn corrupt_outer_gain_for_test(&self, raw: u16) {
        self.position_p_value.store(raw, Ordering::Relaxed);
    }

    /// Validates and publishes a complete motion scale to its channel mirrors.
    ///
    /// # Errors
    ///
    /// Returns [`crate::motion_scale::MotionScaleError`] without changing any
    /// mirror when either encoder or planner resolution is invalid.
    pub fn configure_motion_scale(
        &self,
        stepper_state: &crate::state::stepper::StepperChannelState,
        encoder_ppr: u32,
        planner_steps_per_rev: u32,
    ) -> Result<crate::motion_scale::MotionScale, crate::motion_scale::MotionScaleError> {
        let scale = crate::motion_scale::MotionScale::new(encoder_ppr, planner_steps_per_rev)?;

        self.motion_scale_configured.store(false, Ordering::Release);
        self.encoder_ppr
            .store(scale.encoder_ppr(), Ordering::Relaxed);
        self.encoder_counts_per_rev
            .store(scale.encoder_counts_per_rev(), Ordering::Relaxed);
        stepper_state.store_planner_steps_per_rev(scale.planner_steps_per_rev());
        self.tmc_grid.store(scale.tmc_grid(), Ordering::Relaxed);
        self.step_width
            .store(scale.physical_step_width(), Ordering::Relaxed);
        self.motion_scale_configured.store(true, Ordering::Release);
        Ok(scale)
    }

    pub(crate) fn clear_motion_scale(
        &self,
        stepper_state: &crate::state::stepper::StepperChannelState,
    ) {
        self.motion_scale_configured.store(false, Ordering::Release);
        self.encoder_ppr.store(0, Ordering::Relaxed);
        self.encoder_counts_per_rev.store(0, Ordering::Relaxed);
        stepper_state.store_planner_steps_per_rev(0);
        self.tmc_grid.store(0, Ordering::Relaxed);
        self.step_width.store(0, Ordering::Relaxed);
    }

    /// Returns whether a complete motion scale has been published.
    pub fn motion_scale_configured(&self) -> bool {
        self.motion_scale_configured.load(Ordering::Acquire)
    }

    /// Returns configured encoder pulses per mechanical revolution.
    pub fn encoder_ppr(&self) -> Option<u32> {
        self.motion_scale_configured()
            .then(|| self.encoder_ppr.load(Ordering::Relaxed))
    }

    /// Returns configured quadrature encoder counts per mechanical revolution.
    pub fn encoder_counts_per_rev(&self) -> Option<u32> {
        self.motion_scale_configured()
            .then(|| self.encoder_counts_per_rev.load(Ordering::Relaxed))
    }

    /// Returns configured physical TMC pulses per mechanical revolution.
    pub fn tmc_grid(&self) -> Option<u32> {
        self.motion_scale_configured()
            .then(|| self.tmc_grid.load(Ordering::Relaxed))
    }

    /// Returns configured TMC4671 position units per physical pulse.
    pub fn physical_step_width(&self) -> Option<u32> {
        self.motion_scale_configured()
            .then(|| self.step_width.load(Ordering::Relaxed))
    }

    /// Store the ADC_VM mode-5 startup offset captured for this channel.
    pub fn store_adc_vm_offset(&self, offset_raw: u16, sample_count: u8) {
        use core::sync::atomic::Ordering::Relaxed;

        self.adc_vm_offset_raw.store(offset_raw, Relaxed);
        self.adc_vm_offset_sample_count.store(sample_count, Relaxed);
    }

    /// Return the cached ADC_VM startup offset when one has been captured.
    pub fn adc_vm_offset_snapshot(&self) -> Option<AdcVmOffsetSnapshot> {
        use core::sync::atomic::Ordering::Relaxed;

        let sample_count = self.adc_vm_offset_sample_count.load(Relaxed);
        if sample_count == 0 {
            return None;
        }
        Some(AdcVmOffsetSnapshot {
            offset_raw: self.adc_vm_offset_raw.load(Relaxed),
            sample_count,
        })
    }

    /// Latch the velocity feedforward clamp flag and saturating-increment
    /// its engagement counter.
    pub fn record_velocity_clamp_engagement(&self) {
        self.velocity_ff_clamp_latched
            .store(true, Ordering::Relaxed);

        let mut current = self.velocity_ff_clamp_count.load(Ordering::Relaxed);
        while current < u32::MAX {
            match self.velocity_ff_clamp_count.compare_exchange_weak(
                current,
                current + 1,
                Ordering::Relaxed,
                Ordering::Relaxed,
            ) {
                Ok(_) => break,
                Err(next) => current = next,
            }
        }
    }

    /// Clear the latched velocity feedforward clamp flag and its counter.
    pub fn clear_velocity_clamp_latch(&self) {
        self.velocity_ff_clamp_latched
            .store(false, Ordering::Relaxed);
        self.velocity_ff_clamp_count.store(0, Ordering::Relaxed);
    }

    /// Return whether the velocity feedforward clamp has engaged since the
    /// last `reset()`.
    pub fn velocity_clamp_latched(&self) -> bool {
        self.velocity_ff_clamp_latched.load(Ordering::Relaxed)
    }

    /// Return the saturating count of velocity feedforward clamp engagements
    /// since the last `reset()`.
    pub fn velocity_clamp_count(&self) -> u32 {
        self.velocity_ff_clamp_count.load(Ordering::Relaxed)
    }
}

#[cfg(test)]
mod motion_scale_tests {
    use super::*;
    use crate::state::stepper::StepperChannelState;

    #[test]
    fn motion_scale_configuration_publishes_named_mirrors() {
        let tmc = TmcChannelState::new();
        let stepper = StepperChannelState::new();

        let scale = tmc.configure_motion_scale(&stepper, 1_000, 3_200).unwrap();

        assert!(tmc.motion_scale_configured());
        assert_eq!(tmc.encoder_ppr(), Some(1_000));
        assert_eq!(tmc.encoder_counts_per_rev(), Some(4_000));
        assert_eq!(stepper.planner_steps_per_rev(&tmc), Some(3_200));
        assert_eq!(tmc.tmc_grid(), Some(4_096));
        assert_eq!(tmc.physical_step_width(), Some(16));
        assert_eq!(scale.physical_step_width(), 16);
    }

    #[test]
    fn invalid_motion_scale_does_not_partially_update_mirrors() {
        let tmc = TmcChannelState::new();
        let stepper = StepperChannelState::new();
        tmc.configure_motion_scale(&stepper, 1_000, 3_200).unwrap();

        assert!(
            tmc.configure_motion_scale(&stepper, u32::MAX, 4_000)
                .is_err()
        );

        assert_eq!(tmc.encoder_ppr(), Some(1_000));
        assert_eq!(tmc.encoder_counts_per_rev(), Some(4_000));
        assert_eq!(stepper.planner_steps_per_rev(&tmc), Some(3_200));
        assert_eq!(tmc.physical_step_width(), Some(16));
    }

    #[test]
    fn reset_unpublishes_motion_scale_before_clearing_mirrors() {
        let tmc = TmcChannelState::new();
        let stepper = StepperChannelState::new();
        tmc.configure_motion_scale(&stepper, 1_000, 3_200).unwrap();

        tmc.reset();
        stepper.reset();

        assert!(!tmc.motion_scale_configured());
        assert_eq!(tmc.encoder_counts_per_rev(), None);
        assert_eq!(stepper.planner_steps_per_rev(&tmc), None);
        assert_eq!(tmc.physical_step_width(), None);
    }
}

/// Cached ADC_VM mode-5 offset evidence for one TMC channel.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct AdcVmOffsetSnapshot {
    /// Averaged raw ADC_VM count captured in mode 5 (`VDD/2`).
    pub offset_raw: u16,
    /// Number of samples folded into `offset_raw`.
    pub sample_count: u8,
}

impl Default for TmcChannelState {
    fn default() -> Self {
        Self::new()
    }
}

/// Channel-indexed TMC runtime state.
pub static TMC_CHANNELS: [TmcChannelState; MAX_TMC_CHANNELS] =
    [const { TmcChannelState::new() }; MAX_TMC_CHANNELS];

/// Return runtime state for a board-local TMC channel.
pub fn tmc_channel(channel: u8) -> Option<&'static TmcChannelState> {
    TMC_CHANNELS.get(channel as usize)
}

/// Return the OpenFFBoard-compatible channel-0 runtime state.
pub fn tmc_channel0() -> &'static TmcChannelState {
    &TMC_CHANNELS[0]
}

/// Reset all TMC channel states to firmware defaults.
pub fn reset_all_tmc_channels() {
    for channel in &TMC_CHANNELS {
        channel.reset();
    }
}

/// Emergency motor disable signal. Set by enter_shutdown(), consumed
/// by tmc_control. Uses a Signal so tmc_control wakes immediately
/// even if blocked on an empty channel.
pub static TMC_DISABLE_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Signal tmc_control to drain all pending requests. Set by config_reset
/// (P2), cleared by tmc_control (P1) after draining.
pub static TMC_QUEUE_FLUSH: AtomicBool = AtomicBool::new(false);

#[cfg(test)]
mod tests {
    use super::*;
    use core::mem::size_of;
    use core::sync::atomic::Ordering::Relaxed;
    use foci_firmware_macros::fixture_test;
    use foci_tmc4671::current_limit::PidTorqueFluxLimit;

    #[test]
    fn tmc_channel_state_size_is_pinned() {
        assert_eq!(size_of::<TmcChannelState>(), 160);
    }

    #[test]
    fn optional_filter_hz_distinguishes_unset_disable_and_cutoff() {
        let filter = AtomicOptionalFilterHz::new(None);

        assert_eq!(filter.load(Relaxed), None);

        filter.store(Some(0), Relaxed);
        assert_eq!(filter.load(Relaxed), Some(0));

        filter.store(Some(3000), Relaxed);
        assert_eq!(filter.load(Relaxed), Some(3000));

        filter.store(None, Relaxed);
        assert_eq!(filter.load(Relaxed), None);
    }

    #[test]
    fn outer_gain_defaults_load_through_typed_boundary() {
        let state = TmcChannelState::new();

        assert_eq!(
            state.load_outer_gains(Relaxed),
            Ok(crate::pid::DEFAULT_OUTER_GAINS)
        );
    }

    #[test]
    fn outer_gain_store_round_trips_as_one_aggregate() {
        let state = TmcChannelState::new();
        let gains = crate::pid::test_outer_gains(0x7fff, 1, 2, 0x7fff);

        state.store_outer_gains(gains, Relaxed);

        assert_eq!(state.load_outer_gains(Relaxed), Ok(gains));
    }

    #[test]
    fn outer_gain_load_rejects_corrupt_raw_storage() {
        let state = TmcChannelState::new();
        state.corrupt_outer_gain_for_test(0x8000);

        assert!(state.load_outer_gains(Relaxed).is_err());
    }

    #[fixture_test]
    fn tmc_channels_have_independent_current_limits() {
        let channel0 = tmc_channel(0).unwrap();
        let channel1 = tmc_channel(1).unwrap();
        let limit0 = PidTorqueFluxLimit::new(0x1111).unwrap();
        let limit1 = PidTorqueFluxLimit::new(0x2222).unwrap();

        channel0.store_current_limit(limit0, Relaxed);
        channel1.store_current_limit(limit1, Relaxed);

        assert_eq!(channel0.load_current_limit(Relaxed), limit0);
        assert_eq!(channel1.load_current_limit(Relaxed), limit1);

        channel0.store_current_limit(PidTorqueFluxLimit::ZERO, Relaxed);
        channel1.store_current_limit(PidTorqueFluxLimit::ZERO, Relaxed);
    }

    #[test]
    fn tmc_channel_rejects_out_of_range_index() {
        assert!(tmc_channel(MAX_TMC_CHANNELS as u8).is_none());
    }

    #[test]
    fn reset_clears_phase_advance_state() {
        let channel = TmcChannelState::new();
        channel.phase_advance.store(true, Relaxed);
        channel.phase_advance_gain_ppm.store(60_000, Relaxed);
        channel.phase_advance_max_counts.store(64, Relaxed);
        channel.phase_advance_deadband.store(4, Relaxed);

        channel.reset();

        assert!(!channel.phase_advance.load(Relaxed));
        assert_eq!(channel.phase_advance_gain_ppm.load(Relaxed), 0);
        assert_eq!(channel.phase_advance_max_counts.load(Relaxed), 0);
        assert_eq!(channel.phase_advance_deadband.load(Relaxed), 16);
    }

    #[test]
    fn motor_sentinel_defaults_to_zero_and_reset_clears_it() {
        let channel = TmcChannelState::new();
        assert_eq!(channel.motor_sentinel.load(Relaxed), 0);

        channel.motor_sentinel.store(0x0002_0032, Relaxed);
        channel.reset();

        assert_eq!(channel.motor_sentinel.load(Relaxed), 0);
    }

    #[test]
    fn adc_vm_offset_cache_starts_empty_and_reset_preserves_it() {
        let channel = TmcChannelState::new();
        assert_eq!(channel.adc_vm_offset_snapshot(), None);

        channel.store_adc_vm_offset(33_662, 8);
        assert_eq!(
            channel.adc_vm_offset_snapshot(),
            Some(AdcVmOffsetSnapshot {
                offset_raw: 33_662,
                sample_count: 8,
            })
        );

        channel.reset();

        assert_eq!(
            channel.adc_vm_offset_snapshot(),
            Some(AdcVmOffsetSnapshot {
                offset_raw: 33_662,
                sample_count: 8,
            })
        );
    }

    /// No-auto-energize invariant: a freshly constructed channel (the boot
    /// state) must be disarmed and must not permit enable-driven
    /// auto-calibration. A motor can only be energized after an explicit
    /// host command flips these fields, never on power-on.
    #[test]
    fn fresh_channel_is_disarmed_and_auto_energize_gated() {
        let channel = TmcChannelState::new();

        assert!(
            !channel.motor_armed.load(Relaxed),
            "boot state must leave the motor disarmed"
        );
        assert!(
            !channel.commission_in_flight.load(Relaxed),
            "boot state must not have commissioning in flight"
        );
        assert!(
            !channel.auto_calibrate_on_enable_allowed.load(Relaxed),
            "auto-calibrate-on-enable must be gated closed at boot; only an \
             explicit host command may open it"
        );
    }

    /// `reset()` (the path config_reset funnels through) must re-close every
    /// energize gate, so a shutdown/reset cycle can never leave the motor
    /// armed or auto-calibration enabled.
    #[test]
    fn reset_recloses_auto_energize_gates() {
        let channel = TmcChannelState::new();
        channel.motor_armed.store(true, Relaxed);
        channel.commission_in_flight.store(true, Relaxed);
        channel
            .auto_calibrate_on_enable_allowed
            .store(true, Relaxed);

        channel.reset();

        assert!(!channel.motor_armed.load(Relaxed));
        assert!(!channel.commission_in_flight.load(Relaxed));
        assert!(!channel.auto_calibrate_on_enable_allowed.load(Relaxed));
    }

    #[test]
    fn reset_clears_optional_filter_values() {
        let channel = TmcChannelState::new();
        channel.velocity_filter_hz.store(Some(100), Relaxed);
        channel.torque_filter_hz.store(Some(0), Relaxed);
        channel.position_filter_hz.store(Some(200), Relaxed);
        channel.flux_filter_hz.store(Some(300), Relaxed);

        channel.reset();

        assert_eq!(channel.velocity_filter_hz.load(Relaxed), None);
        assert_eq!(channel.torque_filter_hz.load(Relaxed), None);
        assert_eq!(channel.position_filter_hz.load(Relaxed), None);
        assert_eq!(channel.flux_filter_hz.load(Relaxed), None);
    }

    #[test]
    fn clamp_latch_records_and_clears() {
        let state = TmcChannelState::new();
        assert!(!state.velocity_clamp_latched());
        state.record_velocity_clamp_engagement();
        state.record_velocity_clamp_engagement();
        assert!(state.velocity_clamp_latched());
        assert_eq!(state.velocity_clamp_count(), 2);
        state.clear_velocity_clamp_latch();
        assert!(!state.velocity_clamp_latched());
        assert_eq!(state.velocity_clamp_count(), 0);
    }

    #[test]
    fn enable_pin_asserted_defaults_false() {
        let ch = TmcChannelState::new();
        assert!(!ch.enable_pin_asserted.load(Ordering::Relaxed));
    }

    #[test]
    fn cancel_requested_defaults_to_false() {
        let state = TmcChannelState::new();
        assert!(
            !state
                .cancel_requested
                .load(core::sync::atomic::Ordering::Relaxed)
        );
    }

    #[test]
    fn homing_config_and_stall_result_default_to_inactive() {
        let state = TmcChannelState::new();
        assert_eq!(state.homing_current_ma.load(Relaxed), 0);
        assert_eq!(state.stall_threshold_units.load(Relaxed), 0);
        assert_eq!(state.stall_persistence.load(Relaxed), 0);
        assert!(!state.stall_endstop_configured.load(Relaxed));
        assert!(!state.stall_latched.load(Relaxed));
        assert!(!state.homing_clamp_active.load(Relaxed));
        assert!(!state.stall_result_latched.load(Relaxed));
        assert_eq!(state.stall_peak_error_units.load(Relaxed), 0);
        assert_eq!(state.stall_trigger_tick.load(Relaxed), 0);
    }
}
