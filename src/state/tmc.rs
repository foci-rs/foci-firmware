//! TMC4671 protocol state shared across priorities.

use core::sync::atomic::{AtomicBool, AtomicU8, AtomicU16, AtomicU32, Ordering};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use foci_tmc4671::current_limit::PidTorqueFluxLimit;
use foci_tmc4671::pid::{PidGainOutOfRange, PidIGain, PidPGain};

use crate::pid::{InnerLoopGains, OuterLoopGains};

/// Maximum number of board-local TMC4671 channels supported by shared firmware.
pub const MAX_TMC_CHANNELS: usize = 2;
/// Conservative operational PIDOUT_UQ_UD_LIMITS value used after closed-loop entry.
pub const DEFAULT_OPERATIONAL_PIDOUT_UQ_UD_LIMIT: u32 = 16_000;
/// Lowest accepted raw PIDOUT_UQ_UD_LIMITS value.
pub const MIN_OPERATIONAL_PIDOUT_UQ_UD_LIMIT: u32 = 0;
/// Highest accepted diagnostic PIDOUT_UQ_UD_LIMITS value.
pub const MAX_OPERATIONAL_PIDOUT_UQ_UD_LIMIT: u32 = 32_767;

/// Bitmask constant for pending voltage limit configuration.
pub const PENDING_VOLTAGE_LIMIT: u8 = 1 << 0;
/// Bitmask constant for pending filter configuration.
pub const PENDING_FILTERS: u8 = 1 << 1;
/// Bitmask constant for pending current configuration.
pub const PENDING_CURRENT: u8 = 1 << 2;
/// Bitmask constant for pending encoder direction configuration.
pub const PENDING_ENCODER_DIR: u8 = 1 << 3;
/// Bitmask constant for pending inner gains configuration.
pub const PENDING_INNER_GAINS: u8 = 1 << 4;
/// Bitmask constant for pending outer gains configuration.
pub const PENDING_OUTER_GAINS: u8 = 1 << 5;

/// Worst-case TMC request queue slots a connect-time config handler may occupy per channel;
/// each board asserts at compile time that `TMC_CHANNEL_SIZE` covers this times the channel count.
pub const CONNECT_TIME_QUEUE_SLOTS_PER_CHANNEL: usize = 1;

const OPTIONAL_FILTER_HZ_UNSET: u32 = u32::MAX;

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

const OPTIONAL_INVERT_UNSET: u8 = u8::MAX;

/// `None` means the host has never configured encoder direction on this
/// channel — the drain must not write a default to `ABN_DECODER_MODE` in
/// that case, since there is no safe direction to pick blind. `Some(false)`
/// and `Some(true)` are both real, host-supplied configurations.
pub struct AtomicOptionalInvert {
    raw: AtomicU8,
}

impl AtomicOptionalInvert {
    /// Create optional invert storage with the given initial value.
    pub const fn new(invert: Option<bool>) -> Self {
        Self {
            raw: AtomicU8::new(Self::encode(invert)),
        }
    }

    /// Load the optional invert flag.
    pub fn load(&self, ordering: Ordering) -> Option<bool> {
        match self.raw.load(ordering) {
            OPTIONAL_INVERT_UNSET => None,
            raw => Some(raw != 0),
        }
    }

    /// Store the optional invert flag.
    pub fn store(&self, invert: Option<bool>, ordering: Ordering) {
        self.raw.store(Self::encode(invert), ordering);
    }

    const fn encode(invert: Option<bool>) -> u8 {
        match invert {
            Some(true) => 1,
            Some(false) => 0,
            None => OPTIONAL_INVERT_UNSET,
        }
    }
}

/// Runtime state for one board-local TMC4671 channel.
pub struct TmcChannelState {
    current_limit_value: AtomicU16,
    pub motor_armed: AtomicBool,
    /// Mirrors the physical enable-pin GPIO level; written only by
    /// `SharedCommissioningBackend::set_commission_enable_pin`.
    pub enable_pin_asserted: AtomicBool,
    /// Captured at commission time from the `MOTOR_TYPE_N_POLE_PAIRS` write; zero means no
    /// commission yet
    /// (sentinel check inactive). Used by `arm_motor_electrical` to detect silent chip resets.
    pub motor_sentinel: AtomicU32,
    /// Set by every `restore_safe_state_via_backend` pass, clean or not; cleared only by
    /// `arm_motor_electrical` after the sentinel and PI-representation readbacks succeed, or by
    /// `reset()`.
    /// A restore alone does not confirm the chip is safe to arm.
    pub safe_state_incomplete: AtomicBool,
    /// Set when the most recent restore pass had a verified-write or PID-error-sum-clear failure;
    /// `arm_motor_electrical` refuses to arm while set. Cleared by the next failure-free restore
    /// pass.
    pub safe_state_last_pass_failed: AtomicBool,
    pub safe_state_failed_writes: AtomicU8,
    /// Address of the first write that failed in the most recent restore pass; `0xFF` means none.
    pub safe_state_first_failed_register: AtomicU8,
    /// Logical sub-register selector (4=torque, 5=flux, 6=velocity, 7=position) when the first
    /// failure
    /// was a PID error-sum clear, since all four share one physical selector; `0xFF` otherwise.
    pub safe_state_first_failed_sub_addr: AtomicU8,
    /// Board-monotonic ms timestamp of the most recent restore pass; diagnostic only, wraps every
    /// ~49 days.
    pub safe_state_failure_timestamp_ms: AtomicU32,
    /// Completed restore passes, clean or not; saturating, cleared only by
    /// [`TmcChannelState::reset`].
    pub safe_state_pass_count: AtomicU16,
    pi_representation_established: AtomicBool,
    pub encoder_ppr: AtomicU32,
    encoder_counts_per_rev: AtomicU32,
    tmc_grid: AtomicU32,
    step_width: AtomicU32,
    motion_scale_configured: AtomicBool,
    /// Bitmask of `PENDING_*` configuration items the TMC task must apply on its next control tick.
    ///
    /// Config-time setters record state here instead of enqueueing requests, keeping the
    /// connect-time
    /// burst off the bounded TMC request channel (a dropped `#[klipper_command]` is an MCU
    /// shutdown).
    pub config_apply_pending: AtomicU8,
    pub status_pin_asserted: AtomicBool,
    pub velocity_filter_hz: AtomicOptionalFilterHz,
    pub torque_filter_hz: AtomicOptionalFilterHz,
    pub position_filter_hz: AtomicOptionalFilterHz,
    pub flux_filter_hz: AtomicOptionalFilterHz,
    pub velocity_feedforward: AtomicBool,
    /// Runtime gain in permille applied to the RPM-domain velocity feedforward
    /// offset (1000 = unity).
    pub velocity_feedforward_gain_permille: AtomicU16,
    pub accel_feedforward: AtomicBool,
    /// Runtime gain in permille while step intervals shrink.
    pub accel_feedforward_accel_gain_permille: AtomicU16,
    /// Runtime gain in permille while step intervals grow.
    pub accel_feedforward_decel_gain_permille: AtomicU16,
    /// One-tick feedforward reset request after direction changes.
    pub feedforward_reset: AtomicBool,
    /// PID error-sum clear request after motion drains.
    pub pid_error_sums_clear: AtomicBool,
    pub velocity_limit_cfg: AtomicU32,
    pub voltage_limit_cfg: AtomicU32,
    /// Requested RMS run current in milliamps, applied on the control tick.
    pub run_current_ma_cfg: AtomicU32,
    /// Requested encoder-direction invert flag, applied on the control tick.
    pub encoder_invert_cfg: AtomicOptionalInvert,
    pending_inner_gain_flux_p: AtomicU16,
    pending_inner_gain_flux_i: AtomicU16,
    pending_inner_gain_torque_p: AtomicU16,
    pending_inner_gain_torque_i: AtomicU16,
    pending_outer_gain_position_p: AtomicU16,
    pending_outer_gain_position_i: AtomicU16,
    pending_outer_gain_velocity_p: AtomicU16,
    pending_outer_gain_velocity_i: AtomicU16,
    adc_vm_offset_raw: AtomicU16,
    adc_vm_offset_sample_count: AtomicU8,
    pub encoder_reversed: AtomicBool,
    pub commission_in_flight: AtomicBool,
    /// Operator-requested cancel of this channel's in-flight commissioning run. Set by
    /// `foci_commission_cancel` (P2); reset to `false` at every run start so a stale write cannot
    /// cancel the next run.
    pub cancel_requested: AtomicBool,
    /// A commissioning phase is driving the step queue and wants the planner-referenced control
    /// paths
    /// (feedforward, fast control period) live. Set and cleared only by the stimulus facet.
    pub commission_stimulus_active: AtomicBool,
    /// Written by the control tick every pass: true when no feedforward surface is left applied on
    /// the chip.
    pub feedforward_surfaces_idle: AtomicBool,
    /// `motor_armed` as it was when a commissioning stimulus armed the step
    /// timer; `stimulus_stop` puts it back. Owned by the stimulus facet.
    pub stimulus_prior_motor_armed: AtomicBool,
    /// Whether `stimulus_prior_motor_armed` holds this session's captured value; guards against a
    /// second
    /// arm call clobbering the real pre-session value. Cleared by `stimulus_restore_motor_armed`.
    stimulus_prior_motor_armed_captured: AtomicBool,
    /// Host-controlled gate for raw-enable auto-calibration.
    pub auto_calibrate_on_enable_allowed: AtomicBool,
    position_p_value: AtomicU16,
    position_i_value: AtomicU16,
    velocity_p_value: AtomicU16,
    velocity_i_value: AtomicU16,
    velocity_ff_clamp_latched: AtomicBool,
    velocity_ff_clamp_count: AtomicU32,
    /// Homing-move current clamp in mA RMS; 0 disables the clamp and detector.
    pub homing_current_ma: AtomicU32,
    /// Unconditional stall-detector ceiling in TMC position units (65536 per revolution); always
    /// active.
    pub stall_ceiling_units: AtomicU32,
    /// Baseline-relative stall-detector margin in TMC position units; active once the EMA baseline
    /// has warmed up.
    pub stall_margin_units: AtomicU32,
    /// Consecutive control ticks over threshold before either check's latch
    /// sets.
    pub stall_persistence: AtomicU8,
    pub stall_endstop_configured: AtomicBool,
    pub stall_latched: AtomicBool,
    pub homing_clamp_active: AtomicBool,
    pub stall_result_latched: AtomicBool,
    pub stall_peak_error_units: AtomicU32,
    /// Detector tick at which the most recent latch set, 0 if none.
    pub stall_trigger_tick: AtomicU32,
    /// Which check latched during the most recent completed homing mode.
    /// Wire-encoded per `stall_detector::StallTriggerPath::as_u8` (0 = None,
    /// 1 = Ceiling, 2 = Margin).
    pub stall_trigger_path: AtomicU8,
    /// Largest `error - baseline` observed during the most recent completed
    /// homing mode, while the baseline was valid. 0 if it never warmed up.
    pub stall_peak_margin_delta_units: AtomicU32,
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
            config_apply_pending: AtomicU8::new(0),
            status_pin_asserted: AtomicBool::new(false),
            velocity_filter_hz: AtomicOptionalFilterHz::new(None),
            torque_filter_hz: AtomicOptionalFilterHz::new(None),
            position_filter_hz: AtomicOptionalFilterHz::new(None),
            flux_filter_hz: AtomicOptionalFilterHz::new(None),
            velocity_feedforward: AtomicBool::new(false),
            velocity_feedforward_gain_permille: AtomicU16::new(1_000),
            accel_feedforward: AtomicBool::new(false),
            accel_feedforward_accel_gain_permille: AtomicU16::new(1000),
            accel_feedforward_decel_gain_permille: AtomicU16::new(1000),
            feedforward_reset: AtomicBool::new(false),
            pid_error_sums_clear: AtomicBool::new(false),
            velocity_limit_cfg: AtomicU32::new(500_000),
            voltage_limit_cfg: AtomicU32::new(DEFAULT_OPERATIONAL_PIDOUT_UQ_UD_LIMIT),
            run_current_ma_cfg: AtomicU32::new(0),
            encoder_invert_cfg: AtomicOptionalInvert::new(None),
            pending_inner_gain_flux_p: AtomicU16::new(0),
            pending_inner_gain_flux_i: AtomicU16::new(0),
            pending_inner_gain_torque_p: AtomicU16::new(0),
            pending_inner_gain_torque_i: AtomicU16::new(0),
            pending_outer_gain_position_p: AtomicU16::new(0),
            pending_outer_gain_position_i: AtomicU16::new(0),
            pending_outer_gain_velocity_p: AtomicU16::new(0),
            pending_outer_gain_velocity_i: AtomicU16::new(0),
            adc_vm_offset_raw: AtomicU16::new(0),
            adc_vm_offset_sample_count: AtomicU8::new(0),
            encoder_reversed: AtomicBool::new(false),
            commission_in_flight: AtomicBool::new(false),
            cancel_requested: AtomicBool::new(false),
            commission_stimulus_active: AtomicBool::new(false),
            feedforward_surfaces_idle: AtomicBool::new(false),
            stimulus_prior_motor_armed: AtomicBool::new(false),
            stimulus_prior_motor_armed_captured: AtomicBool::new(false),
            auto_calibrate_on_enable_allowed: AtomicBool::new(false),
            position_p_value: AtomicU16::new(
                crate::pid::CONFIG_RESET_PLACEHOLDER_GAINS.position_p.raw(),
            ),
            position_i_value: AtomicU16::new(
                crate::pid::CONFIG_RESET_PLACEHOLDER_GAINS.position_i.raw(),
            ),
            velocity_p_value: AtomicU16::new(
                crate::pid::CONFIG_RESET_PLACEHOLDER_GAINS.velocity_p.raw(),
            ),
            velocity_i_value: AtomicU16::new(
                crate::pid::CONFIG_RESET_PLACEHOLDER_GAINS.velocity_i.raw(),
            ),
            velocity_ff_clamp_latched: AtomicBool::new(false),
            velocity_ff_clamp_count: AtomicU32::new(0),
            homing_current_ma: AtomicU32::new(0),
            stall_ceiling_units: AtomicU32::new(0),
            stall_margin_units: AtomicU32::new(0),
            stall_persistence: AtomicU8::new(0),
            stall_endstop_configured: AtomicBool::new(false),
            stall_latched: AtomicBool::new(false),
            homing_clamp_active: AtomicBool::new(false),
            stall_result_latched: AtomicBool::new(false),
            stall_peak_error_units: AtomicU32::new(0),
            stall_trigger_tick: AtomicU32::new(0),
            stall_trigger_path: AtomicU8::new(0),
            stall_peak_margin_delta_units: AtomicU32::new(0),
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
        self.status_pin_asserted.store(false, Relaxed);
        self.velocity_filter_hz.store(None, Relaxed);
        self.torque_filter_hz.store(None, Relaxed);
        self.position_filter_hz.store(None, Relaxed);
        self.flux_filter_hz.store(None, Relaxed);
        self.velocity_feedforward.store(false, Relaxed);
        self.velocity_feedforward_gain_permille
            .store(1_000, Relaxed);
        self.accel_feedforward.store(false, Relaxed);
        self.accel_feedforward_accel_gain_permille
            .store(1000, Relaxed);
        self.accel_feedforward_decel_gain_permille
            .store(1000, Relaxed);
        self.feedforward_reset.store(false, Relaxed);
        self.pid_error_sums_clear.store(false, Relaxed);
        self.velocity_limit_cfg.store(500_000, Relaxed);
        self.voltage_limit_cfg
            .store(DEFAULT_OPERATIONAL_PIDOUT_UQ_UD_LIMIT, Relaxed);
        self.run_current_ma_cfg.store(0, Relaxed);
        self.encoder_invert_cfg.store(None, Relaxed);
        self.pending_inner_gain_flux_p.store(0, Relaxed);
        self.pending_inner_gain_flux_i.store(0, Relaxed);
        self.pending_inner_gain_torque_p.store(0, Relaxed);
        self.pending_inner_gain_torque_i.store(0, Relaxed);
        self.pending_outer_gain_position_p.store(0, Relaxed);
        self.pending_outer_gain_position_i.store(0, Relaxed);
        self.pending_outer_gain_velocity_p.store(0, Relaxed);
        self.pending_outer_gain_velocity_i.store(0, Relaxed);
        self.config_apply_pending.store(0, Relaxed);
        self.encoder_reversed.store(false, Relaxed);
        self.commission_in_flight.store(false, Relaxed);
        self.cancel_requested.store(false, Relaxed);
        self.commission_stimulus_active.store(false, Relaxed);
        self.feedforward_surfaces_idle.store(false, Relaxed);
        self.stimulus_prior_motor_armed.store(false, Relaxed);
        self.stimulus_prior_motor_armed_captured
            .store(false, Relaxed);
        self.auto_calibrate_on_enable_allowed.store(false, Relaxed);
        self.store_outer_gains(crate::pid::CONFIG_RESET_PLACEHOLDER_GAINS, Relaxed);
        self.clear_velocity_clamp_latch();
        self.homing_current_ma.store(0, Relaxed);
        self.stall_ceiling_units.store(0, Relaxed);
        self.stall_margin_units.store(0, Relaxed);
        self.stall_persistence.store(0, Relaxed);
        self.stall_endstop_configured.store(false, Relaxed);
        self.stall_latched.store(false, Relaxed);
        self.homing_clamp_active.store(false, Relaxed);
        self.stall_result_latched.store(false, Relaxed);
        self.stall_peak_error_units.store(0, Relaxed);
        self.stall_trigger_tick.store(0, Relaxed);
        self.stall_trigger_path.store(0, Relaxed);
        self.stall_peak_margin_delta_units.store(0, Relaxed);
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

    pub(crate) fn clear_pi_representation_established(&self) {
        self.pi_representation_established
            .store(false, Ordering::Release);
    }

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

    /// Record requested inner-loop gains for the TMC task to apply.
    pub fn store_pending_inner_gains(&self, gains: InnerLoopGains, ordering: Ordering) {
        self.pending_inner_gain_flux_p
            .store(gains.flux_p.raw(), ordering);
        self.pending_inner_gain_flux_i
            .store(gains.flux_i.raw(), ordering);
        self.pending_inner_gain_torque_p
            .store(gains.torque_p.raw(), ordering);
        self.pending_inner_gain_torque_i
            .store(gains.torque_i.raw(), ordering);
    }

    /// Load and validate the recorded pending inner-loop gains.
    pub fn load_pending_inner_gains(
        &self,
        ordering: Ordering,
    ) -> Result<InnerLoopGains, PidGainOutOfRange> {
        Ok(InnerLoopGains {
            flux_p: PidPGain::new(self.pending_inner_gain_flux_p.load(ordering))?,
            flux_i: PidIGain::new(self.pending_inner_gain_flux_i.load(ordering))?,
            torque_p: PidPGain::new(self.pending_inner_gain_torque_p.load(ordering))?,
            torque_i: PidIGain::new(self.pending_inner_gain_torque_i.load(ordering))?,
        })
    }

    /// Record requested outer-loop gains for the TMC task to apply.
    pub fn store_pending_outer_gains(&self, gains: OuterLoopGains, ordering: Ordering) {
        self.pending_outer_gain_position_p
            .store(gains.position_p.raw(), ordering);
        self.pending_outer_gain_position_i
            .store(gains.position_i.raw(), ordering);
        self.pending_outer_gain_velocity_p
            .store(gains.velocity_p.raw(), ordering);
        self.pending_outer_gain_velocity_i
            .store(gains.velocity_i.raw(), ordering);
    }

    /// Load and validate the recorded pending outer-loop gains.
    pub fn load_pending_outer_gains(
        &self,
        ordering: Ordering,
    ) -> Result<OuterLoopGains, PidGainOutOfRange> {
        Ok(OuterLoopGains {
            position_p: PidPGain::new(self.pending_outer_gain_position_p.load(ordering))?,
            position_i: PidIGain::new(self.pending_outer_gain_position_i.load(ordering))?,
            velocity_p: PidPGain::new(self.pending_outer_gain_velocity_p.load(ordering))?,
            velocity_i: PidIGain::new(self.pending_outer_gain_velocity_i.load(ordering))?,
        })
    }

    #[cfg(test)]
    pub(crate) fn corrupt_outer_gain_for_test(&self, raw: u16) {
        self.position_p_value.store(raw, Ordering::Relaxed);
    }

    /// Validates and publishes a complete motion scale to its channel mirrors.
    ///
    /// # Errors
    ///
    /// Returns [`crate::motion_scale::MotionScaleError`] without changing any mirror when encoder
    /// or
    /// planner resolution is invalid.
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

    /// Capture `motor_armed` into `stimulus_prior_motor_armed` once per stimulus session.
    ///
    /// Returns `true` on the first call, `false` afterwards until
    /// [`Self::stimulus_restore_motor_armed`]
    /// closes the session; only the first reading is the real pre-session value.
    pub fn stimulus_save_motor_armed(&self) -> bool {
        if self
            .stimulus_prior_motor_armed_captured
            .swap(true, Ordering::Relaxed)
        {
            false
        } else {
            self.stimulus_prior_motor_armed
                .store(self.motor_armed.load(Ordering::Relaxed), Ordering::Relaxed);
            true
        }
    }

    /// Restore `motor_armed` to the value `stimulus_save_motor_armed` captured and close the
    /// session.
    ///
    /// A no-op when no session is open, so it never overwrites a genuinely armed state with a stale
    /// value.
    pub fn stimulus_restore_motor_armed(&self) {
        if self
            .stimulus_prior_motor_armed_captured
            .swap(false, Ordering::Relaxed)
        {
            self.motor_armed.store(
                self.stimulus_prior_motor_armed.load(Ordering::Relaxed),
                Ordering::Relaxed,
            );
        }
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

/// Runtime state for board-local channel 0.
pub fn tmc_channel0() -> &'static TmcChannelState {
    &TMC_CHANNELS[0]
}

/// Reset all TMC channel states to firmware defaults.
pub fn reset_all_tmc_channels() {
    for channel in &TMC_CHANNELS {
        channel.reset();
    }
}

/// Emergency motor disable signal. Set by enter_shutdown(), consumed by tmc_control.
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
        assert_eq!(size_of::<TmcChannelState>(), 148);
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
    fn optional_invert_distinguishes_unset_from_both_configured_values() {
        let invert = AtomicOptionalInvert::new(None);

        assert_eq!(invert.load(Relaxed), None);

        invert.store(Some(false), Relaxed);
        assert_eq!(invert.load(Relaxed), Some(false));

        invert.store(Some(true), Relaxed);
        assert_eq!(invert.load(Relaxed), Some(true));

        invert.store(None, Relaxed);
        assert_eq!(invert.load(Relaxed), None);
    }

    #[test]
    fn outer_gain_defaults_load_through_typed_boundary() {
        let state = TmcChannelState::new();

        assert_eq!(
            state.load_outer_gains(Relaxed),
            Ok(crate::pid::CONFIG_RESET_PLACEHOLDER_GAINS)
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
    fn reset_clears_pending_config_bits_with_the_values_they_point_at() {
        let channel = TmcChannelState::new();
        channel.run_current_ma_cfg.store(1_500, Relaxed);
        channel.encoder_invert_cfg.store(Some(true), Relaxed);
        channel.pending_inner_gain_flux_p.store(100, Relaxed);
        channel.pending_outer_gain_velocity_i.store(200, Relaxed);
        channel.config_apply_pending.store(
            PENDING_CURRENT
                | PENDING_ENCODER_DIR
                | PENDING_FILTERS
                | PENDING_INNER_GAINS
                | PENDING_OUTER_GAINS,
            Relaxed,
        );

        channel.reset();

        assert_eq!(
            channel.config_apply_pending.load(Relaxed),
            0,
            "a bit surviving the reset that wiped its recorded value points \
             the drain at a value the host never sent"
        );
        assert_eq!(channel.pending_inner_gain_flux_p.load(Relaxed), 0);
        assert_eq!(channel.pending_outer_gain_velocity_i.load(Relaxed), 0);
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
        assert_eq!(state.stall_ceiling_units.load(Relaxed), 0);
        assert_eq!(state.stall_margin_units.load(Relaxed), 0);
        assert_eq!(state.stall_persistence.load(Relaxed), 0);
        assert!(!state.stall_endstop_configured.load(Relaxed));
        assert!(!state.stall_latched.load(Relaxed));
        assert!(!state.homing_clamp_active.load(Relaxed));
        assert!(!state.stall_result_latched.load(Relaxed));
        assert_eq!(state.stall_peak_error_units.load(Relaxed), 0);
        assert_eq!(state.stall_trigger_tick.load(Relaxed), 0);
        assert_eq!(state.stall_trigger_path.load(Relaxed), 0);
        assert_eq!(state.stall_peak_margin_delta_units.load(Relaxed), 0);
    }

    #[test]
    fn reset_clears_the_commission_stimulus_opt_in_and_idle_mirror() {
        let channel = TmcChannelState::new();
        assert!(!channel.commission_stimulus_active.load(Relaxed));
        assert!(!channel.feedforward_surfaces_idle.load(Relaxed));

        channel.commission_stimulus_active.store(true, Relaxed);
        channel.feedforward_surfaces_idle.store(true, Relaxed);
        channel.reset();

        assert!(!channel.commission_stimulus_active.load(Relaxed));
        assert!(!channel.feedforward_surfaces_idle.load(Relaxed));

        channel.stimulus_prior_motor_armed.store(true, Relaxed);
        channel.reset();
        assert!(!channel.stimulus_prior_motor_armed.load(Relaxed));
    }

    #[test]
    fn stimulus_save_motor_armed_captures_only_the_first_call() {
        let channel = TmcChannelState::new();
        channel.motor_armed.store(false, Relaxed);

        assert!(channel.stimulus_save_motor_armed());
        assert!(!channel.stimulus_prior_motor_armed.load(Relaxed));

        channel.motor_armed.store(true, Relaxed);
        assert!(!channel.stimulus_save_motor_armed());
        assert!(!channel.stimulus_prior_motor_armed.load(Relaxed));
    }

    #[test]
    fn stimulus_save_motor_armed_captures_a_true_prior_value() {
        let channel = TmcChannelState::new();
        channel.motor_armed.store(true, Relaxed);

        assert!(channel.stimulus_save_motor_armed());
        assert!(channel.stimulus_prior_motor_armed.load(Relaxed));

        assert!(!channel.stimulus_save_motor_armed());
        assert!(channel.stimulus_prior_motor_armed.load(Relaxed));
    }

    #[test]
    fn stimulus_restore_motor_armed_restores_and_reopens_capture() {
        let channel = TmcChannelState::new();
        channel.motor_armed.store(true, Relaxed);
        assert!(channel.stimulus_save_motor_armed());

        channel.motor_armed.store(false, Relaxed);
        channel.stimulus_restore_motor_armed();
        assert!(channel.motor_armed.load(Relaxed));

        channel.motor_armed.store(false, Relaxed);
        assert!(channel.stimulus_save_motor_armed());
        assert!(!channel.stimulus_prior_motor_armed.load(Relaxed));
    }

    #[test]
    fn stimulus_restore_motor_armed_without_a_capture_leaves_motor_armed_untouched() {
        let channel = TmcChannelState::new();

        channel.motor_armed.store(false, Relaxed);
        assert!(channel.stimulus_save_motor_armed());
        channel.motor_armed.store(true, Relaxed);
        channel.stimulus_restore_motor_armed();
        assert!(!channel.motor_armed.load(Relaxed));
        assert!(!channel.stimulus_prior_motor_armed_captured.load(Relaxed));

        channel.motor_armed.store(true, Relaxed);

        channel.stimulus_restore_motor_armed();
        assert!(channel.motor_armed.load(Relaxed));
    }
}
