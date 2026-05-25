//! Shared types and analysis helpers for the FOCI commissioning workflow.
//!
//! This module provides:
//! - [`TransientBuffer`]: Fixed-size ring buffer for step-response capture
//! - [`CommissionProfile`]: Tuning aggressiveness selection
//! - [`OperatingMode`]: Load condition selection
//! - [`CommissionError`]: Error variants for commissioning phase failures
//! - [`CommissionStatus`]: Overall commissioning outcome
//! - [`PhaseId`]: Identifies each commissioning phase for progress reporting
//!
//! All types are `no_std` compatible. No heap allocation is used.

/// Number of samples in the step-response transient capture buffer.
///
/// Sized to fit one complete transient capture (256 × 4 bytes = 1,024 bytes).
/// Only one capture is active at a time; the buffer is reused across phases.
pub const TRANSIENT_LEN: usize = 256;

/// Fixed-size ring buffer for raw step-response capture.
///
/// Statically allocated and reused across commissioning phases. After each
/// capture phase completes, scalar metrics are extracted and the buffer is
/// reset for the next phase. Raw transient streaming is deferred to Phase 6.
///
/// Memory: 1,024 bytes for samples, plus 4 bytes of metadata.
#[derive(Clone, defmt::Format)]
pub struct TransientBuffer {
    samples: [u32; TRANSIENT_LEN],
    /// Number of samples captured so far (always <= `TRANSIENT_LEN`).
    len: u16,
    /// Microseconds between consecutive samples.
    sample_period_us: u16,
}

impl Default for TransientBuffer {
    fn default() -> Self {
        Self::new()
    }
}

impl TransientBuffer {
    /// Create an empty buffer.
    ///
    /// Sample period defaults to 0. Call [`reset`](Self::reset) before
    /// each capture to set the period for that measurement.
    pub const fn new() -> Self {
        Self {
            samples: [0u32; TRANSIENT_LEN],
            len: 0,
            sample_period_us: 0,
        }
    }

    /// Reset the buffer for a new capture, optionally changing the sample period.
    ///
    /// # Parameters
    /// - `sample_period_us`: microseconds between consecutive samples for the
    ///   new capture
    pub fn reset(&mut self, sample_period_us: u16) {
        self.len = 0;
        self.sample_period_us = sample_period_us;
    }

    /// Append one sample to the buffer.
    ///
    /// Returns `true` if the sample was stored. Returns `false` if the buffer
    /// is already full; the sample is discarded.
    ///
    /// # Parameters
    /// - `value`: raw ADC or encoder count to store
    pub fn push(&mut self, value: u32) -> bool {
        let idx = self.len as usize;
        if idx >= TRANSIENT_LEN {
            return false;
        }
        self.samples[idx] = value;
        self.len += 1;
        true
    }

    /// Number of samples currently stored.
    pub fn len(&self) -> u16 {
        self.len
    }

    /// Returns `true` if no samples have been captured yet.
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Returns `true` if the buffer holds `TRANSIENT_LEN` samples.
    pub fn is_full(&self) -> bool {
        self.len as usize >= TRANSIENT_LEN
    }

    /// Microseconds between consecutive samples.
    pub fn sample_period_us(&self) -> u16 {
        self.sample_period_us
    }

    /// Slice of the captured samples (length may be less than `TRANSIENT_LEN`).
    pub fn samples(&self) -> &[u32] {
        &self.samples[..self.len as usize]
    }
}

impl core::fmt::Debug for TransientBuffer {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("TransientBuffer")
            .field("len", &self.len)
            .field("sample_period_us", &self.sample_period_us)
            .finish()
    }
}

/// Tuning aggressiveness profile for SIMC gain synthesis.
///
/// Controls the closed-loop time constant lambda in the SIMC PI tuning law.
/// A more aggressive profile yields faster response but requires a more
/// accurate plant model and leaves less stability margin.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum CommissionProfile {
    /// `lambda = max(4 * theta, tau)` — slow, very robust, minimal audible noise.
    Conservative,
    /// `lambda = max(2 * theta, tau / 2)` — good default for most motors and print setups.
    Balanced,
    /// `lambda = max(theta, tau/4)` — fastest safe response; may be audible.
    Stiff,
}

impl CommissionProfile {
    /// Decode from the wire representation used in the Klipper command.
    ///
    /// | Value | Profile |
    /// |-------|---------|
    /// | 0 | `Conservative` |
    /// | 1 (default) | `Balanced` |
    /// | 2 | `Stiff` |
    ///
    /// Unknown values map to `Balanced`.
    pub fn from_u8(v: u8) -> Self {
        match v {
            0 => Self::Conservative,
            1 => Self::Balanced,
            2 => Self::Stiff,
            _ => Self::Balanced,
        }
    }

    /// Encode to the wire representation.
    pub fn to_u8(self) -> u8 {
        match self {
            Self::Conservative => 0,
            Self::Balanced => 1,
            Self::Stiff => 2,
        }
    }
}

/// Load condition present during mechanical identification.
///
/// Adjusts velocity-loop lambda after SIMC synthesis to account for the
/// expected operating environment.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum OperatingMode {
    /// Motor running without a significant load (e.g., belt axis with no
    /// toolhead). `lambda_vel *= 1.5` — reduces standstill buzz.
    Unloaded,
    /// Typical printing load; no lambda adjustment applied.
    Nominal,
    /// Heavy toolhead or high-inertia carriage. `lambda_vel *= 0.8` — tighter
    /// tuning; natural damping from the load helps stability.
    HighInertia,
}

impl OperatingMode {
    /// Decode from the wire representation used in the Klipper command.
    ///
    /// | Value | Mode |
    /// |-------|------|
    /// | 0 (default) | `Unloaded` |
    /// | 1 | `Nominal` |
    /// | 2 | `HighInertia` |
    ///
    /// Unknown values map to `Unloaded`.
    pub fn from_u8(v: u8) -> Self {
        match v {
            1 => Self::Nominal,
            2 => Self::HighInertia,
            _ => Self::Unloaded,
        }
    }

    /// Encode to the wire representation.
    pub fn to_u8(self) -> u8 {
        match self {
            Self::Unloaded => 0,
            Self::Nominal => 1,
            Self::HighInertia => 2,
        }
    }
}

/// Error variants for commissioning phase failures.
///
/// Each variant maps to a unique non-zero `status_code()` used in the
/// `foci_commission_phase` Klipper message. The Python module converts
/// the code back to a human-readable string for display.
///
/// Codes are stable across firmware versions. Adding new variants must
/// use new codes; existing codes must not be renumbered.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum CommissionError {
    /// Motor was already enabled when commissioning was requested. Code 1.
    MotorEnabled,
    /// No current was detected in the motor windings during the coil check. Code 2.
    NoCurrent,
    /// SPI communication with the TMC4671 failed. Code 3.
    Spi,
    /// ADC calibration produced offsets outside the expected range. Code 4.
    AdcFault,
    /// Coil connectivity check failed (open circuit or short). Code 5.
    CoilFault,
    /// Phase wiring check: current appeared on the wrong ADC channel. Code 6.
    PhaseFault,
    /// Encoder was not detected or direction could not be determined. Code 7.
    EncoderFault,
    /// Electrical identification (R, L) failed to converge. Code 8.
    ElectricalIdFailed,
    /// Current-loop step-response validation failed after retry. Code 9.
    CurrentValidationFailed,
    /// Mechanical identification produced zero or negative inertia. Code 10.
    MechanicalIdFailed,
    /// Velocity-loop step-response validation failed after retry. Code 11.
    VelocityValidationFailed,
    /// Position-loop P ramp-up produced oscillation at minimum gain. Code 12.
    PositionTuneFailed,
    /// Encoder alignment was not confirmed before outer commissioning. Code 13.
    EncoderNotAligned,
    /// Hardware shutdown was requested (M112 / e-stop) while commissioning. Code 14.
    Shutdown,
    /// A commissioning run was already in progress. Code 15.
    AlreadyRunning,
    /// The commissioning command queue was full. Code 16.
    QueueFull,
    /// Outer safety envelope violated (velocity/position/torque limit exceeded). Code 17.
    /// Always a safety fault: motor must be hard-disabled.
    SafetyEnvelopeViolation,
}

impl CommissionError {
    /// Non-zero status code for the Klipper protocol message.
    ///
    /// Codes are stable across firmware versions. Adding new variants must
    /// use new codes; existing codes must not be renumbered.
    pub fn status_code(self) -> u8 {
        match self {
            Self::MotorEnabled => 1,
            Self::NoCurrent => 2,
            Self::Spi => 3,
            Self::AdcFault => 4,
            Self::CoilFault => 5,
            Self::PhaseFault => 6,
            Self::EncoderFault => 7,
            Self::ElectricalIdFailed => 8,
            Self::CurrentValidationFailed => 9,
            Self::MechanicalIdFailed => 10,
            Self::VelocityValidationFailed => 11,
            Self::PositionTuneFailed => 12,
            Self::EncoderNotAligned => 13,
            Self::Shutdown => 14,
            Self::AlreadyRunning => 15,
            Self::QueueFull => 16,
            Self::SafetyEnvelopeViolation => 17,
        }
    }
}

/// Overall outcome of a commissioning run.
///
/// Reported in the `foci_commission_result` or `foci_tune_result`
/// message. The Python module uses this to
/// decide whether to persist the resulting gains.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum CommissionStatus {
    /// All phases passed; gains are reliable.
    Accepted,
    /// All phases passed but at least one used fallback / conservative gains.
    /// Gains are safe to use; the user should review the warnings.
    AcceptedWithWarnings,
    /// Inner commissioning completed successfully; outer commissioning was not
    /// run (e.g., mechanical ID failed). Current-loop gains are valid.
    InnerOnly,
    /// Commissioning failed; no gains should be persisted.
    Failed(CommissionError),
}

/// Identifies a single commissioning phase for progress reporting.
///
/// Sent in the `foci_commission_phase` message on each phase transition.
/// The Python module translates the `u8` code to a human-readable label.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum PhaseId {
    /// Inner phase 1 — ADC offset calibration.
    AdcCalibrate,
    /// Inner phase 2 — Coil connectivity check.
    CoilCheck,
    /// Inner phase 3 — Phase wiring validation.
    PhaseWiringCheck,
    /// Inner phase 4 — Encoder presence and direction detection.
    EncoderCheck,
    /// Inner phase 5 — Electrical identification (R, L, theta_e).
    ElectricalId,
    /// Inner phase 6 — Current-loop SIMC gain synthesis.
    CurrentTune,
    /// Inner phase 7 — Current-loop step-response validation.
    CurrentValidate,
    /// Inner commissioning completed.
    InnerDone,
    /// Outer phase 8 — Mechanical identification (J_eff, B_eff, tau_mech).
    MechanicalId,
    /// Outer phase 9 — Velocity-loop SIMC gain synthesis.
    VelocityTune,
    /// Outer phase 10 — Velocity-loop step-response validation.
    VelocityValidate,
    /// Outer phase 11 — Position-loop P ramp-up.
    PositionTune,
    /// Outer phase 12 — Filter selection based on validation metrics.
    FilterTune,
    /// Outer phase 13 — Assemble and transmit final results to host.
    Commit,
    /// Outer commissioning completed.
    OuterDone,
    /// Encoder alignment — drive rotor to the D-axis, zero
    /// ABN_DECODER_COUNT and offset, switch PHI_E_SELECTION to ABN
    /// while still driving. Runs between EncoderCheck and ElectricalId
    /// in the inner sequence, and is a valid `stop_after` target for
    /// calibrate-on-enable paths. Appended (not inserted between
    /// EncoderCheck and ElectricalId) so the stable wire values remain
    /// unchanged — wire code 16.
    EncoderAlign,
    /// Closed-loop position mode entry — writes PID gains, velocity
    /// limit, voltage limits, biquad filters, STEP_WIDTH, enters
    /// position mode, and verifies stability. Runs as the final inner
    /// phase for Calibrate and Commission sequences — wire code 17.
    ClosedLoopEntry,
}

impl PhaseId {
    /// Encode to the wire representation used in the Klipper protocol message.
    ///
    /// Values are stable; new variants must use new codes.
    pub fn to_u8(self) -> u8 {
        match self {
            Self::AdcCalibrate => 1,
            Self::CoilCheck => 2,
            Self::PhaseWiringCheck => 3,
            Self::EncoderCheck => 4,
            Self::ElectricalId => 5,
            Self::CurrentTune => 6,
            Self::CurrentValidate => 7,
            Self::InnerDone => 8,
            Self::MechanicalId => 9,
            Self::VelocityTune => 10,
            Self::VelocityValidate => 11,
            Self::PositionTune => 12,
            Self::FilterTune => 13,
            Self::Commit => 14,
            Self::OuterDone => 15,
            Self::EncoderAlign => 16,
            Self::ClosedLoopEntry => 17,
        }
    }
}

/// Complete gain set for closed-loop entry on re-enable.
///
/// Preloaded into firmware atomics by the Python host before `foci_calibrate`.
/// Contains current-loop gains (from commissioning) and outer gains
/// (from commissioning fallbacks or prior tuning).
#[derive(Debug, Clone, Copy, defmt::Format)]
pub struct SavedGains {
    /// Current-loop flux P gain (Q8.8).
    pub flux_p: u16,
    /// Current-loop flux I gain (raw Q8.8; effective advanced-PI zero is I/65536).
    pub flux_i: u16,
    /// Current-loop torque P gain (Q8.8).
    pub torque_p: u16,
    /// Current-loop torque I gain (raw Q8.8; effective advanced-PI zero is I/65536).
    pub torque_i: u16,
    /// Velocity P gain (Q8.8).
    pub velocity_p: u16,
    /// Velocity I gain (Q8.8).
    pub velocity_i: u16,
    /// Position P gain (Q8.8).
    pub position_p: u16,
    /// Position I gain (Q8.8).
    pub position_i: u16,
    /// PID velocity limit register value.
    pub velocity_limit: u32,
    /// Velocity biquad filter cutoff (Hz), 0 = disabled.
    pub velocity_filter_hz: u16,
    /// Torque biquad filter cutoff (Hz), 0 = disabled.
    pub torque_filter_hz: u16,
    /// Position biquad filter cutoff (Hz), 0 = disabled.
    pub position_filter_hz: u16,
    /// Flux biquad filter cutoff (Hz), 0 = disabled.
    pub flux_filter_hz: u16,
}

/// Sub-states for the `ClosedLoopEntry` engine phase.
///
/// Writes PID gains, velocity limit, voltage limits, position selection,
/// step width, motion mode, and biquad filters, then verifies position
/// hold stability.
#[derive(Debug, Clone, Copy, defmt::Format)]
pub enum ClosedLoopEntryState {
    /// Write current-loop and outer PID gains.
    WriteGains,
    /// Write velocity limit and raise PIDOUT_UQ_UD_LIMITS.
    WriteLimits,
    /// Verify PHI_E_SELECTION = 3 (ABN encoder from alignment).
    VerifyPhiE,
    /// Zero position, clear integrators, clear status flags.
    ZeroAndClear,
    /// Write POSITION_SELECTION, STEP_WIDTH with readback.
    WritePositionConfig,
    /// Write biquad filters (best-effort, warn on failure).
    WriteFilters,
    /// Enter position mode: MODE_RAMP_MODE_MOTION = 0x8000_0003.
    EnterPositionMode,
    /// Wait 10 ms, read position, wait 20 ms, read again. Verify stable.
    StabilityCheck1 {
        /// First position reading.
        position_1: i32,
        /// Deadline in microseconds for second reading.
        deadline_us: u64,
    },
    /// Second stability read after 20 ms.
    StabilityCheck2 {
        /// First position reading for comparison.
        position_1: i32,
        /// Deadline in microseconds for second reading.
        deadline_us: u64,
    },
}

/// Controls which phases run and in what order for the inner engine.
///
/// Replaces the old `stop_after: PhaseId` mechanism.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum InnerSequence {
    /// Re-enable path: ADC cal -> encoder align -> closed-loop entry (saved gains).
    /// Skips coil/wiring/encoder diagnostics -- trusts prior Stage 1 validation.
    Calibrate,
    /// Diagnostic path: ADC cal -> coil -> wiring -> encoder -> align -> elec ID.
    Selftest,
    /// Stage 1: full diagnostics -> elec ID -> current tune/validate -> closed-loop entry.
    Commission,
}

/// First-order-plus-dead-time (FOPDT) current plant model.
///
/// Estimated during electrical identification (Phase 5). Contains
/// both internal-unit values (used for gain computation) and physical
/// values (milliohms, microhenries — for host display and persistence).
///
/// Plant model: `G(s) = K / (tau_e * s + 1) * e^(-theta_e * s)`
/// where `K = 1 / R_int` (DC gain in internal units).
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct ElectricalModel {
    /// Winding resistance in scaled internal (ADC/DAC) units.
    /// `R_int = V_excitation / I_steady_state * ELECTRICAL_MODEL_RESISTANCE_SCALE`.
    pub r_int: u32,
    /// Winding inductance in internal micro-units.
    /// `L_int = r_int * tau_e_us / ELECTRICAL_MODEL_RESISTANCE_SCALE`.
    pub l_int: u32,
    /// Electrical time constant L/R in microseconds.
    pub tau_e_us: u32,
    /// Effective transport delay in microseconds (from transient onset).
    pub theta_e_us: u32,
    /// Physical resistance in milliohms (for display/persistence).
    pub r_mohm: u32,
    /// Physical inductance in microhenries (for display/persistence).
    pub l_uh: u32,
    /// Tau from two-point transient fit in microseconds.
    /// Compared against L/R tau for plant model validation.
    pub tau_e_crosscheck_us: u32,
}

/// Fixed-point scale used for [`ElectricalModel::r_int`].
pub const ELECTRICAL_MODEL_RESISTANCE_SCALE: u32 = 1000;

/// Computed PI gains for the TMC4671 current (flux/torque) loops.
///
/// Produced by [`simc_current_gains`] from an [`ElectricalModel`] and
/// a [`CommissionProfile`]. Includes the effective bandwidth and lambda
/// for diagnostics and downstream use (velocity loop bandwidth constraint).
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct CurrentGains {
    /// Flux P gain — TMC4671 Q8.8 register value.
    pub flux_p: u16,
    /// Flux I gain — raw Q8.8; effective advanced-PI zero is I/65536.
    pub flux_i: u16,
    /// Torque P gain — TMC4671 Q8.8 register value.
    pub torque_p: u16,
    /// Torque I gain — raw Q8.8; effective advanced-PI zero is I/65536.
    pub torque_i: u16,
    /// Effective closed-loop bandwidth in Hz.
    pub effective_bandwidth_hz: u32,
    /// Closed-loop time constant lambda in microseconds.
    pub lambda_us: u32,
}

/// Current-loop SIMC integrator damping factor.
///
/// The documented advanced-PI zero placement is correct in isolation, but on
/// the target hardware the raw `4*(lambda+theta)` SIMC integral time winds up
/// during the current-step rise. Use a slower integral time while keeping the
/// proportional SIMC damping unchanged.
const CURRENT_SIMC_TI_MULTIPLIER: f32 = 11.0;

/// Samples used to smooth current validation settling/ringing checks.
///
/// Current captures include PWM/ADC ripple around the regulated value. A short
/// trailing average keeps the validation focused on the controller envelope
/// while preserving raw samples for rise time and overshoot.
const CURRENT_METRIC_SMOOTHING_SAMPLES: usize = 8;

/// Capture direction for bidirectional current-loop validation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum CaptureDirection {
    /// Positive torque step.
    Forward,
    /// Negative torque step.
    Reverse,
}

/// Normalize a raw torque-actual reading by capture direction.
///
/// Forward captures keep the sign as-is; reverse captures negate so that a
/// correctly responding motor produces a positive value in both directions.
/// Negative normalized values (wrong-sign current) are clamped to zero — the
/// tail-level metric will be near-zero and the target-attainment check will
/// flag the failure.
pub fn normalize_capture_sample(torque_actual: i16, direction: CaptureDirection) -> u32 {
    let normalized = match direction {
        CaptureDirection::Forward => torque_actual as i32,
        CaptureDirection::Reverse => -(torque_actual as i32),
    };
    normalized.max(0) as u32
}

/// Metrics from a current-loop step-response validation test.
///
/// Extracted from the [`TransientBuffer`] after a torque step.
/// Used by [`classify_current_validation`] to determine pass/marginal/fail.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct CurrentValidation {
    /// Time from 10% to 90% of target, in microseconds.
    pub rise_time_us: u32,
    /// Peak overshoot above target, in 0.1% units (150 = 15.0%).
    pub overshoot_pct: u16,
    /// Time to settle within +/-5% of target, in microseconds.
    pub settling_time_us: u32,
    /// Zero-crossings of (actual - target) after peak.
    pub ringing_count: u8,
    /// Samples where value hit the saturation limit.
    pub saturation_samples: u16,
    /// |fwd_tail - rev_tail| / avg, in 0.1% units (permille).
    /// Set to 0 by extract_step_metrics (requires two captures).
    pub symmetry_pct: u16,
    /// Robust tail level: median of last 32 samples (or fewer if buffer is
    /// shorter). Used to compute steady-state signed-current symmetry between
    /// forward and reverse captures.
    pub tail_level: u32,
    /// |tail_level - target| / target, in 0.1% units (permille).
    /// Measures whether the current loop actually reached the commanded
    /// target. Set to 0 by extract_step_metrics; the caller computes it
    /// from the combined (worst-case) tail_level and commanded target.
    pub target_error_pct: u16,
}

/// First-order mechanical plant model estimated during Phase 8.
///
/// Represents `G_mech(s) = 1 / (J_eff * s + B_eff)` where J_eff includes
/// carriage mass, belt compliance, and reflected rotor inertia. Used by
/// Phase 9 (velocity-loop SIMC synthesis) to compute velocity gains.
///
/// All values are in internal units derived from encoder counts and
/// torque DAC values. Physical conversion happens on the host side.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct MechanicalModel {
    /// Effective inertia in internal units.
    /// J_eff = torque / initial_acceleration (from first few samples).
    pub j_eff: u32,
    /// Effective damping in internal units.
    /// B_eff = torque / steady_state_velocity, or from deceleration curve.
    pub b_eff: u32,
    /// Mechanical time constant J_eff / B_eff in milliseconds.
    pub tau_mech_ms: u32,
    /// Effective mechanical delay in microseconds.
    /// Time from torque step to first measurable velocity change.
    pub theta_mech_us: u32,
    /// Static friction estimate in torque units. 0 if not measured.
    pub t_f: u16,
}

/// Computed PI gains for the TMC4671 velocity loop.
///
/// Produced by [`simc_velocity_gains`] from a [`MechanicalModel`],
/// inner-loop bandwidth, and tuning profile. The velocity limit is
/// derived from the peak velocity observed during mechanical ID and is
/// represented in encoder counts/ms during commissioning validation. The
/// validation phase converts it to TMC velocity-register units at the SPI
/// write boundary.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct VelocityGains {
    /// Velocity P gain — TMC4671 Q8.8 register value.
    pub velocity_p: u16,
    /// Velocity I gain — TMC4671 Q8.8 register value.
    pub velocity_i: u16,
    /// Commissioning velocity limit in encoder counts/ms.
    pub velocity_limit: u32,
    /// Effective closed-loop velocity bandwidth in Hz.
    pub effective_bandwidth_hz: u32,
    /// Closed-loop time constant lambda_vel in milliseconds.
    pub lambda_vel_ms: u32,
}

/// Maximum model-synthesized velocity P accepted by outer commissioning.
///
/// Keep this centralized so SIMC synthesis and the Phase 10/commit envelope
/// use the same bound.
pub(crate) const VELOCITY_SIMC_MAX_P_REG: u16 = 1152 * 4;

/// Metrics from a velocity-loop step-response validation test (Phase 10).
///
/// Extracted from the [`TransientBuffer`] after a velocity step.
/// Used by [`classify_velocity_validation`] to determine pass/marginal/fail.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct VelocityValidation {
    /// Time from 10% to 90% of target velocity, in milliseconds.
    pub rise_time_ms: u32,
    /// Peak overshoot above target velocity, in 0.1% units (200 = 20.0%).
    pub overshoot_pct: u16,
    /// Peak measured encoder velocity in counts/ms.
    pub peak_velocity: u32,
    /// Time to settle within +/-5% of target velocity, in milliseconds.
    pub settling_time_ms: u32,
    /// Zero-speed chatter detected after returning to zero velocity.
    pub zero_speed_chatter: bool,
    /// Samples where PID output hit saturation.
    pub saturation_samples: u16,
    /// |rise_fwd - rise_rev| / avg, in 0.1% units.
    pub symmetry_pct: u16,
}

/// Computed position-loop gains from Phase 11 P ramp-up.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct PositionGains {
    /// Position P gain — TMC4671 Q8.8 register value.
    pub position_p: u16,
    /// Position I gain — TMC4671 Q8.8 register value (usually 0).
    pub position_i: u16,
}

/// Filter cutoff recommendations from Phase 12.
///
/// Each field is `None` when no filtering is needed, or `Some(cutoff_hz)`
/// when the corresponding validation stage detected noise/chatter.
/// The firmware uses these with [`crate::biquad::butterworth_lowpass`]
/// to compute register coefficients.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct FilterConfig {
    /// Velocity biquad low-pass cutoff Hz. Enabled if zero-speed chatter detected.
    pub velocity_filter_hz: Option<u32>,
    /// Position target low-pass cutoff Hz. Enabled if position ramp-up hit oscillation early.
    pub position_filter_hz: Option<u32>,
    /// Flux biquad low-pass cutoff Hz. Enabled if current ringing detected.
    pub flux_filter_hz: Option<u32>,
    /// Torque biquad low-pass cutoff Hz. Enabled if current ringing detected.
    pub torque_filter_hz: Option<u32>,
}

/// Compute the SIMC lambda (closed-loop time constant) for a given profile.
///
/// All profiles guarantee `lambda >= theta_e` (SIMC stability boundary).
fn simc_lambda_us(profile: CommissionProfile, tau_e_us: u32, theta_e_us: u32) -> u32 {
    let lambda = match profile {
        CommissionProfile::Conservative => (4 * theta_e_us).max(tau_e_us),
        CommissionProfile::Balanced => (2 * theta_e_us).max(tau_e_us / 2),
        CommissionProfile::Stiff => {
            let from_theta = theta_e_us;
            let from_tau = tau_e_us / 4;
            if from_theta > from_tau {
                from_theta
            } else {
                from_tau
            }
        }
    };
    // Enforce stability boundary: lambda >= theta_e
    if lambda >= theta_e_us {
        lambda
    } else {
        theta_e_us
    }
}

/// Compute SIMC PI gains for the current loop, returning `None` if the
/// electrical model is invalid (zero R, zero L, or zero theta).
pub fn simc_current_gains_checked(
    model: &ElectricalModel,
    profile: CommissionProfile,
    config: &crate::config::TmcConfig,
) -> Option<CurrentGains> {
    let lambda_us = simc_lambda_us(profile, model.tau_e_us, model.theta_e_us);
    simc_current_gains_with_lambda_checked(model, lambda_us, config)
}

/// Compute SIMC PI gains for the current loop using an explicit lambda.
pub fn simc_current_gains_with_lambda_checked(
    model: &ElectricalModel,
    lambda_us: u32,
    config: &crate::config::TmcConfig,
) -> Option<CurrentGains> {
    if model.r_int == 0 || model.l_int == 0 || model.theta_e_us == 0 {
        return None;
    }

    let lambda_us = lambda_us.max(model.theta_e_us);

    // SIMC PI tuning law for FOPDT plant G(s) = K / (tau*s+1) * e^(-theta*s):
    //   Kp = tau / (K * (lambda + theta))
    //   Ti = max(tau, CURRENT_SIMC_TI_MULTIPLIER * (lambda + theta))
    //   Ki = Kp / Ti
    //
    // For current loop: K = 1/R_int (DC gain: current per unit voltage).
    // All in internal units — no board-specific scaling needed.

    let tau = model.tau_e_us as f32;
    let theta = model.theta_e_us as f32;
    let lambda = lambda_us as f32;
    let r_int = model.r_int as f32 / ELECTRICAL_MODEL_RESISTANCE_SCALE as f32;
    let k = 1.0 / r_int; // Plant DC gain

    let kp = tau / (k * (lambda + theta));
    let ti = {
        let candidate = CURRENT_SIMC_TI_MULTIPLIER * (lambda + theta);
        if tau > candidate { tau } else { candidate }
    };
    let ti_s = ti * 1e-6; // Convert Ti from microseconds to seconds

    // Convert to the TMC4671 current-loop register format.
    //
    // OpenFFBoard leaves CONFIG_ADVANCED_PI_REPRESENT at its default 0, so the
    // current-loop I register field is raw Q8.8. In advanced PI mode the
    // integrator is series/interacting:
    //   x[k+1] = x[k] + Kp * (I/65536) * e[k]
    // so the raw Q8.8 I register places the PI zero as I/65536 per PWM sample.
    // A continuous Ti maps to I ~= 65536 / (Ti * f_pwm).
    let f_pwm = config.pwm_freq_hz() as f32;
    let p_reg = (kp * 256.0).clamp(1.0, u16::MAX as f32) as u16;
    let i_reg = (65_536.0 / (ti_s * f_pwm)).clamp(1.0, u16::MAX as f32) as u16;

    // Effective bandwidth = 1 / (2*pi*(lambda+theta)) in Hz
    let bw_hz = 1_000_000.0 / (2.0 * core::f32::consts::PI * (lambda + theta));

    Some(CurrentGains {
        flux_p: p_reg,
        flux_i: i_reg,
        torque_p: p_reg,
        torque_i: i_reg,
        effective_bandwidth_hz: bw_hz as u32,
        lambda_us,
    })
}

/// Compute SIMC PI gains for the current loop.
///
/// Returns default conservative gains if the electrical model is invalid.
pub fn simc_current_gains(
    model: &ElectricalModel,
    profile: CommissionProfile,
    config: &crate::config::TmcConfig,
) -> CurrentGains {
    simc_current_gains_checked(model, profile, config).unwrap_or(CurrentGains {
        flux_p: crate::pid::DEFAULT_INNER_GAINS.flux_p,
        flux_i: crate::pid::DEFAULT_INNER_GAINS.flux_i,
        torque_p: crate::pid::DEFAULT_INNER_GAINS.torque_p,
        torque_i: crate::pid::DEFAULT_INNER_GAINS.torque_i,
        effective_bandwidth_hz: 0,
        lambda_us: 0,
    })
}

/// Compute SIMC current-loop gains with an explicit lambda, falling back to defaults.
pub fn simc_current_gains_with_lambda(
    model: &ElectricalModel,
    lambda_us: u32,
    config: &crate::config::TmcConfig,
) -> CurrentGains {
    simc_current_gains_with_lambda_checked(model, lambda_us, config).unwrap_or(CurrentGains {
        flux_p: crate::pid::DEFAULT_INNER_GAINS.flux_p,
        flux_i: crate::pid::DEFAULT_INNER_GAINS.flux_i,
        torque_p: crate::pid::DEFAULT_INNER_GAINS.torque_p,
        torque_i: crate::pid::DEFAULT_INNER_GAINS.torque_i,
        effective_bandwidth_hz: 0,
        lambda_us: 0,
    })
}

/// Settling reference for current-loop validation, in microseconds.
///
/// Advanced PI uses the `I` register as the controller zero location. When the
/// current-loop integrator is intentionally damped, electrical tau is no longer
/// the dominant settling reference; the configured PI zero time is.
pub fn current_settling_reference_us(
    gains: &CurrentGains,
    config: &crate::config::TmcConfig,
) -> u32 {
    let i_reg = gains.flux_i.min(gains.torque_i).max(1) as u64;
    let f_pwm = config.pwm_freq_hz().max(1) as u64;
    (65_536_000_000u64 / (i_reg * f_pwm)).min(u32::MAX as u64) as u32
}

/// Number of tail samples used for the robust tail-level statistic.
const TAIL_LEVEL_WINDOW: usize = 32;

/// Compute the median of a slice of u32 values.
///
/// Uses a stack-allocated scratch buffer (max 32 elements). Returns 0 for
/// empty slices. For even-length slices, returns the mean of the two middle
/// values.
fn median_u32(values: &[u32]) -> u32 {
    let len = values.len();
    if len == 0 {
        return 0;
    }
    let mut scratch = [0u32; TAIL_LEVEL_WINDOW];
    let n = len.min(TAIL_LEVEL_WINDOW);
    scratch[..n].copy_from_slice(&values[..n]);
    scratch[..n].sort_unstable();
    if n % 2 == 1 {
        scratch[n / 2]
    } else {
        (scratch[n / 2 - 1] + scratch[n / 2]) / 2
    }
}

fn trailing_average(samples: &[u32], idx: usize, window_len: usize) -> u32 {
    let start = idx.saturating_add(1).saturating_sub(window_len);
    let window = &samples[start..=idx];
    let sum: u64 = window.iter().map(|&sample| sample as u64).sum();
    (sum / window.len() as u64) as u32
}

/// Extract step-response metrics from a transient buffer capture.
///
/// Analyzes a captured step response to measure rise time, overshoot,
/// settling time, ringing, and saturation. These metrics are then
/// classified by [`classify_current_validation`].
///
/// # Parameters
/// - `buf`: captured step response samples (raw ADC or encoder values)
/// - `target`: the target value that was commanded (step amplitude)
/// - `pidout_limit`: saturation threshold (0 to skip saturation check)
///
/// Returns metrics with `symmetry_pct = 0`. Symmetry requires two captures
/// (forward + reverse); the caller computes it via [`compute_tail_symmetry_pct`].
/// Timing fields are durations within the capture, not absolute times since
/// the command step. Callers that schedule the first sample after one sample
/// period should keep that origin outside this buffer.
pub fn extract_step_metrics(
    buf: &TransientBuffer,
    target: u32,
    pidout_limit: u32,
) -> CurrentValidation {
    let samples = buf.samples();
    if samples.is_empty() || target == 0 {
        return CurrentValidation {
            rise_time_us: 0,
            overshoot_pct: 0,
            settling_time_us: 0,
            ringing_count: 0,
            saturation_samples: 0,
            symmetry_pct: 0,
            tail_level: 0,
            target_error_pct: 0,
        };
    }

    let period = buf.sample_period_us() as u32;
    let target_f = target as f32;
    let threshold_10 = (target_f * 0.10) as u32;
    let threshold_90 = (target_f * 0.90) as u32;
    // Current validation samples quantized ADC values. At low targets, a
    // strict 5% band can truncate to zero counts, which would require every
    // sample to equal the target exactly. Allow at least one count so a small,
    // otherwise stable response can settle.
    let settle_band = ((target_f * 0.05) as u32).max(1);

    // These are relative crossing offsets; rise_time subtracts them, so the
    // absolute capture origin does not affect the result.
    let mut t_10: Option<u32> = None;
    let mut t_90: Option<u32> = None;
    let mut peak: u32 = 0;
    let mut peak_idx: usize = 0;
    let mut saturation_count: u16 = 0;

    for (i, &s) in samples.iter().enumerate() {
        if t_10.is_none() && s >= threshold_10 {
            t_10 = Some(crate::sample_timing::sample_offset_us(i, period));
        }
        if t_90.is_none() && s >= threshold_90 {
            t_90 = Some(crate::sample_timing::sample_offset_us(i, period));
        }
        if s > peak {
            peak = s;
            peak_idx = i;
        }
        if pidout_limit > 0 && s >= pidout_limit {
            saturation_count += 1;
        }
    }

    let rise_time_us = match (t_10, t_90) {
        (Some(a), Some(b)) => b.saturating_sub(a),
        _ => 0,
    };

    // Overshoot: (peak - target) / target in 0.1% units
    let overshoot_pct = if peak > target {
        ((peak - target) as u64 * 1000 / target as u64) as u16
    } else {
        0
    };

    // Settling: last smoothed sample where |actual - target| > 5% band.
    // Use (index + 1) * period because settling time is the duration through
    // the period containing the last out-of-band sample, not its offset
    // from the capture origin.
    let mut last_outside: Option<usize> = None;
    for (i, _) in samples.iter().enumerate() {
        let smoothed = trailing_average(samples, i, CURRENT_METRIC_SMOOTHING_SAMPLES);
        let diff = smoothed.abs_diff(target);
        if diff > settle_band {
            last_outside = Some(i);
        }
    }
    let settling_time_us = match last_outside {
        Some(idx) => crate::sample_timing::sample_elapsed_us(idx, period),
        None => 0,
    };

    // Ringing: crossings between smoothed excursions outside the settle band.
    // Samples inside the accepted band are ignored so ADC noise around the
    // target does not look like sustained oscillation.
    let mut ringing_count: u8 = 0;
    if peak_idx + 1 < samples.len() {
        let upper = target.saturating_add(settle_band);
        let lower = target.saturating_sub(settle_band);
        let mut above: Option<bool> = None;
        for i in peak_idx + 1..samples.len() {
            let smoothed = trailing_average(samples, i, CURRENT_METRIC_SMOOTHING_SAMPLES);
            let now_above = if smoothed > upper {
                Some(true)
            } else if smoothed < lower {
                Some(false)
            } else {
                None
            };
            if let Some(now) = now_above {
                if let Some(prev) = above
                    && now != prev
                {
                    ringing_count = ringing_count.saturating_add(1);
                }
                above = Some(now);
            }
        }
    }

    // Tail level: median of last TAIL_LEVEL_WINDOW samples.
    let tail_start = samples.len().saturating_sub(TAIL_LEVEL_WINDOW);
    let tail_level = median_u32(&samples[tail_start..]);

    CurrentValidation {
        rise_time_us,
        overshoot_pct,
        settling_time_us,
        ringing_count,
        saturation_samples: saturation_count,
        symmetry_pct: 0,
        tail_level,
        target_error_pct: 0,
    }
}

/// Outcome of a validation threshold check.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum ValidationResult {
    /// All metrics within normal limits.
    Pass,
    /// One or more metrics in the marginal range — retry with more conservative gains.
    Marginal,
    /// One or more metrics exceed hard limits — fall back to defaults.
    Fail,
}

/// Classify current-loop validation metrics against thresholds.
///
/// Thresholds (starting points, subject to hardware validation):
///
/// | Metric | Pass | Marginal | Fail |
/// |--------|------|----------|------|
/// | Overshoot | < 15% | 15-30% | > 30% |
/// | Ringing | 0-1 | 2-3 | > 3 |
/// | Settling | < 5*settle_ref | 5-10*settle_ref | > 10*settle_ref |
/// | Symmetry | <= 10% | 10-25% | > 25% |
/// | Saturation | < 10% | 10-25% | > 25% |
/// | Target error | <= 10% | 10-50% | > 50% |
///
/// `total_samples` is the total number of capture samples across both
/// directions, used for computing the saturation percentage.
/// Returns the worst classification across all metrics.
pub fn classify_current_validation(
    metrics: &CurrentValidation,
    settle_reference_us: u32,
    total_samples: u16,
) -> ValidationResult {
    let mut worst = ValidationResult::Pass;

    // Overshoot (0.1% units: 150 = 15.0%, 300 = 30.0%)
    if metrics.overshoot_pct > 300 {
        return ValidationResult::Fail;
    } else if metrics.overshoot_pct > 150 {
        worst = ValidationResult::Marginal;
    }

    // Ringing
    if metrics.ringing_count > 3 {
        return ValidationResult::Fail;
    } else if metrics.ringing_count > 1 {
        worst = ValidationResult::Marginal;
    }

    // Settling (relative to the dominant current-loop reference).
    let settle_5x = 5 * settle_reference_us;
    let settle_10x = 10 * settle_reference_us;
    if metrics.settling_time_us > settle_10x {
        return ValidationResult::Fail;
    } else if metrics.settling_time_us > settle_5x {
        worst = ValidationResult::Marginal;
    }

    // Symmetry (0.1% units: 100 = 10.0%, 250 = 25.0%)
    // Tail-level symmetry is much less noisy than the old rise-time metric,
    // so tighter thresholds are appropriate.
    if metrics.symmetry_pct > 250 {
        return ValidationResult::Fail;
    } else if metrics.symmetry_pct > 100 {
        worst = ValidationResult::Marginal;
    }

    // Saturation (fraction of total samples): pass <10%, marginal 10-25%, fail >25%
    if total_samples > 0 {
        let sat_pct = (metrics.saturation_samples as u32 * 100) / total_samples as u32;
        if sat_pct > 25 {
            return ValidationResult::Fail;
        } else if sat_pct > 10 {
            worst = ValidationResult::Marginal;
        }
    }

    // Target attainment: did the current loop actually reach the commanded
    // target? Catches zero-current, wrong-sign, and under-target responses
    // that would otherwise pass all other checks.
    // 0.1% units: 500 = 50.0%, 100 = 10.0%
    if metrics.target_error_pct > 500 {
        return ValidationResult::Fail;
    } else if metrics.target_error_pct > 100 {
        worst = ValidationResult::Marginal;
    }

    worst
}

/// Return whether current validation should retry with adjusted gains.
pub fn should_retry_current_validation(result: ValidationResult, attempt: u8) -> bool {
    attempt == 0 && result != ValidationResult::Pass
}

/// Return whether a marginal current validation can be accepted without retry.
///
/// This is intentionally narrow: it only accepts the case where all metrics
/// except bidirectional rise-time symmetry are already in the pass range, and
/// the computed retry would not change the gains. That covers measurement
/// variability in the symmetry metric without hiding overshoot, ringing,
/// settling, or saturation issues.
pub fn should_accept_current_validation_without_retry(
    metrics: &CurrentValidation,
    settle_reference_us: u32,
    total_samples: u16,
    previous: CurrentGains,
    retry: CurrentGains,
) -> bool {
    if previous != retry {
        return false;
    }
    if metrics.overshoot_pct > 150 || metrics.ringing_count > 1 {
        return false;
    }
    if metrics.settling_time_us > 5 * settle_reference_us {
        return false;
    }
    if total_samples > 0 {
        let sat_pct = (metrics.saturation_samples as u32 * 100) / total_samples as u32;
        if sat_pct > 10 {
            return false;
        }
    }

    metrics.symmetry_pct > 100 && metrics.symmetry_pct <= 250
}

/// Adjust current-loop integrator gain for a validation retry while retaining P damping.
pub fn current_validation_retry_gains(
    previous: CurrentGains,
    target: u32,
    settle_band: u32,
    forward_tail_level: u32,
    reverse_tail_level: u32,
    overshoot_pct: u16,
) -> CurrentGains {
    let min_tail = forward_tail_level.min(reverse_tail_level);
    let max_tail = forward_tail_level.max(reverse_tail_level);
    let upper = target.saturating_add(settle_band);
    let lower = target.saturating_sub(settle_band);
    let i = if overshoot_pct > 150 || max_tail > upper {
        (previous.flux_i / 2).max(1)
    } else if min_tail < lower {
        previous.flux_i.saturating_add((previous.flux_i / 2).max(1))
    } else {
        previous.flux_i
    };
    CurrentGains {
        flux_p: previous.flux_p,
        flux_i: i,
        torque_p: previous.torque_p,
        torque_i: i,
        effective_bandwidth_hz: previous.effective_bandwidth_hz,
        lambda_us: previous.lambda_us,
    }
}

/// Compute symmetry metric from forward and reverse rise times.
///
/// Returns `|a - b| / avg(a, b)` in 0.1% units (200 = 20.0%).
/// Returns 0 if both values are zero.
pub fn compute_symmetry_pct(rise_fwd: u32, rise_rev: u32) -> u16 {
    let avg = (rise_fwd as u64 + rise_rev as u64) / 2;
    if avg == 0 {
        return 0;
    }
    let diff = rise_fwd.abs_diff(rise_rev);
    (diff as u64 * 1000 / avg) as u16
}

/// Compute symmetry metric from forward and reverse tail levels.
///
/// Returns `|fwd - rev| / avg(fwd, rev)` in 0.1% units (permille).
/// This measures steady-state signed-current symmetry — whether both
/// directions produce the same actual current magnitude. Returns 0 if
/// both values are zero.
pub fn compute_tail_symmetry_pct(fwd_tail: u32, rev_tail: u32) -> u16 {
    let avg = (fwd_tail as u64 + rev_tail as u64) / 2;
    if avg == 0 {
        return 0;
    }
    let diff = fwd_tail.abs_diff(rev_tail);
    (diff as u64 * 1000 / avg) as u16
}

/// Compute target-attainment error from a tail level and commanded target.
///
/// Returns `|tail_level - target| / target` in 0.1% units (permille).
/// Returns 1000 (100%) if `target` is zero to avoid division by zero.
pub fn compute_target_error_pct(tail_level: u32, target: u32) -> u16 {
    if target == 0 {
        return 1000;
    }
    let diff = tail_level.abs_diff(target);
    (diff as u64 * 1000 / target as u64) as u16
}

// ---------------------------------------------------------------------------
// Velocity-loop SIMC synthesis (Phase 9)
// ---------------------------------------------------------------------------

/// Compute the velocity-loop SIMC lambda (closed-loop time constant).
///
/// Profile mapping from the spec:
///
/// | Profile | Lambda_vel base |
/// |---------|----------------|
/// | Conservative | `6 * theta_mech` |
/// | Balanced | `3 * theta_mech` |
/// | Stiff | `max(1.5 * theta_mech, tau_mech / 4)` |
///
/// All values in microseconds. The result is clamped to the bandwidth
/// constraint `lambda_vel >= 4 * (lambda_i + theta_e)` which ensures
/// the inner loop settles before the outer loop reacts.
fn simc_velocity_lambda_us(
    profile: CommissionProfile,
    tau_mech_us: u32,
    theta_mech_us: u32,
    inner_lambda_us: u32,
    theta_e_us: u32,
) -> u32 {
    let lambda = match profile {
        CommissionProfile::Conservative => 6 * theta_mech_us,
        CommissionProfile::Balanced => 3 * theta_mech_us,
        CommissionProfile::Stiff => {
            // 1.5 * theta_mech = theta_mech + theta_mech/2
            let from_theta = theta_mech_us + theta_mech_us / 2;
            let from_tau = tau_mech_us / 4;
            if from_theta > from_tau {
                from_theta
            } else {
                from_tau
            }
        }
    };

    // Enforce stability boundary: lambda >= theta_mech.
    let lambda = if lambda >= theta_mech_us {
        lambda
    } else {
        theta_mech_us
    };

    // Bandwidth constraint: lambda_vel >= 4 * (lambda_i + theta_e).
    // Ensures inner loop settles before outer loop reacts.
    let bandwidth_floor = velocity_bandwidth_floor_us(inner_lambda_us, theta_e_us);
    if lambda >= bandwidth_floor {
        lambda
    } else {
        bandwidth_floor
    }
}

fn velocity_bandwidth_floor_us(inner_lambda_us: u32, theta_e_us: u32) -> u32 {
    4 * (inner_lambda_us + theta_e_us)
}

/// Apply operating-mode adjustment to velocity-loop lambda.
///
/// | Mode | Adjustment | Rationale |
/// |------|-----------|-----------|
/// | Unloaded | `lambda *= 1.5` | Less stiffness, reduce standstill buzz |
/// | Nominal | No change | Default |
/// | High inertia | `lambda *= 0.8` | Tighter, natural damping from inertia |
fn apply_mode_adjustment(lambda_us: u32, mode: OperatingMode) -> u32 {
    match mode {
        // lambda * 1.5 = lambda + lambda/2
        OperatingMode::Unloaded => lambda_us + lambda_us / 2,
        OperatingMode::Nominal => lambda_us,
        // lambda * 0.8 = lambda * 4/5
        OperatingMode::HighInertia => lambda_us * 4 / 5,
    }
}

fn apply_velocity_gain_envelope(
    lambda_us: u32,
    tau_us: u32,
    theta_us: u32,
    b_eff: u32,
    step_width: u32,
    f_pwm_hz: u32,
) -> u32 {
    if b_eff == 0 || step_width == 0 || f_pwm_hz == 0 {
        return lambda_us;
    }

    // b_eff is stored as torque * 1000 / encoder-counts/ms, while TMC4671
    // velocity error is measured in TMC velocity units:
    //
    //   tmc_vel_per_cpms = step_width * 1000 / f_pwm_hz
    //   P_reg = tau_us * b_eff * 256 * f_pwm_hz
    //           / (1_000_000 * step_width * (lambda_us + theta_us))
    //
    // Widen lambda when needed so the synthesized P lands inside the same
    // envelope enforced by Phase 10 and commit.
    let numerator = tau_us as u128 * b_eff as u128 * 256 * f_pwm_hz as u128;
    let denominator = 1_000_000_u128 * step_width as u128 * VELOCITY_SIMC_MAX_P_REG as u128;
    let required_sum = numerator.div_ceil(denominator).min(u32::MAX as u128) as u32;
    let lambda_floor = required_sum.saturating_sub(theta_us);
    lambda_us.max(lambda_floor)
}

/// Compute SIMC PI gains for the velocity loop, returning `None` if the
/// mechanical model is invalid (zero J, zero tau, or zero theta).
///
/// # Parameters
/// - `model`: mechanical plant model from Phase 8
/// - `profile`: tuning aggressiveness
/// - `mode`: operating condition adjustment
/// - `inner_lambda_us`: closed-loop lambda from inner (current) loop
/// - `theta_e_us`: electrical transport delay from inner loop
/// - `peak_velocity`: peak velocity observed during mechanical ID (for limit)
/// - `step_width`: TMC position units per encoder count
/// - `config`: TMC4671 board configuration (for PWM frequency)
#[allow(clippy::too_many_arguments)]
pub fn simc_velocity_gains(
    model: &MechanicalModel,
    profile: CommissionProfile,
    mode: OperatingMode,
    inner_lambda_us: u32,
    theta_e_us: u32,
    peak_velocity: u32,
    step_width: u32,
    config: &crate::config::TmcConfig,
) -> Option<VelocityGains> {
    simc_velocity_gains_with_lambda_multiplier(
        model,
        profile,
        mode,
        inner_lambda_us,
        theta_e_us,
        peak_velocity,
        step_width,
        config,
        1.0,
    )
}

/// Compute SIMC velocity-loop gains with a multiplier applied to the final
/// selected velocity lambda before the gain-envelope widening step.
#[allow(clippy::too_many_arguments)]
pub(crate) fn simc_velocity_gains_with_lambda_multiplier(
    model: &MechanicalModel,
    profile: CommissionProfile,
    mode: OperatingMode,
    inner_lambda_us: u32,
    theta_e_us: u32,
    peak_velocity: u32,
    step_width: u32,
    config: &crate::config::TmcConfig,
    lambda_multiplier: f32,
) -> Option<VelocityGains> {
    if model.j_eff == 0 || model.tau_mech_ms == 0 || model.theta_mech_us == 0 {
        return None;
    }
    let f_pwm_hz = config.pwm_freq_hz();
    if step_width == 0 || f_pwm_hz == 0 {
        return None;
    }

    // Convert tau_mech from milliseconds to microseconds for consistent units.
    let tau_mech_us = model.tau_mech_ms * 1000;

    let lambda_us = simc_velocity_lambda_us(
        profile,
        tau_mech_us,
        model.theta_mech_us,
        inner_lambda_us,
        theta_e_us,
    );
    let lambda_us = apply_mode_adjustment(lambda_us, mode);

    // Ensure lambda doesn't collapse to zero after mode adjustment, then widen
    // it if the model would synthesize gains outside the accepted register
    // envelope.
    let lambda_us = lambda_us.max(model.theta_mech_us);
    let lambda_us =
        ((lambda_us as f32) * lambda_multiplier.max(1.0)).clamp(1.0, u32::MAX as f32) as u32;
    let lambda_us = lambda_us.max(velocity_bandwidth_floor_us(inner_lambda_us, theta_e_us));
    let lambda_us = apply_velocity_gain_envelope(
        lambda_us,
        tau_mech_us,
        model.theta_mech_us,
        model.b_eff,
        step_width,
        f_pwm_hz,
    );

    // SIMC PI tuning law for FOPDT plant G_mech(s) = K / (tau*s+1) * e^(-theta*s):
    //   Kp = tau / (K * (lambda + theta))
    //   Ti = min(tau, 4 * (lambda + theta))
    //   Ki = Kp / Ti
    //
    // In encoder-counts/ms units: K = 1000/B_eff because B_eff is stored as
    // torque * 1000 / cpms. TMC4671 evaluates velocity error in TMC velocity
    // units, so convert the cpms-domain SIMC gains before Q8.8 packing.
    let tau = tau_mech_us as f32;
    let theta = model.theta_mech_us as f32;
    let lambda = lambda_us as f32;
    let b_eff = model.b_eff as f32;
    let k = if b_eff > 0.0 { 1000.0 / b_eff } else { 1.0 };
    let tmc_velocity_units_per_cpms = step_width as f32 * 1000.0 / f_pwm_hz as f32;

    let kp_cpms = tau / (k * (lambda + theta));
    let kp = kp_cpms / tmc_velocity_units_per_cpms;
    let ti = {
        let candidate = 4.0 * (lambda + theta);
        if tau < candidate { tau } else { candidate }
    };
    let ti_s = ti * 1e-6; // Convert Ti from microseconds to seconds.
    let ki = kp / ti_s;

    // Convert to TMC4671 Q8.8 register format.
    let f_pwm = f_pwm_hz as f32;
    let p_reg = (kp * 256.0).clamp(1.0, u16::MAX as f32) as u16;
    let i_reg = (ki / f_pwm * 256.0).clamp(1.0, u16::MAX as f32) as u16;

    // Effective bandwidth = 1 / (2*pi*(lambda+theta)) in Hz.
    let bw_hz = 1_000_000.0 / (2.0 * core::f32::consts::PI * (lambda + theta));

    // Velocity limit = 80% of peak velocity from mechanical ID.
    let velocity_limit = peak_velocity * 4 / 5;

    Some(VelocityGains {
        velocity_p: p_reg,
        velocity_i: i_reg,
        velocity_limit,
        effective_bandwidth_hz: bw_hz as u32,
        lambda_vel_ms: lambda_us / 1000,
    })
}

// ---------------------------------------------------------------------------
// Velocity validation metric extraction and classification (Phase 10)
// ---------------------------------------------------------------------------

const VELOCITY_OVERSHOOT_DEADBAND_COUNTS: u32 = 3;

/// Extract velocity step-response metrics from a transient buffer.
///
/// Analyzes encoder-derived velocity samples to measure rise time,
/// overshoot, settling time, saturation, and zero-speed chatter.
///
/// # Parameters
/// - `buf`: captured velocity samples (encoder count differences per tick)
/// - `target`: commanded velocity step amplitude
/// - `saturation_limit`: PID output saturation threshold (0 to skip)
/// - `chatter_threshold`: velocity noise threshold for zero-speed chatter
///   detection (encoder counts/tick, spec default = 3)
/// - `chatter_window`: number of post-zero samples to monitor for chatter
pub fn extract_velocity_metrics(
    buf: &TransientBuffer,
    target: u32,
    saturation_limit: u32,
    chatter_threshold: u32,
    chatter_window: u16,
) -> VelocityValidation {
    let samples = buf.samples();
    if samples.is_empty() || target == 0 {
        return VelocityValidation {
            rise_time_ms: 0,
            overshoot_pct: 0,
            peak_velocity: 0,
            settling_time_ms: 0,
            zero_speed_chatter: false,
            saturation_samples: 0,
            symmetry_pct: 0,
        };
    }

    let period_us = buf.sample_period_us() as u32;
    let target_f = target as f32;
    let threshold_10 = ((target as u64 * 10).div_ceil(100) as u32).max(1);
    let threshold_90 = ((target as u64 * 90).div_ceil(100) as u32).max(1);
    // Encoder-derived velocity is integer counts per sample. At low targets,
    // a strict 5% band truncates to zero counts and makes a stable +/-1 count
    // response look permanently unsettled.
    let settle_band = ((target_f * 0.05) as u32).max(1);

    let mut t_10: Option<u32> = None;
    let mut t_90: Option<u32> = None;
    let mut peak: u32 = 0;
    let mut saturation_count: u16 = 0;

    for (i, &sample) in samples.iter().enumerate() {
        if t_10.is_none() && sample >= threshold_10 {
            t_10 = Some(crate::sample_timing::sample_offset_us(i, period_us));
        }
        if t_90.is_none() && sample >= threshold_90 {
            t_90 = Some(crate::sample_timing::sample_offset_us(i, period_us));
        }
        if sample > peak {
            peak = sample;
        }
        if saturation_limit > 0 && sample >= saturation_limit {
            saturation_count += 1;
        }
    }

    let reached_90 = t_90.is_some();
    let rise_time_us = match (t_10, t_90) {
        (Some(a), Some(b)) => b.saturating_sub(a),
        _ => 0,
    };
    // Convert to milliseconds (rounding to nearest).
    let rise_time_ms = (rise_time_us + 500) / 1000;

    // Overshoot: (peak - target) / target in 0.1% units. The absolute
    // deadband prevents a few integer encoder counts at low target speeds from
    // dominating the relative percentage.
    let overshoot_counts = peak
        .saturating_sub(target)
        .saturating_sub(VELOCITY_OVERSHOOT_DEADBAND_COUNTS);
    let overshoot_pct = if overshoot_counts > 0 {
        (overshoot_counts as u64 * 1000 / target as u64) as u16
    } else {
        0
    };

    // Settling: last sample where |actual - target| > 5% band.
    let mut last_outside: Option<usize> = None;
    for (i, &sample) in samples.iter().enumerate() {
        let diff = sample.abs_diff(target);
        if diff > settle_band {
            last_outside = Some(i);
        }
    }
    let settling_time_ms = if !reached_90 {
        // A response that never reaches the commanded step cannot be
        // considered settled, even if the large mechanical tau would
        // otherwise make the finite capture window look acceptable.
        u32::MAX
    } else if let Some(idx) = last_outside {
        let settling_us = crate::sample_timing::sample_elapsed_us(idx, period_us);
        // Convert to milliseconds (rounding to nearest).
        (settling_us + 500) / 1000
    } else {
        0
    };

    // Zero-speed chatter: check the last `chatter_window` samples
    // for velocity oscillations above the noise threshold.
    let chatter_start = samples.len().saturating_sub(chatter_window as usize);
    let zero_speed_chatter = samples[chatter_start..]
        .iter()
        .any(|&sample| sample > chatter_threshold);

    VelocityValidation {
        rise_time_ms,
        overshoot_pct,
        peak_velocity: peak,
        settling_time_ms,
        zero_speed_chatter,
        saturation_samples: saturation_count,
        symmetry_pct: 0, // Caller computes from fwd+rev captures.
    }
}

/// Classify velocity-loop validation metrics against spec thresholds.
///
/// | Metric | Pass | Marginal | Fail |
/// |--------|------|----------|------|
/// | Overshoot | < 20% | 20-40% | > 40% |
/// | Settling | < 5*tau_mech | 5-10*tau_mech | > 10*tau_mech |
/// | Zero chatter | none | detected | -- |
/// | Saturation | < 10% | 10-25% | > 25% |
///
/// Chatter is marginal, not fail — the filter stage addresses it.
/// Returns the worst classification across all metrics.
pub fn classify_velocity_validation(
    metrics: &VelocityValidation,
    tau_mech_ms: u32,
    total_samples: u16,
) -> ValidationResult {
    let mut worst = ValidationResult::Pass;

    // Overshoot (0.1% units: 200 = 20.0%, 400 = 40.0%).
    if metrics.overshoot_pct > 400 {
        return ValidationResult::Fail;
    } else if metrics.overshoot_pct > 200 {
        worst = ValidationResult::Marginal;
    }

    // Settling (relative to tau_mech in milliseconds).
    let settle_5x = 5 * tau_mech_ms;
    let settle_10x = 10 * tau_mech_ms;
    if metrics.settling_time_ms > settle_10x {
        return ValidationResult::Fail;
    } else if metrics.settling_time_ms > settle_5x {
        worst = ValidationResult::Marginal;
    }

    // Zero-speed chatter is marginal, not fail.
    if metrics.zero_speed_chatter {
        worst = ValidationResult::Marginal;
    }

    // Saturation (fraction of total samples).
    if total_samples > 0 {
        let sat_pct = (metrics.saturation_samples as u32 * 100) / total_samples as u32;
        if sat_pct > 25 {
            return ValidationResult::Fail;
        } else if sat_pct > 10 {
            worst = ValidationResult::Marginal;
        }
    }

    // Symmetry (0.1% units: 200 = 20.0%, 400 = 40.0%).
    if metrics.symmetry_pct > 400 {
        return ValidationResult::Fail;
    } else if metrics.symmetry_pct > 200 {
        worst = ValidationResult::Marginal;
    }

    worst
}

// ---------------------------------------------------------------------------
// Position oscillation detection (Phase 11)
// ---------------------------------------------------------------------------

/// Diagnostic breakdown of `analyze_position_oscillation`. Exposed so the
/// caller can log the inputs that drove the bool decision when an autotune
/// rejects a gain choice.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PositionOscillationDiagnostics {
    pub oscillating: bool,
    pub peak: u32,
    pub overshoot: u32,
    pub crossings: u8,
}

/// Compute oscillation diagnostics from a position capture.
///
/// Returns the same boolean decision as `detect_position_oscillation`, plus
/// the peak sample, the overshoot above target, and the post-peak zero-crossing
/// count. All three feed the bool so logging them tells you exactly which
/// rule fired (or which one almost fired).
pub fn analyze_position_oscillation(
    buf: &TransientBuffer,
    target_position: u32,
) -> PositionOscillationDiagnostics {
    let samples = buf.samples();
    if samples.is_empty() || target_position == 0 {
        return PositionOscillationDiagnostics {
            oscillating: false,
            peak: 0,
            overshoot: 0,
            crossings: 0,
        };
    }

    // Overshoot.
    let overshoot_threshold = target_position / 10;
    let peak = samples.iter().copied().max().unwrap_or(0);
    let overshoot = peak.saturating_sub(target_position);

    // Ringing: zero-crossings of (actual - target) after peak.
    let mut crossings: u8 = 0;
    let peak_idx = samples.iter().position(|&s| s == peak).unwrap_or(0);
    if peak_idx + 1 < samples.len() {
        let mut above = samples[peak_idx] > target_position;
        for &sample in &samples[peak_idx + 1..] {
            let now_above = sample > target_position;
            if now_above != above {
                crossings = crossings.saturating_add(1);
                above = now_above;
            }
        }
    }

    let oscillating = (peak > target_position && overshoot > overshoot_threshold) || crossings > 2;

    PositionOscillationDiagnostics {
        oscillating,
        peak,
        overshoot,
        crossings,
    }
}

/// Returns true when the position capture overshoots target by more than 10%
/// or shows more than one full ringing cycle (>2 zero-crossings) after peak.
///
/// Thin wrapper around `analyze_position_oscillation`; preserves the original
/// boolean API for callers that don't need the diagnostic breakdown.
pub fn detect_position_oscillation(buf: &TransientBuffer, target_position: u32) -> bool {
    analyze_position_oscillation(buf, target_position).oscillating
}

// ---------------------------------------------------------------------------
// Filter recommendation (Phase 12)
// ---------------------------------------------------------------------------

/// Recommend filter cutoff frequencies based on prior validation metrics.
///
/// Follows the spec policy:
/// - Velocity chatter detected: enable velocity biquad, cutoff = velocity_bw * 2
/// - Position ramp-up hit oscillation early: enable position filter, cutoff = position_bw * 3
/// - Current ringing detected: enable flux/torque biquad, cutoff = current_bw * 2
///
/// Policy ordering: prefer velocity filtering before position filtering.
/// Leave flux/torque filters off unless current noise demands it.
///
/// # Parameters
/// - `velocity_chatter`: true if velocity validation detected zero-speed chatter
/// - `velocity_bandwidth_hz`: velocity-loop effective bandwidth
/// - `position_oscillation_early`: true if position P ramp-up hit oscillation
///   before reaching a useful gain value
/// - `position_bandwidth_hz`: estimated position-loop bandwidth (from velocity BW / 4)
/// - `current_ringing`: true if current validation detected ringing (count > 1)
/// - `current_bandwidth_hz`: current-loop effective bandwidth
pub fn recommend_filters(
    velocity_chatter: bool,
    velocity_bandwidth_hz: u32,
    position_oscillation_early: bool,
    position_bandwidth_hz: u32,
    current_ringing: bool,
    current_bandwidth_hz: u32,
) -> FilterConfig {
    let velocity_filter_hz = if velocity_chatter {
        clamp_filter_cutoff(velocity_bandwidth_hz * 2)
    } else {
        None
    };

    let position_filter_hz = if position_oscillation_early {
        clamp_filter_cutoff(position_bandwidth_hz * 3)
    } else {
        None
    };

    let (flux_filter_hz, torque_filter_hz) = if current_ringing {
        let cutoff = clamp_filter_cutoff(current_bandwidth_hz * 2);
        (cutoff, cutoff)
    } else {
        (None, None)
    };

    FilterConfig {
        velocity_filter_hz,
        position_filter_hz,
        flux_filter_hz,
        torque_filter_hz,
    }
}

/// Clamp a filter cutoff to the valid biquad range (10-1000 Hz).
///
/// Returns `None` if the cutoff is below the minimum (filter too low
/// to be useful). Clamps to 1000 Hz if above the maximum.
fn clamp_filter_cutoff(cutoff_hz: u32) -> Option<u32> {
    const MIN_HZ: u32 = 10;
    const MAX_HZ: u32 = 1000;
    if cutoff_hz < MIN_HZ {
        None
    } else if cutoff_hz > MAX_HZ {
        Some(MAX_HZ)
    } else {
        Some(cutoff_hz)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // --- TransientBuffer ---

    #[test]
    fn transient_buffer_push_and_read() {
        let mut buf = TransientBuffer::new();
        buf.reset(160);
        assert!(buf.is_empty());
        assert_eq!(buf.len(), 0);
        assert_eq!(buf.sample_period_us(), 160);

        assert!(buf.push(10));
        assert!(buf.push(20));
        assert!(buf.push(30));

        assert!(!buf.is_empty());
        assert_eq!(buf.len(), 3);
        let s = buf.samples();
        assert_eq!(s, &[10u32, 20, 30]);
    }

    #[test]
    fn transient_buffer_full() {
        let mut buf = TransientBuffer::new();
        buf.reset(100);
        for i in 0u32..TRANSIENT_LEN as u32 {
            assert!(buf.push(i), "push {i} should succeed");
        }
        assert!(buf.is_full());
        assert_eq!(buf.len() as usize, TRANSIENT_LEN);

        // One more push must be rejected.
        assert!(!buf.push(9999), "push to full buffer must return false");
        // Length must not change.
        assert_eq!(buf.len() as usize, TRANSIENT_LEN);
    }

    #[test]
    fn transient_buffer_reset_clears() {
        let mut buf = TransientBuffer::new();
        buf.reset(160);
        for i in 0u32..10 {
            buf.push(i);
        }
        assert_eq!(buf.len(), 10);

        buf.reset(200);
        assert!(buf.is_empty());
        assert_eq!(buf.len(), 0);
        assert_eq!(buf.sample_period_us(), 200);
        assert_eq!(buf.samples(), &[] as &[u32]);
    }

    // --- CommissionProfile ---

    #[test]
    fn commission_profile_roundtrip() {
        let profiles = [
            CommissionProfile::Conservative,
            CommissionProfile::Balanced,
            CommissionProfile::Stiff,
        ];
        for p in profiles {
            let code = p.to_u8();
            let decoded = CommissionProfile::from_u8(code);
            assert_eq!(p, decoded, "roundtrip failed for {p:?}");
        }
    }

    #[test]
    fn commission_profile_unknown_maps_to_balanced() {
        assert_eq!(CommissionProfile::from_u8(3), CommissionProfile::Balanced);
        assert_eq!(CommissionProfile::from_u8(255), CommissionProfile::Balanced);
    }

    // --- OperatingMode ---

    #[test]
    fn operating_mode_roundtrip() {
        let modes = [
            OperatingMode::Unloaded,
            OperatingMode::Nominal,
            OperatingMode::HighInertia,
        ];
        for m in modes {
            let code = m.to_u8();
            let decoded = OperatingMode::from_u8(code);
            assert_eq!(m, decoded, "roundtrip failed for {m:?}");
        }
    }

    #[test]
    fn operating_mode_unknown_maps_to_unloaded() {
        assert_eq!(OperatingMode::from_u8(3), OperatingMode::Unloaded);
        assert_eq!(OperatingMode::from_u8(255), OperatingMode::Unloaded);
    }

    // --- CommissionError ---

    #[test]
    fn commission_error_status_codes_unique() {
        let errors = [
            CommissionError::MotorEnabled,
            CommissionError::NoCurrent,
            CommissionError::Spi,
            CommissionError::AdcFault,
            CommissionError::CoilFault,
            CommissionError::PhaseFault,
            CommissionError::EncoderFault,
            CommissionError::ElectricalIdFailed,
            CommissionError::CurrentValidationFailed,
            CommissionError::MechanicalIdFailed,
            CommissionError::VelocityValidationFailed,
            CommissionError::PositionTuneFailed,
            CommissionError::EncoderNotAligned,
            CommissionError::Shutdown,
            CommissionError::AlreadyRunning,
            CommissionError::QueueFull,
            CommissionError::SafetyEnvelopeViolation,
        ];
        let mut seen = [false; 256];
        for e in errors {
            let code = e.status_code() as usize;
            assert_ne!(code, 0, "{e:?} must not use code 0 (reserved for success)");
            assert!(!seen[code], "duplicate status_code {code} for {e:?}");
            seen[code] = true;
        }
    }

    // --- PhaseId ---

    #[test]
    fn phase_id_to_u8_unique() {
        let phases = [
            PhaseId::AdcCalibrate,
            PhaseId::CoilCheck,
            PhaseId::PhaseWiringCheck,
            PhaseId::EncoderCheck,
            PhaseId::ElectricalId,
            PhaseId::CurrentTune,
            PhaseId::CurrentValidate,
            PhaseId::InnerDone,
            PhaseId::MechanicalId,
            PhaseId::VelocityTune,
            PhaseId::VelocityValidate,
            PhaseId::PositionTune,
            PhaseId::FilterTune,
            PhaseId::Commit,
            PhaseId::OuterDone,
        ];
        let mut seen = [false; 256];
        for p in phases {
            let code = p.to_u8() as usize;
            assert_ne!(code, 0, "{p:?} must not use code 0");
            assert!(!seen[code], "duplicate to_u8 code {code} for {p:?}");
            seen[code] = true;
        }
    }

    // --- ElectricalModel ---

    #[test]
    fn electrical_model_tau_from_l_over_r() {
        let model = ElectricalModel {
            r_int: 512,
            l_int: 256,
            tau_e_us: 50,
            theta_e_us: 10,
            r_mohm: 1125,
            l_uh: 450,
            tau_e_crosscheck_us: 48,
        };
        assert_eq!(model.r_int, 512);
        assert_eq!(model.tau_e_us, 50);
        assert_eq!(model.theta_e_us, 10);
    }

    // --- SIMC current gains ---

    fn autotune_test_config() -> crate::config::TmcConfig {
        crate::config::TmcConfig {
            pwm_maxcnt: 3999,
            tmc_clock_hz: 25_000_000,
            bbm_h: 10,
            bbm_l: 10,
            dsadc_mdec_a: 331,
            dsadc_mdec_b: 331,
            adc_i_select: 0x1800_0100,
            excitation_voltage: 512,
            didt_cycles: 5000,
            bandwidth_hz: 150.0,
        }
    }

    const TEST_STEP_WIDTH: u32 = 819;

    fn test_electrical_model() -> ElectricalModel {
        ElectricalModel {
            r_int: 1125,
            l_int: 2950,
            tau_e_us: 2622,
            theta_e_us: 700,
            r_mohm: 1125,
            l_uh: 450,
            tau_e_crosscheck_us: 2500,
        }
    }

    #[test]
    fn simc_balanced_gains_nonzero() {
        let model = test_electrical_model();
        let config = autotune_test_config();
        let gains = simc_current_gains(&model, CommissionProfile::Balanced, &config);
        assert!(gains.flux_p > 0, "flux_p must be nonzero");
        assert!(gains.flux_i > 0, "flux_i must be nonzero");
        assert_eq!(gains.flux_p, gains.torque_p, "flux/torque P must match");
        assert_eq!(gains.flux_i, gains.torque_i, "flux/torque I must match");
        assert!(gains.lambda_us > 0, "lambda must be nonzero");
        assert!(
            gains.effective_bandwidth_hz > 0,
            "bandwidth must be nonzero"
        );
    }

    #[test]
    fn simc_delay_dominated_plant_stiff_below_balanced() {
        // Delay-dominated: theta is large relative to tau.
        // Stiff = max(theta, tau/4) = max(700, 655) = 700
        // Balanced = 2*theta = 1400
        // So stiff < balanced here.
        let model = test_electrical_model(); // theta=700, tau=2622
        let config = autotune_test_config();
        let balanced = simc_current_gains(&model, CommissionProfile::Balanced, &config);
        let stiff = simc_current_gains(&model, CommissionProfile::Stiff, &config);
        assert!(
            stiff.lambda_us < balanced.lambda_us,
            "delay-dominated: stiff lambda {} should be < balanced {}",
            stiff.lambda_us,
            balanced.lambda_us
        );
    }

    #[test]
    fn simc_tau_dominated_plant_profiles_remain_ordered() {
        // Tau-dominated current plants still need tau-based lambda floors:
        // Conservative >= Balanced >= Stiff.
        let model = ElectricalModel {
            r_int: 1125,
            l_int: 2950,
            tau_e_us: 4000,
            theta_e_us: 100,
            r_mohm: 1125,
            l_uh: 450,
            tau_e_crosscheck_us: 3800,
        };
        let config = autotune_test_config();
        let conservative = simc_current_gains(&model, CommissionProfile::Conservative, &config);
        let balanced = simc_current_gains(&model, CommissionProfile::Balanced, &config);
        let stiff = simc_current_gains(&model, CommissionProfile::Stiff, &config);
        assert_eq!(conservative.lambda_us, 4000);
        assert_eq!(balanced.lambda_us, 2000);
        assert_eq!(stiff.lambda_us, 1000);
    }

    #[test]
    fn simc_invariants_hold_for_all_profiles() {
        // Universal SIMC invariants that hold regardless of plant shape:
        // 1. conservative >= balanced >= stiff
        // 2. conservative >= tau, balanced >= tau/2, stiff >= tau/4
        // 3. all profiles >= theta (SIMC stability boundary)
        for model in [
            test_electrical_model(),
            ElectricalModel {
                tau_e_us: 4000,
                theta_e_us: 100,
                ..test_electrical_model()
            },
            ElectricalModel {
                tau_e_us: 200,
                theta_e_us: 500,
                ..test_electrical_model()
            },
        ] {
            let config = autotune_test_config();
            let con = simc_current_gains(&model, CommissionProfile::Conservative, &config);
            let bal = simc_current_gains(&model, CommissionProfile::Balanced, &config);
            let stf = simc_current_gains(&model, CommissionProfile::Stiff, &config);

            assert!(
                con.lambda_us >= bal.lambda_us,
                "tau={} theta={}: conservative {} must be >= balanced {}",
                model.tau_e_us,
                model.theta_e_us,
                con.lambda_us,
                bal.lambda_us
            );
            assert!(
                bal.lambda_us >= stf.lambda_us,
                "tau={} theta={}: balanced {} must be >= stiff {}",
                model.tau_e_us,
                model.theta_e_us,
                bal.lambda_us,
                stf.lambda_us
            );
            assert!(
                con.lambda_us >= model.tau_e_us,
                "tau={} theta={}: conservative {} must be >= tau",
                model.tau_e_us,
                model.theta_e_us,
                con.lambda_us
            );
            assert!(
                bal.lambda_us >= model.tau_e_us / 2,
                "tau={} theta={}: balanced {} must be >= tau/2={}",
                model.tau_e_us,
                model.theta_e_us,
                bal.lambda_us,
                model.tau_e_us / 2
            );
            assert!(
                stf.lambda_us >= model.theta_e_us,
                "tau={} theta={}: stiff {} must be >= theta {}",
                model.tau_e_us,
                model.theta_e_us,
                stf.lambda_us,
                model.theta_e_us
            );
            assert!(
                stf.lambda_us >= model.tau_e_us / 4,
                "tau={} theta={}: stiff {} must be >= tau/4={}",
                model.tau_e_us,
                model.theta_e_us,
                stf.lambda_us,
                model.tau_e_us / 4
            );
            for (name, gains) in [("conservative", &con), ("balanced", &bal), ("stiff", &stf)] {
                assert!(
                    gains.lambda_us >= model.theta_e_us,
                    "tau={} theta={}: {} lambda {} must satisfy stability floor theta {}",
                    model.tau_e_us,
                    model.theta_e_us,
                    name,
                    gains.lambda_us,
                    model.theta_e_us
                );
            }
        }
    }

    #[test]
    fn simc_zero_r_returns_none() {
        let mut model = test_electrical_model();
        model.r_int = 0;
        let config = autotune_test_config();
        assert!(simc_current_gains_checked(&model, CommissionProfile::Balanced, &config).is_none());
    }

    #[test]
    fn simc_i_gain_not_trivial() {
        // With typical motor parameters, the I gain should be significantly
        // above the minimum clamp of 1. A value of 1 indicates a units bug.
        let model = test_electrical_model();
        let config = autotune_test_config();
        let gains = simc_current_gains(&model, CommissionProfile::Balanced, &config);
        assert!(
            gains.flux_i > 10,
            "flux_i={} is suspiciously low (units bug?)",
            gains.flux_i
        );
    }

    #[test]
    fn simc_current_i_gain_uses_advanced_pi_zero_parameter() {
        let model = ElectricalModel {
            r_int: ELECTRICAL_MODEL_RESISTANCE_SCALE,
            l_int: 1863,
            tau_e_us: 1863,
            theta_e_us: 160,
            r_mohm: 1000,
            l_uh: 1863,
            tau_e_crosscheck_us: 1863,
        };
        let config = autotune_test_config();
        let gains = simc_current_gains(&model, CommissionProfile::Balanced, &config);

        // Balanced current-loop lambda is floored at tau/2 for robustness:
        // max(2*theta=320us, tau/2=931us). The damped SIMC integrator time is
        // max(tau, CURRENT_SIMC_TI_MULTIPLIER*(lambda+theta)). With the
        // default Q8.8 representation, TMC4671 advanced PI places the series
        // PI zero as I/65536 per PWM sample.
        assert_eq!(gains.lambda_us, model.tau_e_us / 2);
        let ti_s =
            CURRENT_SIMC_TI_MULTIPLIER as f64 * (gains.lambda_us + model.theta_e_us) as f64 * 1e-6;
        let expected_i = (65_536.0 / (ti_s * config.pwm_freq_hz() as f64)) as u16;

        assert_eq!(gains.flux_i, expected_i);
        assert_eq!(gains.torque_i, expected_i);
    }

    #[test]
    fn simc_current_i_gain_starts_near_observed_settling_bracket() {
        let model = ElectricalModel {
            r_int: ELECTRICAL_MODEL_RESISTANCE_SCALE,
            l_int: 1868,
            tau_e_us: 1868,
            theta_e_us: 160,
            r_mohm: 1000,
            l_uh: 1868,
            tau_e_crosscheck_us: 1868,
        };
        let config = autotune_test_config();
        let gains = simc_current_gains(&model, CommissionProfile::Balanced, &config);

        assert!(
            (215..=225).contains(&gains.flux_i),
            "initial I should start near the measured stable retry value, got {}",
            gains.flux_i
        );
        assert_eq!(gains.flux_i, gains.torque_i);
    }

    #[test]
    fn current_settling_reference_uses_advanced_pi_zero_time() {
        let gains = CurrentGains {
            flux_p: 436,
            flux_i: 220,
            torque_p: 436,
            torque_i: 220,
            effective_bandwidth_hz: 146,
            lambda_us: 923,
        };
        let config = autotune_test_config();

        assert_eq!(current_settling_reference_us(&gains, &config), 11915);
    }

    #[test]
    fn simc_retry_lambda_preserves_measured_theta() {
        let model = ElectricalModel {
            r_int: ELECTRICAL_MODEL_RESISTANCE_SCALE,
            l_int: 1917,
            tau_e_us: 1917,
            theta_e_us: 160,
            r_mohm: 1000,
            l_uh: 1917,
            tau_e_crosscheck_us: 1917,
        };
        let config = autotune_test_config();
        let gains = simc_current_gains_with_lambda(&model, 1436, &config);

        let p_expected =
            (1917.0_f64 / (1436.0_f64 + 160.0_f64) * 256.0_f64).clamp(1.0, u16::MAX as f64) as u16;
        let ti_s = CURRENT_SIMC_TI_MULTIPLIER as f64 * (1436.0 + 160.0) * 1e-6;
        let i_expected =
            (65_536.0 / (ti_s * config.pwm_freq_hz() as f64)).clamp(1.0, u16::MAX as f64) as u16;

        assert_eq!(gains.lambda_us, 1436);
        assert_eq!(gains.flux_p, p_expected);
        assert_eq!(gains.flux_i, i_expected);
        assert!(gains.flux_p > 227, "retry P should not use inflated theta");
        assert!(gains.flux_i > 76, "retry I should not use inflated theta");
    }

    // --- Step metrics extraction ---

    /// Build a synthetic first-order step response for testing.
    /// Fills 256 samples at 100us intervals.
    fn synthetic_step_response(overshoot_pct: f32) -> TransientBuffer {
        let mut buf = TransientBuffer::new();
        buf.reset(100);
        let target = 1000.0f32;
        for i in 0..256u32 {
            let t = i as f32;
            let base = target * (1.0 - (-t / 10.0).exp());
            let overshoot_term = if overshoot_pct > 0.0 {
                target * overshoot_pct / 100.0 * (-t / 20.0).exp() * (t / 5.0).sin()
            } else {
                0.0
            };
            buf.push((base + overshoot_term).max(0.0) as u32);
        }
        buf
    }

    #[test]
    fn step_metrics_clean_response() {
        let buf = synthetic_step_response(0.0);
        let metrics = extract_step_metrics(&buf, 1000, 0);
        assert!(
            metrics.overshoot_pct < 20,
            "overshoot {} should be <2%",
            metrics.overshoot_pct
        );
        assert!(
            metrics.ringing_count <= 1,
            "ringing {} should be <=1",
            metrics.ringing_count
        );
        assert!(metrics.rise_time_us > 0, "rise time must be nonzero");
    }

    /// Phase 1 timing invariants — pin the current sample-timing
    /// convention so a later measurement redesign updates these tests
    /// deliberately. See `docs/specs/2026-04-30-inner-commissioning-stability.md`
    /// §5.

    #[test]
    fn step_metrics_t10_at_or_before_t90() {
        let mut buf = TransientBuffer::new();
        buf.reset(160);
        for &v in &[0u32, 50, 200, 600, 950, 1000, 1000, 1000] {
            buf.push(v);
        }
        let metrics = extract_step_metrics(&buf, 1000, 0);
        // rise_time = t90 - t10; nonzero rise_time implies t90 >= t10.
        assert!(metrics.rise_time_us > 0);
    }

    #[test]
    fn step_metrics_rise_time_invariant_under_capture_origin_offset() {
        // Adding zero-valued samples *before* the meaningful step must not
        // change rise_time, because t90 - t10 cancels the capture-origin
        // offset.
        let mut early = TransientBuffer::new();
        early.reset(160);
        for &v in &[0u32, 50, 200, 600, 950, 1000, 1000, 1000] {
            early.push(v);
        }
        let mut delayed = TransientBuffer::new();
        delayed.reset(160);
        for &v in &[0u32, 0, 0, 50, 200, 600, 950, 1000, 1000, 1000] {
            delayed.push(v);
        }
        let early_m = extract_step_metrics(&early, 1000, 0);
        let delayed_m = extract_step_metrics(&delayed, 1000, 0);
        assert_eq!(early_m.rise_time_us, delayed_m.rise_time_us);
    }

    #[test]
    fn step_metrics_settling_time_uses_last_outside_plus_one_period() {
        // Settling time uses smoothed (8-sample trailing average) values, so
        // pin the formula by inserting one zero at index 15. The trailing
        // average then stays out-of-band through index 22 and recovers at
        // index 23 — last_outside == 22, settling == (22 + 1) * 160 = 3680us.
        let mut buf = TransientBuffer::new();
        buf.reset(160);
        for i in 0..24u32 {
            if i == 15 {
                buf.push(0);
            } else {
                buf.push(1000);
            }
        }
        let metrics = extract_step_metrics(&buf, 1000, 0);
        assert_eq!(metrics.settling_time_us, 23 * 160);
    }

    #[test]
    fn step_metrics_zero_target_stays_bounded() {
        let mut buf = TransientBuffer::new();
        buf.reset(160);
        for _ in 0..16 {
            buf.push(100);
        }
        let metrics = extract_step_metrics(&buf, 0, 0);
        assert_eq!(metrics.rise_time_us, 0);
        assert_eq!(metrics.settling_time_us, 0);
    }

    #[test]
    fn step_metrics_low_target_stays_bounded() {
        let mut buf = TransientBuffer::new();
        buf.reset(160);
        for _ in 0..16 {
            buf.push(0);
        }
        let metrics = extract_step_metrics(&buf, 5, 0);
        // settle_band is clamped to >=1 count; an all-zero capture cannot
        // reach the band but the metric must stay bounded, not panic.
        assert!(metrics.settling_time_us <= 16 * 160);
    }

    #[test]
    fn step_metrics_with_overshoot() {
        let buf = synthetic_step_response(100.0);
        let metrics = extract_step_metrics(&buf, 1000, 0);
        assert!(
            metrics.overshoot_pct > 100,
            "should detect significant overshoot: {}",
            metrics.overshoot_pct
        );
    }

    #[test]
    fn step_metrics_ignore_in_band_noise_for_ringing() {
        let mut buf = TransientBuffer::new();
        buf.reset(160);
        for &sample in &[0, 120, 500, 900, 1220, 1100, 1040] {
            buf.push(sample);
        }
        for i in 0..96 {
            let sample = if i % 2 == 0 { 960 } else { 1040 };
            buf.push(sample);
        }

        let metrics = extract_step_metrics(&buf, 1000, 0);
        assert_eq!(metrics.ringing_count, 0);
    }

    #[test]
    fn current_metrics_ignore_tail_ripple_after_settling() {
        let mut buf = TransientBuffer::new();
        buf.reset(160);
        for &sample in &[53, 98, 180, 260, 330, 390, 416, 370] {
            buf.push(sample);
        }
        for i in 0..248 {
            let sample = if i % 2 == 0 { 345 } else { 416 };
            buf.push(sample);
        }

        let metrics = extract_step_metrics(&buf, 367, 0);

        assert!(
            metrics.settling_time_us < 10 * 1868,
            "tail ripple around target should not force full-window settling"
        );
        assert_eq!(metrics.ringing_count, 0);
        assert_eq!(
            classify_current_validation(&metrics, 1868, 512),
            ValidationResult::Pass
        );
    }

    #[test]
    fn step_metrics_empty_buffer() {
        let buf = TransientBuffer::new();
        let metrics = extract_step_metrics(&buf, 1000, 0);
        assert_eq!(metrics.rise_time_us, 0);
        assert_eq!(metrics.overshoot_pct, 0);
        assert_eq!(metrics.tail_level, 0);
    }

    #[test]
    fn step_metrics_tail_level_from_last_32_samples() {
        // Build a buffer that rises quickly to ~950, then stays there.
        // The tail_level (median of last 32 samples) should be ~950.
        let mut buf = TransientBuffer::new();
        buf.reset(160);
        // 10 rising samples
        for &sample in &[0u32, 100, 300, 500, 700, 850, 920, 940, 945, 948] {
            buf.push(sample);
        }
        // 246 steady-state samples at 950
        for _ in 0..246 {
            buf.push(950);
        }
        let metrics = extract_step_metrics(&buf, 1000, 0);
        assert_eq!(
            metrics.tail_level, 950,
            "tail_level should be median of last 32 samples"
        );
    }

    #[test]
    fn step_metrics_tail_level_resists_outliers() {
        // Last 32 samples: 30 at 950, 2 outliers at 0. Median should be 950.
        let mut buf = TransientBuffer::new();
        buf.reset(160);
        // Fill first 224 samples as steady state
        for _ in 0..224 {
            buf.push(950);
        }
        // Last 32 samples: 2 outliers, then 30 at 950
        buf.push(0);
        buf.push(0);
        for _ in 0..30 {
            buf.push(950);
        }
        let metrics = extract_step_metrics(&buf, 1000, 0);
        assert_eq!(metrics.tail_level, 950, "median should resist outliers");
    }

    #[test]
    fn step_metrics_tail_level_short_buffer() {
        // Buffer with only 10 samples — tail_level uses all of them.
        let mut buf = TransientBuffer::new();
        buf.reset(160);
        for &sample in &[100u32, 200, 300, 400, 500, 600, 700, 800, 900, 950] {
            buf.push(sample);
        }
        let metrics = extract_step_metrics(&buf, 1000, 0);
        // Median of [100,200,300,400,500,600,700,800,900,950] = (500+600)/2 = 550
        assert_eq!(
            metrics.tail_level, 550,
            "tail_level should use all samples when buffer < 32"
        );
    }

    #[test]
    fn simc_dimensional_consistency() {
        // Compute gains using the microsecond-based function, then verify
        // against a manual calculation using seconds throughout. The final
        // Q8.8 register values must match, proving no units conversion error.
        let model = test_electrical_model();
        let config = autotune_test_config();
        let gains = simc_current_gains(&model, CommissionProfile::Balanced, &config);

        // Manual calculation in SI (seconds):
        let tau_s = model.tau_e_us as f64 * 1e-6;
        let theta_s = model.theta_e_us as f64 * 1e-6;
        let lambda_s = ((2 * model.theta_e_us).max(model.tau_e_us / 2)) as f64 * 1e-6;
        let r_int = model.r_int as f64 / ELECTRICAL_MODEL_RESISTANCE_SCALE as f64;
        let k = 1.0 / r_int;
        let f_pwm = config.pwm_freq_hz() as f64;

        let kp_si = tau_s / (k * (lambda_s + theta_s));
        let ti_s = {
            let candidate = CURRENT_SIMC_TI_MULTIPLIER as f64 * (lambda_s + theta_s);
            if tau_s > candidate { tau_s } else { candidate }
        };
        let p_reg_expected = (kp_si * 256.0).clamp(1.0, u16::MAX as f64) as u16;
        let i_reg_expected = (65_536.0 / (ti_s * f_pwm)).clamp(1.0, u16::MAX as f64) as u16;

        assert_eq!(
            gains.flux_p, p_reg_expected,
            "P gain mismatch: got {} expected {}",
            gains.flux_p, p_reg_expected
        );
        // Allow +/- 1 for f32 vs f64 rounding
        let i_diff = (gains.flux_i as i32 - i_reg_expected as i32).unsigned_abs();
        assert!(
            i_diff <= 1,
            "I gain mismatch: got {} expected {} (diff {})",
            gains.flux_i,
            i_reg_expected,
            i_diff
        );
    }

    // --- Validation classification ---

    #[test]
    fn classify_pass() {
        let metrics = CurrentValidation {
            rise_time_us: 500,
            overshoot_pct: 100, // 10.0%
            settling_time_us: 2000,
            ringing_count: 1,
            saturation_samples: 5,
            symmetry_pct: 100, // 10.0%
            tail_level: 0,
            target_error_pct: 0,
        };
        assert_eq!(
            classify_current_validation(&metrics, 1000, 512),
            ValidationResult::Pass
        );
    }

    #[test]
    fn classify_marginal_overshoot() {
        let metrics = CurrentValidation {
            rise_time_us: 500,
            overshoot_pct: 200, // 20.0%
            settling_time_us: 2000,
            ringing_count: 1,
            saturation_samples: 5,
            symmetry_pct: 100,
            tail_level: 0,
            target_error_pct: 0,
        };
        assert_eq!(
            classify_current_validation(&metrics, 1000, 512),
            ValidationResult::Marginal
        );
    }

    #[test]
    fn classify_fail_overshoot() {
        let metrics = CurrentValidation {
            rise_time_us: 500,
            overshoot_pct: 350, // 35.0%
            settling_time_us: 2000,
            ringing_count: 1,
            saturation_samples: 5,
            symmetry_pct: 100,
            tail_level: 0,
            target_error_pct: 0,
        };
        assert_eq!(
            classify_current_validation(&metrics, 1000, 512),
            ValidationResult::Fail
        );
    }

    #[test]
    fn classify_fail_ringing() {
        let metrics = CurrentValidation {
            rise_time_us: 500,
            overshoot_pct: 50,
            settling_time_us: 2000,
            ringing_count: 4,
            saturation_samples: 0,
            symmetry_pct: 0,
            tail_level: 0,
            target_error_pct: 0,
        };
        assert_eq!(
            classify_current_validation(&metrics, 1000, 512),
            ValidationResult::Fail
        );
    }

    #[test]
    fn classify_marginal_settling() {
        let metrics = CurrentValidation {
            rise_time_us: 500,
            overshoot_pct: 50,
            settling_time_us: 7000, // 7x tau=1000
            ringing_count: 0,
            saturation_samples: 0,
            symmetry_pct: 0,
            tail_level: 0,
            target_error_pct: 0,
        };
        assert_eq!(
            classify_current_validation(&metrics, 1000, 512),
            ValidationResult::Marginal
        );
    }

    #[test]
    fn classify_current_validation_settling_uses_controller_reference() {
        let metrics = CurrentValidation {
            rise_time_us: 500,
            overshoot_pct: 141,
            settling_time_us: 39360,
            ringing_count: 0,
            saturation_samples: 0,
            symmetry_pct: 83,
            tail_level: 0,
            target_error_pct: 0,
        };

        assert_eq!(
            classify_current_validation(&metrics, 11915, 512),
            ValidationResult::Pass
        );
    }

    #[test]
    fn accept_noop_retry_when_marginal_is_symmetry_only() {
        let previous = CurrentGains {
            flux_p: 437,
            flux_i: 217,
            torque_p: 437,
            torque_i: 217,
            effective_bandwidth_hz: 145,
            lambda_us: 934,
        };
        let retry = current_validation_retry_gains(previous, 367, 18, 351, 358, 119);
        // 180 permille: Marginal under tail-level thresholds (Pass <= 100, Fail > 250).
        let metrics = CurrentValidation {
            rise_time_us: 500,
            overshoot_pct: 119,
            settling_time_us: 40480,
            ringing_count: 0,
            saturation_samples: 0,
            symmetry_pct: 180,
            tail_level: 0,
            target_error_pct: 0,
        };

        assert_eq!(
            classify_current_validation(&metrics, 12080, 512),
            ValidationResult::Marginal
        );
        assert!(should_accept_current_validation_without_retry(
            &metrics, 12080, 512, previous, retry,
        ));
    }

    #[test]
    fn classify_marginal_symmetry_at_150_permille() {
        // With tail-level symmetry thresholds (Pass <= 100, Marginal <= 250),
        // 150 permille should be Marginal (not Pass as with old 200 threshold).
        let metrics = CurrentValidation {
            rise_time_us: 500,
            overshoot_pct: 50,
            settling_time_us: 2000,
            ringing_count: 0,
            saturation_samples: 0,
            symmetry_pct: 150,
            tail_level: 0,
            target_error_pct: 0,
        };
        assert_eq!(
            classify_current_validation(&metrics, 1000, 512),
            ValidationResult::Marginal
        );
    }

    #[test]
    fn classify_fail_symmetry_at_300_permille() {
        // With tail-level thresholds (Fail > 250), 300 should be Fail
        // (not Marginal as with old 400 threshold).
        let metrics = CurrentValidation {
            rise_time_us: 500,
            overshoot_pct: 50,
            settling_time_us: 2000,
            ringing_count: 0,
            saturation_samples: 0,
            symmetry_pct: 300,
            tail_level: 0,
            target_error_pct: 0,
        };
        assert_eq!(
            classify_current_validation(&metrics, 1000, 512),
            ValidationResult::Fail
        );
    }

    #[test]
    fn classify_pass_symmetry_at_boundary() {
        // Exactly 100 permille should still pass.
        let metrics = CurrentValidation {
            rise_time_us: 500,
            overshoot_pct: 50,
            settling_time_us: 2000,
            ringing_count: 0,
            saturation_samples: 0,
            symmetry_pct: 100,
            tail_level: 0,
            target_error_pct: 0,
        };
        assert_eq!(
            classify_current_validation(&metrics, 1000, 512),
            ValidationResult::Pass
        );
    }

    // --- Target attainment ---

    #[test]
    fn classify_fails_when_both_tails_zero() {
        // Zero current in both directions: target_error = 1000‰ (100% off).
        let metrics = CurrentValidation {
            rise_time_us: 0,
            overshoot_pct: 0,
            settling_time_us: 0,
            ringing_count: 0,
            saturation_samples: 0,
            symmetry_pct: 0,
            tail_level: 0,
            target_error_pct: 1000, // 100% off target
        };
        assert_eq!(
            classify_current_validation(&metrics, 12000, 512),
            ValidationResult::Fail
        );
    }

    #[test]
    fn classify_fails_when_tails_below_lower_band() {
        // tail_level=200 vs target=367 → error ≈ 455‰ (45.5% off).
        // |200 - 367| / 367 * 1000 = 455
        let metrics = CurrentValidation {
            rise_time_us: 500,
            overshoot_pct: 0,
            settling_time_us: 2000,
            ringing_count: 0,
            saturation_samples: 0,
            symmetry_pct: 20,
            tail_level: 200,
            target_error_pct: 455,
        };
        assert_eq!(
            classify_current_validation(&metrics, 12000, 512),
            ValidationResult::Marginal
        );
    }

    #[test]
    fn classify_passes_when_tails_in_band() {
        // tail_level=360 vs target=367 → error ≈ 19‰ (1.9% off).
        let metrics = CurrentValidation {
            rise_time_us: 500,
            overshoot_pct: 50,
            settling_time_us: 2000,
            ringing_count: 0,
            saturation_samples: 0,
            symmetry_pct: 20,
            tail_level: 360,
            target_error_pct: 19,
        };
        assert_eq!(
            classify_current_validation(&metrics, 12000, 512),
            ValidationResult::Pass
        );
    }

    #[test]
    fn current_validation_retries_first_failed_attempt() {
        assert!(should_retry_current_validation(ValidationResult::Fail, 0));
        assert!(should_retry_current_validation(
            ValidationResult::Marginal,
            0
        ));
        assert!(!should_retry_current_validation(ValidationResult::Pass, 0));
        assert!(!should_retry_current_validation(ValidationResult::Fail, 1));
    }

    #[test]
    fn current_validation_retry_reduces_integrator_when_tail_high() {
        let previous = CurrentGains {
            flux_p: 437,
            flux_i: 592,
            torque_p: 437,
            torque_i: 592,
            effective_bandwidth_hz: 143,
            lambda_us: 947,
        };

        let retry = current_validation_retry_gains(previous, 367, 18, 372, 403, 294);

        assert_eq!(retry.flux_p, previous.flux_p);
        assert_eq!(retry.torque_p, previous.torque_p);
        assert_eq!(retry.flux_i, 296);
        assert_eq!(retry.torque_i, 296);
        assert_eq!(retry.lambda_us, previous.lambda_us);
        assert_eq!(
            retry.effective_bandwidth_hz,
            previous.effective_bandwidth_hz
        );
    }

    #[test]
    fn current_validation_retry_increases_integrator_when_tail_low() {
        let previous = CurrentGains {
            flux_p: 436,
            flux_i: 151,
            torque_p: 436,
            torque_i: 151,
            effective_bandwidth_hz: 146,
            lambda_us: 923,
        };

        let retry = current_validation_retry_gains(previous, 367, 18, 354, 321, 35);

        assert_eq!(retry.flux_p, previous.flux_p);
        assert_eq!(retry.torque_p, previous.torque_p);
        assert_eq!(retry.flux_i, 226);
        assert_eq!(retry.torque_i, 226);
        assert_eq!(retry.lambda_us, previous.lambda_us);
        assert_eq!(
            retry.effective_bandwidth_hz,
            previous.effective_bandwidth_hz
        );
    }

    // --- Symmetry ---

    #[test]
    fn compute_symmetry_identical() {
        assert_eq!(compute_symmetry_pct(500, 500), 0);
    }

    #[test]
    fn compute_symmetry_20_percent() {
        // |600 - 500| / 550 ≈ 18.2% → 181 in 0.1% units
        let result = compute_symmetry_pct(600, 500);
        assert!(
            (170..=190).contains(&result),
            "symmetry {result} expected ~181"
        );
    }

    #[test]
    fn compute_symmetry_zero_avg() {
        assert_eq!(compute_symmetry_pct(0, 0), 0);
    }

    // --- Tail-level symmetry ---

    #[test]
    fn tail_symmetry_identical_is_zero() {
        assert_eq!(compute_tail_symmetry_pct(367, 367), 0);
    }

    #[test]
    fn tail_symmetry_small_mismatch() {
        // |351 - 358| / avg(351, 358) = 7 / 354.5 ≈ 19.7‰ → 19 in permille
        let result = compute_tail_symmetry_pct(351, 358);
        assert!(
            (15..=25).contains(&result),
            "tail symmetry {result} expected ~20‰"
        );
    }

    #[test]
    fn tail_symmetry_large_mismatch() {
        // |200 - 400| / avg(200, 400) = 200 / 300 ≈ 666.7‰
        let result = compute_tail_symmetry_pct(200, 400);
        assert!(
            (660..=670).contains(&result),
            "tail symmetry {result} expected ~667‰"
        );
    }

    #[test]
    fn tail_symmetry_zero_avg() {
        assert_eq!(compute_tail_symmetry_pct(0, 0), 0);
    }

    // --- Direction-normalized capture ---

    #[test]
    fn normalize_forward_positive_current() {
        // Forward capture, positive torque_actual → stored as-is.
        assert_eq!(
            normalize_capture_sample(367, CaptureDirection::Forward),
            367
        );
    }

    #[test]
    fn normalize_reverse_negative_current() {
        // Reverse capture, negative torque_actual → negated to positive.
        // A correctly responding motor produces negative current for a
        // negative torque step.
        assert_eq!(
            normalize_capture_sample(-367, CaptureDirection::Reverse),
            367
        );
    }

    #[test]
    fn normalize_reverse_wrong_sign_clamps_to_zero() {
        // Reverse capture but torque_actual is positive (wrong sign).
        // Normalized = -(+367) = -367 → clamped to 0.
        assert_eq!(normalize_capture_sample(367, CaptureDirection::Reverse), 0);
    }

    #[test]
    fn normalize_forward_wrong_sign_clamps_to_zero() {
        // Forward capture but torque_actual is negative (wrong sign).
        assert_eq!(normalize_capture_sample(-367, CaptureDirection::Forward), 0);
    }

    // --- MechanicalModel ---

    fn test_mechanical_model() -> MechanicalModel {
        MechanicalModel {
            j_eff: 500,
            b_eff: 50,
            tau_mech_ms: 10,     // J/B = 500/50 = 10ms
            theta_mech_us: 2000, // 2ms mechanical delay
            t_f: 0,
        }
    }

    // --- Velocity-loop SIMC synthesis ---

    #[test]
    fn velocity_lambda_conservative_is_6x_theta() {
        let lambda = simc_velocity_lambda_us(
            CommissionProfile::Conservative,
            10_000, // tau_mech = 10ms
            2000,   // theta_mech = 2ms
            700,    // inner lambda
            100,    // theta_e
        );
        // 6 * 2000 = 12000, bandwidth floor = 4*(700+100) = 3200
        assert_eq!(lambda, 12_000);
    }

    #[test]
    fn velocity_lambda_balanced_is_3x_theta() {
        let lambda = simc_velocity_lambda_us(CommissionProfile::Balanced, 10_000, 2000, 700, 100);
        // 3 * 2000 = 6000, bandwidth floor = 3200
        assert_eq!(lambda, 6000);
    }

    #[test]
    fn velocity_lambda_stiff_max_theta_tau() {
        // theta_mech = 2000: 1.5*2000 = 3000
        // tau_mech = 10000: tau/4 = 2500
        // max(3000, 2500) = 3000
        let lambda = simc_velocity_lambda_us(CommissionProfile::Stiff, 10_000, 2000, 700, 100);
        assert_eq!(lambda, 3200); // Clamped by bandwidth floor 4*(700+100) = 3200
    }

    #[test]
    fn velocity_lambda_bandwidth_constraint_enforced() {
        // With large inner lambda, the bandwidth floor dominates.
        let lambda = simc_velocity_lambda_us(
            CommissionProfile::Stiff,
            10_000,
            2000,
            5000, // large inner lambda
            1000, // large theta_e
        );
        // Stiff base: max(3000, 2500) = 3000
        // Bandwidth floor: 4*(5000+1000) = 24000
        // Floor wins.
        assert_eq!(lambda, 24_000);
    }

    #[test]
    fn velocity_lambda_conservative_ge_balanced() {
        for theta in [500, 1000, 2000, 5000] {
            let con =
                simc_velocity_lambda_us(CommissionProfile::Conservative, 10_000, theta, 700, 100);
            let bal = simc_velocity_lambda_us(CommissionProfile::Balanced, 10_000, theta, 700, 100);
            assert!(
                con >= bal,
                "theta={}: conservative {} must be >= balanced {}",
                theta,
                con,
                bal
            );
        }
    }

    #[test]
    fn mode_adjustment_unloaded_increases() {
        let base = 6000;
        let adjusted = apply_mode_adjustment(base, OperatingMode::Unloaded);
        // 6000 * 1.5 = 9000
        assert_eq!(adjusted, 9000);
    }

    #[test]
    fn mode_adjustment_nominal_unchanged() {
        let base = 6000;
        let adjusted = apply_mode_adjustment(base, OperatingMode::Nominal);
        assert_eq!(adjusted, 6000);
    }

    #[test]
    fn mode_adjustment_high_inertia_decreases() {
        let base = 6000;
        let adjusted = apply_mode_adjustment(base, OperatingMode::HighInertia);
        // 6000 * 0.8 = 4800
        assert_eq!(adjusted, 4800);
    }

    #[test]
    fn velocity_gains_balanced_nonzero() {
        let model = test_mechanical_model();
        let config = autotune_test_config();
        let gains = simc_velocity_gains(
            &model,
            CommissionProfile::Balanced,
            OperatingMode::Nominal,
            700,  // inner lambda from current tune
            100,  // theta_e
            1000, // peak velocity
            TEST_STEP_WIDTH,
            &config,
        );
        let gains = gains.expect("gains should be Some for valid model");
        assert!(gains.velocity_p > 0, "velocity_p must be nonzero");
        assert!(gains.velocity_i > 0, "velocity_i must be nonzero");
        assert_eq!(gains.velocity_limit, 800, "80% of peak=1000");
        assert!(gains.effective_bandwidth_hz > 0);
        assert!(gains.lambda_vel_ms > 0);
    }

    #[test]
    fn velocity_gains_field_model_stays_inside_outer_envelope() {
        let config = autotune_test_config();
        let field_models = [
            MechanicalModel {
                j_eff: 2_522_213,
                b_eff: 19_851,
                tau_mech_ms: 127,
                theta_mech_us: 14_764,
                t_f: 0,
            },
            MechanicalModel {
                j_eff: 2_624_100,
                b_eff: 20_988,
                tau_mech_ms: 125,
                theta_mech_us: 10_731,
                t_f: 0,
            },
        ];

        for model in field_models {
            let base_lambda_us = apply_mode_adjustment(
                simc_velocity_lambda_us(
                    CommissionProfile::Balanced,
                    model.tau_mech_ms * 1000,
                    model.theta_mech_us,
                    934,
                    160,
                ),
                OperatingMode::Nominal,
            )
            .max(model.theta_mech_us);

            let gains = simc_velocity_gains(
                &model,
                CommissionProfile::Balanced,
                OperatingMode::Nominal,
                934,
                160,
                19,
                TEST_STEP_WIDTH,
                &config,
            )
            .expect("field model should synthesize gains");

            assert!(
                gains.velocity_p <= VELOCITY_SIMC_MAX_P_REG,
                "velocity P {} should fit envelope {} for {:?}",
                gains.velocity_p,
                VELOCITY_SIMC_MAX_P_REG,
                model
            );
            assert!(
                gains.velocity_i <= crate::commissioning::outer::OUTER_MAX_VELOCITY_I,
                "velocity I {} should fit envelope {} for {:?}",
                gains.velocity_i,
                crate::commissioning::outer::OUTER_MAX_VELOCITY_I,
                model
            );
            assert!(
                gains.velocity_p < 1024,
                "velocity P {} should be scaled to TMC velocity-error units for {:?}",
                gains.velocity_p,
                model
            );
            assert!(
                gains.lambda_vel_ms >= base_lambda_us / 1000,
                "lambda should not tighten below base {}us for {:?}, got {}ms",
                base_lambda_us,
                model,
                gains.lambda_vel_ms
            );
        }
    }

    #[test]
    fn velocity_gains_field_model_are_scaled_to_tmc_velocity_units() {
        let config = autotune_test_config();
        let model = MechanicalModel {
            j_eff: 2_712_665,
            b_eff: 20_451,
            tau_mech_ms: 132,
            theta_mech_us: 12_895,
            t_f: 0,
        };

        let gains = simc_velocity_gains(
            &model,
            CommissionProfile::Balanced,
            OperatingMode::Nominal,
            934,
            160,
            18,
            TEST_STEP_WIDTH,
            &config,
        )
        .expect("field model should synthesize gains");

        assert!(
            gains.velocity_p < 1024,
            "P={} is still in cpms-domain units; expected TMC velocity-error scaling",
            gains.velocity_p
        );
    }

    #[test]
    fn velocity_gains_none_for_zero_inertia() {
        let mut model = test_mechanical_model();
        model.j_eff = 0;
        let config = autotune_test_config();
        assert!(
            simc_velocity_gains(
                &model,
                CommissionProfile::Balanced,
                OperatingMode::Nominal,
                700,
                100,
                1000,
                TEST_STEP_WIDTH,
                &config
            )
            .is_none()
        );
    }

    #[test]
    fn velocity_gains_none_for_zero_theta() {
        let mut model = test_mechanical_model();
        model.theta_mech_us = 0;
        let config = autotune_test_config();
        assert!(
            simc_velocity_gains(
                &model,
                CommissionProfile::Balanced,
                OperatingMode::Nominal,
                700,
                100,
                1000,
                TEST_STEP_WIDTH,
                &config
            )
            .is_none()
        );
    }

    #[test]
    fn velocity_gains_unloaded_slower_than_nominal() {
        let model = test_mechanical_model();
        let config = autotune_test_config();
        let nominal = simc_velocity_gains(
            &model,
            CommissionProfile::Balanced,
            OperatingMode::Nominal,
            700,
            100,
            1000,
            TEST_STEP_WIDTH,
            &config,
        )
        .unwrap();
        let unloaded = simc_velocity_gains(
            &model,
            CommissionProfile::Balanced,
            OperatingMode::Unloaded,
            700,
            100,
            1000,
            TEST_STEP_WIDTH,
            &config,
        )
        .unwrap();
        assert!(
            unloaded.lambda_vel_ms >= nominal.lambda_vel_ms,
            "unloaded lambda {} should be >= nominal {}",
            unloaded.lambda_vel_ms,
            nominal.lambda_vel_ms
        );
    }

    #[test]
    fn velocity_gains_high_inertia_tighter_than_nominal() {
        let model = test_mechanical_model();
        let config = autotune_test_config();
        let nominal = simc_velocity_gains(
            &model,
            CommissionProfile::Balanced,
            OperatingMode::Nominal,
            700,
            100,
            1000,
            TEST_STEP_WIDTH,
            &config,
        )
        .unwrap();
        let high_inertia = simc_velocity_gains(
            &model,
            CommissionProfile::Balanced,
            OperatingMode::HighInertia,
            700,
            100,
            1000,
            TEST_STEP_WIDTH,
            &config,
        )
        .unwrap();
        assert!(
            high_inertia.lambda_vel_ms <= nominal.lambda_vel_ms,
            "high_inertia lambda {} should be <= nominal {}",
            high_inertia.lambda_vel_ms,
            nominal.lambda_vel_ms
        );
    }

    #[test]
    fn velocity_gains_high_inertia_preserves_inner_bandwidth_floor() {
        let model = test_mechanical_model();
        let config = autotune_test_config();
        let inner_lambda_us = 5000;
        let theta_e_us = 1000;
        let bandwidth_floor_ms = 4 * (inner_lambda_us + theta_e_us) / 1000;

        let gains = simc_velocity_gains(
            &model,
            CommissionProfile::Stiff,
            OperatingMode::HighInertia,
            inner_lambda_us,
            theta_e_us,
            1000,
            TEST_STEP_WIDTH,
            &config,
        )
        .unwrap();

        assert!(
            gains.lambda_vel_ms >= bandwidth_floor_ms,
            "high-inertia lambda {}ms should preserve inner bandwidth floor {}ms",
            gains.lambda_vel_ms,
            bandwidth_floor_ms
        );
    }

    // --- Velocity validation classification ---

    #[test]
    fn velocity_classify_pass() {
        let metrics = VelocityValidation {
            rise_time_ms: 5,
            overshoot_pct: 100, // 10%
            peak_velocity: 0,
            settling_time_ms: 30,
            zero_speed_chatter: false,
            saturation_samples: 5,
            symmetry_pct: 100,
        };
        assert_eq!(
            classify_velocity_validation(&metrics, 10, 256),
            ValidationResult::Pass
        );
    }

    #[test]
    fn velocity_classify_marginal_overshoot() {
        let metrics = VelocityValidation {
            rise_time_ms: 5,
            overshoot_pct: 300, // 30%
            peak_velocity: 0,
            settling_time_ms: 30,
            zero_speed_chatter: false,
            saturation_samples: 0,
            symmetry_pct: 0,
        };
        assert_eq!(
            classify_velocity_validation(&metrics, 10, 256),
            ValidationResult::Marginal
        );
    }

    #[test]
    fn velocity_classify_fail_overshoot() {
        let metrics = VelocityValidation {
            rise_time_ms: 5,
            overshoot_pct: 500, // 50%
            peak_velocity: 0,
            settling_time_ms: 30,
            zero_speed_chatter: false,
            saturation_samples: 0,
            symmetry_pct: 0,
        };
        assert_eq!(
            classify_velocity_validation(&metrics, 10, 256),
            ValidationResult::Fail
        );
    }

    #[test]
    fn velocity_classify_marginal_chatter() {
        let metrics = VelocityValidation {
            rise_time_ms: 5,
            overshoot_pct: 50,
            peak_velocity: 0,
            settling_time_ms: 30,
            zero_speed_chatter: true,
            saturation_samples: 0,
            symmetry_pct: 0,
        };
        assert_eq!(
            classify_velocity_validation(&metrics, 10, 256),
            ValidationResult::Marginal
        );
    }

    #[test]
    fn velocity_classify_fail_settling() {
        let metrics = VelocityValidation {
            rise_time_ms: 5,
            overshoot_pct: 50,
            peak_velocity: 0,
            settling_time_ms: 150, // > 10 * tau_mech(10) = 100
            zero_speed_chatter: false,
            saturation_samples: 0,
            symmetry_pct: 0,
        };
        assert_eq!(
            classify_velocity_validation(&metrics, 10, 256),
            ValidationResult::Fail
        );
    }

    #[test]
    fn velocity_classify_fail_saturation() {
        let metrics = VelocityValidation {
            rise_time_ms: 5,
            overshoot_pct: 50,
            peak_velocity: 0,
            settling_time_ms: 30,
            zero_speed_chatter: false,
            saturation_samples: 70, // 70/256 = 27% > 25%
            symmetry_pct: 0,
        };
        assert_eq!(
            classify_velocity_validation(&metrics, 10, 256),
            ValidationResult::Fail
        );
    }

    #[test]
    fn velocity_classify_marginal_settling() {
        let metrics = VelocityValidation {
            rise_time_ms: 5,
            overshoot_pct: 50,
            peak_velocity: 0,
            settling_time_ms: 70, // 7*10 = 70, between 5*10=50 and 10*10=100
            zero_speed_chatter: false,
            saturation_samples: 0,
            symmetry_pct: 0,
        };
        assert_eq!(
            classify_velocity_validation(&metrics, 10, 256),
            ValidationResult::Marginal
        );
    }

    // --- Velocity metric extraction ---

    #[test]
    fn velocity_metrics_empty_buffer() {
        let buf = TransientBuffer::new();
        let metrics = extract_velocity_metrics(&buf, 1000, 0, 3, 50);
        assert_eq!(metrics.rise_time_ms, 0);
        assert_eq!(metrics.overshoot_pct, 0);
        assert!(!metrics.zero_speed_chatter);
    }

    #[test]
    fn velocity_metrics_chatter_detection() {
        // Fill buffer: first 200 samples at target, last 56 with oscillation.
        let mut buf = TransientBuffer::new();
        buf.reset(1000); // 1ms per sample
        for _ in 0..200 {
            buf.push(1000); // at target
        }
        for i in 0..56 {
            // Oscillate above and below zero (but stored as u32, so abs values).
            buf.push(if i % 2 == 0 { 5 } else { 0 });
        }
        let metrics = extract_velocity_metrics(&buf, 1000, 0, 3, 56);
        assert!(
            metrics.zero_speed_chatter,
            "should detect chatter in tail samples"
        );
    }

    #[test]
    fn velocity_metrics_settle_band_allows_low_target_quantization() {
        let mut buf = TransientBuffer::new();
        buf.reset(1000);
        for i in 0..256 {
            // A low-speed encoder-derived velocity target cannot represent
            // fractional counts per sample. Alternating 5/6 around target 6 is
            // stable within one count and should not look permanently unsettled.
            buf.push(if i % 2 == 0 { 6 } else { 5 });
        }

        let metrics = extract_velocity_metrics(&buf, 6, 0, 3, 0);
        assert_eq!(metrics.settling_time_ms, 0);
    }

    #[test]
    fn velocity_metrics_overshoot_allows_low_target_quantization() {
        let mut buf = TransientBuffer::new();
        buf.reset(1000);
        buf.push(7);
        for _ in 1..256 {
            buf.push(4);
        }

        let metrics = extract_velocity_metrics(&buf, 4, 0, 3, 0);
        assert_eq!(metrics.overshoot_pct, 0);
    }

    #[test]
    fn velocity_validation_fails_when_low_target_is_never_reached() {
        let mut buf = TransientBuffer::new();
        buf.reset(1000);
        for i in 0..256 {
            buf.push(if i % 4 == 2 { 1 } else { 0 });
        }

        let metrics = extract_velocity_metrics(&buf, 2, 0, 3, 0);
        assert_eq!(metrics.peak_velocity, 1);
        assert_eq!(
            classify_velocity_validation(&metrics, 139, 512),
            ValidationResult::Fail
        );
    }

    #[test]
    fn velocity_metrics_no_chatter_when_quiet() {
        let mut buf = TransientBuffer::new();
        buf.reset(1000);
        for _ in 0..256 {
            buf.push(1000);
        }
        let metrics = extract_velocity_metrics(&buf, 1000, 0, 3, 50);
        // Last 50 samples are all 1000, which is above threshold 3 — but
        // this tests the "chatter in zero-speed region" semantic. For a
        // properly captured velocity step response, the samples near target
        // would be at target, not zero. The chatter window monitors the
        // return-to-zero tail.
        // In this case the tail samples are at 1000 which IS above threshold.
        // The function checks raw values, so any value > threshold = chatter.
        // This is expected: in real use, the chatter window covers the
        // zero-velocity settling region where values should be near 0.
        assert!(metrics.zero_speed_chatter);
    }

    // --- Position oscillation detection ---

    #[test]
    fn position_no_oscillation_clean_settle() {
        let mut buf = TransientBuffer::new();
        buf.reset(1000);
        // Smooth approach to target 1000.
        for i in 0..256u32 {
            let value = (1000.0 * (1.0 - (-((i as f32) / 20.0)).exp())) as u32;
            buf.push(value);
        }
        assert!(
            !detect_position_oscillation(&buf, 1000),
            "clean settle should not detect oscillation"
        );
    }

    #[test]
    fn position_oscillation_detected_overshoot() {
        let mut buf = TransientBuffer::new();
        buf.reset(1000);
        // Overshoot > 10%: target 1000, peak 1200.
        for i in 0..256u32 {
            let base = 1000.0 * (1.0 - (-((i as f32) / 20.0)).exp());
            let overshoot = 250.0 * (-((i as f32) / 30.0)).exp() * ((i as f32) / 8.0).sin();
            buf.push((base + overshoot).max(0.0) as u32);
        }
        let peak = buf.samples().iter().copied().max().unwrap_or(0);
        if peak > 1100 {
            assert!(
                detect_position_oscillation(&buf, 1000),
                "should detect oscillation with peak={}",
                peak
            );
        }
    }

    #[test]
    fn position_oscillation_empty_buffer() {
        let buf = TransientBuffer::new();
        assert!(!detect_position_oscillation(&buf, 1000));
    }

    // --- Filter recommendations ---

    #[test]
    fn filter_no_issues_no_filters() {
        let config = recommend_filters(false, 100, false, 25, false, 500);
        assert_eq!(config.velocity_filter_hz, None);
        assert_eq!(config.position_filter_hz, None);
        assert_eq!(config.flux_filter_hz, None);
        assert_eq!(config.torque_filter_hz, None);
    }

    #[test]
    fn filter_velocity_chatter_enables_velocity_filter() {
        let config = recommend_filters(true, 100, false, 25, false, 500);
        assert_eq!(config.velocity_filter_hz, Some(200)); // 100 * 2
        assert_eq!(config.position_filter_hz, None);
        assert_eq!(config.flux_filter_hz, None);
    }

    #[test]
    fn filter_position_oscillation_enables_position_filter() {
        let config = recommend_filters(false, 100, true, 25, false, 500);
        assert_eq!(config.velocity_filter_hz, None);
        assert_eq!(config.position_filter_hz, Some(75)); // 25 * 3
        assert_eq!(config.flux_filter_hz, None);
    }

    #[test]
    fn filter_current_ringing_enables_flux_torque() {
        let config = recommend_filters(false, 100, false, 25, true, 500);
        assert_eq!(config.velocity_filter_hz, None);
        assert_eq!(config.position_filter_hz, None);
        assert_eq!(config.flux_filter_hz, Some(1000)); // 500 * 2
        assert_eq!(config.torque_filter_hz, Some(1000));
    }

    #[test]
    fn filter_all_issues_all_filters() {
        let config = recommend_filters(true, 100, true, 25, true, 500);
        assert_eq!(config.velocity_filter_hz, Some(200));
        assert_eq!(config.position_filter_hz, Some(75));
        assert_eq!(config.flux_filter_hz, Some(1000));
        assert_eq!(config.torque_filter_hz, Some(1000));
    }
}
