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
#[derive(Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TransientBuffer {
    samples: [u32; TRANSIENT_LEN],
    /// Number of samples captured so far (always <= `TRANSIENT_LEN`).
    len: u16,
    /// Microseconds between consecutive samples.
    sample_period_us: u16,
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
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CommissionProfile {
    /// `lambda = 4 * theta` — slow, very robust, minimal audible noise.
    Conservative,
    /// `lambda = 2 * theta` — good default for most motors and print setups.
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
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
        }
    }
}

/// Overall outcome of a commissioning run.
///
/// Reported in the `foci_commission_inner_result` or
/// `foci_commission_outer_result` message. The Python module uses this to
/// decide whether to persist the resulting gains.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
        }
    }
}

/// First-order-plus-dead-time (FOPDT) current plant model.
///
/// Estimated during electrical identification (Phase 5). Contains
/// both internal-unit values (used for gain computation) and physical
/// values (milliohms, microhenries — for host display and persistence).
///
/// Plant model: `G(s) = K / (tau_e * s + 1) * e^(-theta_e * s)`
/// where `K = 1 / R_int` (DC gain in internal units).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ElectricalModel {
    /// Winding resistance in internal (ADC/DAC) units.
    /// R_int = V_excitation / I_steady_state.
    pub r_int: u32,
    /// Winding inductance in internal units.
    /// L_int = V * dt_s / delta_I.
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
}
