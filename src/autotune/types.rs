use super::HoldGateAccumulator;

/// Tuning aggressiveness profile for commissioning gain synthesis.
///
/// Current-loop P synthesis uses the configured physical bandwidth directly;
/// retained SIMC helpers and outer-loop synthesis use this profile to select
/// their closed-loop time scale. A more aggressive profile yields faster
/// response but requires a more accurate plant model and leaves less stability
/// margin.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum CommissionProfile {
    /// Slow, very robust, minimal audible noise.
    Conservative,
    /// Good default for most motors and print setups.
    Balanced,
    /// Fastest safe response; may be audible.
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
    /// Sentinel register mismatch at arm time — chip silently lost the
    /// state established at commission. Channel must be re-commissioned
    /// before any further arm attempt. Code 18.
    ChipResetDetected,
    /// Shared multi-point resistance identification failed a quality gate or
    /// backend transaction during electrical identification before the
    /// firmware could classify the specific resistance cause. Code 19.
    ResistanceIdFailed,
    /// Resistance identification could not run because ADC calibration was not ready. Code 20.
    ResistanceAdcNotReady,
    /// Resistance identification could not run because the motor was not armed. Code 21.
    ResistanceNotArmed,
    /// Resistance identification found an unexpected power-stage setup. Code 22.
    ResistanceWrongPowerStage,
    /// Resistance identification did not find enough linear-region points. Code 23.
    ResistanceInsufficientLinearPoints,
    /// Resistance identification rejected thermal drift across the signed anchor. Code 24.
    ResistanceThermalDrift,
    /// Resistance identification found mismatched electrical-axis slopes. Code 25.
    ResistanceElectricalAxisMismatch,
    /// Resistance identification rejected a high linear-fit residual. Code 26.
    ResistanceHighResidual,
    /// Resistance identification ran with an unsupported motor type. Code 27.
    ResistanceInvalidMotorType,
    /// Resistance identification exceeded the profile current limit. Code 28.
    ResistancePeakCurrentExceeded,
    /// Resistance identification received an unsupported profile. Code 29.
    ResistanceUnsupportedProfile,
    /// Resistance identification received an out-of-bounds profile parameter. Code 30.
    ResistanceParameterOutOfBounds,
    /// Resistance identification could not find a safe measurement envelope. Code 31.
    ResistanceMeasurementEnvelopeUnsupported,
    /// Inductance identification did not reach the commanded open-loop velocity. Code 32.
    InductanceVelocityTimeout,
    /// Inductance identification rejected the AC capture quality gates. Code 33.
    InductanceCaptureRejected,
    /// Inductance identification saw encoder motion during the AC capture. Code 34.
    InductanceEncoderMotion,
    /// Inductance identification did not complete the required phi window. Code 35.
    InductanceCaptureWindow,
    /// Inductance identification measured an out-of-range field frequency. Code 36.
    InductanceFrequencyOutOfRange,
    /// Inductance identification did not measure enough quadrature signal. Code 37.
    InductanceLowQuadratureSignal,
    /// Inductance identification was missing resistance evidence. Code 38.
    InductanceMissingResistanceEvidence,
    /// Inductance identification could not compute a valid reactance. Code 39.
    InductanceReactanceInvalid,
    /// Inductance identification could not compute the drift cross-check. Code 40.
    InductanceDriftInvalid,
    /// Inductance identification rejected saliency evidence. Code 41.
    InductanceSaliencyInvalid,
    /// Sustained current-loop hold validation failed. Code 42.
    CurrentHoldFailed,
    /// Closed-loop entry stability check saw runaway or drift. Code 43.
    ClosedLoopEntryUnstable,
    /// Electrical model assembly had no usable current or inductance samples. Code 44.
    ElectricalModelNoSamples,
    /// Electrical model assembly rounded the inductance scale to zero. Code 45.
    ElectricalModelScaleZero,
    /// Electrical model assembly rejected theta as too large relative to tau. Code 46.
    ElectricalThetaTooLarge,
    /// A commissioning state returned an invalid or unrepresentable deadline. Code 47.
    InvalidSchedule,
    /// Resistance acquisition violated its timing envelope. Code 48.
    ResistanceTimingRejected,
    /// Resistance acquisition exceeded its phase-local deadline. Code 49.
    ResistanceDeadlineExceeded,
    /// Inductance acquisition violated its timing envelope. Code 50.
    InductanceTimingRejected,
    /// Electrical-delay acquisition violated its timing envelope. Code 51.
    DelayTimingRejected,
    /// Velocity-sweep analysis exceeded its declared cooperative budget. Code 52.
    AnalysisOverrun,
    /// A completed velocity rest analysis did not confirm rest. Code 53.
    VelocityRestNotConfirmed,
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
            Self::ChipResetDetected => 18,
            Self::ResistanceIdFailed => 19,
            Self::ResistanceAdcNotReady => 20,
            Self::ResistanceNotArmed => 21,
            Self::ResistanceWrongPowerStage => 22,
            Self::ResistanceInsufficientLinearPoints => 23,
            Self::ResistanceThermalDrift => 24,
            Self::ResistanceElectricalAxisMismatch => 25,
            Self::ResistanceHighResidual => 26,
            Self::ResistanceInvalidMotorType => 27,
            Self::ResistancePeakCurrentExceeded => 28,
            Self::ResistanceUnsupportedProfile => 29,
            Self::ResistanceParameterOutOfBounds => 30,
            Self::ResistanceMeasurementEnvelopeUnsupported => 31,
            Self::InductanceVelocityTimeout => 32,
            Self::InductanceCaptureRejected => 33,
            Self::InductanceEncoderMotion => 34,
            Self::InductanceCaptureWindow => 35,
            Self::InductanceFrequencyOutOfRange => 36,
            Self::InductanceLowQuadratureSignal => 37,
            Self::InductanceMissingResistanceEvidence => 38,
            Self::InductanceReactanceInvalid => 39,
            Self::InductanceDriftInvalid => 40,
            Self::InductanceSaliencyInvalid => 41,
            Self::CurrentHoldFailed => 42,
            Self::ClosedLoopEntryUnstable => 43,
            Self::ElectricalModelNoSamples => 44,
            Self::ElectricalModelScaleZero => 45,
            Self::ElectricalThetaTooLarge => 46,
            Self::InvalidSchedule => 47,
            Self::ResistanceTimingRejected => 48,
            Self::ResistanceDeadlineExceeded => 49,
            Self::InductanceTimingRejected => 50,
            Self::DelayTimingRejected => 51,
            Self::AnalysisOverrun => 52,
            Self::VelocityRestNotConfirmed => 53,
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

impl CommissionStatus {
    /// Encode to the stable status code used in host replies and trace events.
    pub fn to_u8(self) -> u8 {
        match self {
            Self::Accepted => 0,
            Self::AcceptedWithWarnings => 1,
            Self::InnerOnly => 2,
            Self::Failed(e) => e.status_code(),
        }
    }
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
    /// Inner phase 6 — Current-loop gain synthesis.
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
    /// Current-loop flux I gain (raw Q4.12).
    pub flux_i: u16,
    /// Current-loop torque P gain (Q8.8).
    pub torque_p: u16,
    /// Current-loop torque I gain (raw Q4.12).
    pub torque_i: u16,
    /// Velocity P gain (Q8.8).
    pub velocity_p: u16,
    /// Velocity I gain (Q4.12).
    pub velocity_i: u16,
    /// Position P gain (Q8.8).
    pub position_p: u16,
    /// Position I gain (Q4.12).
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
    /// Prepare the commissioning-only sustained hold accumulator.
    SustainedHoldStart,
    /// Sample closed-loop hold current, position, and status until the deadline.
    SustainedHoldSample {
        /// Sustained-hold metrics accumulator.
        accumulator: HoldGateAccumulator,
        /// Hold window start timestamp in microseconds.
        started_us: u64,
        /// Next requested sample timestamp in microseconds.
        next_sample_us: u64,
        /// Hold window deadline in microseconds.
        deadline_us: u64,
        /// Requested sample period in microseconds.
        requested_sample_period_us: u16,
        /// True when the cooperative sampler missed the requested cadence.
        sample_overrun: bool,
    },
}

/// Closed-loop entry evidence has not been captured.
pub const CLOSED_LOOP_ENTRY_STATUS_NOT_RUN: u8 = 0;
/// Closed-loop entry fast stability check passed.
pub const CLOSED_LOOP_ENTRY_STATUS_PASS: u8 = 1;
/// Closed-loop entry detected runaway position growth.
pub const CLOSED_LOOP_ENTRY_STATUS_FAIL_RUNAWAY: u8 = 2;
/// Closed-loop entry detected bounded position drift above the threshold.
pub const CLOSED_LOOP_ENTRY_STATUS_FAIL_DRIFT: u8 = 3;
/// Closed-loop entry detected bounded drift accepted for downstream hold sampling.
pub const CLOSED_LOOP_ENTRY_STATUS_WARN_DRIFT: u8 = 4;

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

#[cfg(test)]
mod tests {
    use super::*;

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
            CommissionError::ChipResetDetected,
            CommissionError::ResistanceIdFailed,
            CommissionError::ResistanceAdcNotReady,
            CommissionError::ResistanceNotArmed,
            CommissionError::ResistanceWrongPowerStage,
            CommissionError::ResistanceInsufficientLinearPoints,
            CommissionError::ResistanceThermalDrift,
            CommissionError::ResistanceElectricalAxisMismatch,
            CommissionError::ResistanceHighResidual,
            CommissionError::ResistanceInvalidMotorType,
            CommissionError::ResistancePeakCurrentExceeded,
            CommissionError::ResistanceUnsupportedProfile,
            CommissionError::ResistanceParameterOutOfBounds,
            CommissionError::ResistanceMeasurementEnvelopeUnsupported,
            CommissionError::InductanceVelocityTimeout,
            CommissionError::InductanceCaptureRejected,
            CommissionError::InductanceEncoderMotion,
            CommissionError::InductanceCaptureWindow,
            CommissionError::InductanceFrequencyOutOfRange,
            CommissionError::InductanceLowQuadratureSignal,
            CommissionError::InductanceMissingResistanceEvidence,
            CommissionError::InductanceReactanceInvalid,
            CommissionError::InductanceDriftInvalid,
            CommissionError::InductanceSaliencyInvalid,
            CommissionError::CurrentHoldFailed,
            CommissionError::ClosedLoopEntryUnstable,
            CommissionError::ElectricalModelNoSamples,
            CommissionError::ElectricalModelScaleZero,
            CommissionError::ElectricalThetaTooLarge,
            CommissionError::InvalidSchedule,
            CommissionError::ResistanceTimingRejected,
            CommissionError::ResistanceDeadlineExceeded,
            CommissionError::InductanceTimingRejected,
            CommissionError::DelayTimingRejected,
            CommissionError::AnalysisOverrun,
            CommissionError::VelocityRestNotConfirmed,
        ];
        let mut seen = [false; 256];
        for e in errors {
            let code = e.status_code() as usize;
            assert_ne!(code, 0, "{e:?} must not use code 0 (reserved for success)");
            assert!(!seen[code], "duplicate status_code {code} for {e:?}");
            seen[code] = true;
        }
        assert_eq!(CommissionError::CurrentHoldFailed.status_code(), 42);
        assert_eq!(CommissionError::ClosedLoopEntryUnstable.status_code(), 43);
        assert_eq!(CommissionError::ElectricalModelNoSamples.status_code(), 44);
        assert_eq!(CommissionError::ElectricalModelScaleZero.status_code(), 45);
        assert_eq!(CommissionError::ElectricalThetaTooLarge.status_code(), 46);
        assert_eq!(CommissionError::InvalidSchedule.status_code(), 47);
        assert_eq!(CommissionError::ResistanceTimingRejected.status_code(), 48);
        assert_eq!(
            CommissionError::ResistanceDeadlineExceeded.status_code(),
            49
        );
        assert_eq!(CommissionError::InductanceTimingRejected.status_code(), 50);
        assert_eq!(CommissionError::DelayTimingRejected.status_code(), 51);
        assert_eq!(CommissionError::AnalysisOverrun.status_code(), 52);
        assert_eq!(CommissionError::VelocityRestNotConfirmed.status_code(), 53);
    }

    #[test]
    fn commission_chip_reset_detected_has_status_code_18() {
        assert_eq!(CommissionError::ChipResetDetected.status_code(), 18);
    }

    #[test]
    fn commission_status_to_u8_uses_stable_status_codes() {
        assert_eq!(CommissionStatus::Accepted.to_u8(), 0);
        assert_eq!(CommissionStatus::AcceptedWithWarnings.to_u8(), 1);
        assert_eq!(CommissionStatus::InnerOnly.to_u8(), 2);
        assert_eq!(
            CommissionStatus::Failed(CommissionError::ChipResetDetected).to_u8(),
            18
        );
    }

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
}
