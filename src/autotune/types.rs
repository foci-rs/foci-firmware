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
    ElectricalIdentificationFailed,
    /// Current-loop step-response validation failed after retry. Code 9.
    CurrentValidationFailed,
    /// Reserved for historical decoding: the retired mechanical-identification
    /// phase's failure code. No live phase raises this. Code 10.
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
    ClosedLoopActivationUnstable,
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
    /// A torque-flux limit was zero or otherwise unrepresentable. Code 54.
    InvalidCurrentLimit,
    /// A bounded evidence buffer reached capacity. Code 55.
    EvidenceBufferFull,
    /// A rung index addressed a slot beyond the frozen plan's rung count. Code 56.
    RungIndexOutOfRange,
    /// A frozen plan input changed after the plan was admitted: encoder counts
    /// per revolution or the configured current limit no longer match what the
    /// campaign froze. Code 57.
    ///
    /// Distinct from the other internal faults because it reports external
    /// mutation of a safety-relevant input mid-campaign, not a firmware
    /// invariant violation.
    PlanContractChanged,
    /// Rung-origin recovery could not be planned. Code 58.
    RecoveryPlanFailed,
    /// An encoder domain could not be constructed, or an encoder sample was
    /// rejected by it. Code 59.
    EncoderDomainInvalid,
    /// Retained capture state was absent where an invariant requires it. Code 60.
    MissingCaptureState,
    /// A breakaway search grid was structurally invalid. Code 61.
    SearchGridInvalid,
    /// A directional target rate could not be represented. Code 62.
    TargetRateInvalid,
    /// Two consecutive encoder samples reported the same timestamp, so the
    /// interval between them was zero and no rate is derivable. Code 63.
    ZeroElapsedSample,
    /// A campaign plan, ladder, envelope, or sub-plan could not be constructed
    /// before motion. Code 64.
    CampaignPlanInvalid,
    /// Stage-C authority could not be bound from an accepted confirmation. Code 65.
    IntegralAuthorityDenied,
    /// A periodic sampling grid was configured with a zero period. Code 67.
    SchedulePeriodZero,
    /// A grid deadline was advanced before its previous deadline was due, so
    /// the elapsed-time subtraction had no defined result. Code 68.
    ///
    /// Distinct from an arithmetic overflow: this means the caller polled
    /// early, which is a sequencing error rather than an unrepresentable value.
    ScheduleEarlyInvocation,
    /// Advancing a grid deadline overflowed. Code 69.
    ScheduleDeadlineOverflow,
    /// A safety primitive was opened while one was already open. Code 70.
    ///
    /// A sequencing fault: some path reached the next stroke without closing
    /// the previous primitive's window.
    PrimitiveAlreadyOpen,
    /// Origin recovery found the axis away from the offset it recorded before
    /// the stroke. Code 72.
    RecoveryOriginMismatch,
    /// The outer safety check found the active primitive's start timestamp
    /// ahead of the current time. Code 71.
    ///
    /// Raised outside the campaign, on the per-tick safety check, so it is
    /// stamped with whatever phase happened to be active rather than with the
    /// phase that caused it.
    PrimitiveStartInFuture,
    /// Discovery finished without a selected in-band ladder, so a path
    /// requiring a nominated band had nothing to act on. Code 66.
    ///
    /// The accompanying `BreakawayTerminalCause` carries the specific reason
    /// discovery did not select; this code says only that none was selected.
    DiscoveryBandNotSelected,
    /// Resistance identification's fitted or signed-anchor slope was zero or
    /// negative -- a physically impossible reading indicating reversed
    /// current polarity or a sign error, not a data-quality gate failure.
    /// Code 73.
    ResistanceNonPositiveSlope,
    /// Operator-requested cancel reached GPIO-first safe-state gracefully;
    /// the MCU stays running and `commission_in_flight` is cleared. Code 74.
    Cancelled,
    /// The most recent safe-state restore pass had one or more write
    /// failures; `arm_motor_electrical` refuses to arm until a subsequent
    /// restore pass completes with zero failures. Code 75.
    SafeStateIncomplete,
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
            Self::ElectricalIdentificationFailed => 8,
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
            Self::ClosedLoopActivationUnstable => 43,
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
            Self::InvalidCurrentLimit => 54,
            Self::EvidenceBufferFull => 55,
            Self::RungIndexOutOfRange => 56,
            Self::PlanContractChanged => 57,
            Self::RecoveryPlanFailed => 58,
            Self::EncoderDomainInvalid => 59,
            Self::MissingCaptureState => 60,
            Self::SearchGridInvalid => 61,
            Self::TargetRateInvalid => 62,
            Self::ZeroElapsedSample => 63,
            Self::CampaignPlanInvalid => 64,
            Self::IntegralAuthorityDenied => 65,
            Self::DiscoveryBandNotSelected => 66,
            Self::SchedulePeriodZero => 67,
            Self::ScheduleEarlyInvocation => 68,
            Self::ScheduleDeadlineOverflow => 69,
            Self::PrimitiveAlreadyOpen => 70,
            Self::PrimitiveStartInFuture => 71,
            Self::RecoveryOriginMismatch => 72,
            Self::ResistanceNonPositiveSlope => 73,
            Self::Cancelled => 74,
            Self::SafeStateIncomplete => 75,
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

pub use crate::commissioning::vocabulary_generated::CommissionPhase;

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

/// Sub-states for the `ClosedLoopActivation` engine phase.
///
/// Writes PID gains, velocity limit, voltage limits, position selection,
/// step width, motion mode, and biquad filters, then verifies position
/// hold stability.
#[derive(Debug, Clone, Copy, defmt::Format)]
pub enum ClosedLoopActivationState {
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
pub const CLOSED_LOOP_ACTIVATION_STATUS_NOT_RUN: u8 = 0;
/// Closed-loop entry fast stability check passed.
pub const CLOSED_LOOP_ACTIVATION_STATUS_PASS: u8 = 1;
/// Closed-loop entry detected runaway position growth.
pub const CLOSED_LOOP_ACTIVATION_STATUS_FAIL_RUNAWAY: u8 = 2;
/// Closed-loop entry detected bounded position drift above the threshold.
pub const CLOSED_LOOP_ACTIVATION_STATUS_FAIL_DRIFT: u8 = 3;
/// Closed-loop entry detected bounded drift accepted for downstream hold sampling.
pub const CLOSED_LOOP_ACTIVATION_STATUS_WARN_DRIFT: u8 = 4;

/// Controls which phases run and in what order for the inner engine.
///
/// Replaces the old `stop_after: CommissionPhase` mechanism.
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
            CommissionError::ElectricalIdentificationFailed,
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
            CommissionError::ClosedLoopActivationUnstable,
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
            CommissionError::InvalidCurrentLimit,
            CommissionError::EvidenceBufferFull,
            CommissionError::RungIndexOutOfRange,
            CommissionError::PlanContractChanged,
            CommissionError::RecoveryPlanFailed,
            CommissionError::EncoderDomainInvalid,
            CommissionError::MissingCaptureState,
            CommissionError::SearchGridInvalid,
            CommissionError::TargetRateInvalid,
            CommissionError::ZeroElapsedSample,
            CommissionError::CampaignPlanInvalid,
            CommissionError::IntegralAuthorityDenied,
            CommissionError::DiscoveryBandNotSelected,
            CommissionError::SchedulePeriodZero,
            CommissionError::ScheduleEarlyInvocation,
            CommissionError::ScheduleDeadlineOverflow,
            CommissionError::PrimitiveAlreadyOpen,
            CommissionError::PrimitiveStartInFuture,
            CommissionError::RecoveryOriginMismatch,
            CommissionError::Cancelled,
            CommissionError::SafeStateIncomplete,
        ];
        let mut seen = [false; 256];
        for e in errors {
            let code = e.status_code() as usize;
            assert_ne!(code, 0, "{e:?} must not use code 0 (reserved for success)");
            assert!(!seen[code], "duplicate status_code {code} for {e:?}");
            seen[code] = true;
        }
        assert_eq!(CommissionError::CurrentHoldFailed.status_code(), 42);
        assert_eq!(
            CommissionError::ClosedLoopActivationUnstable.status_code(),
            43
        );
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
        assert_eq!(CommissionError::InvalidCurrentLimit.status_code(), 54);
        assert_eq!(CommissionError::EvidenceBufferFull.status_code(), 55);
        assert_eq!(CommissionError::RungIndexOutOfRange.status_code(), 56);
        assert_eq!(CommissionError::PlanContractChanged.status_code(), 57);
        assert_eq!(CommissionError::RecoveryPlanFailed.status_code(), 58);
        assert_eq!(CommissionError::EncoderDomainInvalid.status_code(), 59);
        assert_eq!(CommissionError::MissingCaptureState.status_code(), 60);
        assert_eq!(CommissionError::SearchGridInvalid.status_code(), 61);
        assert_eq!(CommissionError::TargetRateInvalid.status_code(), 62);
        assert_eq!(CommissionError::ZeroElapsedSample.status_code(), 63);
        assert_eq!(CommissionError::CampaignPlanInvalid.status_code(), 64);
        assert_eq!(CommissionError::IntegralAuthorityDenied.status_code(), 65);
        assert_eq!(CommissionError::DiscoveryBandNotSelected.status_code(), 66);
        assert_eq!(CommissionError::SchedulePeriodZero.status_code(), 67);
        assert_eq!(CommissionError::ScheduleEarlyInvocation.status_code(), 68);
        assert_eq!(CommissionError::ScheduleDeadlineOverflow.status_code(), 69);
        assert_eq!(CommissionError::PrimitiveAlreadyOpen.status_code(), 70);
        assert_eq!(CommissionError::PrimitiveStartInFuture.status_code(), 71);
        assert_eq!(CommissionError::RecoveryOriginMismatch.status_code(), 72);
    }

    #[test]
    fn commission_chip_reset_detected_has_status_code_18() {
        assert_eq!(CommissionError::ChipResetDetected.status_code(), 18);
    }

    #[test]
    fn commission_cancelled_has_status_code_74() {
        assert_eq!(CommissionError::Cancelled.status_code(), 74);
    }

    #[test]
    fn commission_safe_state_incomplete_has_status_code_75() {
        assert_eq!(CommissionError::SafeStateIncomplete.status_code(), 75);
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

    /// Inner codes 1-8, 16, 17 are unchanged by the velocity-vocabulary
    /// renumber; code 9 (formerly `MechanicalId`) stays permanently reserved.
    #[test]
    fn inner_phase_wire_codes_unchanged() {
        assert_eq!(CommissionPhase::AdcCalibrate.to_u8(), 1);
        assert_eq!(CommissionPhase::CoilCheck.to_u8(), 2);
        assert_eq!(CommissionPhase::PhaseWiringCheck.to_u8(), 3);
        assert_eq!(CommissionPhase::EncoderCheck.to_u8(), 4);
        assert_eq!(CommissionPhase::ElectricalIdentification.to_u8(), 5);
        assert_eq!(CommissionPhase::CurrentTune.to_u8(), 6);
        assert_eq!(CommissionPhase::CurrentValidate.to_u8(), 7);
        assert_eq!(CommissionPhase::InnerDone.to_u8(), 8);
        assert_eq!(CommissionPhase::EncoderAlign.to_u8(), 16);
        assert_eq!(CommissionPhase::ClosedLoopActivation.to_u8(), 17);
    }

    /// The outer block is a contiguous sequence: the four velocity workflow
    /// codes in workflow order, then the retained non-velocity outer codes,
    /// placed after `EncoderAlign`/`ClosedLoopActivation` since those two pinned
    /// codes (16, 17) leave no 8-wide contiguous gap below them.
    #[test]
    fn outer_block_is_contiguous_in_workflow_order() {
        assert_eq!(
            CommissionPhase::BreakawayProportionalAcquisition.to_u8(),
            18
        );
        assert_eq!(
            CommissionPhase::IntegralGainResponseCharacterization.to_u8(),
            19
        );
        assert_eq!(CommissionPhase::FixedGainAmplitudeValidation.to_u8(), 20);
        assert_eq!(
            CommissionPhase::ReversalStandstillRobustnessGate.to_u8(),
            21
        );
    }

    #[test]
    fn declaration_order_is_frozen() {
        use crate::commissioning::vocabulary_generated::{
            AutotuneAction, BreakawayPhase, VelocityAutotuneWorkflowKind,
        };

        assert_eq!(
            CommissionPhase::DECLARATION_ORDER,
            &[
                ("AdcCalibrate", 1),
                ("CoilCheck", 2),
                ("PhaseWiringCheck", 3),
                ("EncoderCheck", 4),
                ("ElectricalIdentification", 5),
                ("CurrentTune", 6),
                ("CurrentValidate", 7),
                ("InnerDone", 8),
                ("BreakawayProportionalAcquisition", 18),
                ("IntegralGainResponseCharacterization", 19),
                ("FixedGainAmplitudeValidation", 20),
                ("ReversalStandstillRobustnessGate", 21),
                ("EncoderAlign", 16),
                ("ClosedLoopActivation", 17),
            ]
        );
        assert_eq!(
            BreakawayPhase::DECLARATION_ORDER,
            &[
                ("BreakawayProbe", 0),
                ("ProportionalResponseDiscovery", 1),
                ("ProportionalBandConfirmation", 2),
            ]
        );
        assert_eq!(
            VelocityAutotuneWorkflowKind::DECLARATION_ORDER,
            &[
                ("Resume", 0),
                ("FixedGainAmplitudeAscending", 1),
                ("FixedGainAmplitudeDescending", 2),
                ("BreakawaySeededProportionalThenIntegral", 3),
                ("ReversalStandstillRobustnessGate", 4),
            ]
        );
        assert_eq!(
            AutotuneAction::DECLARATION_ORDER,
            &[
                ("FixedGainAmplitudeAscending", 1),
                ("FixedGainAmplitudeDescending", 2),
                ("BreakawaySeeded", 7),
                ("IntegralResume", 8),
                ("RobustnessReversal", 9),
            ]
        );
    }
}
