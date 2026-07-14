//! Outer commissioning phase state and phase-local data.

use crate::autotune::{
    CommissionError, CommissionProfile, FilterConfig, MechanicalModel, PhaseId, PositionGains,
    TRANSIENT_LEN, VelocityGains, VelocityValidation,
};
use crate::commissioning::encoder_domain::EncoderCount;
use crate::commissioning::types::{OuterResult, OuterSafetyFault};

use super::OUTER_WARNING_NONE;
use super::safety::OuterSafetyLimits;

pub(crate) const OUTER_EVIDENCE_UNDER_DRIVEN_PROBE: u8 = 1 << 0;
pub(crate) const OUTER_EVIDENCE_LOW_CONFIDENCE_MECH: u8 = 1 << 1;
pub(crate) const OUTER_EVIDENCE_TAU_DEFAULTED: u8 = 1 << 2;
pub(crate) const OUTER_EVIDENCE_MOTION_BUDGET_CLAMPED: u8 = 1 << 3;
pub(crate) const OUTER_EVIDENCE_CONSERVATIVE_REQUIRED: u8 = 1 << 4;
pub(crate) const OUTER_EVIDENCE_LAMBDA_WIDENED: u8 = 1 << 5;
pub(crate) const OUTER_POLICY_WARNING_MASK: u8 =
    OUTER_EVIDENCE_CONSERVATIVE_REQUIRED | OUTER_EVIDENCE_LAMBDA_WIDENED;

/// Mechanical identification sub-state machine (Phase 8).
#[derive(Debug, defmt::Format)]
pub(crate) enum MechanicalIdState {
    /// Record starting position, configure torque mode.
    Setup,
    /// Hold zero torque briefly after the mode switch, then arm safety.
    PostSwitchSettle {
        next_sample_us: u64,
        timeout_deadline_us: u64,
        stable_ticks: u8,
    },
    /// Sample encoder position into transient buffer at 1 kHz.
    ForwardCapture {
        next_sample_us: u64,
        capture_start_us: u64,
    },
    /// Return to zero torque and wait for settling.
    ForwardSettle { deadline_us: u64 },
    /// Apply reverse torque step and begin reverse capture.
    ReverseStart,
    /// Sample encoder position for reverse direction.
    ReverseCapture {
        next_sample_us: u64,
        capture_start_us: u64,
    },
    /// Return to zero torque and wait for settling.
    ReverseSettle { deadline_us: u64 },
    /// Estimate mechanical parameters from captured data.
    Estimate,
    /// Validate gates and finalize.
    Finalize,
}

/// Accumulated data for mechanical identification.
#[derive(Debug, defmt::Format)]
pub(crate) struct MechanicalIdData {
    /// Starting encoder position recorded at setup.
    pub(crate) start_position: Option<EncoderCount>,
    /// Retry attempt index for adaptive torque steps.
    pub(crate) attempt: u8,
    /// Torque step amplitude (PIDOUT units).
    pub(crate) torque_step: u16,
    /// Forward capture slope estimate + confidence.
    pub(crate) fwd_slope: crate::commissioning::outer::slope_estimator::SlopeEstimate,
    pub(crate) fwd_response: crate::commissioning::outer::response::MechanicalResponseEstimate,
    pub(crate) forward_peak_velocity: i32,
    pub(crate) forward_delay_us: u32,
    pub(crate) forward_travel: u32,
    /// Reverse capture slope estimate + confidence.
    pub(crate) rev_slope: crate::commissioning::outer::slope_estimator::SlopeEstimate,
    pub(crate) rev_response: crate::commissioning::outer::response::MechanicalResponseEstimate,
    pub(crate) reverse_peak_velocity: i32,
    pub(crate) reverse_delay_us: u32,
    pub(crate) reverse_travel: u32,
    /// Timestamp of each sample in the currently active MechanicalId capture.
    pub(crate) sample_timestamps_us: [u64; TRANSIENT_LEN],
    /// Number of valid entries in `sample_timestamps_us`.
    pub(crate) sample_timestamp_len: u16,
}

impl MechanicalIdData {
    pub(crate) fn new(torque_step: u16) -> Self {
        Self {
            start_position: None,
            attempt: 0,
            torque_step,
            fwd_slope: crate::commissioning::outer::slope_estimator::SlopeEstimate::default(),
            fwd_response:
                crate::commissioning::outer::response::MechanicalResponseEstimate::default(),
            forward_peak_velocity: 0,
            forward_delay_us: 0,
            forward_travel: 0,
            rev_slope: crate::commissioning::outer::slope_estimator::SlopeEstimate::default(),
            rev_response:
                crate::commissioning::outer::response::MechanicalResponseEstimate::default(),
            reverse_peak_velocity: 0,
            reverse_delay_us: 0,
            reverse_travel: 0,
            sample_timestamps_us: [0; TRANSIENT_LEN],
            sample_timestamp_len: 0,
        }
    }

    pub(crate) fn reset_captures(&mut self) {
        self.fwd_slope = crate::commissioning::outer::slope_estimator::SlopeEstimate::default();
        self.fwd_response =
            crate::commissioning::outer::response::MechanicalResponseEstimate::default();
        self.forward_peak_velocity = 0;
        self.forward_delay_us = 0;
        self.forward_travel = 0;
        self.rev_slope = crate::commissioning::outer::slope_estimator::SlopeEstimate::default();
        self.rev_response =
            crate::commissioning::outer::response::MechanicalResponseEstimate::default();
        self.reverse_peak_velocity = 0;
        self.reverse_delay_us = 0;
        self.reverse_travel = 0;
        self.sample_timestamp_len = 0;
    }

    pub(crate) fn reset_sample_timestamps(&mut self) {
        self.sample_timestamp_len = 0;
    }

    pub(crate) fn push_sample_timestamp(&mut self, timestamp_us: u64) {
        let idx = self.sample_timestamp_len as usize;
        if idx < self.sample_timestamps_us.len() {
            self.sample_timestamps_us[idx] = timestamp_us;
            self.sample_timestamp_len += 1;
        }
    }

    pub(crate) fn sample_timestamps(&self) -> &[u64] {
        &self.sample_timestamps_us[..self.sample_timestamp_len as usize]
    }
}

/// Mechanical-ID diagnostics carried into the final host result.
#[derive(Debug, Clone, Copy, defmt::Format)]
pub(crate) struct OuterDiagnostics {
    pub(crate) warning_code: u8,
    pub(crate) mech_torque_step: u16,
    pub(crate) mech_fwd_accel: u32,
    pub(crate) mech_rev_accel: u32,
    pub(crate) mech_fwd_peak_velocity: u32,
    pub(crate) mech_rev_peak_velocity: u32,
    pub(crate) mech_fwd_travel: u32,
    pub(crate) mech_rev_travel: u32,
    /// True when mechanical-ID fell back instead of deriving tau from J/B.
    pub(crate) mech_tau_defaulted: bool,
    pub(crate) synth_velocity_p: u16,
    pub(crate) synth_velocity_i: u16,
    pub(crate) synth_position_p: u16,
    pub(crate) synth_position_i: u16,
    pub(crate) outer_evidence_flags: u8,
    pub(crate) stiffness_timebase_ms: u16,
    pub(crate) velocity_search_stop_reason: u8,
    pub(crate) motion_budget_mrev: u16,
}

impl OuterDiagnostics {
    pub(crate) fn from_mechanical_id(data: &MechanicalIdData) -> Self {
        Self {
            warning_code: OUTER_WARNING_NONE,
            mech_torque_step: data.torque_step,
            mech_fwd_accel: data.fwd_slope.accel.unsigned_abs(),
            mech_rev_accel: data.rev_slope.accel.unsigned_abs(),
            mech_fwd_peak_velocity: data.forward_peak_velocity.unsigned_abs(),
            mech_rev_peak_velocity: data.reverse_peak_velocity.unsigned_abs(),
            mech_fwd_travel: data.forward_travel,
            mech_rev_travel: data.reverse_travel,
            mech_tau_defaulted: false,
            synth_velocity_p: 0,
            synth_velocity_i: 0,
            synth_position_p: 0,
            synth_position_i: 0,
            outer_evidence_flags: 0,
            stiffness_timebase_ms: 0,
            velocity_search_stop_reason: 0,
            motion_budget_mrev: 0,
        }
    }
}

/// Velocity-loop validation sub-state machine (Phase 10).
#[derive(Debug, defmt::Format)]
pub(crate) enum VelocityValidateState {
    /// Write velocity gains to TMC4671 registers.
    WriteGains,
    /// Switch to velocity mode and apply forward velocity step.
    ForwardStep,
    /// Capture forward step response at 1 kHz.
    ForwardCapture {
        next_sample_us: u64,
        last_sample_us: u64,
        ambiguity_retried: bool,
        saturation_start: u16,
    },
    /// Return to zero velocity and wait for settling.
    ForwardSettle { deadline_us: u64 },
    /// Apply reverse velocity step.
    ReverseStep,
    /// Capture reverse step response.
    ReverseCapture {
        next_sample_us: u64,
        last_sample_us: u64,
        ambiguity_retried: bool,
        saturation_start: u16,
    },
    /// Return to zero velocity and wait for settling.
    ReverseSettle { deadline_us: u64 },
    /// Monitor zero-speed chatter for 200ms.
    ChatterCheck {
        next_sample_us: u64,
        last_sample_us: u64,
        remaining: u16,
        ambiguity_retried: bool,
    },
    /// Extract metrics and classify response.
    Classify,
    /// Restore safe state before finalize or retry.
    SafeState { deadline_us: u64 },
    /// Emit result or transition.
    Finalize,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) enum VelocityTuneStrategy {
    /// Validate gains synthesized from the mechanical model.
    Model,
    /// Tune motor-local velocity P from bounded closed-loop response.
    ResponseRamp,
}

pub(crate) const VELOCITY_SEARCH_STOP_CLEAN: u8 = 1;
pub(crate) const VELOCITY_SEARCH_STOP_UNDER_DRIVEN_BUDGET: u8 = 2;
pub(crate) const VELOCITY_SEARCH_STOP_UNSAFE_BACKOFF: u8 = 3;
pub(crate) const VELOCITY_SEARCH_STOP_LOW_CONFIDENCE: u8 = 4;

#[derive(Debug, Clone, Copy, PartialEq, defmt::Format)]
pub(crate) struct EffectiveVelocityPolicy {
    pub(crate) effective_profile: CommissionProfile,
    pub(crate) lambda_multiplier: f32,
    pub(crate) conservative_required: bool,
    pub(crate) lambda_widened: bool,
}

/// Accumulated data for velocity-loop validation.
#[derive(Debug, defmt::Format)]
pub(crate) struct VelocityValidateData {
    /// Synthesized velocity gains for the current attempt.
    pub(crate) gains: VelocityGains,
    /// Last response-ramp gain set that validated cleanly.
    pub(crate) accepted_gains: VelocityGains,
    /// Whether `accepted_gains` came from a passing response-ramp attempt.
    pub(crate) has_accepted_gains: bool,
    /// Highest response-ramp P that still looked under-driven.
    pub(crate) response_lower_p: u16,
    /// Lowest response-ramp P that looked unsafe before any gain passed.
    pub(crate) response_upper_p: u16,
    /// The mechanical model from Phase 8 (needed for retry re-synthesis).
    pub(crate) model: MechanicalModel,
    /// Effective policy derived once from the requested profile and inner confidence.
    pub(crate) policy: EffectiveVelocityPolicy,
    /// Velocity tuning strategy for this run.
    pub(crate) strategy: VelocityTuneStrategy,
    /// Timebase used for production stiffness classification.
    pub(crate) stiffness_timebase_ms: u32,
    /// True when mechanical timing is trusted enough for production verdict.
    pub(crate) mech_tau_trusted: bool,
    /// Reason the velocity search stopped.
    pub(crate) velocity_search_stop_reason: u8,
    /// Current attempt (0 = first, 1 = retry).
    pub(crate) attempt: u8,
    /// Velocity step amplitude (encoder counts/ms or velocity target units).
    pub(crate) step_target: u32,
    /// Forward capture metrics.
    pub(crate) forward_metrics: Option<VelocityValidation>,
    /// Reverse capture metrics.
    pub(crate) reverse_metrics: Option<VelocityValidation>,
    /// Maximum chatter value observed during zero-speed check.
    pub(crate) max_chatter: u32,
    /// Peak velocity from mechanical ID (for velocity limit derivation).
    pub(crate) peak_velocity: u32,
    /// Previous encoder count for velocity derivation (position delta per tick).
    pub(crate) prev_encoder_for_velocity: Option<EncoderCount>,
    /// Saturation counter: samples where the velocity PID's torque output
    /// is near the current limit (indicating the controller is effort-limited).
    pub(crate) saturation_count: u16,
    /// Current limit for saturation detection (from `backend.current_limit()`).
    pub(crate) torque_limit: u32,
    /// Operational velocity limit to restore after validation.
    pub(crate) operational_velocity_limit: u32,
    /// Diagnostics carried into the final result.
    pub(crate) diagnostics: OuterDiagnostics,
}

/// Position-loop tuning sub-state machine (Phase 11).
#[derive(Debug, defmt::Format)]
pub(crate) enum PositionTuneState {
    /// Switch to position mode and write initial P gain.
    Setup,
    /// Command a position step and capture response.
    StepCapture { next_sample_us: u64 },
    /// Wait for settling after capture.
    Settle { deadline_us: u64 },
    /// Analyze capture: oscillation check and decide to increase P or stop.
    Analyze,
    /// Restore safe state and finalize.
    Finalize,
}

/// Accumulated data for position-loop tuning.
#[derive(Debug, defmt::Format)]
pub(crate) struct PositionTuneData {
    /// Current candidate P gain.
    pub(crate) current_p: u16,
    /// Last known stable P gain (no oscillation).
    pub(crate) stable_p: u16,
    /// Position step target in ABN encoder counts.
    pub(crate) step_target_counts: u32,
    /// Position step target in TMC electrical-position units.
    pub(crate) step_target_tmc: u32,
    /// Starting ABN encoder count for response measurement.
    pub(crate) step_start_counts: Option<EncoderCount>,
    /// Starting TMC position for target writes.
    pub(crate) step_start_tmc: i32,
    /// Whether oscillation was detected early (at the first P value).
    pub(crate) oscillation_at_minimum: bool,
    /// Accepted velocity gains from Phase 10.
    pub(crate) velocity_gains: VelocityGains,
    /// Whether zero-speed chatter was detected during velocity validation.
    pub(crate) velocity_chatter: bool,
    /// Whether current ringing was detected (from inner validation).
    pub(crate) current_ringing: bool,
    /// Current-loop effective bandwidth Hz (for filter recommendation).
    pub(crate) current_bandwidth_hz: u32,
    /// Identified effective inertia from mechanical ID (internal units).
    pub(crate) j_eff: u32,
    /// Identified effective damping from mechanical ID (internal units).
    pub(crate) b_eff: u32,
    /// Current limit for saturation detection during the position step capture.
    pub(crate) torque_limit: u32,
    /// Saturation counter: samples where the inner-loop torque output is at
    /// or above 90% of `torque_limit` during the position step capture.
    pub(crate) saturation_count: u16,
    /// Diagnostics carried into the final result.
    pub(crate) diagnostics: OuterDiagnostics,
}

/// Outer commissioning phase state machine.
#[derive(Debug, defmt::Format)]
#[allow(dead_code)] // Later phases (9-13) are placeholders for future plans.
#[allow(clippy::large_enum_variant)]
// MechanicalId carries a ~2 KB sample_timestamps_us array for time-aware
// metrics extraction. On STM32F407's 128 KB SRAM the variant size is
// acceptable; boxing is not available in no_std without an allocator.
pub(crate) enum OuterPhase {
    /// Phase 8: Mechanical identification.
    MechanicalId {
        state: MechanicalIdState,
        data: MechanicalIdData,
        safety: OuterSafetyLimits,
    },
    /// Phase 9: Velocity-loop SIMC gain synthesis.
    VelocityTune {
        model: MechanicalModel,
        /// Peak velocity from mechanical ID (for velocity limit derivation).
        peak_velocity: u32,
        diagnostics: OuterDiagnostics,
        safety: OuterSafetyLimits,
    },
    /// Phase 10: Velocity-loop step-response validation.
    VelocityValidate {
        state: VelocityValidateState,
        data: VelocityValidateData,
        safety: OuterSafetyLimits,
    },
    /// Phase 11: Position-loop P ramp-up.
    PositionTune {
        state: PositionTuneState,
        data: PositionTuneData,
        safety: OuterSafetyLimits,
    },
    /// Phase 12: Filter selection.
    FilterTune {
        /// Accepted velocity gains.
        velocity_gains: VelocityGains,
        /// Accepted position gains.
        position_gains: PositionGains,
        /// Whether position oscillation was detected early (at minimum P).
        position_oscillation_early: bool,
        /// Whether velocity chatter was detected.
        velocity_chatter: bool,
        /// Whether current ringing was detected.
        current_ringing: bool,
        /// Current-loop bandwidth Hz.
        current_bandwidth_hz: u32,
        /// Identified effective inertia from mechanical ID (internal units).
        j_eff: u32,
        /// Identified effective damping from mechanical ID (internal units).
        b_eff: u32,
        diagnostics: OuterDiagnostics,
        safety: OuterSafetyLimits,
    },
    /// Phase 13: Assemble and transmit results.
    Commit {
        velocity_gains: VelocityGains,
        position_gains: PositionGains,
        filter_config: FilterConfig,
        /// Identified effective inertia from mechanical ID (internal units).
        j_eff: u32,
        /// Identified effective damping from mechanical ID (internal units).
        b_eff: u32,
        diagnostics: OuterDiagnostics,
    },
    /// All outer phases completed.
    Done(OuterResult),
    /// A phase failed.
    Failed(CommissionError),
    /// A phase failed because a Stage 2 safety envelope tripped.
    FailedWithSafety {
        /// Terminal Stage 2 status.
        error: CommissionError,
        /// Host-visible safety-envelope evidence.
        safety_fault: OuterSafetyFault,
    },
}

impl OuterPhase {
    /// Return the stable phase ID for the active non-terminal outer phase.
    pub(crate) fn current_phase_id(&self) -> Option<PhaseId> {
        match self {
            Self::MechanicalId { .. } => Some(PhaseId::MechanicalId),
            Self::VelocityTune { .. } => Some(PhaseId::VelocityTune),
            Self::VelocityValidate { .. } => Some(PhaseId::VelocityValidate),
            Self::PositionTune { .. } => Some(PhaseId::PositionTune),
            Self::FilterTune { .. } => Some(PhaseId::FilterTune),
            Self::Commit { .. } => Some(PhaseId::Commit),
            Self::Done(_) => Some(PhaseId::OuterDone),
            Self::Failed(_) | Self::FailedWithSafety { .. } => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use core::mem::size_of;

    use super::{MechanicalIdData, MechanicalIdState, OuterPhase};
    use crate::autotune::{CommissionError, PhaseId};

    use crate::commissioning::outer::OuterSafetyLimits;

    #[test]
    fn outer_phase_size_stays_within_reasonable_stack_budget() {
        assert!(
            size_of::<OuterPhase>() <= 4096,
            "OuterPhase grew to {} bytes",
            size_of::<OuterPhase>()
        );
    }

    #[test]
    fn outer_phase_id_maps_active_and_failed_phases() {
        assert_eq!(
            OuterPhase::MechanicalId {
                state: MechanicalIdState::Setup,
                data: MechanicalIdData::new(0),
                safety: OuterSafetyLimits::new(
                    0,
                    crate::commissioning::outer::OuterMotionBudget::firmware_caps()
                ),
            }
            .current_phase_id(),
            Some(PhaseId::MechanicalId)
        );
        assert_eq!(
            OuterPhase::Failed(CommissionError::Spi).current_phase_id(),
            None
        );
    }
}
