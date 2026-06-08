//! Stage 2 (mechanical / velocity / position / filter) commissioning.
//!
//! `OuterPhase` is the cooperative state-machine type. `step_outer`
//! dispatches one tick to the per-phase submodule, prefixed by a
//! safety-envelope check that aborts on velocity- or position-window
//! violations. The phase variants and their sub-state types live here
//! so each per-phase submodule can pattern-match and construct them.

pub mod commit;
pub mod filter;
mod limits;
pub mod mechanical;
pub mod position;
pub mod response;
pub mod slope_estimator;
mod trace;
pub mod velocity;

use crate::autotune::{
    CommissionError, CommissionProfile, FilterConfig, MechanicalModel, OperatingMode, PhaseId,
    PositionGains, TRANSIENT_LEN, TransientBuffer, VelocityGains, VelocityValidation,
};
use crate::commissioning::backend::CommissioningBackend;
use crate::commissioning::types::{OuterResult, StepResult};

pub(crate) use limits::{
    MECHANICAL_ID_VELOCITY_CAP_COUNTS_PER_MS, OUTER_FALLBACK_POSITION_I, OUTER_FALLBACK_POSITION_P,
    OUTER_FALLBACK_VELOCITY_I, OUTER_FALLBACK_VELOCITY_P, OUTER_MAX_POSITION_I,
    OUTER_MAX_POSITION_P, OUTER_MAX_VELOCITY_I, OUTER_MAX_VELOCITY_P, OUTER_WARNING_NONE,
    OUTER_WARNING_UNSAFE_SYNTH_GAINS, POS_TUNE_INITIAL_P, REFERENCE_ENCODER_COUNTS_PER_REV,
    mechanical_id_velocity_cap_counts_per_ms, position_tune_step_counts, position_window_counts,
    response_fallback_velocity_limit_counts_per_ms,
};
use limits::{POSITION_WINDOW_COUNTS, normalized_counts_per_rev};
pub(crate) use trace::{
    TraceSummarySeqs, trace_diagnostic_warning, trace_pack_i16_pair, trace_pack_u16_pair,
    trace_summary_event,
};

// ---------------------------------------------------------------------------
// Outer sub-state types
// ---------------------------------------------------------------------------

/// Mechanical identification sub-state machine (Phase 8).
#[derive(Debug, defmt::Format)]
pub(crate) enum MechanicalIdState {
    /// Record starting position, configure torque mode.
    Setup,
    /// Hold zero torque briefly after the mode switch, then arm safety.
    PostSwitchSettle {
        earliest_sample_us: u64,
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
    pub(crate) start_position: i32,
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
    pub(crate) fn new(start_position: i32, torque_step: u16) -> Self {
        Self {
            start_position,
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
    pub(crate) synth_velocity_p: u16,
    pub(crate) synth_velocity_i: u16,
    pub(crate) synth_position_p: u16,
    pub(crate) synth_position_i: u16,
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
            synth_velocity_p: 0,
            synth_velocity_i: 0,
            synth_position_p: 0,
            synth_position_i: 0,
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
    },
    /// Return to zero velocity and wait for settling.
    ForwardSettle { deadline_us: u64 },
    /// Apply reverse velocity step.
    ReverseStep,
    /// Capture reverse step response.
    ReverseCapture {
        next_sample_us: u64,
        last_sample_us: u64,
    },
    /// Return to zero velocity and wait for settling.
    ReverseSettle { deadline_us: u64 },
    /// Monitor zero-speed chatter for 200ms.
    ChatterCheck {
        next_sample_us: u64,
        last_sample_us: u64,
        remaining: u16,
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
    /// Velocity tuning strategy for this run.
    pub(crate) strategy: VelocityTuneStrategy,
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
    pub(crate) prev_encoder_for_velocity: u32,
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
    pub(crate) step_start_counts: i32,
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

/// Outer safety limits, computed once at the start of outer commissioning.
#[derive(Debug, Clone, Copy, defmt::Format)]
pub(crate) struct OuterSafetyLimits {
    /// Maximum absolute velocity in encoder counts per millisecond.
    pub(crate) velocity_cap: i32,
    /// Maximum distance from start position in encoder counts.
    pub(crate) position_window: i32,
    /// Unwrapped signed position accumulated from wrap-aware deltas.
    pub(crate) unwrapped_position: i32,
    /// Torque cap: the configured run_current limit.
    pub(crate) torque_cap: u16,
    /// Previous raw encoder count for delta computation.
    pub(crate) prev_encoder_count: i32,
    /// Encoder counts per revolution (PPR * 4). Used to normalize
    /// wrap-around deltas when the ABN decoder counter rolls over.
    pub(crate) counts_per_rev: i32,
    /// Timestamp (microseconds) of the previous safety sample.
    /// Zero means the next sample initializes the baseline.
    pub(crate) last_sample_us: u64,
}

impl OuterSafetyLimits {
    pub(crate) fn new(start_position: i32, torque_cap: u16) -> Self {
        Self {
            velocity_cap: MECHANICAL_ID_VELOCITY_CAP_COUNTS_PER_MS,
            position_window: POSITION_WINDOW_COUNTS,
            unwrapped_position: 0,
            torque_cap,
            prev_encoder_count: start_position,
            counts_per_rev: 0, // Set by MechanicalId::Setup after reading ABN_DECODER_PPR
            last_sample_us: 0,
        }
    }

    pub(crate) fn configure_encoder_resolution(&mut self, counts_per_rev: i32) {
        let counts_per_rev = normalized_counts_per_rev(counts_per_rev);
        self.counts_per_rev = counts_per_rev;
        self.velocity_cap = mechanical_id_velocity_cap_counts_per_ms(counts_per_rev);
        self.position_window = position_window_counts(counts_per_rev);
    }
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
            Self::Failed(_) => None,
        }
    }
}

// ---------------------------------------------------------------------------
// Safety helpers
// ---------------------------------------------------------------------------

/// Transition the outer phase to `Failed` and emit an `OuterFailed` result.
pub(crate) fn fail_outer(phase: &mut OuterPhase, err: CommissionError) -> StepResult {
    *phase = OuterPhase::Failed(err);
    StepResult::OuterFailed(err)
}

/// Map a safety-envelope violation to the dedicated error variant.
///
/// Always returns `SafetyEnvelopeViolation` so the OuterFailed handler
/// can classify it as a safety fault requiring hard disable, distinct
/// from normal phase failures that are eligible for soft restore.
pub(crate) fn safety_error_for_phase(_phase: &OuterPhase) -> CommissionError {
    CommissionError::SafetyEnvelopeViolation
}

/// Per-tick safety envelope check for all outer phases.
///
/// Reads the current encoder position, computes a wrap-aware delta,
/// normalizes velocity by actual elapsed time, and accumulates an
/// unwrapped signed position. Checks:
/// - Time-normalized velocity within cap.
/// - Unwrapped position within window from start.
///
/// Returns `true` if a safety limit is violated.
pub(crate) async fn check_outer_safety<B: CommissioningBackend>(
    safety: &mut OuterSafetyLimits,
    backend: &mut B,
) -> Result<bool, B::Error> {
    // Safety limits are initialized by MechanicalId::Setup after it reads
    // the real encoder baseline. Skip the pre-setup placeholder values.
    if safety.torque_cap == 0 {
        return Ok(false);
    }

    let encoder_raw = backend.try_read_encoder_count().await?;
    let encoder_count = encoder_raw as i32;
    let now_us = backend.elapsed_us();

    // Wrap-aware delta in encoder counts since the last safety sample.
    let delta_counts = response::encoder_delta(
        encoder_count,
        safety.prev_encoder_count,
        safety.counts_per_rev,
    );
    let dt_us = if safety.last_sample_us == 0 {
        0
    } else {
        now_us.wrapping_sub(safety.last_sample_us)
    };
    let velocity_cpms = response::counts_per_ms(delta_counts, dt_us);
    safety.prev_encoder_count = encoder_count;
    safety.last_sample_us = now_us;
    safety.unwrapped_position += delta_counts;

    // Check velocity cap (absolute value).
    let abs_velocity = velocity_cpms.unsigned_abs() as i32;
    if abs_velocity > safety.velocity_cap {
        defmt::warn!(
            "outer safety: velocity {} exceeds cap {} (delta={} dt_us={})",
            abs_velocity,
            safety.velocity_cap,
            delta_counts,
            dt_us,
        );
        return Ok(true);
    }

    // Check position window against unwrapped accumulator.
    let abs_position = safety.unwrapped_position.unsigned_abs() as i32;
    if abs_position > safety.position_window {
        defmt::warn!(
            "outer safety: position {} exceeds window {}",
            abs_position,
            safety.position_window
        );
        return Ok(true);
    }

    Ok(false)
}

/// Get a mutable reference to the safety limits for the current phase.
///
/// Returns `None` for terminal phases (Done/Failed/Commit) that don't
/// need per-tick safety checks.
fn outer_safety_mut(phase: &mut OuterPhase) -> Option<&mut OuterSafetyLimits> {
    match phase {
        OuterPhase::MechanicalId { safety, .. }
        | OuterPhase::VelocityTune { safety, .. }
        | OuterPhase::VelocityValidate { safety, .. }
        | OuterPhase::PositionTune { safety, .. }
        | OuterPhase::FilterTune { safety, .. } => Some(safety),
        OuterPhase::Commit { .. } | OuterPhase::Done(_) | OuterPhase::Failed(_) => None,
    }
}

// ---------------------------------------------------------------------------
// Dispatcher
// ---------------------------------------------------------------------------

/// Step outer phase state machine by one cooperative tick.
///
/// Dispatches to the per-phase handler in the matching submodule. Every
/// tick runs the safety envelope check before the phase-specific logic;
/// a violation aborts to `Failed(SafetyEnvelopeViolation)` after zeroing
/// excitation.
#[allow(clippy::too_many_arguments)]
#[allow(dead_code)]
pub(crate) async fn step_outer<B: CommissioningBackend>(
    phase: &mut OuterPhase,
    buf: &mut TransientBuffer,
    profile: CommissionProfile,
    mode: OperatingMode,
    inner_lambda_us: u32,
    theta_e_us: u32,
    current_ringing: bool,
    current_bandwidth_hz: u32,
    tau_e_us: u32,
    tau_e_crosscheck_us: u32,
    tau_residual_permille: u16,
    inner_warning_flags: u8,
    backend: &mut B,
) -> StepResult {
    let mut trace_summary_seqs = TraceSummarySeqs::new();
    step_outer_with_trace(
        phase,
        buf,
        profile,
        mode,
        inner_lambda_us,
        theta_e_us,
        current_ringing,
        current_bandwidth_hz,
        tau_e_us,
        tau_e_crosscheck_us,
        tau_residual_permille,
        inner_warning_flags,
        &mut trace_summary_seqs,
        backend,
    )
    .await
}

#[allow(clippy::too_many_arguments)]
pub(crate) async fn step_outer_with_trace<B: CommissioningBackend>(
    phase: &mut OuterPhase,
    buf: &mut TransientBuffer,
    profile: CommissionProfile,
    mode: OperatingMode,
    inner_lambda_us: u32,
    theta_e_us: u32,
    current_ringing: bool,
    current_bandwidth_hz: u32,
    tau_e_us: u32,
    tau_e_crosscheck_us: u32,
    tau_residual_permille: u16,
    inner_warning_flags: u8,
    trace_summary_seqs: &mut TraceSummarySeqs,
    backend: &mut B,
) -> StepResult {
    // `tau_e_us` and `tau_e_crosscheck_us` are persisted in the outer
    // engine for Phase 2 (adaptive measurement / fitted-tau crosscheck);
    // Phase 1 outer logic uses only `tau_residual_permille` and
    // `inner_warning_flags`. Keeping the wire/struct contract unchanged
    // avoids a Phase 2 schema bump.
    let _ = (tau_e_us, tau_e_crosscheck_us);
    if let Some(safety) = outer_safety_mut(phase) {
        match check_outer_safety(safety, backend).await {
            Ok(true) => {
                // Zero excitation immediately on any safety violation.
                let _ = backend
                    .write_register(foci_tmc4671::registers::PID_TORQUE_FLUX_TARGET, 0)
                    .await;
                let _ = backend
                    .write_register(foci_tmc4671::registers::MODE_RAMP_MODE_MOTION, 0)
                    .await;
                let err = safety_error_for_phase(phase);
                return fail_outer(phase, err);
            }
            Ok(false) => {}
            Err(_) => {
                defmt::warn!("outer safety: encoder read failed");
                return fail_outer(phase, CommissionError::Spi);
            }
        }
    }

    match phase {
        OuterPhase::MechanicalId { .. } => {
            mechanical::step_mechanical_id_with_trace(phase, buf, trace_summary_seqs, backend).await
        }
        OuterPhase::VelocityTune { .. } => {
            velocity::step_velocity_tune(
                phase,
                profile,
                mode,
                inner_lambda_us,
                theta_e_us,
                tau_residual_permille,
                inner_warning_flags,
                current_bandwidth_hz,
                backend,
            )
            .await
        }
        OuterPhase::VelocityValidate { .. } => {
            velocity::step_velocity_validate_with_trace(
                phase,
                buf,
                profile,
                mode,
                inner_lambda_us,
                theta_e_us,
                current_ringing,
                current_bandwidth_hz,
                trace_summary_seqs,
                backend,
            )
            .await
        }
        OuterPhase::PositionTune { .. } => {
            position::step_position_tune_with_trace(phase, buf, mode, trace_summary_seqs, backend)
                .await
        }
        OuterPhase::FilterTune { .. } => filter::step_filter_tune(phase, backend).await,
        OuterPhase::Commit { .. } => commit::step_commit(phase),
        OuterPhase::Done(result) => StepResult::OuterDone(result.clone()),
        OuterPhase::Failed(err) => StepResult::OuterFailed(*err),
    }
}

#[cfg(test)]
mod tests {
    use core::mem::size_of;

    use super::{
        MECHANICAL_ID_VELOCITY_CAP_COUNTS_PER_MS, MechanicalIdData, MechanicalIdState,
        OuterDiagnostics, OuterPhase, OuterSafetyLimits, VelocityTuneStrategy,
        VelocityValidateData, VelocityValidateState, check_outer_safety, mechanical, step_outer,
        velocity,
    };
    use crate::autotune::{
        CommissionError, CommissionProfile, MechanicalModel, OperatingMode, PhaseId,
        TransientBuffer, VelocityGains, VelocityValidation,
    };
    use crate::commission_dispatch::test_support::{MockBackend, SpiOp};
    use crate::commissioning::types::StepResult;
    use foci_tmc4671::registers::{
        ABN_DECODER_COUNT, ABN_DECODER_PPR, MODE_RAMP_MODE_MOTION, PID_TORQUE_FLUX_TARGET,
    };

    #[test]
    fn outer_safety_uses_mechanical_id_velocity_cap() {
        let safety = OuterSafetyLimits::new(0, 100);
        assert_eq!(
            safety.velocity_cap,
            MECHANICAL_ID_VELOCITY_CAP_COUNTS_PER_MS
        );
    }

    #[test]
    fn outer_safety_limits_scale_with_encoder_resolution() {
        let mut safety = OuterSafetyLimits::new(0, 100);

        safety.configure_encoder_resolution(8_000);

        assert_eq!(safety.counts_per_rev, 8_000);
        assert_eq!(safety.velocity_cap, 48);
        assert_eq!(safety.position_window, 16_000);
    }

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
                data: MechanicalIdData::new(0, 0),
                safety: OuterSafetyLimits::new(0, 0),
            }
            .current_phase_id(),
            Some(PhaseId::MechanicalId)
        );
        assert_eq!(
            OuterPhase::Failed(CommissionError::Spi).current_phase_id(),
            None
        );
    }

    #[test]
    fn outer_safety_normalizes_velocity_by_elapsed_time() {
        pollster::block_on(async {
            let mut safety = OuterSafetyLimits::new(0, 100);
            safety.counts_per_rev = 4000;
            safety.prev_encoder_count = 2328;
            safety.last_sample_us = 100_000;
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 124_909;
            backend.encoder_count_value = 2525; // delta=197 over 24.909ms => ~7 counts/ms

            let violated = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("encoder read should succeed");

            assert!(
                !violated,
                "time-normalized velocity should stay below the cap"
            );
            assert_eq!(safety.prev_encoder_count, 2525);
            assert_eq!(safety.last_sample_us, 124_909);
            assert_eq!(safety.unwrapped_position, 197);
        });
    }

    #[test]
    fn outer_safety_read_error_fails_with_spi() {
        pollster::block_on(async {
            let mut safety = OuterSafetyLimits::new(0, 2448);
            safety.counts_per_rev = 4000;
            safety.last_sample_us = 1_000;
            let mut phase = OuterPhase::MechanicalId {
                state: MechanicalIdState::ForwardCapture {
                    next_sample_us: 2_000,
                    capture_start_us: 1_000,
                },
                data: MechanicalIdData::new(0, 367),
                safety,
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 2_000;
            backend.fail_on_read = Some(ABN_DECODER_COUNT.addr());

            let result = step_outer(
                &mut phase,
                &mut buf,
                CommissionProfile::Balanced,
                OperatingMode::Nominal,
                0,
                0,
                false,
                0,
                0,
                0,
                0,
                0,
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::OuterFailed(CommissionError::Spi)
            ));
            assert!(matches!(phase, OuterPhase::Failed(CommissionError::Spi)));
        });
    }

    #[test]
    fn mechanical_id_setup_zeroes_torque_before_switching_modes() {
        pollster::block_on(async {
            let mut phase = OuterPhase::MechanicalId {
                state: MechanicalIdState::Setup,
                data: MechanicalIdData::new(0, 0),
                safety: OuterSafetyLimits::new(0, 0),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();
            backend.current_limit_value = 2448;
            backend.encoder_count_value = 1234;
            let _ = backend.registers.insert(ABN_DECODER_PPR.addr(), 4000);

            let result = step_outer(
                &mut phase,
                &mut buf,
                CommissionProfile::Balanced,
                OperatingMode::Nominal,
                0,
                0,
                false,
                0,
                0,
                0,
                0,
                0,
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::PhaseChanged(PhaseId::MechanicalId)
            ));

            let zero_torque_idx = backend
                .spi_log
                .iter()
                .position(|op| {
                    matches!(
                        op,
                        SpiOp::Write(addr, 0) if *addr == PID_TORQUE_FLUX_TARGET.addr()
                    )
                })
                .expect("setup should zero PID_TORQUE_FLUX_TARGET");
            let mode_switch_idx = backend
                .spi_log
                .iter()
                .position(|op| {
                    matches!(
                        op,
                        SpiOp::Write(addr, _) if *addr == MODE_RAMP_MODE_MOTION.addr()
                    )
                })
                .expect("setup should switch MODE_RAMP_MODE_MOTION");

            assert!(
                zero_torque_idx < mode_switch_idx,
                "setup must zero torque before entering torque mode"
            );
        });
    }

    #[test]
    fn mechanical_id_ignores_immediate_post_switch_velocity_spike() {
        pollster::block_on(async {
            let mut phase = OuterPhase::MechanicalId {
                state: MechanicalIdState::Setup,
                data: MechanicalIdData::new(0, 0),
                safety: OuterSafetyLimits::new(0, 0),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();
            backend.current_limit_value = 2448;
            backend.encoder_count_value = 1000;
            let _ = backend.registers.insert(ABN_DECODER_PPR.addr(), 4000);

            let first = step_outer(
                &mut phase,
                &mut buf,
                CommissionProfile::Balanced,
                OperatingMode::Nominal,
                0,
                0,
                false,
                0,
                0,
                0,
                0,
                0,
                &mut backend,
            )
            .await;
            assert!(matches!(
                first,
                StepResult::PhaseChanged(PhaseId::MechanicalId)
            ));

            // Reproduce the field failure: the first sample after the position->torque
            // mode switch jumps by 25 counts/ms before the intended torque step starts.
            backend.encoder_count_value = 1025;

            let second = step_outer(
                &mut phase,
                &mut buf,
                CommissionProfile::Balanced,
                OperatingMode::Nominal,
                0,
                0,
                false,
                0,
                0,
                0,
                0,
                0,
                &mut backend,
            )
            .await;

            assert!(
                !matches!(
                    second,
                    StepResult::OuterFailed(CommissionError::SafetyEnvelopeViolation)
                ),
                "mechanical-id should not abort on the immediate post-switch spike"
            );
            assert!(
                !matches!(
                    phase,
                    OuterPhase::Failed(CommissionError::SafetyEnvelopeViolation)
                ),
                "phase should survive the immediate post-switch spike"
            );
        });
    }

    #[test]
    fn mechanical_id_waits_for_post_switch_settle_before_arming_safety() {
        pollster::block_on(async {
            let mut phase = OuterPhase::MechanicalId {
                state: MechanicalIdState::Setup,
                data: MechanicalIdData::new(0, 0),
                safety: OuterSafetyLimits::new(0, 0),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();
            backend.current_limit_value = 2448;
            backend.encoder_count_value = 1000;
            let _ = backend.registers.insert(ABN_DECODER_PPR.addr(), 4000);

            let first = step_outer(
                &mut phase,
                &mut buf,
                CommissionProfile::Balanced,
                OperatingMode::Nominal,
                0,
                0,
                false,
                0,
                0,
                0,
                0,
                0,
                &mut backend,
            )
            .await;
            assert!(matches!(
                first,
                StepResult::PhaseChanged(PhaseId::MechanicalId)
            ));

            backend.elapsed_us_value = 10_000;
            backend.encoder_count_value = 1025;
            let second = step_outer(
                &mut phase,
                &mut buf,
                CommissionProfile::Balanced,
                OperatingMode::Nominal,
                0,
                0,
                false,
                0,
                0,
                0,
                0,
                0,
                &mut backend,
            )
            .await;
            assert!(matches!(second, StepResult::Continue));
            match &phase {
                OuterPhase::MechanicalId { state, safety, .. } => {
                    assert!(matches!(state, MechanicalIdState::PostSwitchSettle { .. }));
                    assert_eq!(
                        safety.torque_cap, 0,
                        "safety should remain disarmed while drifting"
                    );
                }
                _ => panic!("expected MechanicalId phase"),
            }

            for (us, pos) in [(11_000, 1026), (12_000, 1027), (13_000, 1028)] {
                backend.elapsed_us_value = us;
                backend.encoder_count_value = pos;
                let result = step_outer(
                    &mut phase,
                    &mut buf,
                    CommissionProfile::Balanced,
                    OperatingMode::Nominal,
                    0,
                    0,
                    false,
                    0,
                    0,
                    0,
                    0,
                    0,
                    &mut backend,
                )
                .await;
                assert!(matches!(result, StepResult::Continue));
            }

            match &phase {
                OuterPhase::MechanicalId {
                    state,
                    safety,
                    data,
                } => {
                    assert!(matches!(state, MechanicalIdState::ForwardCapture { .. }));
                    assert_eq!(safety.torque_cap, 2448);
                    assert_eq!(safety.prev_encoder_count, 1028);
                    assert_eq!(data.start_position, 1027);
                }
                _ => panic!("expected MechanicalId phase"),
            }

            assert!(
                backend.spi_log.iter().any(|op| matches!(
                    op,
                    SpiOp::Write(addr, val)
                        if *addr == PID_TORQUE_FLUX_TARGET.addr() && (*val >> 16) != 0
                )),
                "settle completion should apply the first torque step immediately"
            );
        });
    }

    #[test]
    fn mechanical_id_fails_if_post_switch_motion_never_settles() {
        pollster::block_on(async {
            let mut phase = OuterPhase::MechanicalId {
                state: MechanicalIdState::Setup,
                data: MechanicalIdData::new(0, 0),
                safety: OuterSafetyLimits::new(0, 0),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();
            backend.current_limit_value = 2448;
            backend.encoder_count_value = 1000;
            let _ = backend.registers.insert(ABN_DECODER_PPR.addr(), 4000);

            let first = step_outer(
                &mut phase,
                &mut buf,
                CommissionProfile::Balanced,
                OperatingMode::Nominal,
                0,
                0,
                false,
                0,
                0,
                0,
                0,
                0,
                &mut backend,
            )
            .await;
            assert!(matches!(
                first,
                StepResult::PhaseChanged(PhaseId::MechanicalId)
            ));

            for (us, pos) in [
                (10_000, 1050),
                (20_000, 1100),
                (30_000, 1150),
                (40_000, 1200),
            ] {
                backend.elapsed_us_value = us;
                backend.encoder_count_value = pos;
                let _ = step_outer(
                    &mut phase,
                    &mut buf,
                    CommissionProfile::Balanced,
                    OperatingMode::Nominal,
                    0,
                    0,
                    false,
                    0,
                    0,
                    0,
                    0,
                    0,
                    &mut backend,
                )
                .await;
            }

            backend.elapsed_us_value = 150_000;
            backend.encoder_count_value = 1250;
            let result = step_outer(
                &mut phase,
                &mut buf,
                CommissionProfile::Balanced,
                OperatingMode::Nominal,
                0,
                0,
                false,
                0,
                0,
                0,
                0,
                0,
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::OuterFailed(CommissionError::SafetyEnvelopeViolation)
            ));
            assert!(matches!(
                phase,
                OuterPhase::Failed(CommissionError::SafetyEnvelopeViolation)
            ));
        });
    }

    #[test]
    fn low_confidence_slope_falls_back_instead_of_retrying_more_torque() {
        pollster::block_on(async {
            use crate::commissioning::outer::slope_estimator::SlopeEstimate;
            let low_conf = SlopeEstimate::default(); // fit_sample_count = 0 → low-conf
            let no_response =
                crate::commissioning::outer::response::MechanicalResponseEstimate::default();
            let mut phase = OuterPhase::MechanicalId {
                state: MechanicalIdState::Finalize,
                data: MechanicalIdData {
                    start_position: 0,
                    attempt: 0,
                    torque_step: 367,
                    fwd_slope: low_conf,
                    fwd_response: no_response,
                    forward_peak_velocity: 28,
                    forward_delay_us: 1000,
                    forward_travel: 3830,
                    rev_slope: low_conf,
                    rev_response: no_response,
                    reverse_peak_velocity: 29,
                    reverse_delay_us: 1000,
                    reverse_travel: 3888,
                    sample_timestamps_us: [0; crate::autotune::TRANSIENT_LEN],
                    sample_timestamp_len: 0,
                },
                safety: OuterSafetyLimits::new(0, 2448),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();

            let result = mechanical::step_mechanical_id(&mut phase, &mut buf, &mut backend).await;

            assert!(matches!(
                result,
                StepResult::PhaseChanged(PhaseId::VelocityTune)
            ));
            assert!(
                matches!(phase, OuterPhase::VelocityTune { .. }),
                "low-confidence should fall back to VelocityTune (response ramp), not retry"
            );
        });
    }

    #[test]
    fn full_response_model_takes_precedence_over_low_confidence_slope() {
        pollster::block_on(async {
            use crate::commissioning::outer::response::MechanicalResponseEstimate;
            use crate::commissioning::outer::slope_estimator::SlopeEstimate;
            let low_conf = SlopeEstimate::default();
            let fwd_response = MechanicalResponseEstimate {
                j_eff: 100_000,
                b_eff: 20_000,
                tau_mech_ms: 5,
                theta_mech_us: 10_000,
                steady_velocity_cpms: 18,
                travel: 3000,
                residual_mcpms: 0,
                fit_sample_count: 4,
                sign_match: true,
            };
            let rev_response = MechanicalResponseEstimate {
                steady_velocity_cpms: -18,
                ..fwd_response
            };
            let mut phase = OuterPhase::MechanicalId {
                state: MechanicalIdState::Finalize,
                data: MechanicalIdData {
                    start_position: 0,
                    attempt: 0,
                    torque_step: 122,
                    fwd_slope: low_conf,
                    fwd_response,
                    forward_peak_velocity: 18,
                    forward_delay_us: 10_000,
                    forward_travel: 3000,
                    rev_slope: low_conf,
                    rev_response,
                    reverse_peak_velocity: 18,
                    reverse_delay_us: 10_000,
                    reverse_travel: 3000,
                    sample_timestamps_us: [0; crate::autotune::TRANSIENT_LEN],
                    sample_timestamp_len: 0,
                },
                safety: OuterSafetyLimits::new(0, 2448),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();

            let result = mechanical::step_mechanical_id(&mut phase, &mut buf, &mut backend).await;

            assert!(matches!(
                result,
                StepResult::PhaseChanged(PhaseId::VelocityTune)
            ));
            match phase {
                OuterPhase::VelocityTune {
                    model, diagnostics, ..
                } => {
                    assert_eq!(diagnostics.warning_code, super::OUTER_WARNING_NONE);
                    assert_eq!(model.j_eff, 100_000);
                    assert_eq!(model.b_eff, 20_000);
                    assert_eq!(model.tau_mech_ms, 5);
                }
                other => panic!("expected VelocityTune phase, got {:?}", other),
            }
        });
    }

    #[test]
    fn asymmetric_full_response_retries_with_more_torque() {
        pollster::block_on(async {
            use crate::commissioning::outer::response::MechanicalResponseEstimate;
            let fwd_response = MechanicalResponseEstimate {
                j_eff: 100_000,
                b_eff: 20_000,
                tau_mech_ms: 5,
                theta_mech_us: 10_000,
                steady_velocity_cpms: 18,
                travel: 3000,
                residual_mcpms: 0,
                fit_sample_count: 4,
                sign_match: true,
            };
            let rev_response = MechanicalResponseEstimate {
                j_eff: 200_000,
                steady_velocity_cpms: -18,
                ..fwd_response
            };
            let mut phase = OuterPhase::MechanicalId {
                state: MechanicalIdState::Finalize,
                data: MechanicalIdData {
                    start_position: 0,
                    attempt: 0,
                    torque_step: 367,
                    fwd_slope: Default::default(),
                    fwd_response,
                    forward_peak_velocity: 18,
                    forward_delay_us: 10_000,
                    forward_travel: 3000,
                    rev_slope: Default::default(),
                    rev_response,
                    reverse_peak_velocity: 18,
                    reverse_delay_us: 10_000,
                    reverse_travel: 3000,
                    sample_timestamps_us: [0; crate::autotune::TRANSIENT_LEN],
                    sample_timestamp_len: 0,
                },
                safety: OuterSafetyLimits::new(0, 2448),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();

            let result = mechanical::step_mechanical_id(&mut phase, &mut buf, &mut backend).await;

            assert!(matches!(result, StepResult::Continue));
            match phase {
                OuterPhase::MechanicalId { state, data, .. } => {
                    assert_eq!(data.attempt, 1);
                    assert!(matches!(state, MechanicalIdState::Setup));
                }
                other => panic!("expected MechanicalId retry, got {:?}", other),
            }
        });
    }

    #[test]
    fn high_confidence_asymmetry_still_retries_with_more_torque() {
        pollster::block_on(async {
            use crate::commissioning::outer::slope_estimator::SlopeEstimate;
            // Both slopes high-confidence, asymmetric by >30%.
            let high_conf_fwd = SlopeEstimate {
                accel: 20,
                residual_counts: 1,
                velocity_rise_mcpms: 20_000,
                fit_sample_count: 6,
                sign_match: true,
            };
            let high_conf_rev = SlopeEstimate {
                accel: -50,
                residual_counts: 1,
                velocity_rise_mcpms: -50_000,
                fit_sample_count: 6,
                sign_match: true,
            };
            let no_response =
                crate::commissioning::outer::response::MechanicalResponseEstimate::default();
            let mut phase = OuterPhase::MechanicalId {
                state: MechanicalIdState::Finalize,
                data: MechanicalIdData {
                    start_position: 0,
                    attempt: 0,
                    torque_step: 367,
                    fwd_slope: high_conf_fwd,
                    fwd_response: no_response,
                    forward_peak_velocity: 20,
                    forward_delay_us: 1000,
                    forward_travel: 3000,
                    rev_slope: high_conf_rev,
                    rev_response: no_response,
                    reverse_peak_velocity: 50,
                    reverse_delay_us: 1000,
                    reverse_travel: 5000,
                    sample_timestamps_us: [0; crate::autotune::TRANSIENT_LEN],
                    sample_timestamp_len: 0,
                },
                safety: OuterSafetyLimits::new(0, 2448),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();

            let _ = mechanical::step_mechanical_id(&mut phase, &mut buf, &mut backend).await;

            // Expect retry: attempt increments, state returns to Setup.
            match &phase {
                OuterPhase::MechanicalId { state, data, .. } => {
                    assert_eq!(data.attempt, 1, "expected attempt increment on retry");
                    assert!(
                        matches!(state, MechanicalIdState::Setup),
                        "expected return to Setup for retry, got {:?}",
                        state,
                    );
                }
                other => panic!("expected MechanicalId phase after retry, got {:?}", other),
            }
        });
    }

    #[test]
    fn high_confidence_zero_accel_still_retries() {
        pollster::block_on(async {
            use crate::commissioning::outer::slope_estimator::SlopeEstimate;
            // Forward slope "high-confidence" but accel is zero: a
            // high-confidence fit could still produce 0 if the rotor
            // genuinely didn't move during the fit window. The zero-accel
            // retry path is gated on confidence, not on magnitude, so
            // this retries.
            let zero_accel = SlopeEstimate {
                accel: 0,
                residual_counts: 1,
                velocity_rise_mcpms: 10_000,
                fit_sample_count: 6,
                sign_match: true,
            };
            let no_response =
                crate::commissioning::outer::response::MechanicalResponseEstimate::default();
            let mut phase = OuterPhase::MechanicalId {
                state: MechanicalIdState::Finalize,
                data: MechanicalIdData {
                    start_position: 0,
                    attempt: 0,
                    torque_step: 367,
                    fwd_slope: zero_accel,
                    fwd_response: no_response,
                    forward_peak_velocity: 10,
                    forward_delay_us: 1000,
                    forward_travel: 100,
                    rev_slope: zero_accel,
                    rev_response: no_response,
                    reverse_peak_velocity: 10,
                    reverse_delay_us: 1000,
                    reverse_travel: 100,
                    sample_timestamps_us: [0; crate::autotune::TRANSIENT_LEN],
                    sample_timestamp_len: 0,
                },
                safety: OuterSafetyLimits::new(0, 2448),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();

            let _ = mechanical::step_mechanical_id(&mut phase, &mut buf, &mut backend).await;

            match &phase {
                OuterPhase::MechanicalId { state, data, .. } => {
                    assert_eq!(data.attempt, 1, "high-confidence zero-accel should retry");
                    assert!(matches!(state, MechanicalIdState::Setup));
                }
                other => panic!("expected retry, got {:?}", other),
            }
        });
    }

    #[test]
    fn high_signal_asymmetry_still_retries_with_more_torque() {
        pollster::block_on(async {
            use crate::commissioning::outer::slope_estimator::SlopeEstimate;
            // Both slopes high-confidence, asymmetric by >30%.
            let high_conf_fwd = SlopeEstimate {
                accel: 20,
                residual_counts: 1,
                velocity_rise_mcpms: 20_000,
                fit_sample_count: 6,
                sign_match: true,
            };
            let high_conf_rev = SlopeEstimate {
                accel: -50,
                residual_counts: 1,
                velocity_rise_mcpms: -50_000,
                fit_sample_count: 6,
                sign_match: true,
            };
            let no_response =
                crate::commissioning::outer::response::MechanicalResponseEstimate::default();
            let mut phase = OuterPhase::MechanicalId {
                state: MechanicalIdState::Finalize,
                data: MechanicalIdData {
                    start_position: 0,
                    attempt: 0,
                    torque_step: 367,
                    fwd_slope: high_conf_fwd,
                    fwd_response: no_response,
                    forward_peak_velocity: 28,
                    forward_delay_us: 1000,
                    forward_travel: 3830,
                    rev_slope: high_conf_rev,
                    rev_response: no_response,
                    reverse_peak_velocity: 29,
                    reverse_delay_us: 1000,
                    reverse_travel: 3888,
                    sample_timestamps_us: [0; crate::autotune::TRANSIENT_LEN],
                    sample_timestamp_len: 0,
                },
                safety: OuterSafetyLimits::new(0, 2448),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();

            let _ = mechanical::step_mechanical_id(&mut phase, &mut buf, &mut backend).await;

            match &phase {
                OuterPhase::MechanicalId { state, data, .. } => {
                    assert_eq!(data.attempt, 1, "expected attempt increment on retry");
                    assert!(
                        matches!(state, MechanicalIdState::Setup),
                        "expected return to Setup for retry, got {:?}",
                        state,
                    );
                }
                other => panic!("expected MechanicalId phase after retry, got {:?}", other),
            }
        });
    }

    #[test]
    fn response_ramp_fail_does_not_advance_to_position_tune() {
        pollster::block_on(async {
            let mut phase = OuterPhase::VelocityValidate {
                state: VelocityValidateState::Classify,
                data: VelocityValidateData {
                    gains: VelocityGains {
                        velocity_p: 1152,
                        velocity_i: 0,
                        velocity_limit: 16,
                        effective_bandwidth_hz: 0,
                        lambda_vel_ms: 0,
                    },
                    accepted_gains: VelocityGains {
                        velocity_p: 1152,
                        velocity_i: 0,
                        velocity_limit: 16,
                        effective_bandwidth_hz: 0,
                        lambda_vel_ms: 0,
                    },
                    has_accepted_gains: false,
                    response_lower_p: 0,
                    response_upper_p: 0,
                    model: MechanicalModel {
                        j_eff: 1,
                        b_eff: 1,
                        tau_mech_ms: 50,
                        theta_mech_us: 0,
                        t_f: 0,
                    },
                    strategy: VelocityTuneStrategy::ResponseRamp,
                    attempt: 0,
                    step_target: 4,
                    forward_metrics: Some(VelocityValidation {
                        rise_time_ms: 20,
                        overshoot_pct: 5250,
                        peak_velocity: 25,
                        settling_time_ms: 256,
                        zero_speed_chatter: false,
                        saturation_samples: 0,
                        symmetry_pct: 0,
                    }),
                    reverse_metrics: Some(VelocityValidation {
                        rise_time_ms: 0,
                        overshoot_pct: 5000,
                        peak_velocity: 24,
                        settling_time_ms: 256,
                        zero_speed_chatter: false,
                        saturation_samples: 0,
                        symmetry_pct: 0,
                    }),
                    max_chatter: 24,
                    peak_velocity: 18,
                    prev_encoder_for_velocity: 0,
                    saturation_count: 0,
                    torque_limit: 2448,
                    operational_velocity_limit: 500_000,
                    diagnostics: OuterDiagnostics {
                        warning_code: 1,
                        mech_torque_step: 367,
                        mech_fwd_accel: 0,
                        mech_rev_accel: 0,
                        mech_fwd_peak_velocity: 18,
                        mech_rev_peak_velocity: 18,
                        mech_fwd_travel: 3907,
                        mech_rev_travel: 3886,
                        synth_velocity_p: 0,
                        synth_velocity_i: 0,
                        synth_position_p: 0,
                        synth_position_i: 0,
                    },
                },
                safety: OuterSafetyLimits::new(0, 2448),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();

            let result = velocity::step_velocity_validate(
                &mut phase,
                &mut buf,
                CommissionProfile::Balanced,
                OperatingMode::Nominal,
                0,
                0,
                false,
                0,
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::OuterFailed(CommissionError::VelocityValidationFailed)
            ));
            assert!(matches!(
                phase,
                OuterPhase::Failed(CommissionError::VelocityValidationFailed)
            ));
        });
    }
}
