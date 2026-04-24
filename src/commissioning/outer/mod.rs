//! Stage 2 (mechanical / velocity / position / filter) commissioning.
//!
//! `OuterPhase` is the cooperative state-machine type. `step_outer`
//! dispatches one tick to the per-phase submodule, prefixed by a
//! safety-envelope check that aborts on velocity- or position-window
//! violations. The phase variants and their sub-state types live here
//! so each per-phase submodule can pattern-match and construct them.

pub mod commit;
pub mod filter;
pub mod mechanical;
pub mod position;
pub mod response;
pub mod velocity;

use crate::autotune::{
    CommissionError, CommissionProfile, FilterConfig, MechanicalModel, OperatingMode,
    PositionGains, TransientBuffer, VelocityGains, VelocityValidation,
};
use crate::commissioning::types::{CommissioningBackend, OuterResult, StepResult};

// ---------------------------------------------------------------------------
// Outer safety-envelope constants
// ---------------------------------------------------------------------------

/// MechanicalId safety velocity cap in encoder counts per millisecond.
/// 6 rev/s at 4000 counts/rev = 24000 counts/s = 24 counts/ms.
/// Kept separate from the conservative runtime fallback velocity limit so
/// outer safety tuning does not silently retune response-ramp behavior.
pub(crate) const MECHANICAL_ID_VELOCITY_CAP_COUNTS_PER_MS: i32 = 24;

/// Conservative runtime fallback velocity limit in encoder counts/ms.
/// Used by response-ramp fallback gains when autotuned outer gains are not
/// accepted. Keep this lower than the MechanicalId safety cap.
pub(crate) const RESPONSE_FALLBACK_VELOCITY_LIMIT_COUNTS_PER_MS: i32 = 16;

/// Default position window: ± encoder counts from start position.
/// 2 revolutions at 4096 counts/rev = 8192 counts.
const POSITION_WINDOW_COUNTS: i32 = 8192;

/// Conservative velocity-loop runtime fallback used when autotuned outer
/// gains exceed the accepted safety envelope. These are the same defaults
/// used by closed-loop entry when no outer gains are configured.
pub(crate) const OUTER_FALLBACK_VELOCITY_P: u16 = 1152;
pub(crate) const OUTER_FALLBACK_VELOCITY_I: u16 = 0;

/// Conservative position-loop runtime fallback used with
/// `OUTER_FALLBACK_VELOCITY_*`.
pub(crate) const OUTER_FALLBACK_POSITION_P: u16 = 640;
pub(crate) const OUTER_FALLBACK_POSITION_I: u16 = 0;

/// Maximum accepted autotuned velocity P gain.
///
/// The current mechanical-ID scaling can saturate the velocity P register
/// on real hardware. Accepting those gains caused immediate
/// velocity/position limit windup during normal motion, so keep runtime
/// gains near the known-good default envelope until the mechanical model
/// scaling is reworked.
pub(crate) const OUTER_MAX_VELOCITY_P: u16 = OUTER_FALLBACK_VELOCITY_P * 4;
pub(crate) const OUTER_MAX_VELOCITY_I: u16 = 512;
pub(crate) const OUTER_MAX_POSITION_P: u16 = OUTER_FALLBACK_POSITION_P * 4;
pub(crate) const OUTER_MAX_POSITION_I: u16 = 128;

pub(crate) const OUTER_WARNING_NONE: u8 = 0;
pub(crate) const OUTER_WARNING_UNSAFE_SYNTH_GAINS: u8 = 2;

/// Starting position P gain: 128 = 0.5 in Q8.8 (conservative).
pub(crate) const POS_TUNE_INITIAL_P: u16 = 128;

/// Motor-local position step for position-loop tuning, in ABN encoder
/// counts. The TMC position target is written in electrical-position
/// units, while response capture uses ABN counts. Keep the commanded move
/// small enough for installed mechanics but larger than one-count
/// quantization noise.
pub(crate) const POS_TUNE_STEP_COUNTS: u32 = 8;

// ---------------------------------------------------------------------------
// Outer sub-state types
// ---------------------------------------------------------------------------

/// Mechanical identification sub-state machine (Phase 8).
#[derive(Debug, defmt::Format)]
pub(crate) enum MechanicalIdState {
    /// Record starting position, configure torque mode.
    Setup,
    /// Apply torque step and begin forward capture.
    ForwardStart,
    /// Sample encoder position into transient buffer at 1 kHz.
    ForwardCapture { next_sample_us: u64 },
    /// Return to zero torque and wait for settling.
    ForwardSettle { deadline_us: u64 },
    /// Apply reverse torque step and begin reverse capture.
    ReverseStart,
    /// Sample encoder position for reverse direction.
    ReverseCapture { next_sample_us: u64 },
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
    /// Forward capture: per-sample velocity (encoder counts/ms).
    /// Stored as [initial_acceleration, peak_velocity] pairs.
    pub(crate) forward_initial_accel: i32,
    pub(crate) forward_peak_velocity: i32,
    pub(crate) forward_delay_samples: u16,
    pub(crate) forward_travel: u32,
    /// Reverse capture: same metrics.
    pub(crate) reverse_initial_accel: i32,
    pub(crate) reverse_peak_velocity: i32,
    pub(crate) reverse_delay_samples: u16,
    pub(crate) reverse_travel: u32,
}

impl MechanicalIdData {
    pub(crate) fn new(start_position: i32, torque_step: u16) -> Self {
        Self {
            start_position,
            attempt: 0,
            torque_step,
            forward_initial_accel: 0,
            forward_peak_velocity: 0,
            forward_delay_samples: 0,
            forward_travel: 0,
            reverse_initial_accel: 0,
            reverse_peak_velocity: 0,
            reverse_delay_samples: 0,
            reverse_travel: 0,
        }
    }

    pub(crate) fn reset_captures(&mut self) {
        self.forward_initial_accel = 0;
        self.forward_peak_velocity = 0;
        self.forward_delay_samples = 0;
        self.forward_travel = 0;
        self.reverse_initial_accel = 0;
        self.reverse_peak_velocity = 0;
        self.reverse_delay_samples = 0;
        self.reverse_travel = 0;
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
            mech_fwd_accel: data.forward_initial_accel.unsigned_abs(),
            mech_rev_accel: data.reverse_initial_accel.unsigned_abs(),
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
    ForwardCapture { next_sample_us: u64 },
    /// Return to zero velocity and wait for settling.
    ForwardSettle { deadline_us: u64 },
    /// Apply reverse velocity step.
    ReverseStep,
    /// Capture reverse step response.
    ReverseCapture { next_sample_us: u64 },
    /// Return to zero velocity and wait for settling.
    ReverseSettle { deadline_us: u64 },
    /// Monitor zero-speed chatter for 200ms.
    ChatterCheck { next_sample_us: u64, remaining: u16 },
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
}

/// Outer commissioning phase state machine.
#[derive(Debug, defmt::Format)]
#[allow(dead_code)] // Later phases (9-13) are placeholders for future plans.
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
) -> bool {
    // Safety limits are initialized by MechanicalId::Setup after it reads
    // the real encoder baseline. Skip the pre-setup placeholder values.
    if safety.torque_cap == 0 {
        return false;
    }

    let encoder_raw = backend.read_encoder_count().await;
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
        return true;
    }

    // Check position window against unwrapped accumulator.
    let abs_position = safety.unwrapped_position.unsigned_abs() as i32;
    if abs_position > safety.position_window {
        defmt::warn!(
            "outer safety: position {} exceeds window {}",
            abs_position,
            safety.position_window
        );
        return true;
    }

    false
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
pub(crate) async fn step_outer<B: CommissioningBackend>(
    phase: &mut OuterPhase,
    buf: &mut TransientBuffer,
    profile: CommissionProfile,
    mode: OperatingMode,
    inner_lambda_us: u32,
    theta_e_us: u32,
    current_ringing: bool,
    current_bandwidth_hz: u32,
    backend: &mut B,
) -> StepResult {
    if let Some(safety) = outer_safety_mut(phase)
        && check_outer_safety(safety, backend).await
    {
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

    match phase {
        OuterPhase::MechanicalId { .. } => {
            mechanical::step_mechanical_id(phase, buf, backend).await
        }
        OuterPhase::VelocityTune { .. } => {
            velocity::step_velocity_tune(phase, profile, mode, inner_lambda_us, theta_e_us, backend)
                .await
        }
        OuterPhase::VelocityValidate { .. } => {
            velocity::step_velocity_validate(
                phase,
                buf,
                profile,
                mode,
                inner_lambda_us,
                theta_e_us,
                current_ringing,
                current_bandwidth_hz,
                backend,
            )
            .await
        }
        OuterPhase::PositionTune { .. } => {
            position::step_position_tune(phase, buf, mode, backend).await
        }
        OuterPhase::FilterTune { .. } => filter::step_filter_tune(phase, backend).await,
        OuterPhase::Commit { .. } => commit::step_commit(phase),
        OuterPhase::Done(result) => StepResult::OuterDone(result.clone()),
        OuterPhase::Failed(err) => StepResult::OuterFailed(*err),
    }
}

#[cfg(test)]
mod tests {
    use super::{MECHANICAL_ID_VELOCITY_CAP_COUNTS_PER_MS, OuterSafetyLimits, check_outer_safety};
    use crate::commission_dispatch::test_support::MockBackend;

    #[test]
    fn outer_safety_uses_mechanical_id_velocity_cap() {
        let safety = OuterSafetyLimits::new(0, 100);
        assert_eq!(
            safety.velocity_cap,
            MECHANICAL_ID_VELOCITY_CAP_COUNTS_PER_MS
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

            let violated = check_outer_safety(&mut safety, &mut backend).await;

            assert!(
                !violated,
                "time-normalized velocity should stay below the cap"
            );
            assert_eq!(safety.prev_encoder_count, 2525);
            assert_eq!(safety.last_sample_us, 124_909);
            assert_eq!(safety.unwrapped_position, 197);
        });
    }
}
