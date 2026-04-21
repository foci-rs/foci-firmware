//! Commissioning state machine and hardware backend.
//!
//! The [`CommissioningEngine`] is a cooperative state machine invoked
//! every 1ms by the `tmc_control` task. Each `step()` call advances
//! the current phase by one chunk of work (max 1ms runtime).
//!
//! Phases 1-4 (diagnostics) are implemented here. Phases 5-7
//! (electrical ID, current tune, validation) are filled by the
//! electrical-ID and current-tune plans.

use crate::autotune::{
    CommissionError, CommissionProfile, CommissionStatus, CurrentGains, CurrentValidation,
    ElectricalModel, InnerSequence, MechanicalModel, OperatingMode, PhaseId, TransientBuffer,
    ValidationResult, VelocityGains, VelocityValidation,
};

pub(crate) use crate::commissioning::inner::InnerPhase;
pub use crate::commissioning::types::{
    AdcCalResult, CommissionResult, CommissioningBackend, InnerResult, OuterResult, StepResult,
};

// ---------------------------------------------------------------------------
// Diagnostic sub-state constants
// ---------------------------------------------------------------------------

use crate::autotune::CaptureDirection;

/// Current-loop validation sub-state machine (Phase 7).
#[derive(Debug, defmt::Format)]
pub(crate) enum CurrentValidateState {
    /// Write PI gains to TMC4671 registers.
    WriteGains,
    /// Switch to torque mode and apply current step.
    StepSetup { direction: CaptureDirection },
    /// Wait for transient to develop before capturing.
    StepSettle {
        direction: CaptureDirection,
        deadline_us: u64,
    },
    /// Capture step response into TransientBuffer at 160us intervals.
    StepCapture {
        direction: CaptureDirection,
        next_sample_us: u64,
    },
    /// Extract metrics and classify response.
    Classify,
    /// Restore safe state before finalizing.
    SafeState { deadline_us: u64 },
    /// Emit final InnerResult.
    Finalize,
}

/// Accumulated data for current-loop validation.
#[derive(Debug, Clone, defmt::Format)]
pub(crate) struct CurrentValidateData {
    /// The electrical model from Phase 5.
    pub(crate) model: ElectricalModel,
    /// Currently applied gains.
    pub(crate) gains: CurrentGains,
    /// Validation attempt (0 = first, 1 = retry).
    pub(crate) attempt: u8,
    /// Current step amplitude (absolute, PIDOUT units).
    pub(crate) step_amplitude: u16,
    /// Forward capture metrics (filled after forward capture).
    pub(crate) forward_metrics: Option<CurrentValidation>,
    /// Reverse capture metrics (filled after reverse capture).
    pub(crate) reverse_metrics: Option<CurrentValidation>,
    /// Final commissioning status to report.
    pub(crate) final_status: CommissionStatus,
    /// PID-output saturation counter: samples where the current PID's
    /// voltage output is at or near `PIDOUT_UQ_UD_LIMITS`.
    pub(crate) saturation_count: u16,
    /// Configured PIDOUT voltage limit for saturation detection.
    pub(crate) pidout_limit: u32,
}

// ---------------------------------------------------------------------------
// Diagnostic sub-states
// ---------------------------------------------------------------------------

/// ADC calibration sub-state.
#[derive(Debug, defmt::Format)]
pub(crate) enum AdcCalState {
    /// Configure registers and enable outputs.
    Setup,
    /// Wait for ADC outputs to settle.
    Settle { deadline_us: u64 },
    /// Collect ADC samples one per step() call.
    Sample {
        sum_i0: i32,
        sum_i1: i32,
        count: u16,
    },
    /// Validate and program offsets.
    Finalize { sum_i0: i32, sum_i1: i32 },
}

/// Coil test sub-state (used for both coil A and coil B).
#[derive(Debug, defmt::Format)]
pub(crate) enum CoilTestState {
    /// Configure open-loop voltage mode for this phase.
    Setup { phase: u8 },
    /// Wait for current to settle.
    Settle { phase: u8, deadline_us: u64 },
    /// Read ADC and record result.
    Read { phase: u8 },
    /// Disable and cool down before next test.
    Cooldown { phase: u8, deadline_us: u64 },
}

/// Coil check phase state (wraps both coil tests).
#[derive(Debug, defmt::Format)]
pub(crate) enum CoilCheckState {
    /// Testing coil A (phase 0).
    CoilA(CoilTestState),
    /// Testing coil B (phase 1).
    CoilB(CoilTestState),
}

/// Accumulated coil test observations carried between sub-phases.
#[derive(Debug, Clone, Copy, defmt::Format)]
pub(crate) struct CoilObservation {
    /// Current on the expected ADC channel.
    pub(crate) expected: i16,
    /// Current on the other ADC channel.
    pub(crate) other: i16,
}

/// Encoder check sub-state.
#[derive(Debug, defmt::Format)]
pub(crate) enum EncoderCheckState {
    /// Set up current-regulated mode for direction sweep.
    Setup,
    /// Sweep PHI_E_EXT in steps, one per tick.
    Sweep {
        step: u16,
        deadline_us: u64,
        count_start: i32,
    },
    /// Read final count and determine direction.
    Finalize { count_start: i32 },
}

/// Encoder D-axis alignment sub-state.
#[derive(Debug, defmt::Format)]
pub(crate) enum EncoderAlignState {
    /// Configure external PHI_E=0 voltage drive.
    Setup {
        /// Encoder count captured before the direction sweep that ended at PHI_E=0.
        count_before: u32,
    },
    /// Wait for rotor to settle at the nearest D-axis pole.
    Settle {
        count_before: u32,
        deadline_us: u64,
        min_movement: u32,
    },
    /// First post-settle encoder read.
    StabilityCheck1 {
        count_before: u32,
        min_movement: u32,
    },
    /// Second encoder read after a short delay for stability validation.
    StabilityCheck2 {
        count_before: u32,
        count1: u32,
        deadline_us: u64,
        min_movement: u32,
    },
    /// Zero encoder count and offset while the rotor is still held.
    Commit { old_count: u32 },
    /// Stop open-loop drive after PHI_E has been locked to ABN.
    StopDrive { old_count: u32 },
}

/// Accumulated per-coil measurements for electrical identification.
#[derive(Debug, Clone, defmt::Format)]
pub(crate) struct ElectricalIdData {
    /// Excitation voltage used for all measurements (PIDOUT units).
    pub(crate) voltage: u32,
    /// Per-coil accumulated signed current for resistance (sum of i16 readings).
    pub(crate) resistance_current_sum: [i32; 2],
    /// Per-coil number of resistance samples collected.
    pub(crate) resistance_sample_count: [u16; 2],
    /// Per-coil accumulated |delta_I| for di/dt inductance measurement.
    pub(crate) inductance_sum_delta: [u64; 2],
    /// Per-coil number of valid di/dt cycles.
    pub(crate) inductance_count: [u32; 2],
    /// Per-coil elapsed microseconds during di/dt measurement.
    pub(crate) inductance_elapsed_us: [u32; 2],
}

impl ElectricalIdData {
    pub(crate) fn new(voltage: u32) -> Self {
        Self {
            voltage,
            resistance_current_sum: [0; 2],
            resistance_sample_count: [0; 2],
            inductance_sum_delta: [0; 2],
            inductance_count: [0; 2],
            inductance_elapsed_us: [0; 2],
        }
    }
}

/// Electrical identification sub-state machine (Phase 5).
#[derive(Debug, defmt::Format)]
pub(crate) enum ElectricalIdState {
    // --- Resistance measurement (per coil) ---
    /// Configure open-loop voltage mode for resistance measurement.
    ResSetup { coil: u8 },
    /// Wait for steady-state current.
    ResSettle { coil: u8, deadline_us: u64 },
    /// Sample PID_TORQUE_FLUX_ACTUAL at 160us intervals (multiple per step).
    ResSample { coil: u8, next_sample_us: u64 },
    /// Disable motor and cool down before next measurement.
    ResDisable { coil: u8, deadline_us: u64 },

    // --- Inductance measurement (per coil, square-wave di/dt) ---
    /// Configure for di/dt toggling.
    IndSetup { coil: u8 },
    /// Brief settle before toggling.
    IndSettle { coil: u8, deadline_us: u64 },
    /// Toggle voltage and accumulate |delta_I| (chunked per step).
    IndToggle {
        coil: u8,
        remaining_cycles: u32,
        prev_current: i16,
        start_us: u64,
    },
    /// Disable motor and cool down.
    IndDisable { coil: u8, deadline_us: u64 },

    // --- Delay identification ---
    /// Configure for delay step response capture.
    DelaySetup,
    /// Brief settle at zero voltage before step.
    DelaySettle { deadline_us: u64 },
    /// Apply voltage step and start capturing.
    DelayStep,
    /// Capture 256 samples into TransientBuffer at 160us intervals.
    DelaySample { baseline: i16, next_sample_us: u64 },
    /// Disable and restore safe state.
    DelayDisable { deadline_us: u64 },

    // --- Final ---
    /// Assemble ElectricalModel and validate.
    Validate,
}

// ---------------------------------------------------------------------------
// Outer safety-envelope constants
// ---------------------------------------------------------------------------

/// Default velocity cap in encoder counts per millisecond.
/// 4 rev/s at 4000 counts/rev = 16000 counts/s = 16 counts/ms.
/// Raised from 8 (2 rev/s) because MechanicalId at 60% torque
/// on low-inertia motors exceeds 8 counts/ms.
///
const VELOCITY_CAP_COUNTS_PER_MS: i32 = 16;

/// Default position window: +/- encoder counts from start position.
/// 2 revolutions at 4096 counts/rev = 8192 counts.
const POSITION_WINDOW_COUNTS: i32 = 8192;

// ---------------------------------------------------------------------------
// Mechanical identification constants (Phase 8)
// ---------------------------------------------------------------------------

/// Torque step attempts as fractions of run_current.
///
/// Start at the spec midpoint, then retry with stronger bounded torque if the
/// encoder response is too quantized to identify a useful mechanical model.
const MECH_ID_TORQUE_STEP_PERCENT_0: u32 = 25;
const MECH_ID_TORQUE_STEP_PERCENT_1: u32 = 40;
const MECH_ID_TORQUE_STEP_PERCENT_2: u32 = 60;
const MECH_ID_MAX_ATTEMPTS: u8 = 3;

/// Sampling interval for encoder capture (microseconds).
/// 1 kHz = 1000us between samples.
const MECH_ID_SAMPLE_PERIOD_US: u64 = 1000;

/// Settling time after removing torque (microseconds).
/// Allow the motor to decelerate and stop before reverse capture.
const MECH_ID_SETTLE_US: u64 = 500_000;

/// Forward/reverse agreement threshold (30% = 30/100).
/// If |J_fwd - J_rev| / avg > 30%, mechanical ID fails.
const MECH_ID_SYMMETRY_THRESHOLD_PCT: u32 = 30;

/// Operational PID output voltage limit for outer commissioning.
///
/// Matches closed-loop entry so torque, velocity, and position tests are not
/// constrained by the conservative safe-state ceiling.
const OUTER_PIDOUT_LIMIT: u32 = 16000;

/// Torque mode with MODE_PID_TYPE set for the TMC4671 advanced PI structure.
///
/// Current-loop SIMC writes the advanced flux/torque PI registers, so
/// validation and mechanical ID must exercise that same controller path.
const CURRENT_TORQUE_MODE: u32 = 0x8000_0001;

/// Velocity mode with MODE_PID_TYPE set for the TMC4671 advanced PI structure.
///
/// The production closed-loop path uses advanced PID mode because the ADC
/// setup also configures the PWM-synchronized velocity meter expected by that
/// controller structure. Outer validation must exercise the same controller
/// path it is tuning.
const OUTER_VELOCITY_MODE: u32 = 0x8000_0002;

/// Position mode with MODE_PID_TYPE set, matching production closed-loop entry.
const OUTER_POSITION_MODE: u32 = 0x8000_0003;

/// Minimum number of acceleration samples used for inertia estimate.
/// First few samples before friction/damping dominates.
const MECH_ID_ACCEL_SAMPLES: usize = 8;

/// Minimum velocity delta to consider the motor has started moving.
/// In encoder counts per sample period.
const MECH_ID_VELOCITY_ONSET_THRESHOLD: i32 = 2;

/// Minimum mechanical-ID signal before model-based gain synthesis is trusted.
///
/// These thresholds reject the observed 1-count acceleration and
/// 2-count/ms peak velocity case that produced saturated gains.
const MECH_ID_MIN_ACCEL_COUNTS_PER_MS2: u32 = 2;
const MECH_ID_MIN_PEAK_VELOCITY_COUNTS_PER_MS: u32 = 4;
const MECH_ID_MIN_TRAVEL_COUNTS: u32 = 64;

// ---------------------------------------------------------------------------
// Velocity validation constants (Phase 10)
// ---------------------------------------------------------------------------

/// Velocity step as a fraction of velocity_limit (15% = 15/100).
/// Spec says 10-20%.
const VEL_VALIDATE_STEP_PERCENT: u32 = 15;

/// Sampling interval for velocity capture (microseconds).
/// 1 kHz = 1000us between samples.
const VEL_VALIDATE_SAMPLE_PERIOD_US: u64 = 1000;

/// Settling time after removing velocity target (microseconds).
const VEL_VALIDATE_SETTLE_US: u64 = 200_000;

/// Zero-speed chatter monitoring window (samples at 1 kHz).
/// Spec: 200ms = 200 samples.
const VEL_VALIDATE_CHATTER_WINDOW: u16 = 200;

/// Chatter threshold (encoder counts/tick). Spec: > 3.
const VEL_VALIDATE_CHATTER_THRESHOLD: u32 = 3;

/// Lambda multiplier for retry (150% = lambda * 3 / 2).
const VEL_RETRY_LAMBDA_MUL_NUM: u32 = 3;
const VEL_RETRY_LAMBDA_MUL_DEN: u32 = 2;

/// Motor-local response tuning uses small velocity targets so an installed
/// mechanism stays within the existing outer safety envelope.
const VEL_RESPONSE_STEP_TARGET: u32 = 4;
const VEL_RESPONSE_LIMIT: u32 = VELOCITY_CAP_COUNTS_PER_MS as u32;
const VEL_RESPONSE_P_MUL_NUM: u32 = 3;
const VEL_RESPONSE_P_MUL_DEN: u32 = 2;
const VEL_RESPONSE_VALIDATE_TAU_MS: u32 = 50;

// ---------------------------------------------------------------------------
// Position-loop tuning constants (Phase 11)
// ---------------------------------------------------------------------------

/// Starting position P gain: 128 = 0.5 in Q8.8 (conservative).
const POS_TUNE_INITIAL_P: u16 = 128;

/// P gain increase per iteration: 150% = P * 3 / 2.
const POS_TUNE_P_MUL_NUM: u32 = 3;
const POS_TUNE_P_MUL_DEN: u32 = 2;

/// Position I gain as fraction of P for Nominal/HighInertia modes.
/// 5% of P = P * 5 / 100.
const POS_TUNE_I_PERCENT: u32 = 5;

/// Motor-local position step for position-loop tuning, in ABN encoder counts.
///
/// The TMC position target is written in electrical-position units, while
/// response capture uses ABN counts. Keep the commanded move small enough for
/// installed mechanics but larger than one-count quantization noise.
const POS_TUNE_STEP_COUNTS: u32 = 8;

/// Conservative velocity-loop runtime fallback used when autotuned outer gains
/// exceed the accepted safety envelope. These are the same defaults used by
/// closed-loop entry when no outer gains are configured.
const OUTER_FALLBACK_VELOCITY_P: u16 = 1152;
const OUTER_FALLBACK_VELOCITY_I: u16 = 0;

/// Conservative position-loop runtime fallback used with
/// `OUTER_FALLBACK_VELOCITY_*`.
const OUTER_FALLBACK_POSITION_P: u16 = 640;
const OUTER_FALLBACK_POSITION_I: u16 = 0;

/// Maximum accepted autotuned velocity P gain.
///
/// The current mechanical-ID scaling can saturate the velocity P register on
/// real hardware. Accepting those gains caused immediate velocity/position
/// limit windup during normal motion, so keep runtime gains near the known-good
/// default envelope until the mechanical model scaling is reworked.
const OUTER_MAX_VELOCITY_P: u16 = OUTER_FALLBACK_VELOCITY_P * 4;
const OUTER_MAX_VELOCITY_I: u16 = 512;
const OUTER_MAX_POSITION_P: u16 = OUTER_FALLBACK_POSITION_P * 4;
const OUTER_MAX_POSITION_I: u16 = 128;

/// Maximum position P tested during ramp-up.
///
/// Keep Phase 11 inside the same envelope that the final commit guard accepts
/// so tuning never exercises gains that would be rejected for runtime use.
const POS_TUNE_MAX_P: u16 = OUTER_MAX_POSITION_P;

/// Response-tuned velocity is deliberately soft. Keep position P at the
/// fallback value so a few encoder counts of residual error do not command
/// large hold velocity and current-loop integral effort.
const POS_RESPONSE_TUNE_MAX_P: u16 = OUTER_FALLBACK_POSITION_P;

const OUTER_WARNING_NONE: u8 = 0;
const OUTER_WARNING_LOW_CONFIDENCE_MECH_ID: u8 = 1;
const OUTER_WARNING_UNSAFE_SYNTH_GAINS: u8 = 2;

/// Sampling interval for position capture (microseconds). 1 kHz.
const POS_TUNE_SAMPLE_PERIOD_US: u64 = 1000;

/// Settling time after position step (microseconds).
const POS_TUNE_SETTLE_US: u64 = 200_000;

// ---------------------------------------------------------------------------
// Mechanical identification sub-states
// ---------------------------------------------------------------------------

/// Mechanical identification sub-state machine (Phase 8).
#[derive(Debug, defmt::Format)]
enum MechanicalIdState {
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
struct MechanicalIdData {
    /// Starting encoder position recorded at setup.
    start_position: i32,
    /// Retry attempt index for adaptive torque steps.
    attempt: u8,
    /// Torque step amplitude (PIDOUT units).
    torque_step: u16,
    /// Forward capture: per-sample velocity (encoder counts/ms).
    /// Stored as [initial_acceleration, peak_velocity] pairs.
    forward_initial_accel: i32,
    forward_peak_velocity: i32,
    forward_delay_samples: u16,
    forward_travel: u32,
    /// Reverse capture: same metrics.
    reverse_initial_accel: i32,
    reverse_peak_velocity: i32,
    reverse_delay_samples: u16,
    reverse_travel: u32,
}

impl MechanicalIdData {
    fn new(start_position: i32, torque_step: u16) -> Self {
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

    fn reset_captures(&mut self) {
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
struct OuterDiagnostics {
    warning_code: u8,
    mech_torque_step: u16,
    mech_fwd_accel: u32,
    mech_rev_accel: u32,
    mech_fwd_peak_velocity: u32,
    mech_rev_peak_velocity: u32,
    mech_fwd_travel: u32,
    mech_rev_travel: u32,
    synth_velocity_p: u16,
    synth_velocity_i: u16,
    synth_position_p: u16,
    synth_position_i: u16,
}

impl OuterDiagnostics {
    fn from_mechanical_id(data: &MechanicalIdData) -> Self {
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

// ---------------------------------------------------------------------------
// Velocity validation sub-states
// ---------------------------------------------------------------------------

/// Velocity-loop validation sub-state machine (Phase 10).
#[derive(Debug, defmt::Format)]
enum VelocityValidateState {
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
enum VelocityTuneStrategy {
    /// Validate gains synthesized from the mechanical model.
    Model,
    /// Tune motor-local velocity P from bounded closed-loop response.
    ResponseRamp,
}

/// Accumulated data for velocity-loop validation.
#[derive(Debug, defmt::Format)]
struct VelocityValidateData {
    /// Synthesized velocity gains for the current attempt.
    gains: VelocityGains,
    /// Last response-ramp gain set that validated cleanly.
    accepted_gains: VelocityGains,
    /// Whether `accepted_gains` came from a passing response-ramp attempt.
    has_accepted_gains: bool,
    /// The mechanical model from Phase 8 (needed for retry re-synthesis).
    model: MechanicalModel,
    /// Velocity tuning strategy for this run.
    strategy: VelocityTuneStrategy,
    /// Current attempt (0 = first, 1 = retry).
    attempt: u8,
    /// Velocity step amplitude (encoder counts/ms or velocity target units).
    step_target: u32,
    /// Forward capture metrics.
    forward_metrics: Option<VelocityValidation>,
    /// Reverse capture metrics.
    reverse_metrics: Option<VelocityValidation>,
    /// Maximum chatter value observed during zero-speed check.
    max_chatter: u32,
    /// Peak velocity from mechanical ID (for velocity limit derivation).
    peak_velocity: u32,
    /// Previous encoder count for velocity derivation (position delta per tick).
    prev_encoder_for_velocity: u32,
    /// Saturation counter: samples where the velocity PID's torque output
    /// is near the current limit (indicating the controller is effort-limited).
    saturation_count: u16,
    /// Current limit for saturation detection (from `backend.current_limit()`).
    torque_limit: u32,
    /// Operational velocity limit to restore after validation.
    operational_velocity_limit: u32,
    /// Diagnostics carried into the final result.
    diagnostics: OuterDiagnostics,
}

// ---------------------------------------------------------------------------
// Position-loop tuning sub-states (Phase 11)
// ---------------------------------------------------------------------------

/// Position-loop tuning sub-state machine (Phase 11).
#[derive(Debug, defmt::Format)]
enum PositionTuneState {
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
struct PositionTuneData {
    /// Current candidate P gain.
    current_p: u16,
    /// Last known stable P gain (no oscillation).
    stable_p: u16,
    /// Position step target in ABN encoder counts.
    step_target_counts: u32,
    /// Position step target in TMC electrical-position units.
    step_target_tmc: u32,
    /// Starting ABN encoder count for response measurement.
    step_start_counts: i32,
    /// Starting TMC position for target writes.
    step_start_tmc: i32,
    /// Whether oscillation was detected early (at the first P value).
    oscillation_at_minimum: bool,
    /// Accepted velocity gains from Phase 10.
    velocity_gains: VelocityGains,
    /// Whether zero-speed chatter was detected during velocity validation.
    velocity_chatter: bool,
    /// Whether current ringing was detected (from inner validation).
    current_ringing: bool,
    /// Current-loop effective bandwidth Hz (for filter recommendation).
    current_bandwidth_hz: u32,
    /// Identified effective inertia from mechanical ID (internal units).
    j_eff: u32,
    /// Identified effective damping from mechanical ID (internal units).
    b_eff: u32,
    /// Diagnostics carried into the final result.
    diagnostics: OuterDiagnostics,
}

/// Outer safety limits, computed once at the start of outer commissioning.
#[derive(Debug, Clone, Copy, defmt::Format)]
struct OuterSafetyLimits {
    /// Maximum absolute velocity in encoder counts per millisecond.
    velocity_cap: i32,
    /// Maximum distance from start position in encoder counts.
    position_window: i32,
    /// Unwrapped signed position accumulated from wrap-aware deltas.
    unwrapped_position: i32,
    /// Torque cap: the configured run_current limit.
    torque_cap: u16,
    /// Previous raw encoder count for delta computation.
    prev_encoder_count: i32,
    /// Encoder counts per revolution (PPR * 4). Used to normalize
    /// wrap-around deltas when the ABN decoder counter rolls over.
    counts_per_rev: i32,
}

impl OuterSafetyLimits {
    fn new(start_position: i32, torque_cap: u16) -> Self {
        Self {
            velocity_cap: VELOCITY_CAP_COUNTS_PER_MS,
            position_window: POSITION_WINDOW_COUNTS,
            unwrapped_position: 0,
            torque_cap,
            prev_encoder_count: start_position,
            counts_per_rev: 0, // Set by MechanicalId::Setup after reading ABN_DECODER_PPR
        }
    }
}

/// Outer commissioning phase state machine.
#[derive(Debug, defmt::Format)]
#[allow(dead_code)] // Later phases (9-13) are placeholders for future plans.
enum OuterPhase {
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
        position_gains: crate::autotune::PositionGains,
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
        position_gains: crate::autotune::PositionGains,
        filter_config: crate::autotune::FilterConfig,
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

/// Parameters for outer (mechanical) commissioning engine construction.
///
/// Groups the inner-loop characterization results and operating mode
/// that `new_outer()` needs beyond `oid` and `entry_snapshot`.
#[derive(Debug, Clone, Copy, defmt::Format)]
pub struct OuterTuneParams {
    /// Commissioning profile (conservative / balanced / aggressive).
    pub profile: CommissionProfile,
    /// Operating mode (step/dir vs position).
    pub mode: OperatingMode,
    /// Electrical time constant from Stage 1 (microseconds).
    pub inner_lambda_us: u32,
    /// Theta-E delay from Stage 1 (microseconds).
    pub theta_e_us: u32,
    /// Whether the Stage 1 current step response showed ringing.
    pub current_ringing: bool,
    /// Current-loop bandwidth from Stage 1 (Hz).
    pub current_bandwidth_hz: u32,
}

/// Snapshot of outer gains at Stage 2 entry, for soft-failure restoration.
///
/// Current-loop gains are not included because Stage 2 never mutates them.
/// Built by the caller in `main.rs` from firmware atomics and passed to
/// `new_outer()`.
#[derive(Debug, Clone, Copy, defmt::Format)]
pub struct EntrySnapshot {
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
    /// PIDOUT_UQ_UD_LIMITS operational value.
    pub voltage_limit: u32,
    /// Velocity biquad filter cutoff (Hz), 0 = disabled.
    pub velocity_filter_hz: u16,
    /// Torque biquad filter cutoff (Hz), 0 = disabled.
    pub torque_filter_hz: u16,
    /// Position biquad filter cutoff (Hz), 0 = disabled.
    pub position_filter_hz: u16,
    /// Flux biquad filter cutoff (Hz), 0 = disabled.
    pub flux_filter_hz: u16,
}

/// Which commissioning mode is active.
#[derive(Debug, defmt::Format)]
enum EngineMode {
    Inner {
        phase: InnerPhase,
        profile: CommissionProfile,
        sequence: InnerSequence,
        adc_cal: Option<AdcCalResult>,
    },
    Outer {
        phase: OuterPhase,
        profile: CommissionProfile,
        mode: OperatingMode,
        /// Inner-loop closed-loop lambda in microseconds.
        inner_lambda_us: u32,
        /// Electrical transport delay in microseconds.
        theta_e_us: u32,
        /// Whether current-loop ringing was detected during inner validation.
        current_ringing: bool,
        /// Current-loop effective bandwidth Hz from inner commissioning.
        current_bandwidth_hz: u32,
        /// Gains snapshot at entry for soft-failure restoration.
        entry_snapshot: EntrySnapshot,
    },
}

/// Cooperative commissioning state machine.
///
/// Created when a `foci_commission`, `foci_tune`, or `foci_calibrate`
/// command arrives. Stored as `Option<CommissioningEngine>` in the
/// P1 `tmc_control` task. Each 1kHz tick calls `step()`.
pub struct CommissioningEngine {
    oid: u8,
    engine_mode: EngineMode,
    buf: TransientBuffer,
    /// Per-stage results collected during a selftest run. Populated only
    /// when `sequence == InnerSequence::Selftest`; unused otherwise.
    selftest_results: crate::selftest::SelftestResults,
}

impl CommissioningEngine {
    /// Create an engine for calibration (re-enable path).
    ///
    /// Runs ADC calibration, encoder alignment, and closed-loop entry
    /// with saved gains. Skips coil/wiring/encoder diagnostics.
    /// Returns `StepResult::CalibrateDone` on success.
    pub fn new_calibrate(oid: u8) -> Self {
        Self {
            oid,
            engine_mode: EngineMode::Inner {
                phase: InnerPhase::AdcCalibrate(AdcCalState::Setup),
                profile: CommissionProfile::Balanced,
                sequence: InnerSequence::Calibrate,
                adc_cal: None,
            },
            buf: TransientBuffer::new(),
            selftest_results: crate::selftest::SelftestResults::default(),
        }
    }

    /// Create an engine for selftest (diagnostics + electrical ID).
    ///
    /// Runs ADC calibration, coil check, phase wiring, encoder check,
    /// encoder alignment, and electrical identification, then stops with
    /// `StepResult::SelftestDone` after restoring safe state.
    pub fn new_selftest(oid: u8) -> Self {
        Self {
            oid,
            engine_mode: EngineMode::Inner {
                phase: InnerPhase::AdcCalibrate(AdcCalState::Setup),
                profile: CommissionProfile::Balanced,
                sequence: InnerSequence::Selftest,
                adc_cal: None,
            },
            buf: TransientBuffer::new(),
            selftest_results: crate::selftest::SelftestResults::default(),
        }
    }

    /// Create an engine for Stage 1 commissioning.
    ///
    /// Full diagnostics, electrical ID, current tune/validate, then
    /// closed-loop entry with fallback outer gains.
    /// Returns `StepResult::CommissionDone` on success.
    pub fn new_commission(oid: u8, profile: CommissionProfile) -> Self {
        Self {
            oid,
            engine_mode: EngineMode::Inner {
                phase: InnerPhase::AdcCalibrate(AdcCalState::Setup),
                profile,
                sequence: InnerSequence::Commission,
                adc_cal: None,
            },
            buf: TransientBuffer::new(),
            selftest_results: crate::selftest::SelftestResults::default(),
        }
    }

    /// Create an engine for outer (mechanical) commissioning.
    ///
    /// `entry_snapshot` captures the active outer gains at entry for
    /// soft-failure restoration. Built by the caller from firmware atomics.
    pub fn new_outer(oid: u8, params: OuterTuneParams, entry_snapshot: EntrySnapshot) -> Self {
        // Safety limits and data are initialized with placeholder values.
        // The MechanicalIdState::Setup sub-state reads the actual encoder
        // position and current limit from the backend on the first step().
        let safety = OuterSafetyLimits::new(0, 0);
        let data = MechanicalIdData::new(0, 0);
        let OuterTuneParams {
            profile,
            mode,
            inner_lambda_us,
            theta_e_us,
            current_ringing,
            current_bandwidth_hz,
        } = params;
        Self {
            oid,
            engine_mode: EngineMode::Outer {
                phase: OuterPhase::MechanicalId {
                    state: MechanicalIdState::Setup,
                    data,
                    safety,
                },
                profile,
                mode,
                inner_lambda_us,
                theta_e_us,
                current_ringing,
                current_bandwidth_hz,
                entry_snapshot,
            },
            buf: TransientBuffer::new(),
            selftest_results: crate::selftest::SelftestResults::default(),
        }
    }

    /// The OID this engine was started for.
    pub fn oid(&self) -> u8 {
        self.oid
    }

    /// Whether this engine was launched by `foci_calibrate`.
    pub fn is_calibrate(&self) -> bool {
        matches!(
            self.engine_mode,
            EngineMode::Inner {
                sequence: InnerSequence::Calibrate,
                ..
            }
        )
    }

    /// Whether this engine was launched by `foci_selftest`.
    pub fn is_selftest(&self) -> bool {
        matches!(
            self.engine_mode,
            EngineMode::Inner {
                sequence: InnerSequence::Selftest,
                ..
            }
        )
    }

    /// Return the entry snapshot for the outer engine, if active.
    pub fn entry_snapshot(&self) -> Option<EntrySnapshot> {
        match &self.engine_mode {
            EngineMode::Outer { entry_snapshot, .. } => Some(*entry_snapshot),
            _ => None,
        }
    }

    /// Consume the engine's collected selftest results.
    ///
    /// Caller should only invoke this when the most recent `step()`
    /// returned `StepResult::SelftestDone`.
    pub fn take_selftest_results(&mut self) -> crate::selftest::SelftestResults {
        core::mem::take(&mut self.selftest_results)
    }

    /// Advance the state machine by one tick (max 1ms).
    ///
    /// Returns a [`StepResult`] indicating whether to continue,
    /// report a phase transition, or finalize.
    ///
    /// The method destructures `engine_mode` to pass `phase` and
    /// `buf` separately, avoiding a double `&mut self` borrow.
    pub async fn step<B: CommissioningBackend>(&mut self, backend: &mut B) -> StepResult {
        // E-stop check at the top of every step
        if backend.is_shutdown_requested() {
            return self.abort(CommissionError::Shutdown);
        }

        let selftest_results = &mut self.selftest_results;
        match &mut self.engine_mode {
            EngineMode::Inner {
                phase,
                profile,
                sequence,
                adc_cal,
            } => {
                crate::commissioning::inner::step_inner(
                    phase,
                    &mut self.buf,
                    *profile,
                    *sequence,
                    adc_cal,
                    selftest_results,
                    backend,
                )
                .await
            }
            EngineMode::Outer {
                phase,
                profile,
                mode,
                inner_lambda_us,
                theta_e_us,
                current_ringing,
                current_bandwidth_hz,
                ..
            } => {
                Self::step_outer(
                    phase,
                    &mut self.buf,
                    *profile,
                    *mode,
                    *inner_lambda_us,
                    *theta_e_us,
                    *current_ringing,
                    *current_bandwidth_hz,
                    backend,
                )
                .await
            }
        }
    }

    /// Abort commissioning, restoring safe state.
    fn abort(&mut self, error: CommissionError) -> StepResult {
        match &self.engine_mode {
            EngineMode::Inner { .. } => StepResult::InnerFailed(error),
            EngineMode::Outer { .. } => StepResult::OuterFailed(error),
        }
    }

    // -------------------------------------------------------------------
    // Outer phase dispatch and safety
    // -------------------------------------------------------------------

    /// Step outer phase state machine.
    ///
    /// Dispatches to per-phase handlers. Every tick runs the safety
    /// envelope check before the phase-specific logic.
    #[allow(clippy::too_many_arguments)]
    async fn step_outer<B: CommissioningBackend>(
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
        // Run per-tick safety envelope for all active phases.
        if let Some(safety) = Self::outer_safety_mut(phase)
            && Self::check_outer_safety(safety, backend).await
        {
            // Zero excitation immediately on any safety violation.
            let _ = backend
                .write_register(foci_tmc4671::registers::PID_TORQUE_FLUX_TARGET, 0)
                .await;
            let _ = backend
                .write_register(foci_tmc4671::registers::MODE_RAMP_MODE_MOTION, 0)
                .await;
            let err = Self::safety_error_for_phase(phase);
            return Self::fail_outer(phase, err);
        }

        match phase {
            OuterPhase::MechanicalId { .. } => Self::step_mechanical_id(phase, buf, backend).await,
            OuterPhase::VelocityTune { .. } => {
                Self::step_velocity_tune(phase, profile, mode, inner_lambda_us, theta_e_us, backend)
                    .await
            }
            OuterPhase::VelocityValidate { .. } => {
                Self::step_velocity_validate(
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
                Self::step_position_tune(phase, buf, mode, backend).await
            }
            OuterPhase::FilterTune { .. } => Self::step_filter_tune(phase, backend).await,
            OuterPhase::Commit { .. } => Self::step_commit(phase),
            OuterPhase::Done(result) => StepResult::OuterDone(result.clone()),
            OuterPhase::Failed(err) => StepResult::OuterFailed(*err),
        }
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

    /// Compute a wrap-aware encoder delta.
    ///
    /// The ABN decoder counter wraps at `counts_per_rev` (PPR * 4).
    /// A raw subtraction across the wrap boundary produces a delta
    /// near +/- counts_per_rev instead of the true small motion.
    /// This normalizes the delta to `[-half, +half)`.
    fn encoder_delta(current: i32, previous: i32, counts_per_rev: i32) -> i32 {
        let half = counts_per_rev / 2;
        let raw = current.wrapping_sub(previous);
        if raw > half {
            raw - counts_per_rev
        } else if raw < -half {
            raw + counts_per_rev
        } else {
            raw
        }
    }

    /// Per-tick safety envelope check for all outer phases.
    ///
    /// Reads the current encoder position, computes a wrap-aware
    /// velocity delta, and accumulates an unwrapped signed position.
    /// Checks:
    /// - Velocity within cap
    /// - Unwrapped position within window from start
    ///
    /// Returns `true` if a safety limit is violated.
    async fn check_outer_safety<B: CommissioningBackend>(
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

        // Wrap-aware velocity: delta encoder counts per 1ms tick.
        let velocity = Self::encoder_delta(
            encoder_count,
            safety.prev_encoder_count,
            safety.counts_per_rev,
        );
        safety.prev_encoder_count = encoder_count;
        safety.unwrapped_position += velocity;

        // Check velocity cap (absolute value).
        let abs_velocity = if velocity < 0 { -velocity } else { velocity };
        if abs_velocity > safety.velocity_cap {
            defmt::warn!(
                "outer safety: velocity {} exceeds cap {}",
                abs_velocity,
                safety.velocity_cap
            );
            return true;
        }

        // Check position window against unwrapped accumulator.
        let abs_position = if safety.unwrapped_position < 0 {
            -safety.unwrapped_position
        } else {
            safety.unwrapped_position
        };
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

    /// Map a safety-envelope violation to the dedicated error variant.
    ///
    /// Always returns `SafetyEnvelopeViolation` so the OuterFailed handler
    /// can classify it as a safety fault requiring hard disable, distinct
    /// from normal phase failures that are eligible for soft restore.
    fn safety_error_for_phase(_phase: &OuterPhase) -> CommissionError {
        CommissionError::SafetyEnvelopeViolation
    }

    /// Transition outer phase to Failed.
    fn fail_outer(phase: &mut OuterPhase, err: CommissionError) -> StepResult {
        *phase = OuterPhase::Failed(err);
        StepResult::OuterFailed(err)
    }

    fn mech_id_torque_percent(attempt: u8) -> u32 {
        match attempt {
            0 => MECH_ID_TORQUE_STEP_PERCENT_0,
            1 => MECH_ID_TORQUE_STEP_PERCENT_1,
            _ => MECH_ID_TORQUE_STEP_PERCENT_2,
        }
    }

    fn mech_id_torque_step(current_limit: u32, attempt: u8) -> u16 {
        let percent = Self::mech_id_torque_percent(attempt);
        (current_limit * percent / 100).max(1).min(u16::MAX as u32) as u16
    }

    fn mech_id_low_confidence(
        fwd_accel: u32,
        rev_accel: u32,
        fwd_peak: u32,
        rev_peak: u32,
        fwd_travel: u32,
        rev_travel: u32,
    ) -> bool {
        fwd_accel.min(rev_accel) < MECH_ID_MIN_ACCEL_COUNTS_PER_MS2
            || fwd_peak.min(rev_peak) < MECH_ID_MIN_PEAK_VELOCITY_COUNTS_PER_MS
            || fwd_travel.min(rev_travel) < MECH_ID_MIN_TRAVEL_COUNTS
    }

    fn retry_mechanical_id(
        state: &mut MechanicalIdState,
        data: &mut MechanicalIdData,
        reason: &str,
    ) -> bool {
        if data.attempt + 1 >= MECH_ID_MAX_ATTEMPTS {
            return false;
        }

        data.attempt += 1;
        data.reset_captures();
        *state = MechanicalIdState::Setup;
        defmt::warn!(
            "mech-id: {} on attempt {}, retrying at {}% current",
            reason,
            data.attempt,
            Self::mech_id_torque_percent(data.attempt)
        );
        true
    }

    fn response_velocity_gains() -> VelocityGains {
        VelocityGains {
            velocity_p: OUTER_FALLBACK_VELOCITY_P,
            velocity_i: OUTER_FALLBACK_VELOCITY_I,
            velocity_limit: VEL_RESPONSE_LIMIT,
            effective_bandwidth_hz: 0,
            lambda_vel_ms: 0,
        }
    }

    fn response_mech_model(
        data: &MechanicalIdData,
        torque: u32,
        fwd_accel: u32,
        rev_accel: u32,
        fwd_peak: u32,
        rev_peak: u32,
    ) -> (MechanicalModel, u32) {
        let accel_avg = ((fwd_accel.max(1) + rev_accel.max(1)) / 2).max(1);
        let peak_avg = ((fwd_peak + rev_peak) / 2).max(1);
        let j_eff = (torque * 1000) / accel_avg;
        let b_eff = (torque * 1000) / peak_avg;
        let avg_delay_samples = (data.forward_delay_samples + data.reverse_delay_samples) / 2;

        (
            MechanicalModel {
                j_eff,
                b_eff,
                tau_mech_ms: VEL_RESPONSE_VALIDATE_TAU_MS,
                theta_mech_us: (avg_delay_samples as u64 * MECH_ID_SAMPLE_PERIOD_US) as u32,
                t_f: 0,
            },
            peak_avg,
        )
    }

    fn response_velocity_phase(
        model: MechanicalModel,
        peak_velocity: u32,
        mut diagnostics: OuterDiagnostics,
        safety: OuterSafetyLimits,
    ) -> OuterPhase {
        diagnostics.warning_code = OUTER_WARNING_LOW_CONFIDENCE_MECH_ID;
        OuterPhase::VelocityTune {
            model,
            peak_velocity,
            diagnostics,
            safety,
        }
    }

    // -------------------------------------------------------------------
    // Phase 8: Mechanical Identification
    // -------------------------------------------------------------------

    /// Advance mechanical identification sub-state.
    ///
    /// Implements the spec's acceleration test procedure:
    /// 1. Switch to torque mode with tuned current-loop gains active
    /// 2. Record starting position
    /// 3. Apply torque step, sample encoder at 1 kHz
    /// 4. Return to zero torque, wait for settling
    /// 5. Repeat in opposite direction
    /// 6. Estimate J_eff, B_eff, tau_mech, theta_mech
    /// 7. Validate gates and transition
    async fn step_mechanical_id<B: CommissioningBackend>(
        phase: &mut OuterPhase,
        buf: &mut TransientBuffer,
        backend: &mut B,
    ) -> StepResult {
        use foci_tmc4671::registers::*;

        let (state, data, safety) = match phase {
            OuterPhase::MechanicalId {
                state,
                data,
                safety,
            } => (state, data, safety),
            _ => return StepResult::Continue,
        };

        match state {
            MechanicalIdState::Setup => {
                // Read current encoder position as the reference point.
                let encoder_raw = backend.read_encoder_count().await;
                let start_position = encoder_raw as i32;

                // Compute torque step from configured run_current.
                let current_limit = backend.current_limit() & 0xFFFF;
                let torque_step = if current_limit > 0 {
                    Self::mech_id_torque_step(current_limit, data.attempt)
                } else {
                    return Self::fail_outer(phase, CommissionError::NoCurrent);
                };

                // Initialize safety limits with real values.
                safety.prev_encoder_count = start_position;
                safety.torque_cap = current_limit as u16;

                // Initialize data with real values.
                data.start_position = start_position;
                data.torque_step = torque_step;

                // Ensure we are in torque mode with the same advanced PI
                // controller structure used by the tuned current loop.
                if backend
                    .write_register(PIDOUT_UQ_UD_LIMITS, OUTER_PIDOUT_LIMIT)
                    .await
                    .is_err()
                    || backend.write_register(PWM_SV_CHOP, 7).await.is_err()
                    || backend.write_register(PHI_E_SELECTION, 3).await.is_err()
                    || backend
                        .write_register(MODE_RAMP_MODE_MOTION, CURRENT_TORQUE_MODE)
                        .await
                        .is_err()
                {
                    return Self::fail_outer(phase, CommissionError::Spi);
                }

                let phi_ok = backend
                    .read_register(PHI_E_SELECTION)
                    .await
                    .is_ok_and(|v| v == 3);
                if !phi_ok {
                    return Self::fail_outer(phase, CommissionError::EncoderNotAligned);
                }

                backend.set_enable_pin(true);

                // Zero torque target before starting capture.
                let _ = backend.write_register(PID_TORQUE_FLUX_TARGET, 0).await;

                // Prepare transient buffer for forward capture.
                buf.reset(MECH_ID_SAMPLE_PERIOD_US as u16);

                // Read encoder PPR for wrap-aware delta computation.
                let ppr_raw = backend.read_register(ABN_DECODER_PPR).await.unwrap_or(4000);
                let counts_per_rev = (ppr_raw as i32).max(4);
                safety.counts_per_rev = counts_per_rev;

                // Re-read encoder after mode switch to prevent a false
                // velocity spike in the safety check. The transition from
                // position mode to torque mode can cause the motor to
                // drift, producing a large encoder delta on the next tick.
                let post_switch_pos = backend.read_encoder_count().await as i32;
                safety.prev_encoder_count = post_switch_pos;
                safety.unwrapped_position = 0;
                data.start_position = post_switch_pos;

                defmt::info!(
                    "mech-id: attempt={}, start pos={}, torque_step={}, current_limit={}",
                    data.attempt,
                    post_switch_pos,
                    torque_step,
                    current_limit
                );

                *state = MechanicalIdState::ForwardStart;
                StepResult::PhaseChanged(PhaseId::MechanicalId)
            }

            MechanicalIdState::ForwardStart => {
                // Apply positive torque step.
                let torque = data.torque_step as u32;
                // PID_TORQUE_FLUX_TARGET: torque in [31:16], flux=0 in [15:0].
                let target = torque << 16;
                if backend
                    .write_register(PID_TORQUE_FLUX_TARGET, target)
                    .await
                    .is_err()
                {
                    return Self::fail_outer(phase, CommissionError::Spi);
                }

                let next_sample_us = backend.elapsed_us().wrapping_add(MECH_ID_SAMPLE_PERIOD_US);
                *state = MechanicalIdState::ForwardCapture { next_sample_us };
                StepResult::Continue
            }

            MechanicalIdState::ForwardCapture { next_sample_us } => {
                // Check if it's time for the next sample.
                let now = backend.elapsed_us();
                if now.wrapping_sub(*next_sample_us) >= 0x8000_0000 {
                    // Not yet time for next sample.
                    return StepResult::Continue;
                }

                // Sample encoder position.
                let encoder_raw = backend.read_encoder_count().await;
                buf.push(encoder_raw);

                if buf.is_full() {
                    // Forward capture complete — zero torque.
                    let _ = backend.write_register(PID_TORQUE_FLUX_TARGET, 0).await;

                    // Extract forward metrics from buffer before reset.
                    let (initial_accel, peak_velocity, delay_samples, travel) =
                        Self::extract_mech_metrics(buf, data.start_position, safety.counts_per_rev);
                    data.forward_initial_accel = initial_accel;
                    data.forward_peak_velocity = peak_velocity;
                    data.forward_delay_samples = delay_samples;
                    data.forward_travel = travel;

                    defmt::info!(
                        "mech-id fwd: accel={}, peak_vel={}, delay={}, travel={}",
                        initial_accel,
                        peak_velocity,
                        delay_samples,
                        travel
                    );

                    let deadline = backend.elapsed_us().wrapping_add(MECH_ID_SETTLE_US);
                    *state = MechanicalIdState::ForwardSettle {
                        deadline_us: deadline,
                    };
                } else {
                    *next_sample_us = now.wrapping_add(MECH_ID_SAMPLE_PERIOD_US);
                }
                StepResult::Continue
            }

            MechanicalIdState::ForwardSettle { deadline_us } => {
                if backend.elapsed_us().wrapping_sub(*deadline_us) < 0x8000_0000 {
                    // Settled — prepare for reverse capture.
                    buf.reset(MECH_ID_SAMPLE_PERIOD_US as u16);
                    *state = MechanicalIdState::ReverseStart;
                }
                StepResult::Continue
            }

            MechanicalIdState::ReverseStart => {
                // Apply negative torque step.
                // Torque is signed i16 in upper 16 bits.
                let neg_torque = (-(data.torque_step as i16)) as u16;
                let target = (neg_torque as u32) << 16;
                if backend
                    .write_register(PID_TORQUE_FLUX_TARGET, target)
                    .await
                    .is_err()
                {
                    return Self::fail_outer(phase, CommissionError::Spi);
                }

                let next_sample_us = backend.elapsed_us().wrapping_add(MECH_ID_SAMPLE_PERIOD_US);
                *state = MechanicalIdState::ReverseCapture { next_sample_us };
                StepResult::Continue
            }

            MechanicalIdState::ReverseCapture { next_sample_us } => {
                let now = backend.elapsed_us();
                if now.wrapping_sub(*next_sample_us) >= 0x8000_0000 {
                    return StepResult::Continue;
                }

                let encoder_raw = backend.read_encoder_count().await;
                buf.push(encoder_raw);

                if buf.is_full() {
                    // Reverse capture complete — zero torque.
                    let _ = backend.write_register(PID_TORQUE_FLUX_TARGET, 0).await;

                    // For reverse capture, use the position at capture start
                    // (first sample) as the reference, not the original start.
                    let reverse_start = if !buf.is_empty() {
                        buf.samples()[0] as i32
                    } else {
                        data.start_position
                    };
                    let (initial_accel, peak_velocity, delay_samples, travel) =
                        Self::extract_mech_metrics(buf, reverse_start, safety.counts_per_rev);
                    // Store as absolute values for symmetry comparison.
                    data.reverse_initial_accel = if initial_accel < 0 {
                        -initial_accel
                    } else {
                        initial_accel
                    };
                    data.reverse_peak_velocity = if peak_velocity < 0 {
                        -peak_velocity
                    } else {
                        peak_velocity
                    };
                    data.reverse_delay_samples = delay_samples;
                    data.reverse_travel = travel;

                    defmt::info!(
                        "mech-id rev: accel={}, peak_vel={}, delay={}, travel={}",
                        data.reverse_initial_accel,
                        data.reverse_peak_velocity,
                        delay_samples,
                        travel
                    );

                    let deadline = backend.elapsed_us().wrapping_add(MECH_ID_SETTLE_US);
                    *state = MechanicalIdState::ReverseSettle {
                        deadline_us: deadline,
                    };
                } else {
                    *next_sample_us = now.wrapping_add(MECH_ID_SAMPLE_PERIOD_US);
                }
                StepResult::Continue
            }

            MechanicalIdState::ReverseSettle { deadline_us } => {
                if backend.elapsed_us().wrapping_sub(*deadline_us) < 0x8000_0000 {
                    *state = MechanicalIdState::Estimate;
                }
                StepResult::Continue
            }

            MechanicalIdState::Estimate => {
                // Ensure motor is stopped.
                let _ = backend.write_register(PID_TORQUE_FLUX_TARGET, 0).await;
                let _ = backend.write_register(MODE_RAMP_MODE_MOTION, 0).await;

                *state = MechanicalIdState::Finalize;
                StepResult::Continue
            }

            MechanicalIdState::Finalize => {
                // Assemble and validate mechanical model.
                let torque = data.torque_step as u32;

                // Forward metrics (already absolute for accel).
                let fwd_accel = if data.forward_initial_accel < 0 {
                    (-data.forward_initial_accel) as u32
                } else {
                    data.forward_initial_accel as u32
                };
                let rev_accel = data.reverse_initial_accel as u32;

                let fwd_peak = if data.forward_peak_velocity < 0 {
                    (-data.forward_peak_velocity) as u32
                } else {
                    data.forward_peak_velocity as u32
                };
                let rev_peak = data.reverse_peak_velocity as u32;
                let peak_avg = (fwd_peak + rev_peak) / 2;

                // Gate: both accelerations must be positive and non-zero.
                if fwd_accel == 0 || rev_accel == 0 {
                    if Self::retry_mechanical_id(state, data, "zero acceleration") {
                        return StepResult::Continue;
                    }
                    defmt::warn!(
                        "mech-id: zero acceleration after retries, using response velocity tuning"
                    );
                    let (model, peak_velocity) = Self::response_mech_model(
                        data, torque, fwd_accel, rev_accel, fwd_peak, rev_peak,
                    );
                    *phase = Self::response_velocity_phase(
                        model,
                        peak_velocity,
                        OuterDiagnostics::from_mechanical_id(data),
                        *safety,
                    );
                    return StepResult::PhaseChanged(PhaseId::VelocityTune);
                }

                // J_eff = torque / initial_acceleration.
                // Scale: torque is in DAC units, accel is in counts/ms^2.
                // We multiply torque by 1000 to get better fixed-point resolution.
                let j_eff_fwd = (torque * 1000) / fwd_accel;
                let j_eff_rev = (torque * 1000) / rev_accel;

                // Symmetry gate: |J_fwd - J_rev| / avg <= 30%.
                let j_avg = (j_eff_fwd + j_eff_rev) / 2;
                if j_avg == 0 {
                    defmt::warn!("mech-id: zero average inertia");
                    return Self::fail_outer(phase, CommissionError::MechanicalIdFailed);
                }
                let j_diff = j_eff_fwd.abs_diff(j_eff_rev);
                let asymmetry_pct = (j_diff * 100) / j_avg;
                if asymmetry_pct > MECH_ID_SYMMETRY_THRESHOLD_PCT {
                    if Self::retry_mechanical_id(state, data, "asymmetric response") {
                        return StepResult::Continue;
                    }
                    defmt::warn!(
                        "mech-id: asymmetry {}% exceeds {}% after retries, using response velocity tuning",
                        asymmetry_pct,
                        MECH_ID_SYMMETRY_THRESHOLD_PCT
                    );
                    let (model, peak_velocity) = Self::response_mech_model(
                        data, torque, fwd_accel, rev_accel, fwd_peak, rev_peak,
                    );
                    *phase = Self::response_velocity_phase(
                        model,
                        peak_velocity,
                        OuterDiagnostics::from_mechanical_id(data),
                        *safety,
                    );
                    return StepResult::PhaseChanged(PhaseId::VelocityTune);
                }

                let j_eff = j_avg;

                // B_eff = torque / steady_state_velocity.
                // Scale to match J_eff units.
                let b_eff = if peak_avg > 0 {
                    (torque * 1000) / peak_avg
                } else {
                    // No steady-state velocity reached — pure inertial load.
                    // Use a small minimum to avoid division by zero in tau.
                    1
                };

                // tau_mech = J_eff / B_eff in milliseconds.
                let tau_mech_ms = if b_eff > 0 { j_eff / b_eff } else { 1000 };

                // theta_mech from delay samples (time from torque step to
                // first measurable velocity change).
                let avg_delay_samples =
                    (data.forward_delay_samples + data.reverse_delay_samples) / 2;
                let theta_mech_us = (avg_delay_samples as u64 * MECH_ID_SAMPLE_PERIOD_US) as u32;

                let model = MechanicalModel {
                    j_eff,
                    b_eff,
                    tau_mech_ms,
                    theta_mech_us,
                    t_f: 0, // Static friction not measured in this phase.
                };

                defmt::info!(
                    "mech-id: J={}, B={}, tau={}ms, theta={}us",
                    model.j_eff,
                    model.b_eff,
                    model.tau_mech_ms,
                    model.theta_mech_us
                );

                let diagnostics = OuterDiagnostics::from_mechanical_id(data);
                if Self::mech_id_low_confidence(
                    fwd_accel,
                    rev_accel,
                    fwd_peak,
                    rev_peak,
                    data.forward_travel,
                    data.reverse_travel,
                ) {
                    if Self::retry_mechanical_id(state, data, "low-confidence response") {
                        return StepResult::Continue;
                    }

                    defmt::warn!("mech-id: low-confidence data, using response velocity tuning");
                    *phase = Self::response_velocity_phase(model, peak_avg, diagnostics, *safety);
                    return StepResult::PhaseChanged(PhaseId::VelocityTune);
                }

                // Carry safety limits forward to VelocityTune.
                let carried_safety = match phase {
                    OuterPhase::MechanicalId { safety, .. } => *safety,
                    _ => unreachable!(),
                };

                *phase = OuterPhase::VelocityTune {
                    model,
                    peak_velocity: peak_avg,
                    diagnostics,
                    safety: carried_safety,
                };
                StepResult::PhaseChanged(PhaseId::VelocityTune)
            }
        }
    }

    /// Extract mechanical metrics from a position transient buffer.
    ///
    /// Computes velocity as first-order differences of position samples,
    /// then derives:
    /// - Initial acceleration (from first `MECH_ID_ACCEL_SAMPLES` velocities)
    /// - Peak velocity (maximum absolute velocity in the capture)
    /// - Delay samples (number of samples before first velocity onset)
    ///
    /// # Parameters
    /// - `buf`: Transient buffer filled with encoder position samples
    /// - `reference_position`: Starting position for velocity computation
    fn extract_mech_metrics(
        buf: &TransientBuffer,
        reference_position: i32,
        counts_per_rev: i32,
    ) -> (i32, i32, u16, u32) {
        let samples = buf.samples();
        if samples.len() < 2 {
            return (0, 0, 0, 0);
        }

        // Convert absolute encoder samples into per-sample velocity using
        // wrap-aware deltas. The ABN decoder wraps at counts_per_rev, so
        // raw subtraction produces wrong results near the boundary.
        let velocity_at = |idx: usize| -> i32 {
            if idx == 0 {
                Self::encoder_delta(samples[0] as i32, reference_position, counts_per_rev)
            } else {
                Self::encoder_delta(samples[idx] as i32, samples[idx - 1] as i32, counts_per_rev)
            }
        };

        let mut peak_velocity: i32 = 0;
        let mut onset_idx: Option<usize> = None;
        let mut max_travel: u32 = 0;

        // Accumulate unwrapped travel from wrap-aware deltas.
        let mut unwrapped_travel: i32 = 0;

        // Find the first sample with real motion and track the signed peak
        // velocity. Mechanical ID runs against an unloaded motor, but current
        // buildup and static friction can leave a quiet prefix at the start of
        // the capture. Treating that prefix as the acceleration window caused
        // false zero-acceleration failures on hardware.
        for (i, &_sample) in samples.iter().enumerate() {
            let velocity = velocity_at(i);
            unwrapped_travel += velocity;
            let travel = unwrapped_travel.unsigned_abs();
            if travel > max_travel {
                max_travel = travel;
            }
            let abs_cur = velocity.unsigned_abs();
            let abs_peak = peak_velocity.unsigned_abs();
            if abs_cur > abs_peak {
                peak_velocity = velocity;
            }

            if onset_idx.is_none() && abs_cur >= MECH_ID_VELOCITY_ONSET_THRESHOLD as u32 {
                onset_idx = Some(i);
            }
        }

        let onset_idx = match onset_idx {
            Some(idx) => idx,
            None => return (0, peak_velocity, samples.len() as u16, max_travel),
        };
        let delay_samples = onset_idx as u16;

        // Estimate initial acceleration from the first few samples after
        // motion onset, preserving sign so the caller can compare forward and
        // reverse captures. We use the velocity step to the local peak rather
        // than an average over the whole window so integer quantization cannot
        // round a small but real acceleration down to zero.
        let previous_velocity = if onset_idx == 0 {
            0
        } else {
            velocity_at(onset_idx - 1)
        };

        let direction_positive = velocity_at(onset_idx) >= 0;
        let end = (onset_idx + MECH_ID_ACCEL_SAMPLES).min(samples.len());
        let mut window_peak = velocity_at(onset_idx);
        for i in onset_idx + 1..end {
            let velocity = velocity_at(i);
            if direction_positive {
                if velocity > window_peak {
                    window_peak = velocity;
                }
            } else if velocity < window_peak {
                window_peak = velocity;
            }
        }

        let mut initial_accel = window_peak.wrapping_sub(previous_velocity);
        if initial_accel == 0 {
            initial_accel = velocity_at(onset_idx);
        }

        (initial_accel, peak_velocity, delay_samples, max_travel)
    }

    // -------------------------------------------------------------------
    // Phase 9: Velocity-Loop SIMC Synthesis
    // -------------------------------------------------------------------

    /// Advance velocity-tune phase.
    ///
    /// This is a thin pass-through: call the pure synthesis helper,
    /// then immediately transition to VelocityValidate with the
    /// computed gains.
    async fn step_velocity_tune<B: CommissioningBackend>(
        phase: &mut OuterPhase,
        profile: CommissionProfile,
        mode: OperatingMode,
        inner_lambda_us: u32,
        theta_e_us: u32,
        backend: &mut B,
    ) -> StepResult {
        let (model, peak_velocity, mut diagnostics, safety) = match phase {
            OuterPhase::VelocityTune {
                model,
                peak_velocity,
                diagnostics,
                safety,
            } => (*model, *peak_velocity, *diagnostics, *safety),
            _ => return StepResult::Continue,
        };

        let (gains, strategy, step_target) = if diagnostics.warning_code
            == OUTER_WARNING_LOW_CONFIDENCE_MECH_ID
        {
            let gains = Self::response_velocity_gains();
            (
                gains,
                VelocityTuneStrategy::ResponseRamp,
                VEL_RESPONSE_STEP_TARGET,
            )
        } else {
            let config = backend.tmc_config();
            let gains = match crate::autotune::simc_velocity_gains(
                &model,
                profile,
                mode,
                inner_lambda_us,
                theta_e_us,
                peak_velocity,
                config,
            ) {
                Some(gains) => gains,
                None => {
                    defmt::warn!("velocity tune: synthesis returned None");
                    return Self::fail_outer(phase, CommissionError::MechanicalIdFailed);
                }
            };

            diagnostics.synth_velocity_p = gains.velocity_p;
            diagnostics.synth_velocity_i = gains.velocity_i;

            if gains.velocity_p > OUTER_MAX_VELOCITY_P || gains.velocity_i > OUTER_MAX_VELOCITY_I {
                diagnostics.warning_code = OUTER_WARNING_UNSAFE_SYNTH_GAINS;
                defmt::warn!(
                    "velocity tune: synthesized P={}, I={} outside envelope, using response tuning",
                    gains.velocity_p,
                    gains.velocity_i
                );
                (
                    Self::response_velocity_gains(),
                    VelocityTuneStrategy::ResponseRamp,
                    VEL_RESPONSE_STEP_TARGET,
                )
            } else {
                let step_target = if gains.velocity_limit > 0 {
                    (gains.velocity_limit * VEL_VALIDATE_STEP_PERCENT / 100).max(1)
                } else {
                    return Self::fail_outer(phase, CommissionError::VelocityValidationFailed);
                };
                (gains, VelocityTuneStrategy::Model, step_target)
            }
        };

        defmt::info!(
            "velocity tune: P={}, I={}, limit={}, bw={}Hz, lambda={}ms, strategy={}",
            gains.velocity_p,
            gains.velocity_i,
            gains.velocity_limit,
            gains.effective_bandwidth_hz,
            gains.lambda_vel_ms,
            match strategy {
                VelocityTuneStrategy::Model => "model",
                VelocityTuneStrategy::ResponseRamp => "response",
            }
        );

        let data = VelocityValidateData {
            gains,
            accepted_gains: gains,
            has_accepted_gains: false,
            model,
            strategy,
            attempt: 0,
            step_target,
            forward_metrics: None,
            reverse_metrics: None,
            max_chatter: 0,
            peak_velocity,
            prev_encoder_for_velocity: 0,
            saturation_count: 0,
            torque_limit: backend.current_limit() & 0xFFFF,
            operational_velocity_limit: backend.operational_velocity_limit(),
            diagnostics,
        };

        *phase = OuterPhase::VelocityValidate {
            state: VelocityValidateState::WriteGains,
            data,
            safety,
        };
        StepResult::PhaseChanged(PhaseId::VelocityValidate)
    }

    // -------------------------------------------------------------------
    // Phase 10: Velocity-Loop Validation
    // -------------------------------------------------------------------

    /// Advance velocity-validate sub-state machine.
    #[allow(clippy::too_many_arguments)]
    #[allow(clippy::too_many_arguments)]
    async fn step_velocity_validate<B: CommissioningBackend>(
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
        use foci_tmc4671::registers::*;

        let (state, data, safety) = match phase {
            OuterPhase::VelocityValidate {
                state,
                data,
                safety,
            } => (state, data, safety),
            _ => return StepResult::Continue,
        };

        match state {
            VelocityValidateState::WriteGains => {
                // Write velocity PI gains to TMC4671.
                let vel_reg =
                    ((data.gains.velocity_p as u32) << 16) | (data.gains.velocity_i as u32);
                if backend
                    .write_register(PID_VELOCITY_P_VELOCITY_I, vel_reg)
                    .await
                    .is_err()
                {
                    return Self::fail_outer(phase, CommissionError::Spi);
                }

                // Write velocity limit.
                if backend
                    .write_register(PID_VELOCITY_LIMIT, data.gains.velocity_limit)
                    .await
                    .is_err()
                {
                    return Self::fail_outer(phase, CommissionError::Spi);
                }

                // Switch to velocity mode using the same advanced PI
                // controller structure as production closed-loop entry.
                if backend
                    .write_register(MODE_RAMP_MODE_MOTION, OUTER_VELOCITY_MODE)
                    .await
                    .is_err()
                {
                    return Self::fail_outer(phase, CommissionError::Spi);
                }

                // Zero velocity target.
                let _ = backend.write_register(PID_VELOCITY_TARGET, 0).await;

                // Prepare buffer for forward capture.
                buf.reset(VEL_VALIDATE_SAMPLE_PERIOD_US as u16);
                data.forward_metrics = None;
                data.reverse_metrics = None;
                data.max_chatter = 0;
                data.saturation_count = 0;

                *state = VelocityValidateState::ForwardStep;
                StepResult::Continue
            }

            VelocityValidateState::ForwardStep => {
                // Apply positive velocity step.
                if backend
                    .write_register(PID_VELOCITY_TARGET, data.step_target)
                    .await
                    .is_err()
                {
                    return Self::fail_outer(phase, CommissionError::Spi);
                }

                // Record initial encoder position for velocity derivation.
                data.prev_encoder_for_velocity = backend.read_encoder_count().await;

                let next_sample_us = backend
                    .elapsed_us()
                    .wrapping_add(VEL_VALIDATE_SAMPLE_PERIOD_US);
                *state = VelocityValidateState::ForwardCapture { next_sample_us };
                StepResult::Continue
            }

            VelocityValidateState::ForwardCapture { next_sample_us } => {
                let now = backend.elapsed_us();
                if now.wrapping_sub(*next_sample_us) >= 0x8000_0000 {
                    return StepResult::Continue;
                }

                // Derive velocity from encoder position delta (counts/tick).
                let encoder_raw = backend.read_encoder_count().await;
                let velocity = (encoder_raw as i32)
                    .wrapping_sub(data.prev_encoder_for_velocity as i32)
                    .unsigned_abs();
                data.prev_encoder_for_velocity = encoder_raw;
                buf.push(velocity);

                // Detect velocity-controller saturation: read the velocity
                // PID's torque output (INTERIM_PIDOUT_TARGET_TORQUE) and
                // count when it's at or near the configured torque limit.
                if data.torque_limit > 0 {
                    let sub = foci_tmc4671::registers::INTERIM_PIDOUT_TARGET_TORQUE;
                    let _ = backend.write_register(sub.addr, sub.sub_addr as u32).await;
                    if let Ok(pidout) = backend.read_register(sub.data).await {
                        let commanded_torque = (pidout & 0xFFFF) as i16;
                        let abs_commanded = commanded_torque.unsigned_abs() as u32;
                        if abs_commanded >= data.torque_limit * 9 / 10 {
                            data.saturation_count += 1;
                        }
                    }
                }

                if buf.is_full() {
                    // Forward capture complete — zero velocity target.
                    let _ = backend.write_register(PID_VELOCITY_TARGET, 0).await;

                    // Extract forward metrics.
                    let metrics = crate::autotune::extract_velocity_metrics(
                        buf,
                        data.step_target,
                        0, // Saturation counted from velocity PID output, not velocity samples.
                        VEL_VALIDATE_CHATTER_THRESHOLD,
                        VEL_VALIDATE_CHATTER_WINDOW,
                    );
                    data.forward_metrics = Some(metrics);

                    defmt::info!(
                        "vel fwd: rise={}ms, overshoot={}, settle={}ms",
                        metrics.rise_time_ms,
                        metrics.overshoot_pct,
                        metrics.settling_time_ms
                    );

                    let deadline = backend.elapsed_us().wrapping_add(VEL_VALIDATE_SETTLE_US);
                    *state = VelocityValidateState::ForwardSettle {
                        deadline_us: deadline,
                    };
                } else {
                    *next_sample_us = now.wrapping_add(VEL_VALIDATE_SAMPLE_PERIOD_US);
                }
                StepResult::Continue
            }

            VelocityValidateState::ForwardSettle { deadline_us } => {
                if backend.elapsed_us().wrapping_sub(*deadline_us) < 0x8000_0000 {
                    buf.reset(VEL_VALIDATE_SAMPLE_PERIOD_US as u16);
                    *state = VelocityValidateState::ReverseStep;
                }
                StepResult::Continue
            }

            VelocityValidateState::ReverseStep => {
                // Apply negative velocity step.
                // PID_VELOCITY_TARGET is signed i32.
                let neg_target = -(data.step_target as i32);
                if backend
                    .write_register(PID_VELOCITY_TARGET, neg_target as u32)
                    .await
                    .is_err()
                {
                    return Self::fail_outer(phase, CommissionError::Spi);
                }

                // Record initial encoder position for velocity derivation.
                data.prev_encoder_for_velocity = backend.read_encoder_count().await;

                let next_sample_us = backend
                    .elapsed_us()
                    .wrapping_add(VEL_VALIDATE_SAMPLE_PERIOD_US);
                *state = VelocityValidateState::ReverseCapture { next_sample_us };
                StepResult::Continue
            }

            VelocityValidateState::ReverseCapture { next_sample_us } => {
                let now = backend.elapsed_us();
                if now.wrapping_sub(*next_sample_us) >= 0x8000_0000 {
                    return StepResult::Continue;
                }

                // Derive velocity from encoder position delta (counts/tick).
                let encoder_raw = backend.read_encoder_count().await;
                let velocity = (encoder_raw as i32)
                    .wrapping_sub(data.prev_encoder_for_velocity as i32)
                    .unsigned_abs();
                data.prev_encoder_for_velocity = encoder_raw;
                buf.push(velocity);

                // Detect velocity-controller saturation from PID output.
                if data.torque_limit > 0 {
                    let sub = foci_tmc4671::registers::INTERIM_PIDOUT_TARGET_TORQUE;
                    let _ = backend.write_register(sub.addr, sub.sub_addr as u32).await;
                    if let Ok(pidout) = backend.read_register(sub.data).await {
                        let commanded_torque = (pidout & 0xFFFF) as i16;
                        let abs_commanded = commanded_torque.unsigned_abs() as u32;
                        if abs_commanded >= data.torque_limit * 9 / 10 {
                            data.saturation_count += 1;
                        }
                    }
                }

                if buf.is_full() {
                    let _ = backend.write_register(PID_VELOCITY_TARGET, 0).await;

                    let metrics = crate::autotune::extract_velocity_metrics(
                        buf,
                        data.step_target,
                        0, // Saturation counted from velocity PID output, not velocity samples.
                        VEL_VALIDATE_CHATTER_THRESHOLD,
                        VEL_VALIDATE_CHATTER_WINDOW,
                    );
                    data.reverse_metrics = Some(metrics);

                    defmt::info!(
                        "vel rev: rise={}ms, overshoot={}, settle={}ms",
                        metrics.rise_time_ms,
                        metrics.overshoot_pct,
                        metrics.settling_time_ms
                    );

                    let deadline = backend.elapsed_us().wrapping_add(VEL_VALIDATE_SETTLE_US);
                    *state = VelocityValidateState::ReverseSettle {
                        deadline_us: deadline,
                    };
                } else {
                    *next_sample_us = now.wrapping_add(VEL_VALIDATE_SAMPLE_PERIOD_US);
                }
                StepResult::Continue
            }

            VelocityValidateState::ReverseSettle { deadline_us } => {
                if backend.elapsed_us().wrapping_sub(*deadline_us) < 0x8000_0000 {
                    // Seed chatter encoder tracker independently of safety.
                    data.prev_encoder_for_velocity = backend.read_encoder_count().await;
                    buf.reset(VEL_VALIDATE_SAMPLE_PERIOD_US as u16);
                    let next_sample_us = backend
                        .elapsed_us()
                        .wrapping_add(VEL_VALIDATE_SAMPLE_PERIOD_US);
                    *state = VelocityValidateState::ChatterCheck {
                        next_sample_us,
                        remaining: VEL_VALIDATE_CHATTER_WINDOW,
                    };
                }
                StepResult::Continue
            }

            VelocityValidateState::ChatterCheck {
                next_sample_us,
                remaining,
            } => {
                let now = backend.elapsed_us();
                if now.wrapping_sub(*next_sample_us) >= 0x8000_0000 {
                    return StepResult::Continue;
                }

                // Derive velocity from own encoder tracker, independent of
                // the safety check which updates prev_encoder_count earlier.
                let count = backend.read_encoder_count().await;
                let velocity = (count as i32).wrapping_sub(data.prev_encoder_for_velocity as i32);
                data.prev_encoder_for_velocity = count;
                let abs_velocity = if velocity < 0 {
                    (-velocity) as u32
                } else {
                    velocity as u32
                };
                if abs_velocity > data.max_chatter {
                    data.max_chatter = abs_velocity;
                }

                *remaining -= 1;
                if *remaining == 0 {
                    *state = VelocityValidateState::Classify;
                } else {
                    *next_sample_us = now.wrapping_add(VEL_VALIDATE_SAMPLE_PERIOD_US);
                }
                StepResult::Continue
            }

            VelocityValidateState::Classify => {
                // Compute symmetry from forward/reverse rise times.
                let fwd = data.forward_metrics.unwrap_or(VelocityValidation {
                    rise_time_ms: 0,
                    overshoot_pct: 0,
                    settling_time_ms: 0,
                    zero_speed_chatter: false,
                    saturation_samples: 0,
                    symmetry_pct: 0,
                });
                let rev = data.reverse_metrics.unwrap_or(VelocityValidation {
                    rise_time_ms: 0,
                    overshoot_pct: 0,
                    settling_time_ms: 0,
                    zero_speed_chatter: false,
                    saturation_samples: 0,
                    symmetry_pct: 0,
                });

                let symmetry =
                    crate::autotune::compute_symmetry_pct(fwd.rise_time_ms, rev.rise_time_ms);

                // Merge: take the worst of forward/reverse for each metric.
                let merged = VelocityValidation {
                    rise_time_ms: fwd.rise_time_ms.max(rev.rise_time_ms),
                    overshoot_pct: fwd.overshoot_pct.max(rev.overshoot_pct),
                    settling_time_ms: fwd.settling_time_ms.max(rev.settling_time_ms),
                    zero_speed_chatter: data.max_chatter > VEL_VALIDATE_CHATTER_THRESHOLD,
                    saturation_samples: data.saturation_count,
                    symmetry_pct: symmetry,
                };

                let validation_tau_ms = if data.strategy == VelocityTuneStrategy::ResponseRamp {
                    VEL_RESPONSE_VALIDATE_TAU_MS
                } else {
                    data.model.tau_mech_ms.max(1)
                };
                let result = crate::autotune::classify_velocity_validation(
                    &merged,
                    validation_tau_ms,
                    crate::autotune::TRANSIENT_LEN as u16 * 2, // Total samples across both captures.
                );

                defmt::info!(
                    "vel classify: result={}, chatter={}, symmetry={}",
                    match result {
                        ValidationResult::Pass => "pass",
                        ValidationResult::Marginal => "marginal",
                        ValidationResult::Fail => "fail",
                    },
                    data.max_chatter,
                    symmetry
                );

                if data.strategy == VelocityTuneStrategy::ResponseRamp {
                    match result {
                        ValidationResult::Pass => {
                            data.accepted_gains = data.gains;
                            data.has_accepted_gains = true;

                            let next_p = data.gains.velocity_p as u32 * VEL_RESPONSE_P_MUL_NUM
                                / VEL_RESPONSE_P_MUL_DEN;
                            if next_p > OUTER_MAX_VELOCITY_P as u32
                                || next_p <= data.gains.velocity_p as u32
                            {
                                defmt::info!(
                                    "vel response: cap reached, accepted P={}, I={}",
                                    data.accepted_gains.velocity_p,
                                    data.accepted_gains.velocity_i
                                );
                                let _ = backend.write_register(PID_VELOCITY_TARGET, 0).await;
                                let _ = backend.write_register(MODE_RAMP_MODE_MOTION, 0).await;
                                let deadline =
                                    backend.elapsed_us().wrapping_add(VEL_VALIDATE_SETTLE_US);
                                *state = VelocityValidateState::SafeState {
                                    deadline_us: deadline,
                                };
                            } else {
                                data.attempt = data.attempt.saturating_add(1);
                                data.gains.velocity_p = next_p as u16;
                                data.gains.velocity_i = 0;
                                defmt::info!(
                                    "vel response: clean response, trying P={}, I=0",
                                    data.gains.velocity_p
                                );
                                *state = VelocityValidateState::WriteGains;
                            }
                        }
                        ValidationResult::Marginal | ValidationResult::Fail => {
                            let failed_p = data.gains.velocity_p;
                            if data.has_accepted_gains {
                                data.gains = data.accepted_gains;
                            } else {
                                data.gains = Self::response_velocity_gains();
                            }
                            defmt::warn!(
                                "vel response: {} at P={}, accepting P={}",
                                match result {
                                    ValidationResult::Fail => "fail",
                                    _ => "marginal",
                                },
                                failed_p,
                                if data.has_accepted_gains {
                                    data.accepted_gains.velocity_p
                                } else {
                                    OUTER_FALLBACK_VELOCITY_P
                                }
                            );
                            let _ = backend.write_register(PID_VELOCITY_TARGET, 0).await;
                            let _ = backend.write_register(MODE_RAMP_MODE_MOTION, 0).await;
                            let deadline =
                                backend.elapsed_us().wrapping_add(VEL_VALIDATE_SETTLE_US);
                            *state = VelocityValidateState::SafeState {
                                deadline_us: deadline,
                            };
                        }
                    }
                    return StepResult::Continue;
                }

                match result {
                    ValidationResult::Pass => {
                        // Success — restore safe state then transition.
                        let _ = backend.write_register(PID_VELOCITY_TARGET, 0).await;
                        let _ = backend.write_register(MODE_RAMP_MODE_MOTION, 0).await;
                        let deadline = backend.elapsed_us().wrapping_add(VEL_VALIDATE_SETTLE_US);
                        *state = VelocityValidateState::SafeState {
                            deadline_us: deadline,
                        };
                    }
                    ValidationResult::Marginal if data.attempt == 0 => {
                        // Retry: increase lambda_vel by 50% and recompute gains.
                        defmt::info!("vel validate: marginal on attempt 0, retrying");
                        data.attempt = 1;

                        let config = backend.tmc_config();
                        // Compute new gains with increased lambda.
                        // We modify the model's theta_mech to force a larger lambda,
                        // matching the SIMC retry pattern from current validation.
                        let mut retry_model = data.model;
                        let new_theta = data.model.theta_mech_us * VEL_RETRY_LAMBDA_MUL_NUM
                            / VEL_RETRY_LAMBDA_MUL_DEN;
                        retry_model.theta_mech_us = new_theta;

                        match crate::autotune::simc_velocity_gains(
                            &retry_model,
                            profile,
                            mode,
                            inner_lambda_us,
                            theta_e_us,
                            data.peak_velocity,
                            config,
                        ) {
                            Some(mut new_gains) => {
                                data.diagnostics.synth_velocity_p = new_gains.velocity_p;
                                data.diagnostics.synth_velocity_i = new_gains.velocity_i;
                                if new_gains.velocity_p > OUTER_MAX_VELOCITY_P
                                    || new_gains.velocity_i > OUTER_MAX_VELOCITY_I
                                {
                                    data.diagnostics.warning_code =
                                        OUTER_WARNING_UNSAFE_SYNTH_GAINS;
                                    defmt::warn!(
                                        "vel retry: synthesized P={}, I={} outside envelope, using response tuning",
                                        new_gains.velocity_p,
                                        new_gains.velocity_i
                                    );
                                    new_gains = Self::response_velocity_gains();
                                    data.strategy = VelocityTuneStrategy::ResponseRamp;
                                    data.step_target = VEL_RESPONSE_STEP_TARGET;
                                }
                                defmt::info!(
                                    "vel retry: P={}, I={}, lambda={}ms",
                                    new_gains.velocity_p,
                                    new_gains.velocity_i,
                                    new_gains.lambda_vel_ms
                                );
                                data.gains = new_gains;
                                if data.strategy == VelocityTuneStrategy::Model {
                                    data.step_target = (new_gains.velocity_limit
                                        * VEL_VALIDATE_STEP_PERCENT
                                        / 100)
                                        .max(1);
                                }
                                *state = VelocityValidateState::WriteGains;
                            }
                            None => {
                                return Self::fail_outer(
                                    phase,
                                    CommissionError::VelocityValidationFailed,
                                );
                            }
                        }
                    }
                    ValidationResult::Marginal | ValidationResult::Fail => {
                        // Already retried or hard fail — abort.
                        defmt::warn!(
                            "vel validate: {} on attempt {}",
                            match result {
                                ValidationResult::Fail => "fail",
                                _ => "marginal",
                            },
                            data.attempt
                        );
                        let _ = backend.write_register(PID_VELOCITY_TARGET, 0).await;
                        let _ = backend.write_register(MODE_RAMP_MODE_MOTION, 0).await;
                        return Self::fail_outer(phase, CommissionError::VelocityValidationFailed);
                    }
                }
                StepResult::Continue
            }

            VelocityValidateState::SafeState { deadline_us } => {
                if backend.elapsed_us().wrapping_sub(*deadline_us) < 0x8000_0000 {
                    *state = VelocityValidateState::Finalize;
                }
                StepResult::Continue
            }

            VelocityValidateState::Finalize => {
                // Ensure motor is stopped.
                let _ = backend.write_register(PID_VELOCITY_TARGET, 0).await;
                let _ = backend.write_register(MODE_RAMP_MODE_MOTION, 0).await;

                let carried_safety = *safety;
                let mut accepted_gains = data.gains;
                // Keep the bounded validation velocity limit in hardware for
                // Phase 11. Position tuning intentionally uses small, local
                // step responses; restoring the runtime limit here lets the
                // first position step violate the outer safety envelope.
                //
                // The final reported gains still carry the operational limit
                // so SAVE_CONFIG does not persist the commissioning test limit.
                accepted_gains.velocity_limit = data.operational_velocity_limit.max(1);
                let velocity_chatter = data.max_chatter > VEL_VALIDATE_CHATTER_THRESHOLD;

                defmt::info!(
                    "vel validate: accepted P={}, I={}, limit={}",
                    accepted_gains.velocity_p,
                    accepted_gains.velocity_i,
                    accepted_gains.velocity_limit
                );

                // Position capture is measured in ABN counts, but
                // PID_POSITION_TARGET is written in TMC electrical-position
                // units. Convert the small motor-local test move through the
                // configured STEP_WIDTH so the command and measurement agree.
                let step_width = backend.step_width().max(1);
                let step_target_counts = POS_TUNE_STEP_COUNTS;
                let step_target_tmc = step_width.saturating_mul(step_target_counts);

                let j_eff = data.model.j_eff;
                let b_eff = data.model.b_eff;

                let pos_data = PositionTuneData {
                    current_p: POS_TUNE_INITIAL_P,
                    stable_p: 0,
                    step_target_counts,
                    step_target_tmc,
                    step_start_counts: 0,
                    step_start_tmc: 0,
                    oscillation_at_minimum: false,
                    velocity_gains: accepted_gains,
                    velocity_chatter,
                    current_ringing,
                    current_bandwidth_hz,
                    j_eff,
                    b_eff,
                    diagnostics: data.diagnostics,
                };

                *phase = OuterPhase::PositionTune {
                    state: PositionTuneState::Setup,
                    data: pos_data,
                    safety: carried_safety,
                };
                StepResult::PhaseChanged(PhaseId::PositionTune)
            }
        }
    }

    // -------------------------------------------------------------------
    // Phase 11: Position-Loop P Ramp-Up
    // -------------------------------------------------------------------

    /// Advance position-tune sub-state machine.
    ///
    /// Ramps position P from conservative starting value until oscillation
    /// is detected or a cap is reached. Final P = last non-oscillating value.
    async fn step_position_tune<B: CommissioningBackend>(
        phase: &mut OuterPhase,
        buf: &mut TransientBuffer,
        mode: OperatingMode,
        backend: &mut B,
    ) -> StepResult {
        use foci_tmc4671::registers::*;

        let (state, data, safety) = match phase {
            OuterPhase::PositionTune {
                state,
                data,
                safety,
            } => (state, data, safety),
            _ => return StepResult::Continue,
        };

        match state {
            PositionTuneState::Setup => {
                // Switch to position mode using the same advanced PI
                // controller structure as production closed-loop entry.
                if backend
                    .write_register(MODE_RAMP_MODE_MOTION, OUTER_POSITION_MODE)
                    .await
                    .is_err()
                {
                    return Self::fail_outer(phase, CommissionError::Spi);
                }

                // Write initial position P gain, I=0.
                let pos_reg = (data.current_p as u32) << 16;
                if backend
                    .write_register(PID_POSITION_P_POSITION_I, pos_reg)
                    .await
                    .is_err()
                {
                    return Self::fail_outer(phase, CommissionError::Spi);
                }

                // Record both position domains. ABN counts are used for
                // response measurement; PID_POSITION_ACTUAL is the TMC target
                // domain used by PID_POSITION_TARGET.
                let encoder_raw = backend.read_encoder_count().await;
                data.step_start_counts = encoder_raw as i32;
                let position_raw = match backend.read_register(PID_POSITION_ACTUAL).await {
                    Ok(v) => v as i32,
                    Err(_) => return Self::fail_outer(phase, CommissionError::Spi),
                };
                data.step_start_tmc = position_raw;

                // Command a position step.
                let target = position_raw.wrapping_add(data.step_target_tmc as i32);
                if backend
                    .write_register(PID_POSITION_TARGET, target as u32)
                    .await
                    .is_err()
                {
                    return Self::fail_outer(phase, CommissionError::Spi);
                }

                // Prepare capture buffer.
                buf.reset(POS_TUNE_SAMPLE_PERIOD_US as u16);

                let next_sample_us = backend.elapsed_us().wrapping_add(POS_TUNE_SAMPLE_PERIOD_US);
                *state = PositionTuneState::StepCapture { next_sample_us };
                StepResult::Continue
            }

            PositionTuneState::StepCapture { next_sample_us } => {
                let now = backend.elapsed_us();
                if now.wrapping_sub(*next_sample_us) >= 0x8000_0000 {
                    return StepResult::Continue;
                }

                let encoder_raw = backend.read_encoder_count().await;
                // Store position relative to step start so the oscillation
                // detector compares against the step amplitude, not absolute
                // encoder counts.
                let relative = (encoder_raw as i32)
                    .wrapping_sub(data.step_start_counts)
                    .unsigned_abs();
                buf.push(relative);

                if buf.is_full() {
                    // Return to start position.
                    let _ = backend
                        .write_register(PID_POSITION_TARGET, data.step_start_tmc as u32)
                        .await;

                    let deadline = backend.elapsed_us().wrapping_add(POS_TUNE_SETTLE_US);
                    *state = PositionTuneState::Settle {
                        deadline_us: deadline,
                    };
                } else {
                    *next_sample_us = now.wrapping_add(POS_TUNE_SAMPLE_PERIOD_US);
                }
                StepResult::Continue
            }

            PositionTuneState::Settle { deadline_us } => {
                if backend.elapsed_us().wrapping_sub(*deadline_us) < 0x8000_0000 {
                    *state = PositionTuneState::Analyze;
                }
                StepResult::Continue
            }

            PositionTuneState::Analyze => {
                // Check for oscillation in the captured response.
                let target_abs = data.step_target_counts;
                let oscillation = crate::autotune::detect_position_oscillation(buf, target_abs);

                defmt::info!(
                    "pos tune: P={}, oscillation={}",
                    data.current_p,
                    oscillation
                );

                if oscillation {
                    if data.stable_p == 0 {
                        // Minimum P already oscillates — position tune failed.
                        data.oscillation_at_minimum = true;
                        defmt::warn!("pos tune: minimum P={} oscillates", data.current_p);
                    }
                    // Stop ramp-up. Use last stable P (or fail if none).
                    *state = PositionTuneState::Finalize;
                } else {
                    // Clean response — record this as stable and increase P.
                    data.stable_p = data.current_p;

                    let response_tuned = data.velocity_gains.effective_bandwidth_hz == 0;
                    let max_p = if response_tuned {
                        POS_RESPONSE_TUNE_MAX_P
                    } else {
                        POS_TUNE_MAX_P
                    };
                    if data.current_p >= max_p {
                        // Hit cap — finalize with current stable P.
                        defmt::info!("pos tune: P cap reached at {}", data.stable_p);
                        *state = PositionTuneState::Finalize;
                    } else {
                        // Try higher P.
                        let new_p = (data.current_p as u32 * POS_TUNE_P_MUL_NUM
                            / POS_TUNE_P_MUL_DEN)
                            .min(max_p as u32) as u16;
                        if new_p <= data.current_p {
                            defmt::info!("pos tune: P cap reached at {}", data.stable_p);
                            *state = PositionTuneState::Finalize;
                            return StepResult::Continue;
                        }
                        data.current_p = new_p;

                        // Write new P to registers.
                        let pos_reg = (data.current_p as u32) << 16;
                        let _ = backend
                            .write_register(PID_POSITION_P_POSITION_I, pos_reg)
                            .await;

                        // Record new step start and command step.
                        let encoder_raw = backend.read_encoder_count().await;
                        data.step_start_counts = encoder_raw as i32;
                        let position_raw = match backend.read_register(PID_POSITION_ACTUAL).await {
                            Ok(v) => v as i32,
                            Err(_) => return Self::fail_outer(phase, CommissionError::Spi),
                        };
                        data.step_start_tmc = position_raw;
                        let target = position_raw.wrapping_add(data.step_target_tmc as i32);
                        let _ = backend
                            .write_register(PID_POSITION_TARGET, target as u32)
                            .await;

                        buf.reset(POS_TUNE_SAMPLE_PERIOD_US as u16);
                        let next_sample_us =
                            backend.elapsed_us().wrapping_add(POS_TUNE_SAMPLE_PERIOD_US);
                        *state = PositionTuneState::StepCapture { next_sample_us };
                    }
                }
                StepResult::Continue
            }

            PositionTuneState::Finalize => {
                // Ensure motor is stopped.
                let _ = backend.write_register(MODE_RAMP_MODE_MOTION, 0).await;

                if data.oscillation_at_minimum {
                    // Position tune failed — keep velocity gains.
                    return Self::fail_outer(phase, CommissionError::PositionTuneFailed);
                }

                let final_p = data.stable_p;

                // Response-tuned velocity gains are deliberately conservative:
                // the tuner has not measured a steady-state position bias, so
                // leave position integral disabled to avoid windup on installed
                // mechanics.
                let response_tuned = data.velocity_gains.effective_bandwidth_hz == 0;
                let final_i = if response_tuned {
                    0
                } else {
                    match mode {
                        OperatingMode::Nominal | OperatingMode::HighInertia => {
                            (final_p as u32 * POS_TUNE_I_PERCENT / 100).max(1) as u16
                        }
                        OperatingMode::Unloaded => 0,
                    }
                };

                defmt::info!("pos tune: final P={}, I={}", final_p, final_i);

                let carried_safety = *safety;
                let position_gains = crate::autotune::PositionGains {
                    position_p: final_p,
                    position_i: final_i,
                };
                let velocity_gains = data.velocity_gains;
                let velocity_chatter = data.velocity_chatter;
                let current_ringing = data.current_ringing;
                let current_bandwidth_hz = data.current_bandwidth_hz;
                let j_eff = data.j_eff;
                let b_eff = data.b_eff;
                let diagnostics = data.diagnostics;
                // Position oscillation is "early" if the first stable P was
                // reached only at the minimum value (suggesting the system
                // is marginal for position control).
                let position_oscillation_early =
                    final_p == POS_TUNE_INITIAL_P && data.current_p > POS_TUNE_INITIAL_P;

                *phase = OuterPhase::FilterTune {
                    velocity_gains,
                    position_gains,
                    position_oscillation_early,
                    velocity_chatter,
                    current_ringing,
                    current_bandwidth_hz,
                    j_eff,
                    b_eff,
                    diagnostics,
                    safety: carried_safety,
                };
                StepResult::PhaseChanged(PhaseId::FilterTune)
            }
        }
    }

    // -------------------------------------------------------------------
    // Phase 12: Filter Selection
    // -------------------------------------------------------------------

    /// Implement filter selection from prior validation metrics.
    ///
    /// Calls the pure recommendation helper, computes biquad coefficients,
    /// and writes them to the TMC4671 sub-registers.
    async fn step_filter_tune<B: CommissioningBackend>(
        phase: &mut OuterPhase,
        backend: &mut B,
    ) -> StepResult {
        let (
            velocity_gains,
            position_gains,
            position_oscillation_early,
            velocity_chatter,
            current_ringing,
            current_bandwidth_hz,
            j_eff,
            b_eff,
            diagnostics,
        ) = match phase {
            OuterPhase::FilterTune {
                velocity_gains,
                position_gains,
                position_oscillation_early,
                velocity_chatter,
                current_ringing,
                current_bandwidth_hz,
                j_eff,
                b_eff,
                diagnostics,
                ..
            } => (
                *velocity_gains,
                *position_gains,
                *position_oscillation_early,
                *velocity_chatter,
                *current_ringing,
                *current_bandwidth_hz,
                *j_eff,
                *b_eff,
                *diagnostics,
            ),
            _ => return StepResult::Continue,
        };

        // Estimate position bandwidth as velocity_bw / 4.
        let position_bandwidth_hz = velocity_gains.effective_bandwidth_hz / 4;

        let filter_config = crate::autotune::recommend_filters(
            velocity_chatter,
            velocity_gains.effective_bandwidth_hz,
            position_oscillation_early,
            position_bandwidth_hz,
            current_ringing,
            current_bandwidth_hz,
        );

        let pwm_maxcnt = backend.pwm_maxcnt();

        // Enable or explicitly disable each filter to avoid stale biquad
        // state leaking from prior commissioning runs or host configuration.
        if let Some(cutoff_hz) = filter_config.velocity_filter_hz {
            crate::commissioning::helpers::write_biquad_velocity(
                backend,
                cutoff_hz as u16,
                pwm_maxcnt,
            )
            .await;
        } else {
            crate::commissioning::helpers::write_sub_register(
                backend,
                foci_tmc4671::registers::CONFIG_BIQUAD_V_ENABLE,
                0,
            )
            .await;
        }

        if let Some(cutoff_hz) = filter_config.position_filter_hz {
            crate::commissioning::helpers::write_biquad_position(
                backend,
                cutoff_hz as u16,
                pwm_maxcnt,
            )
            .await;
        } else {
            crate::commissioning::helpers::write_sub_register(
                backend,
                foci_tmc4671::registers::CONFIG_BIQUAD_X_ENABLE,
                0,
            )
            .await;
        }

        if let Some(cutoff_hz) = filter_config.flux_filter_hz {
            crate::commissioning::helpers::write_biquad_flux(backend, cutoff_hz as u16, pwm_maxcnt)
                .await;
        } else {
            crate::commissioning::helpers::write_sub_register(
                backend,
                foci_tmc4671::registers::CONFIG_BIQUAD_F_ENABLE,
                0,
            )
            .await;
        }

        if let Some(cutoff_hz) = filter_config.torque_filter_hz {
            crate::commissioning::helpers::write_biquad_torque(
                backend,
                cutoff_hz as u16,
                pwm_maxcnt,
            )
            .await;
        } else {
            crate::commissioning::helpers::write_sub_register(
                backend,
                foci_tmc4671::registers::CONFIG_BIQUAD_T_ENABLE,
                0,
            )
            .await;
        }

        defmt::info!(
            "filter tune: vel={}, pos={}, flux={}, torque={}",
            filter_config.velocity_filter_hz.unwrap_or(0),
            filter_config.position_filter_hz.unwrap_or(0),
            filter_config.flux_filter_hz.unwrap_or(0),
            filter_config.torque_filter_hz.unwrap_or(0)
        );

        *phase = OuterPhase::Commit {
            velocity_gains,
            position_gains,
            filter_config,
            j_eff,
            b_eff,
            diagnostics,
        };
        StepResult::PhaseChanged(PhaseId::Commit)
    }

    // -------------------------------------------------------------------
    // Phase 13: Commit
    // -------------------------------------------------------------------

    /// Assemble and emit the final outer commissioning result.
    fn step_commit(phase: &mut OuterPhase) -> StepResult {
        let (mut velocity_gains, mut position_gains, filter_config, j_eff, b_eff, mut diagnostics) =
            match phase {
                OuterPhase::Commit {
                    velocity_gains,
                    position_gains,
                    filter_config,
                    j_eff,
                    b_eff,
                    diagnostics,
                } => (
                    *velocity_gains,
                    *position_gains,
                    *filter_config,
                    *j_eff,
                    *b_eff,
                    *diagnostics,
                ),
                _ => return StepResult::Continue,
            };

        if diagnostics.synth_velocity_p == 0 && diagnostics.synth_velocity_i == 0 {
            diagnostics.synth_velocity_p = velocity_gains.velocity_p;
            diagnostics.synth_velocity_i = velocity_gains.velocity_i;
        }
        diagnostics.synth_position_p = position_gains.position_p;
        diagnostics.synth_position_i = position_gains.position_i;

        let mut status = if diagnostics.warning_code == OUTER_WARNING_NONE {
            CommissionStatus::Accepted
        } else {
            CommissionStatus::AcceptedWithWarnings
        };
        if velocity_gains.velocity_p > OUTER_MAX_VELOCITY_P
            || velocity_gains.velocity_i > OUTER_MAX_VELOCITY_I
            || position_gains.position_p > OUTER_MAX_POSITION_P
            || position_gains.position_i > OUTER_MAX_POSITION_I
        {
            defmt::warn!(
                "outer commit: rejecting unsafe gains vel_p={}, vel_i={}, pos_p={}, pos_i={}",
                velocity_gains.velocity_p,
                velocity_gains.velocity_i,
                position_gains.position_p,
                position_gains.position_i
            );
            velocity_gains.velocity_p = OUTER_FALLBACK_VELOCITY_P;
            velocity_gains.velocity_i = OUTER_FALLBACK_VELOCITY_I;
            position_gains.position_p = OUTER_FALLBACK_POSITION_P;
            position_gains.position_i = OUTER_FALLBACK_POSITION_I;
            diagnostics.warning_code = OUTER_WARNING_UNSAFE_SYNTH_GAINS;
            status = CommissionStatus::AcceptedWithWarnings;
        }

        let result = OuterResult {
            status,
            velocity_p: velocity_gains.velocity_p,
            velocity_i: velocity_gains.velocity_i,
            velocity_limit: velocity_gains.velocity_limit,
            position_p: position_gains.position_p,
            position_i: position_gains.position_i,
            velocity_filter_hz: filter_config.velocity_filter_hz.unwrap_or(0) as u16,
            position_filter_hz: filter_config.position_filter_hz.unwrap_or(0) as u16,
            flux_filter_hz: filter_config.flux_filter_hz.unwrap_or(0) as u16,
            torque_filter_hz: filter_config.torque_filter_hz.unwrap_or(0) as u16,
            j_eff,
            b_eff,
            warning_code: diagnostics.warning_code,
            mech_torque_step: diagnostics.mech_torque_step,
            mech_fwd_accel: diagnostics.mech_fwd_accel,
            mech_rev_accel: diagnostics.mech_rev_accel,
            mech_fwd_peak_velocity: diagnostics.mech_fwd_peak_velocity,
            mech_rev_peak_velocity: diagnostics.mech_rev_peak_velocity,
            mech_fwd_travel: diagnostics.mech_fwd_travel,
            mech_rev_travel: diagnostics.mech_rev_travel,
            synth_velocity_p: diagnostics.synth_velocity_p,
            synth_velocity_i: diagnostics.synth_velocity_i,
            synth_position_p: diagnostics.synth_position_p,
            synth_position_i: diagnostics.synth_position_i,
        };

        defmt::info!(
            "outer commit: vel_p={}, vel_i={}, pos_p={}, pos_i={}",
            result.velocity_p,
            result.velocity_i,
            result.position_p,
            result.position_i
        );

        *phase = OuterPhase::Done(result.clone());
        StepResult::OuterDone(result)
    }
}
