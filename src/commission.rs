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
    ClosedLoopEntryState, CommissionError, CommissionProfile, CommissionStatus, CurrentGains,
    CurrentValidation, ElectricalModel, InnerSequence, MechanicalModel, OperatingMode, PhaseId,
    TransientBuffer, ValidationResult, VelocityGains, VelocityValidation,
};
use crate::config::TmcConfig;
use foci_tmc4671::registers::Register;

/// Hardware abstraction for commissioning measurements.
///
/// The engine operates through this trait rather than raw SPI.
/// Each board crate provides a concrete implementation wrapping
/// its SPI device, GPIO pins, and timer.
///
/// SPI methods are async to match the DMA-based embassy-stm32
/// driver. The engine's `step()` is also async, called from the
/// async `tmc_control` task — consistent with how `selftest.rs`
/// measurement functions work.
#[allow(async_fn_in_trait)]
pub trait CommissioningBackend {
    /// SPI/hardware error type.
    type Error;

    /// Read a TMC4671 register.
    async fn read_register(&mut self, reg: Register) -> Result<u32, Self::Error>;

    /// Write a TMC4671 register.
    async fn write_register(&mut self, reg: Register, val: u32) -> Result<(), Self::Error>;

    /// Set the TMC4671 enable pin state.
    fn set_enable_pin(&mut self, state: bool);

    /// Read the current ABN encoder count.
    async fn read_encoder_count(&mut self) -> u32;

    /// Microseconds elapsed since an arbitrary monotonic reference point.
    ///
    /// Used for deadline-driven sub-state timing within `step()`.
    /// Must be strictly monotonic (never decreases or wraps) across
    /// a commissioning session.
    ///
    /// Returning `u64` avoids any realistic wraparound: u64 us covers
    /// ~584,942 years. The previous `u32` return type combined with
    /// a `read_tim5() / 84` implementation wrapped every ~51 seconds
    /// on a non-u32-clean boundary, which broke deadline arithmetic
    /// and caused the commissioning engine to hang.
    ///
    /// Board implementations should use a hardware-backed monotonic
    /// (e.g. a TIM2 `stm32_tim2_monotonic!` from rtic-monotonics) to
    /// preserve microsecond precision for sub-ms sampling loops in
    /// Phase 5 electrical identification.
    fn elapsed_us(&self) -> u64;

    /// Whether a shutdown has been requested (M112 / e-stop).
    ///
    /// Checked at the top of every `step()` call.
    fn is_shutdown_requested(&self) -> bool;

    /// Read the configured current limit register value.
    ///
    /// Returns `PID_TORQUE_FLUX_LIMITS` as set by `tmc_set_current`.
    /// Zero means no current limit configured.
    fn current_limit(&self) -> u32;

    /// PWM_MAXCNT register value for the board.
    fn pwm_maxcnt(&self) -> u16;

    /// PWM_BBM_H_BBM_L register value: `(bbm_h << 8) | bbm_l`.
    fn bbm_register(&self) -> u32;

    /// DSADC_MDEC_B_MDEC_A register value (same MDEC for both groups).
    fn dsadc_mdec_register(&self) -> u32;

    /// Test excitation voltage magnitude (PIDOUT units, bits [15:0]).
    fn excitation_voltage(&self) -> u16;

    /// Board-specific TMC4671 configuration.
    ///
    /// Needed by SIMC gain synthesis for PWM frequency.
    fn tmc_config(&self) -> &TmcConfig;

    /// TMC4671 STEP_WIDTH value (phi_e increment per step pulse).
    ///
    /// Used by position-loop tuning as the position step size.
    /// Returns 0 if the encoder has not been configured.
    fn step_width(&self) -> u32;

    /// Configured operational PID_VELOCITY_LIMIT value.
    ///
    /// Phase 10 may use a small limit derived from mechanical ID to keep the
    /// validation step bounded. Once velocity validation passes, later phases
    /// should return to the operational limit used for normal motion.
    fn operational_velocity_limit(&self) -> u32;
}

/// Result of a `step()` call.
#[derive(Debug, defmt::Format)]
pub enum StepResult {
    /// Engine is still working. Call `step()` again next tick.
    Continue,
    /// A phase transition occurred. Report to host.
    PhaseChanged(PhaseId),
    /// Inner commissioning failed.
    InnerFailed(CommissionError),
    /// Outer commissioning completed successfully.
    OuterDone(OuterResult),
    /// Outer commissioning failed.
    OuterFailed(CommissionError),
    /// ADC calibration completed. Hardware remains active for encoder
    /// alignment.
    CalibrateDone(AdcCalResult),
    /// Selftest phases completed. Hardware restored to safe state.
    SelftestDone,
    /// Stage 1 commissioning complete. Motor is in closed-loop position mode.
    /// Carries inner result + fallback outer gains for the Python host to
    /// cache and persist.
    CommissionDone(CommissionResult),
}

/// Result of inner (electrical) commissioning.
#[derive(Debug, Clone, defmt::Format)]
pub struct InnerResult {
    /// Overall status.
    pub status: CommissionStatus,
    /// Resistance in milliohms (for host display/persistence).
    pub r_mohm: u32,
    /// Inductance in microhenries (for host display/persistence).
    pub l_uh: u32,
    /// Computed flux P gain (Q8.8).
    pub flux_p: u16,
    /// Computed flux I gain (Q8.8).
    pub flux_i: u16,
    /// Computed torque P gain (Q8.8).
    pub torque_p: u16,
    /// Computed torque I gain (Q8.8).
    pub torque_i: u16,
    /// Inner-loop closed-loop lambda in microseconds.
    pub lambda_us: u32,
    /// Electrical transport delay in microseconds.
    pub theta_e_us: u32,
    /// Max ringing count from forward/reverse current validation.
    pub ringing_count: u8,
    /// Effective current-loop bandwidth in Hz.
    pub bandwidth_hz: u32,
}

/// Result of outer (mechanical) commissioning.
#[derive(Debug, Clone, defmt::Format)]
pub struct OuterResult {
    /// Overall status.
    pub status: CommissionStatus,
    /// Velocity P gain (Q8.8).
    pub velocity_p: u16,
    /// Velocity I gain (Q8.8).
    pub velocity_i: u16,
    /// Velocity limit.
    pub velocity_limit: u32,
    /// Position P gain (Q8.8).
    pub position_p: u16,
    /// Position I gain (Q8.8).
    pub position_i: u16,
    /// Velocity filter cutoff Hz (0 = disabled).
    pub velocity_filter_hz: u16,
    /// Position filter cutoff Hz (0 = disabled).
    pub position_filter_hz: u16,
    /// Flux filter cutoff Hz (0 = disabled).
    pub flux_filter_hz: u16,
    /// Torque filter cutoff Hz (0 = disabled).
    pub torque_filter_hz: u16,
    /// Identified effective inertia from mechanical ID (internal units).
    pub j_eff: u32,
    /// Identified effective damping from mechanical ID (internal units).
    pub b_eff: u32,
    /// Warning reason for accepted-with-warning results (0 = none).
    pub warning_code: u8,
    /// Torque step used by the final mechanical-ID attempt.
    pub mech_torque_step: u16,
    /// Forward initial acceleration, absolute encoder counts/ms^2.
    pub mech_fwd_accel: u32,
    /// Reverse initial acceleration, absolute encoder counts/ms^2.
    pub mech_rev_accel: u32,
    /// Forward peak velocity, absolute encoder counts/ms.
    pub mech_fwd_peak_velocity: u32,
    /// Reverse peak velocity, absolute encoder counts/ms.
    pub mech_rev_peak_velocity: u32,
    /// Forward total travel during mechanical ID, encoder counts.
    pub mech_fwd_travel: u32,
    /// Reverse total travel during mechanical ID, encoder counts.
    pub mech_rev_travel: u32,
    /// Velocity P before safety fallback, if synthesis reached that far.
    pub synth_velocity_p: u16,
    /// Velocity I before safety fallback, if synthesis reached that far.
    pub synth_velocity_i: u16,
    /// Position P before safety fallback, if position tuning reached that far.
    pub synth_position_p: u16,
    /// Position I before safety fallback, if position tuning reached that far.
    pub synth_position_i: u16,
}

/// Result of the calibration subset requested by `new_calibrate`.
#[derive(Debug, Clone, defmt::Format)]
pub struct AdcCalResult {
    /// ADC I0 offset.
    pub adc_i0: u16,
    /// ADC I1 offset.
    pub adc_i1: u16,
    /// Encoder count captured before the alignment phase zeroed it.
    pub encoder_count: u32,
}

/// Result of Stage 1 commissioning.
///
/// Combines the inner result (current-loop gains, electrical model) with
/// the fallback outer gains applied during closed-loop entry. Matches the
/// wire result `foci_commission_result`.
#[derive(Debug, Clone, defmt::Format)]
pub struct CommissionResult {
    /// Inner commissioning result (current-loop gains, electrical model).
    pub inner: InnerResult,
    /// Fallback velocity P gain (Q8.8) written during closed-loop entry.
    pub fallback_velocity_p: u16,
    /// Fallback velocity I gain (Q8.8) written during closed-loop entry.
    pub fallback_velocity_i: u16,
    /// Fallback position P gain (Q8.8) written during closed-loop entry.
    pub fallback_position_p: u16,
    /// Fallback position I gain (Q8.8) written during closed-loop entry.
    pub fallback_position_i: u16,
    /// Fallback PID velocity limit register value.
    pub fallback_velocity_limit: u32,
}

// ---------------------------------------------------------------------------
// Diagnostic sub-state constants
// ---------------------------------------------------------------------------

/// Number of ADC samples for offset calibration.
const ADC_CAL_SAMPLES: u16 = 128;

/// Settling time after enabling ADC outputs (microseconds).
const ADC_CAL_SETTLE_US: u64 = 250_000;

/// Valid ADC offset range (centered around ~33000).
const ADC_OFFSET_MIN: u16 = 30_000;
const ADC_OFFSET_MAX: u16 = 36_000;

/// Minimum ADC current to consider a coil present (raw ADC units).
const MOTOR_CURRENT_THRESHOLD: u16 = 200;

/// Settling time after applying coil test voltage (microseconds).
const COIL_SETTLE_US: u64 = 100_000;

/// Settling time after disabling coil test (microseconds).
const COIL_COOLDOWN_US: u64 = 50_000;

/// Number of PHI_E steps for encoder direction sweep.
const ENCODER_DIR_STEPS: u16 = 50;

/// Final PHI_E_EXT value for encoder direction sweep.
///
/// Keep this away from 0/65536 so the sweep has a non-zero final rotor
/// displacement. A full electrical revolution can end at the same stable
/// angle it started from, producing a false ENCODER_FAULT.
const ENCODER_DIR_TARGET_PHI: u32 = 32768;

/// Per-step dwell time for encoder direction sweep (microseconds).
const ENCODER_DIR_STEP_US: u64 = 10_000;

/// Settling time after applying encoder alignment voltage.
const ENCODER_ALIGN_SETTLE_US: u64 = 500_000;

/// Stability double-read delay after encoder alignment settle.
const ENCODER_ALIGN_STABILITY_US: u64 = 10_000;

/// Allowed encoder jitter during the alignment double-read.
const ENCODER_ALIGN_STABILITY_COUNTS: u32 = 2;

/// Voltage limit used only while applying encoder movement current.
const ENCODER_ALIGN_VOLTAGE_LIMIT: u32 = 2000;

/// Minimum D-axis current target used while forcing encoder movement.
const ENCODER_MOVE_CURRENT_MIN: u32 = 100;

/// Maximum D-axis current target used while forcing encoder movement.
const ENCODER_MOVE_CURRENT_MAX: u32 = 1000;

// ---------------------------------------------------------------------------
// Electrical identification constants (Phase 5)
// ---------------------------------------------------------------------------

/// Number of steady-state current samples for resistance measurement.
const RES_SAMPLE_COUNT: u16 = 1000;

/// Sampling interval for resistance and transient captures (microseconds).
/// Matches PWM rate (~160us at 25 kHz) for Nyquist-rate current sampling.
const ELEC_SAMPLE_PERIOD_US: u64 = 160;

/// Settling time after applying excitation voltage (microseconds).
const RES_SETTLE_US: u64 = 100_000;

/// Cooldown time after disabling a per-coil measurement (microseconds).
const ELEC_COOLDOWN_US: u64 = 50_000;

/// Brief settle before starting di/dt toggling (microseconds).
const DIDT_PRE_SETTLE_US: u64 = 10_000;

/// Maximum di/dt toggle cycles per step() call.
/// Each cycle is ~2 SPI transfers (~22us at 8 MHz SPI).
/// Budget: ~30 cycles/step ≈ 660us, leaving margin within 1ms tick.
const DIDT_CYCLES_PER_STEP: u32 = 30;

/// Minimum average |delta_I| per di/dt cycle to consider inductance valid.
/// Below this, the current change is too small to extract L reliably.
const DIDT_MIN_DELTA: u32 = 5;

/// Number of transient samples for delay identification.
/// Matches TransientBuffer capacity.
const DELAY_SAMPLE_COUNT: u16 = 256;

/// Minimum detectable current for delay onset (ADC units above baseline).
/// Must be above noise floor but sensitive enough to detect delay.
const DELAY_ONSET_THRESHOLD: u16 = 20;

/// Fraction of steady-state current for 63% rise detection (fixed-point).
/// 0.632 * 1024 ≈ 647 (Q10 format for integer math).
const RISE_63_PCT_Q10: u32 = 647;

// ---------------------------------------------------------------------------
// Current validation constants (Phase 7)
// ---------------------------------------------------------------------------

/// Step amplitude as a fraction of run_current (15% = 15/100).
const VALIDATION_STEP_PERCENT: u32 = 15;

/// Settling time after applying or removing current step (microseconds).
const VALIDATION_SETTLE_US: u64 = 50_000;

/// PID output voltage limit used during current-loop validation.
///
/// ADC calibration leaves `PIDOUT_UQ_UD_LIMITS` at 1024 as a conservative
/// safety cap, but the measured validation captures can hit that cap before
/// reaching the current target. 2000 matches the bounded encoder-alignment
/// limit and remains well below the operational closed-loop limit.
const CURRENT_VALIDATE_PIDOUT_LIMIT: u32 = 2000;

use crate::autotune::CaptureDirection;

/// Current-loop validation sub-state machine (Phase 7).
#[derive(Debug, defmt::Format)]
enum CurrentValidateState {
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
struct CurrentValidateData {
    /// The electrical model from Phase 5.
    model: ElectricalModel,
    /// Currently applied gains.
    gains: CurrentGains,
    /// Validation attempt (0 = first, 1 = retry).
    attempt: u8,
    /// Current step amplitude (absolute, PIDOUT units).
    step_amplitude: u16,
    /// Forward capture metrics (filled after forward capture).
    forward_metrics: Option<CurrentValidation>,
    /// Reverse capture metrics (filled after reverse capture).
    reverse_metrics: Option<CurrentValidation>,
    /// Final commissioning status to report.
    final_status: CommissionStatus,
    /// PID-output saturation counter: samples where the current PID's
    /// voltage output is at or near `PIDOUT_UQ_UD_LIMITS`.
    saturation_count: u16,
    /// Configured PIDOUT voltage limit for saturation detection.
    pidout_limit: u32,
}

// ---------------------------------------------------------------------------
// Diagnostic sub-states
// ---------------------------------------------------------------------------

/// ADC calibration sub-state.
#[derive(Debug, defmt::Format)]
enum AdcCalState {
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
enum CoilTestState {
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
enum CoilCheckState {
    /// Testing coil A (phase 0).
    CoilA(CoilTestState),
    /// Testing coil B (phase 1).
    CoilB(CoilTestState),
}

/// Accumulated coil test observations carried between sub-phases.
#[derive(Debug, Clone, Copy, defmt::Format)]
struct CoilObservation {
    /// Current on the expected ADC channel.
    expected: i16,
    /// Current on the other ADC channel.
    other: i16,
}

/// Encoder check sub-state.
#[derive(Debug, defmt::Format)]
enum EncoderCheckState {
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
enum EncoderAlignState {
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
struct ElectricalIdData {
    /// Excitation voltage used for all measurements (PIDOUT units).
    voltage: u32,
    /// Per-coil accumulated signed current for resistance (sum of i16 readings).
    res_current_sum: [i32; 2],
    /// Per-coil number of resistance samples collected.
    res_sample_count: [u16; 2],
    /// Per-coil accumulated |delta_I| for di/dt inductance measurement.
    ind_sum_delta: [u64; 2],
    /// Per-coil number of valid di/dt cycles.
    ind_count: [u32; 2],
    /// Per-coil elapsed microseconds during di/dt measurement.
    ind_elapsed_us: [u32; 2],
}

impl ElectricalIdData {
    fn new(voltage: u32) -> Self {
        Self {
            voltage,
            res_current_sum: [0; 2],
            res_sample_count: [0; 2],
            ind_sum_delta: [0; 2],
            ind_count: [0; 2],
            ind_elapsed_us: [0; 2],
        }
    }
}

/// Electrical identification sub-state machine (Phase 5).
#[derive(Debug, defmt::Format)]
enum ElectricalIdState {
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

/// Inner commissioning phase state machine.
///
/// Each variant carries its own local state so that multi-step
/// operations can be chunked across 1ms `step()` calls.
#[derive(Debug, defmt::Format)]
enum InnerPhase {
    /// Phase 1: ADC offset calibration.
    AdcCalibrate(AdcCalState),
    /// Phase 2: Coil connectivity check.
    CoilCheck {
        state: CoilCheckState,
        /// Coil A observation (filled after coil A test).
        coil_a: Option<CoilObservation>,
    },
    /// Phase 3: Phase wiring validation.
    PhaseWiringCheck {
        coil_a: CoilObservation,
        coil_b: CoilObservation,
    },
    /// Phase 4: Encoder movement and direction detection.
    EncoderCheck { state: EncoderCheckState },
    /// Encoder D-axis alignment and zeroing.
    EncoderAlign { state: EncoderAlignState },
    /// Phase 5: Electrical identification (R, L, theta_e).
    ElectricalId {
        state: ElectricalIdState,
        data: ElectricalIdData,
    },
    /// Phase 6: Current-loop SIMC gain synthesis.
    CurrentTune { model: ElectricalModel },
    /// Phase 7: Current-loop step-response validation.
    CurrentValidate {
        state: CurrentValidateState,
        data: CurrentValidateData,
    },
    /// Closed-loop position mode entry with PID gains, filters, and stability check.
    ClosedLoopEntry {
        /// Current sub-state.
        state: ClosedLoopEntryState,
        /// Current-loop flux P gain (Q8.8) to write.
        flux_p: u16,
        /// Current-loop flux I gain (Q8.8) to write.
        flux_i: u16,
        /// Current-loop torque P gain (Q8.8) to write.
        torque_p: u16,
        /// Current-loop torque I gain (Q8.8) to write.
        torque_i: u16,
        /// Outer velocity P gain (Q8.8) to write.
        velocity_p: u16,
        /// Outer velocity I gain (Q8.8) to write.
        velocity_i: u16,
        /// Outer position P gain (Q8.8) to write.
        position_p: u16,
        /// Outer position I gain (Q8.8) to write.
        position_i: u16,
        /// PID velocity limit register value.
        velocity_limit: u32,
        /// Velocity biquad filter cutoff (Hz), 0 = disabled.
        velocity_filter_hz: u16,
        /// Torque biquad filter cutoff (Hz), 0 = disabled.
        torque_filter_hz: u16,
        /// Position biquad filter cutoff (Hz), 0 = disabled.
        position_filter_hz: u16,
        /// Flux biquad filter cutoff (Hz), 0 = disabled.
        flux_filter_hz: u16,
        /// Inner result carried forward for CommissionDone (Commission sequence only).
        inner_result: Option<InnerResult>,
    },
    /// A phase failed.
    Failed(CommissionError),
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

    /// Restore outer gains from an entry snapshot after Stage 2 soft failure.
    ///
    /// Follows the spec's restore sequence: zero targets, disable mode,
    /// clear integrators, reload outer gains + filters from snapshot,
    /// reload voltage limit, zero position, re-enter position mode.
    pub async fn restore_entry_gains<B: CommissioningBackend>(
        snapshot: &EntrySnapshot,
        backend: &mut B,
    ) -> Result<(), CommissionError> {
        use foci_tmc4671::registers::*;

        // 1. Zero torque, velocity targets, and velocity offset.
        backend
            .write_register(PID_TORQUE_FLUX_TARGET, 0)
            .await
            .map_err(|_| CommissionError::Spi)?;
        backend
            .write_register(PID_VELOCITY_TARGET, 0)
            .await
            .map_err(|_| CommissionError::Spi)?;
        backend
            .write_register(PID_VELOCITY_OFFSET, 0)
            .await
            .map_err(|_| CommissionError::Spi)?;

        // 2. Disable motion mode.
        backend
            .write_register(MODE_RAMP_MODE_MOTION, 0)
            .await
            .map_err(|_| CommissionError::Spi)?;

        // 3. Clear PID integrators.
        Self::write_sub_register_checked(backend, PID_ERROR_PID_TORQUE_ERROR_SUM, 0)
            .await
            .map_err(|_| CommissionError::Spi)?;
        Self::write_sub_register_checked(backend, PID_ERROR_PID_FLUX_ERROR_SUM, 0)
            .await
            .map_err(|_| CommissionError::Spi)?;
        Self::write_sub_register_checked(backend, PID_ERROR_PID_VELOCITY_ERROR_SUM, 0)
            .await
            .map_err(|_| CommissionError::Spi)?;
        Self::write_sub_register_checked(backend, PID_ERROR_PID_POSITION_ERROR_SUM, 0)
            .await
            .map_err(|_| CommissionError::Spi)?;

        // 4. Clear status flags.
        backend
            .write_register(STATUS_FLAGS, 0)
            .await
            .map_err(|_| CommissionError::Spi)?;

        // 5. Reload outer gains from snapshot.
        let velocity_reg = (snapshot.velocity_p as u32) << 16 | snapshot.velocity_i as u32;
        backend
            .write_register(PID_VELOCITY_P_VELOCITY_I, velocity_reg)
            .await
            .map_err(|_| CommissionError::Spi)?;
        let position_reg = (snapshot.position_p as u32) << 16 | snapshot.position_i as u32;
        backend
            .write_register(PID_POSITION_P_POSITION_I, position_reg)
            .await
            .map_err(|_| CommissionError::Spi)?;
        backend
            .write_register(PID_VELOCITY_LIMIT, snapshot.velocity_limit)
            .await
            .map_err(|_| CommissionError::Spi)?;

        // Reload filters (best-effort). Explicitly disable when Hz=0
        // to restore the full entry filter state.
        let pwm_maxcnt = backend.pwm_maxcnt();
        if snapshot.velocity_filter_hz > 0 {
            Self::write_biquad_velocity(backend, snapshot.velocity_filter_hz, pwm_maxcnt).await;
        } else {
            Self::write_sub_register(backend, CONFIG_BIQUAD_V_ENABLE, 0).await;
        }
        if snapshot.torque_filter_hz > 0 {
            Self::write_biquad_torque(backend, snapshot.torque_filter_hz, pwm_maxcnt).await;
        } else {
            Self::write_sub_register(backend, CONFIG_BIQUAD_T_ENABLE, 0).await;
        }
        if snapshot.position_filter_hz > 0 {
            Self::write_biquad_position(backend, snapshot.position_filter_hz, pwm_maxcnt).await;
        } else {
            Self::write_sub_register(backend, CONFIG_BIQUAD_X_ENABLE, 0).await;
        }
        if snapshot.flux_filter_hz > 0 {
            Self::write_biquad_flux(backend, snapshot.flux_filter_hz, pwm_maxcnt).await;
        } else {
            Self::write_sub_register(backend, CONFIG_BIQUAD_F_ENABLE, 0).await;
        }

        // 6. Reload voltage limit.
        backend
            .write_register(PIDOUT_UQ_UD_LIMITS, snapshot.voltage_limit)
            .await
            .map_err(|_| CommissionError::Spi)?;

        // 7. Zero position (also zeros target per datasheet 0x6B).
        backend
            .write_register(PID_POSITION_ACTUAL, 0)
            .await
            .map_err(|_| CommissionError::Spi)?;

        // 8. Re-enter position mode.
        backend
            .write_register(MODE_RAMP_MODE_MOTION, 0x8000_0003)
            .await
            .map_err(|_| CommissionError::Spi)?;

        // Caller must perform stability check after an appropriate
        // delay (10ms + 10ms with Mono::delay). This function cannot
        // block cooperatively because CommissioningBackend has no
        // delay primitive.

        Ok(())
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
                Self::step_inner(
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

    /// Step inner phase state machine.
    ///
    /// Dispatches to per-phase handlers. Each handler advances the
    /// sub-state by one chunk of work (bounded to ~1ms). Phase
    /// transitions are signalled by replacing `*phase` and returning
    /// `StepResult::PhaseChanged`.
    async fn step_inner<B: CommissioningBackend>(
        phase: &mut InnerPhase,
        buf: &mut TransientBuffer,
        profile: CommissionProfile,
        sequence: InnerSequence,
        adc_cal: &mut Option<AdcCalResult>,
        selftest_results: &mut crate::selftest::SelftestResults,
        backend: &mut B,
    ) -> StepResult {
        match phase {
            InnerPhase::AdcCalibrate(_) => {
                Self::step_adc_calibrate(phase, sequence, adc_cal, selftest_results, backend).await
            }
            InnerPhase::CoilCheck { .. } => {
                Self::step_coil_check(phase, sequence, selftest_results, backend).await
            }
            InnerPhase::PhaseWiringCheck { coil_a, coil_b } => {
                let a = *coil_a;
                let b = *coil_b;
                Self::step_phase_wiring(phase, sequence, selftest_results, a, b)
            }
            InnerPhase::EncoderCheck { .. } => {
                Self::step_encoder_check(phase, sequence, selftest_results, backend).await
            }
            InnerPhase::EncoderAlign { .. } => {
                Self::step_encoder_align(phase, sequence, adc_cal, selftest_results, backend).await
            }
            InnerPhase::ElectricalId { .. } => {
                Self::step_elec_id(phase, buf, sequence, selftest_results, backend).await
            }
            InnerPhase::CurrentTune { .. } => {
                Self::step_current_tune(phase, profile, backend).await
            }
            InnerPhase::CurrentValidate { .. } => {
                Self::step_current_validate(phase, buf, profile, sequence, backend).await
            }
            InnerPhase::ClosedLoopEntry { .. } => {
                Self::step_closed_loop_entry(phase, sequence, adc_cal, backend).await
            }
            InnerPhase::Failed(err) => StepResult::InnerFailed(*err),
        }
    }

    // -------------------------------------------------------------------
    // Phase 1: ADC Calibration
    // -------------------------------------------------------------------

    /// Advance ADC calibration sub-state.
    ///
    /// Ported from `selftest_adc_calibrate`: configures TMC4671 for
    /// zero-current ADC sampling (PWM_CHOP=0), collects 128 samples
    /// one per step() call, validates offsets, programs them back.
    async fn step_adc_calibrate<B: CommissioningBackend>(
        phase: &mut InnerPhase,
        sequence: InnerSequence,
        adc_cal: &mut Option<AdcCalResult>,
        selftest_results: &mut crate::selftest::SelftestResults,
        backend: &mut B,
    ) -> StepResult {
        use foci_tmc4671::registers::*;

        let cal_state = match phase {
            InnerPhase::AdcCalibrate(cal_state) => cal_state,
            _ => return StepResult::Continue,
        };

        match cal_state {
            AdcCalState::Setup => {
                // Stop motor, disable outputs, set voltage limit.
                backend.set_enable_pin(false);
                if backend
                    .write_register(MODE_RAMP_MODE_MOTION, 0)
                    .await
                    .is_err()
                    || backend
                        .write_register(PIDOUT_UQ_UD_LIMITS, 1024)
                        .await
                        .is_err()
                {
                    return Self::fail_inner(phase, CommissionError::Spi);
                }

                // Verify voltage limit write.
                match backend.read_register(PIDOUT_UQ_UD_LIMITS).await {
                    Ok(1024) => {}
                    _ => return Self::fail_inner(phase, CommissionError::Spi),
                }

                let _ = backend.write_register(PID_TORQUE_FLUX_TARGET, 0).await;
                let _ = backend.write_register(UQ_UD_EXT, 0).await;

                // PWM config: chopping disabled for calibration.
                let _ = backend.write_register(PWM_POLARITIES, 0).await;
                let _ = backend
                    .write_register(PWM_MAXCNT, backend.pwm_maxcnt() as u32)
                    .await;
                let _ = backend
                    .write_register(PWM_BBM_H_BBM_L, backend.bbm_register())
                    .await;
                if backend.write_register(PWM_SV_CHOP, 0).await.is_err() {
                    return Self::fail_inner(phase, CommissionError::Spi);
                }

                // Verify MODE=0 and PWM_SV_CHOP=0.
                let mode_ok = backend
                    .read_register(MODE_RAMP_MODE_MOTION)
                    .await
                    .is_ok_and(|v| v == 0);
                let chop_ok = backend
                    .read_register(PWM_SV_CHOP)
                    .await
                    .is_ok_and(|v| v == 0);
                if !mode_ok || !chop_ok {
                    return Self::fail_inner(phase, CommissionError::Spi);
                }

                // ADC configuration.
                let _ = backend
                    .write_register(DSADC_MDEC_B_MDEC_A, backend.dsadc_mdec_register())
                    .await;
                let _ = backend
                    .write_register(ADC_I_SELECT, (1 << 28) | (1 << 8))
                    .await;

                // Motor type: 2-phase stepper, 50 pole pairs.
                let _ = backend
                    .write_register(MOTOR_TYPE_N_POLE_PAIRS, (2 << 16) | 50)
                    .await;

                // Clear offsets, enable outputs.
                let _ = backend.write_register(ADC_I0_SCALE_OFFSET, 256 << 16).await;
                let _ = backend.write_register(ADC_I1_SCALE_OFFSET, 256 << 16).await;
                backend.set_enable_pin(true);

                let deadline = backend.elapsed_us().wrapping_add(ADC_CAL_SETTLE_US);
                *cal_state = AdcCalState::Settle {
                    deadline_us: deadline,
                };
                StepResult::Continue
            }

            AdcCalState::Settle { deadline_us } => {
                if backend.elapsed_us().wrapping_sub(*deadline_us) < 0x8000_0000 {
                    // Deadline passed (unsigned comparison).
                    *cal_state = AdcCalState::Sample {
                        sum_i0: 0,
                        sum_i1: 0,
                        count: 0,
                    };
                }
                StepResult::Continue
            }

            AdcCalState::Sample {
                sum_i0,
                sum_i1,
                count,
            } => {
                // One sample per step() call (~1ms between samples).
                match backend.read_register(ADC_IWY_IUX).await {
                    Ok(raw) => {
                        let i_ux = (raw & 0xFFFF) as i16 as i32;
                        let i_wy = ((raw >> 16) & 0xFFFF) as i16 as i32;
                        *sum_i0 += i_ux;
                        *sum_i1 += i_wy;
                        *count += 1;
                    }
                    Err(_) => {
                        return Self::fail_inner(phase, CommissionError::Spi);
                    }
                }
                if *count >= ADC_CAL_SAMPLES {
                    *cal_state = AdcCalState::Finalize {
                        sum_i0: *sum_i0,
                        sum_i1: *sum_i1,
                    };
                }
                StepResult::Continue
            }

            AdcCalState::Finalize { sum_i0, sum_i1 } => {
                let n = ADC_CAL_SAMPLES as i32;
                let offset_i0 = (*sum_i0 / n) as i16 as u16;
                let offset_i1 = (*sum_i1 / n) as i16 as u16;

                // Validate offsets in expected range.
                if !(ADC_OFFSET_MIN..=ADC_OFFSET_MAX).contains(&offset_i0)
                    || !(ADC_OFFSET_MIN..=ADC_OFFSET_MAX).contains(&offset_i1)
                {
                    if matches!(sequence, InnerSequence::Selftest) {
                        selftest_results.set(1, crate::selftest::SelftestStageValue::fail(0));
                    }
                    return Self::fail_inner(phase, CommissionError::AdcFault);
                }

                if matches!(sequence, InnerSequence::Selftest) {
                    selftest_results.set(
                        1,
                        crate::selftest::SelftestStageValue::adc_offsets(offset_i0, offset_i1),
                    );
                }

                // Program calibrated offsets.
                let _ = backend
                    .write_register(ADC_I0_SCALE_OFFSET, (256 << 16) | offset_i0 as u32)
                    .await;
                let _ = backend
                    .write_register(ADC_I1_SCALE_OFFSET, (256 << 16) | offset_i1 as u32)
                    .await;

                // Re-enable PWM chopping.
                let _ = backend.write_register(PWM_SV_CHOP, 7).await;

                // Capture encoder count for the final calibration result.
                let encoder_count = backend.read_encoder_count().await;
                let result = AdcCalResult {
                    adc_i0: offset_i0,
                    adc_i1: offset_i1,
                    encoder_count,
                };
                *adc_cal = Some(result.clone());

                // Route based on sequence.
                match sequence {
                    InnerSequence::Calibrate => {
                        // Skip coil/wiring diagnostics, but still run the
                        // encoder direction sweep used by commissioning so
                        // the rotor is moved before D-axis alignment.
                        *phase = InnerPhase::EncoderCheck {
                            state: EncoderCheckState::Setup,
                        };
                        StepResult::PhaseChanged(PhaseId::EncoderCheck)
                    }
                    InnerSequence::Selftest | InnerSequence::Commission => {
                        // Run full diagnostics.
                        *phase = InnerPhase::CoilCheck {
                            state: CoilCheckState::CoilA(CoilTestState::Setup { phase: 0 }),
                            coil_a: None,
                        };
                        StepResult::PhaseChanged(PhaseId::CoilCheck)
                    }
                }
            }
        }
    }

    // -------------------------------------------------------------------
    // Phase 2: Coil Connectivity Check
    // -------------------------------------------------------------------

    /// Advance coil check sub-state.
    ///
    /// Tests coil A then coil B via `UQ_UD_EXT` (MODE=8). Validates
    /// current exceeds `MOTOR_CURRENT_THRESHOLD` on the expected
    /// ADC channel. Carries observations forward for phase wiring
    /// check.
    async fn step_coil_check<B: CommissioningBackend>(
        phase: &mut InnerPhase,
        sequence: InnerSequence,
        selftest_results: &mut crate::selftest::SelftestResults,
        backend: &mut B,
    ) -> StepResult {
        use foci_tmc4671::registers::*;

        // Extract mutable fields from phase.
        let (state, coil_a_obs) = match phase {
            InnerPhase::CoilCheck { state, coil_a } => (state, coil_a),
            _ => return StepResult::Continue,
        };

        let is_coil_b = matches!(state, CoilCheckState::CoilB(_));

        match state {
            CoilCheckState::CoilA(coil_test) | CoilCheckState::CoilB(coil_test) => {
                match coil_test {
                    CoilTestState::Setup { phase: coil_phase } => {
                        let p = *coil_phase;
                        // Set PHI_E source to external.
                        if backend.write_register(PHI_E_SELECTION, 1).await.is_err() {
                            return Self::fail_inner(phase, CommissionError::Spi);
                        }
                        // Angle: 0 for coil A (D-axis), 16384 for coil B (Q-axis).
                        let phi_e = if p == 0 { 0u32 } else { 16384 };
                        if backend.write_register(PHI_E_EXT, phi_e).await.is_err() {
                            return Self::fail_inner(phase, CommissionError::Spi);
                        }
                        // Apply voltage.
                        let voltage = backend.excitation_voltage() as u32;
                        if backend
                            .write_register(UQ_UD_EXT, voltage & 0xFFFF)
                            .await
                            .is_err()
                        {
                            return Self::fail_inner(phase, CommissionError::Spi);
                        }
                        if backend
                            .write_register(MODE_RAMP_MODE_MOTION, 8)
                            .await
                            .is_err()
                        {
                            return Self::fail_inner(phase, CommissionError::Spi);
                        }
                        backend.set_enable_pin(true);

                        let deadline = backend.elapsed_us().wrapping_add(COIL_SETTLE_US);
                        *coil_test = CoilTestState::Settle {
                            phase: p,
                            deadline_us: deadline,
                        };
                        StepResult::Continue
                    }

                    CoilTestState::Settle {
                        phase: coil_phase,
                        deadline_us,
                    } => {
                        if backend.elapsed_us().wrapping_sub(*deadline_us) < 0x8000_0000 {
                            *coil_test = CoilTestState::Read { phase: *coil_phase };
                        }
                        StepResult::Continue
                    }

                    CoilTestState::Read { phase: coil_phase } => {
                        let p = *coil_phase;
                        let raw = match backend.read_register(ADC_IWY_IUX).await {
                            Ok(v) => v,
                            Err(_) => {
                                return Self::fail_inner(phase, CommissionError::Spi);
                            }
                        };

                        // Stop motor and disable.
                        let _ = backend.write_register(MODE_RAMP_MODE_MOTION, 0).await;
                        let _ = backend.write_register(UQ_UD_EXT, 0).await;
                        backend.set_enable_pin(false);

                        let i_ux = (raw & 0xFFFF) as i16;
                        let i_wy = ((raw >> 16) & 0xFFFF) as i16;

                        let obs = if p == 0 {
                            CoilObservation {
                                expected: i_ux,
                                other: i_wy,
                            }
                        } else {
                            CoilObservation {
                                expected: i_wy,
                                other: i_ux,
                            }
                        };

                        // Validate coil presence.
                        if obs.expected.unsigned_abs() <= MOTOR_CURRENT_THRESHOLD {
                            if matches!(sequence, InnerSequence::Selftest) {
                                let stage = if is_coil_b { 3 } else { 2 };
                                selftest_results
                                    .set(stage, crate::selftest::SelftestStageValue::fail(0));
                            }
                            return Self::fail_inner(phase, CommissionError::CoilFault);
                        }

                        let deadline = backend.elapsed_us().wrapping_add(COIL_COOLDOWN_US);
                        *coil_test = CoilTestState::Cooldown {
                            phase: p,
                            deadline_us: deadline,
                        };

                        // Store observation.
                        if !is_coil_b {
                            if matches!(sequence, InnerSequence::Selftest) {
                                selftest_results.set(
                                    2,
                                    crate::selftest::SelftestStageValue::coil_current(obs.expected),
                                );
                            }
                            *coil_a_obs = Some(obs);
                        } else {
                            if matches!(sequence, InnerSequence::Selftest) {
                                selftest_results.set(
                                    3,
                                    crate::selftest::SelftestStageValue::coil_current(obs.expected),
                                );
                            }
                            // Both coils tested. Transition to PhaseWiringCheck.
                            let a = coil_a_obs.unwrap_or(CoilObservation {
                                expected: 0,
                                other: 0,
                            });
                            *phase = InnerPhase::PhaseWiringCheck {
                                coil_a: a,
                                coil_b: obs,
                            };
                            return StepResult::PhaseChanged(PhaseId::PhaseWiringCheck);
                        }
                        StepResult::Continue
                    }

                    CoilTestState::Cooldown {
                        phase: _coil_phase,
                        deadline_us,
                    } => {
                        if backend.elapsed_us().wrapping_sub(*deadline_us) < 0x8000_0000 {
                            // Coil A done, move to coil B.
                            *state = CoilCheckState::CoilB(CoilTestState::Setup { phase: 1 });
                        }
                        StepResult::Continue
                    }
                }
            }
        }
    }

    // -------------------------------------------------------------------
    // Phase 3: Phase Wiring Check
    // -------------------------------------------------------------------

    /// Validate phase wiring from coil test observations.
    ///
    /// Current on the expected ADC channel must exceed current on
    /// the other channel. If reversed, the motor phases are swapped.
    fn step_phase_wiring(
        phase: &mut InnerPhase,
        sequence: InnerSequence,
        selftest_results: &mut crate::selftest::SelftestResults,
        coil_a: CoilObservation,
        coil_b: CoilObservation,
    ) -> StepResult {
        // Phase wiring check: expected channel must dominate.
        if coil_a.other.unsigned_abs() >= coil_a.expected.unsigned_abs() {
            if matches!(sequence, InnerSequence::Selftest) {
                selftest_results.set(4, crate::selftest::SelftestStageValue::fail(0));
            }
            return Self::fail_inner(phase, CommissionError::PhaseFault);
        }
        if coil_b.other.unsigned_abs() >= coil_b.expected.unsigned_abs() {
            if matches!(sequence, InnerSequence::Selftest) {
                selftest_results.set(4, crate::selftest::SelftestStageValue::fail(0));
            }
            return Self::fail_inner(phase, CommissionError::PhaseFault);
        }

        if matches!(sequence, InnerSequence::Selftest) {
            selftest_results.set(
                4,
                crate::selftest::SelftestStageValue {
                    status: 0,
                    value: 0,
                },
            );
        }

        // Transition to encoder check.
        *phase = InnerPhase::EncoderCheck {
            state: EncoderCheckState::Setup,
        };
        StepResult::PhaseChanged(PhaseId::EncoderCheck)
    }

    // -------------------------------------------------------------------
    // Phase 4: Encoder Presence and Direction
    // -------------------------------------------------------------------

    /// Start current-regulated open-loop encoder movement.
    ///
    /// Direct voltage mode (`MODE=8` + `UQ_UD_EXT`) is useful for coil
    /// current measurements, but it can fail to move a loaded rotor far
    /// enough for encoder validation. The dev open-loop path uses this
    /// current-regulated pattern: external PHI_E, a D-axis flux target,
    /// current-loop gains, and torque/flux current limits.
    async fn start_encoder_current_drive<B: CommissioningBackend>(
        backend: &mut B,
        phi_e: u32,
    ) -> Result<(), CommissionError> {
        use foci_tmc4671::registers::*;

        let current_limit = backend.current_limit() & 0xFFFF;
        if current_limit == 0 {
            return Err(CommissionError::NoCurrent);
        }

        let target = (current_limit / 2)
            .clamp(ENCODER_MOVE_CURRENT_MIN, ENCODER_MOVE_CURRENT_MAX)
            .min(current_limit) as i16;
        let defaults = crate::pid::DEFAULT_INNER_GAINS;
        let flux_reg = (defaults.flux_p as u32) << 16 | defaults.flux_i as u32;
        let torque_reg = (defaults.torque_p as u32) << 16 | defaults.torque_i as u32;
        let target_reg: u32 = PidTorqueFluxTarget::new()
            .with_pid_flux_target(target)
            .with_pid_torque_target(0)
            .into();

        backend.set_enable_pin(false);
        backend
            .write_register(MODE_RAMP_MODE_MOTION, 0)
            .await
            .map_err(|_| CommissionError::Spi)?;
        backend
            .write_register(PID_TORQUE_FLUX_TARGET, 0)
            .await
            .map_err(|_| CommissionError::Spi)?;
        backend
            .write_register(UQ_UD_EXT, 0)
            .await
            .map_err(|_| CommissionError::Spi)?;
        backend
            .write_register(PID_FLUX_P_FLUX_I, flux_reg)
            .await
            .map_err(|_| CommissionError::Spi)?;
        backend
            .write_register(PID_TORQUE_P_TORQUE_I, torque_reg)
            .await
            .map_err(|_| CommissionError::Spi)?;
        backend
            .write_register(PID_TORQUE_FLUX_LIMITS, backend.current_limit())
            .await
            .map_err(|_| CommissionError::Spi)?;
        backend
            .write_register(PIDOUT_UQ_UD_LIMITS, ENCODER_ALIGN_VOLTAGE_LIMIT)
            .await
            .map_err(|_| CommissionError::Spi)?;
        backend
            .write_register(PHI_E_SELECTION, 1)
            .await
            .map_err(|_| CommissionError::Spi)?;
        backend
            .write_register(PHI_E_EXT, phi_e)
            .await
            .map_err(|_| CommissionError::Spi)?;
        backend
            .write_register(PID_TORQUE_FLUX_TARGET, target_reg)
            .await
            .map_err(|_| CommissionError::Spi)?;
        backend
            .write_register(MODE_RAMP_MODE_MOTION, 0x8000_0001)
            .await
            .map_err(|_| CommissionError::Spi)?;
        backend.set_enable_pin(true);
        Ok(())
    }

    /// Stop current-regulated open-loop encoder movement.
    async fn stop_encoder_current_drive<B: CommissioningBackend>(
        backend: &mut B,
    ) -> Result<(), CommissionError> {
        use foci_tmc4671::registers::*;

        backend
            .write_register(PID_TORQUE_FLUX_TARGET, 0)
            .await
            .map_err(|_| CommissionError::Spi)?;
        backend
            .write_register(UQ_UD_EXT, 0)
            .await
            .map_err(|_| CommissionError::Spi)?;
        backend
            .write_register(MODE_RAMP_MODE_MOTION, 0)
            .await
            .map_err(|_| CommissionError::Spi)?;
        backend.set_enable_pin(false);
        Ok(())
    }

    /// Advance encoder check sub-state.
    ///
    /// Sweeps to a displaced electrical angle via PHI_E_EXT in
    /// resumable sub-steps, then checks whether the encoder count changed.
    async fn step_encoder_check<B: CommissioningBackend>(
        phase: &mut InnerPhase,
        sequence: InnerSequence,
        selftest_results: &mut crate::selftest::SelftestResults,
        backend: &mut B,
    ) -> StepResult {
        use foci_tmc4671::registers::*;

        let state = match phase {
            InnerPhase::EncoderCheck { state } => state,
            _ => return StepResult::Continue,
        };

        match state {
            EncoderCheckState::Setup => {
                let count_start = backend.read_encoder_count().await as i32;

                if let Err(error) = Self::start_encoder_current_drive(backend, 0).await {
                    return Self::fail_inner(phase, error);
                }

                let deadline = backend.elapsed_us().wrapping_add(ENCODER_DIR_STEP_US);
                *state = EncoderCheckState::Sweep {
                    step: 0,
                    deadline_us: deadline,
                    count_start,
                };
                StepResult::Continue
            }

            EncoderCheckState::Sweep {
                step,
                deadline_us,
                count_start,
            } => {
                if backend.elapsed_us().wrapping_sub(*deadline_us) >= 0x8000_0000 {
                    // Not yet time for next step.
                    return StepResult::Continue;
                }

                *step += 1;
                if *step > ENCODER_DIR_STEPS {
                    let cs = *count_start;
                    *state = EncoderCheckState::Finalize { count_start: cs };
                    return StepResult::Continue;
                }

                // Advance PHI_E_EXT toward a non-zero final angle. Ending
                // at 65536/0 can return the rotor to the start angle and
                // make a working encoder look stationary.
                let phi = (ENCODER_DIR_TARGET_PHI as u64 * *step as u64 / ENCODER_DIR_STEPS as u64)
                    as u32;
                let _ = backend.write_register(PHI_E_EXT, phi).await;

                *deadline_us = backend.elapsed_us().wrapping_add(ENCODER_DIR_STEP_US);
                StepResult::Continue
            }

            EncoderCheckState::Finalize { count_start } => {
                let count_end = backend.read_encoder_count().await as i32;

                // Stop and disable (always, regardless of result).
                let _ = Self::stop_encoder_current_drive(backend).await;

                let delta = count_end.wrapping_sub(*count_start);
                defmt::info!(
                    "encoder-dir: start={} end={} delta={}",
                    *count_start,
                    count_end,
                    delta
                );
                if delta == 0 {
                    defmt::warn!(
                        "encoder-dir FAIL: start={} end={} delta=0",
                        *count_start,
                        count_end
                    );
                    if matches!(sequence, InnerSequence::Selftest) {
                        selftest_results.set(5, crate::selftest::SelftestStageValue::fail(0));
                    }
                    return Self::fail_inner(phase, CommissionError::EncoderFault);
                }

                // All sequences that reach EncoderCheck proceed to
                // EncoderAlign. Use the final sweep count as the alignment
                // baseline because the rotor is still held at the displaced
                // sweep angle; EncoderAlign::Setup will drive back to PHI=0.
                if matches!(sequence, InnerSequence::Selftest) {
                    // Raw encoder delta magnitude; direction captured in stage 6.
                    selftest_results.set(
                        5,
                        crate::selftest::SelftestStageValue {
                            status: 0,
                            value: delta.unsigned_abs(),
                        },
                    );
                    // Stage 6: encoder direction (0=forward, 1=reversed).
                    // Reversed when the encoder count decreases as PHI_E increases.
                    let reversed = delta < 0;
                    selftest_results.set(
                        6,
                        crate::selftest::SelftestStageValue {
                            status: 0,
                            value: if reversed { 1 } else { 0 },
                        },
                    );
                }

                // Direction detected. Transition to EncoderAlign, which
                // locks PHI_E_SELECTION to ABN at a known D-axis zero before
                // any later closed-loop or electrical-ID work.
                // Direction value (0=forward, 1=reversed) is carried as
                // commissioning-local data; persistence is outside this plan.
                *phase = InnerPhase::EncoderAlign {
                    state: EncoderAlignState::Setup {
                        count_before: count_end as u32,
                    },
                };
                StepResult::PhaseChanged(PhaseId::EncoderAlign)
            }
        }
    }

    // -------------------------------------------------------------------
    // Encoder D-axis Alignment
    // -------------------------------------------------------------------

    /// Advance encoder D-axis alignment sub-state.
    ///
    /// This is the non-blocking engine equivalent of the old blocking
    /// alignment helper. It applies current-regulated open-loop drive,
    /// verifies the encoder moved and settled, zeros ABN count/offset
    /// while still driving, then locks PHI_E to ABN before stopping the
    /// drive.
    async fn step_encoder_align<B: CommissioningBackend>(
        phase: &mut InnerPhase,
        sequence: InnerSequence,
        adc_cal: &mut Option<AdcCalResult>,
        _selftest_results: &mut crate::selftest::SelftestResults,
        backend: &mut B,
    ) -> StepResult {
        use foci_tmc4671::registers::*;

        let state = match phase {
            InnerPhase::EncoderAlign { state } => state,
            _ => return StepResult::Continue,
        };

        match state {
            EncoderAlignState::Setup { count_before } => {
                if backend.current_limit() == 0 || backend.step_width() == 0 {
                    return Self::fail_inner(phase, CommissionError::NoCurrent);
                }

                if let Err(error) = Self::start_encoder_current_drive(backend, 0).await {
                    return Self::fail_inner(phase, error);
                }

                let phi_ok = backend
                    .read_register(PHI_E_SELECTION)
                    .await
                    .is_ok_and(|v| v == 1);
                let mode_ok = backend
                    .read_register(MODE_RAMP_MODE_MOTION)
                    .await
                    .is_ok_and(|v| v == 0x8000_0001);
                if !phi_ok || !mode_ok {
                    return Self::fail_inner(phase, CommissionError::Spi);
                }

                backend.set_enable_pin(true);

                let step_width = backend.step_width().max(1);
                let min_movement = (65536u32 / (20 * step_width)).max(2);
                let deadline = backend.elapsed_us().wrapping_add(ENCODER_ALIGN_SETTLE_US);
                *state = EncoderAlignState::Settle {
                    count_before: *count_before,
                    deadline_us: deadline,
                    min_movement,
                };
                StepResult::Continue
            }

            EncoderAlignState::Settle {
                count_before,
                deadline_us,
                min_movement,
            } => {
                if backend.elapsed_us().wrapping_sub(*deadline_us) < 0x8000_0000 {
                    *state = EncoderAlignState::StabilityCheck1 {
                        count_before: *count_before,
                        min_movement: *min_movement,
                    };
                }
                StepResult::Continue
            }

            EncoderAlignState::StabilityCheck1 {
                count_before,
                min_movement,
            } => {
                let count1 = match backend.read_register(ABN_DECODER_COUNT).await {
                    Ok(v) => v,
                    Err(_) => return Self::fail_inner(phase, CommissionError::Spi),
                };
                let deadline = backend
                    .elapsed_us()
                    .wrapping_add(ENCODER_ALIGN_STABILITY_US);
                *state = EncoderAlignState::StabilityCheck2 {
                    count_before: *count_before,
                    count1,
                    deadline_us: deadline,
                    min_movement: *min_movement,
                };
                StepResult::Continue
            }

            EncoderAlignState::StabilityCheck2 {
                count_before,
                count1,
                deadline_us,
                min_movement,
            } => {
                if backend.elapsed_us().wrapping_sub(*deadline_us) >= 0x8000_0000 {
                    return StepResult::Continue;
                }

                let count2 = match backend.read_register(ABN_DECODER_COUNT).await {
                    Ok(v) => v,
                    Err(_) => return Self::fail_inner(phase, CommissionError::Spi),
                };

                let movement1 = (*count1 as i32 - *count_before as i32).unsigned_abs();
                let movement2 = (count2 as i32 - *count_before as i32).unsigned_abs();
                let movement = movement1.max(movement2);
                let stable = (*count1 as i32 - count2 as i32).unsigned_abs()
                    <= ENCODER_ALIGN_STABILITY_COUNTS;
                if movement < *min_movement || !stable {
                    defmt::warn!(
                        "encoder-align FAIL: count_before={} count1={} count2={} \
                         movement={} min={} stable={}",
                        *count_before,
                        *count1,
                        count2,
                        movement,
                        *min_movement,
                        stable
                    );
                    let _ = Self::stop_encoder_current_drive(backend).await;
                    return Self::fail_inner(phase, CommissionError::EncoderFault);
                }

                *state = EncoderAlignState::Commit { old_count: *count1 };
                StepResult::Continue
            }

            EncoderAlignState::Commit { old_count } => {
                if backend.write_register(ABN_DECODER_COUNT, 0).await.is_err()
                    || backend
                        .write_register(ABN_DECODER_PHI_E_PHI_M_OFFSET, 0)
                        .await
                        .is_err()
                    || backend.write_register(PHI_E_SELECTION, 3).await.is_err()
                {
                    return Self::fail_inner(phase, CommissionError::Spi);
                }

                let phi_ok = backend
                    .read_register(PHI_E_SELECTION)
                    .await
                    .is_ok_and(|v| v == 3);
                if !phi_ok {
                    return Self::fail_inner(phase, CommissionError::Spi);
                }

                *state = EncoderAlignState::StopDrive {
                    old_count: *old_count,
                };
                StepResult::Continue
            }

            EncoderAlignState::StopDrive { old_count } => {
                if let Err(error) = Self::stop_encoder_current_drive(backend).await {
                    return Self::fail_inner(phase, error);
                }

                if let Some(result) = adc_cal.as_mut() {
                    result.encoder_count = *old_count;
                }

                match sequence {
                    InnerSequence::Calibrate => {
                        // Transition to ClosedLoopEntry with conservative
                        // default gains. The CalibrateDone handler in
                        // main.rs will overwrite current-loop gains with
                        // stored_gains (from prior commissioning) and
                        // re-apply filters from atomics.
                        let defaults = crate::pid::DEFAULT_INNER_GAINS;
                        let vel_limit = backend.operational_velocity_limit();
                        *phase = InnerPhase::ClosedLoopEntry {
                            state: ClosedLoopEntryState::WriteGains,
                            flux_p: defaults.flux_p,
                            flux_i: defaults.flux_i,
                            torque_p: defaults.torque_p,
                            torque_i: defaults.torque_i,
                            velocity_p: 1152, // 4.5 in Q8.8 (conservative default)
                            velocity_i: 0,
                            position_p: 640, // 2.5 in Q8.8 (conservative default)
                            position_i: 0,
                            velocity_limit: vel_limit,
                            velocity_filter_hz: 0,
                            torque_filter_hz: 0,
                            position_filter_hz: 0,
                            flux_filter_hz: 0,
                            inner_result: None,
                        };
                        StepResult::PhaseChanged(PhaseId::ClosedLoopEntry)
                    }
                    InnerSequence::Selftest | InnerSequence::Commission => {
                        let voltage = backend.excitation_voltage() as u32;
                        *phase = InnerPhase::ElectricalId {
                            state: ElectricalIdState::ResSetup { coil: 0 },
                            data: ElectricalIdData::new(voltage),
                        };
                        StepResult::PhaseChanged(PhaseId::ElectricalId)
                    }
                }
            }
        }
    }

    // -------------------------------------------------------------------
    // Phase 5: Electrical Identification
    // -------------------------------------------------------------------

    /// Advance electrical identification sub-state.
    ///
    /// Measures per-coil resistance (1000 samples at 160us in steady
    /// state), per-coil inductance (square-wave di/dt), and delay
    /// (transient capture with two-point fit). Assembles an
    /// `ElectricalModel` and validates before transitioning.
    async fn step_elec_id<B: CommissioningBackend>(
        phase: &mut InnerPhase,
        buf: &mut TransientBuffer,
        sequence: InnerSequence,
        selftest_results: &mut crate::selftest::SelftestResults,
        backend: &mut B,
    ) -> StepResult {
        use foci_tmc4671::registers::*;

        let (elec_id_state, data) = match phase {
            InnerPhase::ElectricalId { state, data } => (state, data),
            _ => return StepResult::Continue,
        };

        match elec_id_state {
            // ---------------------------------------------------------------
            // Resistance measurement (per coil)
            // ---------------------------------------------------------------
            ElectricalIdState::ResSetup { coil } => {
                let coil_idx = *coil;
                // Configure open-loop voltage mode.
                if backend.write_register(PHI_E_SELECTION, 1).await.is_err() {
                    return Self::fail_inner(phase, CommissionError::Spi);
                }
                let phi_e = if coil_idx == 0 { 0u32 } else { 16384 };
                if backend.write_register(PHI_E_EXT, phi_e).await.is_err() {
                    return Self::fail_inner(phase, CommissionError::Spi);
                }
                if backend
                    .write_register(UQ_UD_EXT, data.voltage & 0xFFFF)
                    .await
                    .is_err()
                {
                    return Self::fail_inner(phase, CommissionError::Spi);
                }
                if backend
                    .write_register(MODE_RAMP_MODE_MOTION, 8)
                    .await
                    .is_err()
                {
                    return Self::fail_inner(phase, CommissionError::Spi);
                }
                backend.set_enable_pin(true);

                let deadline = backend.elapsed_us().wrapping_add(RES_SETTLE_US);
                *elec_id_state = ElectricalIdState::ResSettle {
                    coil: coil_idx,
                    deadline_us: deadline,
                };
                StepResult::Continue
            }

            ElectricalIdState::ResSettle { coil, deadline_us } => {
                if backend.elapsed_us().wrapping_sub(*deadline_us) < 0x8000_0000 {
                    let next = backend.elapsed_us().wrapping_add(ELEC_SAMPLE_PERIOD_US);
                    *elec_id_state = ElectricalIdState::ResSample {
                        coil: *coil,
                        next_sample_us: next,
                    };
                }
                StepResult::Continue
            }

            ElectricalIdState::ResSample {
                coil,
                next_sample_us,
            } => {
                let coil_idx = *coil as usize;
                // Take as many samples as fit within this step's time budget.
                loop {
                    if data.res_sample_count[coil_idx] >= RES_SAMPLE_COUNT {
                        // All samples collected. Disable motor.
                        let _ = backend.write_register(MODE_RAMP_MODE_MOTION, 0).await;
                        let _ = backend.write_register(UQ_UD_EXT, 0).await;
                        backend.set_enable_pin(false);
                        let deadline = backend.elapsed_us().wrapping_add(ELEC_COOLDOWN_US);
                        *elec_id_state = ElectricalIdState::ResDisable {
                            coil: *coil,
                            deadline_us: deadline,
                        };
                        return StepResult::Continue;
                    }
                    // Wait for next sample deadline.
                    if backend.elapsed_us().wrapping_sub(*next_sample_us) >= 0x8000_0000 {
                        break; // Not yet time, yield.
                    }
                    // Take one sample.
                    match backend.read_register(PID_TORQUE_FLUX_ACTUAL).await {
                        Ok(actual) => {
                            let current = (actual & 0xFFFF) as i16;
                            data.res_current_sum[coil_idx] += current as i32;
                            data.res_sample_count[coil_idx] += 1;
                        }
                        Err(_) => {
                            return Self::fail_inner(phase, CommissionError::Spi);
                        }
                    }
                    *next_sample_us = next_sample_us.wrapping_add(ELEC_SAMPLE_PERIOD_US);
                }
                StepResult::Continue
            }

            ElectricalIdState::ResDisable { coil, deadline_us } => {
                if backend.elapsed_us().wrapping_sub(*deadline_us) >= 0x8000_0000 {
                    return StepResult::Continue; // Not yet.
                }
                let coil_idx = *coil;
                if coil_idx == 0 {
                    // Move to coil B resistance.
                    *elec_id_state = ElectricalIdState::ResSetup { coil: 1 };
                } else {
                    // Both resistance measurements done. Start inductance.
                    *elec_id_state = ElectricalIdState::IndSetup { coil: 0 };
                }
                StepResult::Continue
            }

            // ---------------------------------------------------------------
            // Inductance measurement (per coil, square-wave di/dt)
            // ---------------------------------------------------------------
            ElectricalIdState::IndSetup { coil } => {
                let coil_idx = *coil;
                if backend.write_register(PHI_E_SELECTION, 1).await.is_err() {
                    return Self::fail_inner(phase, CommissionError::Spi);
                }
                let phi_e = if coil_idx == 0 { 0u32 } else { 16384 };
                if backend.write_register(PHI_E_EXT, phi_e).await.is_err() {
                    return Self::fail_inner(phase, CommissionError::Spi);
                }
                // Start at zero voltage.
                if backend.write_register(UQ_UD_EXT, 0).await.is_err() {
                    return Self::fail_inner(phase, CommissionError::Spi);
                }
                if backend
                    .write_register(MODE_RAMP_MODE_MOTION, 8)
                    .await
                    .is_err()
                {
                    return Self::fail_inner(phase, CommissionError::Spi);
                }
                backend.set_enable_pin(true);

                let deadline = backend.elapsed_us().wrapping_add(DIDT_PRE_SETTLE_US);
                *elec_id_state = ElectricalIdState::IndSettle {
                    coil: coil_idx,
                    deadline_us: deadline,
                };
                StepResult::Continue
            }

            ElectricalIdState::IndSettle { coil, deadline_us } => {
                if backend.elapsed_us().wrapping_sub(*deadline_us) >= 0x8000_0000 {
                    return StepResult::Continue; // Not yet.
                }
                // Read initial current before toggling.
                let initial_current = match backend.read_register(PID_TORQUE_FLUX_ACTUAL).await {
                    Ok(actual) => (actual & 0xFFFF) as i16,
                    Err(_) => return Self::fail_inner(phase, CommissionError::Spi),
                };
                let total_cycles = 5000u32; // From TmcConfig::didt_cycles default
                let start = backend.elapsed_us();
                *elec_id_state = ElectricalIdState::IndToggle {
                    coil: *coil,
                    remaining_cycles: total_cycles,
                    prev_current: initial_current,
                    start_us: start,
                };
                StepResult::Continue
            }

            ElectricalIdState::IndToggle {
                coil,
                remaining_cycles,
                prev_current,
                start_us,
            } => {
                let coil_idx = *coil as usize;
                let voltage = data.voltage;
                let v_pos = voltage & 0xFFFF;
                let v_neg = (-(voltage as i32) as u32) & 0xFFFF;

                let mut cycles_this_step = 0u32;

                while *remaining_cycles > 0 && cycles_this_step < DIDT_CYCLES_PER_STEP {
                    // Determine polarity based on which cycle we're on.
                    let total_done = 5000u32.saturating_sub(*remaining_cycles);
                    let v = if total_done % 2 == 0 { v_pos } else { v_neg };
                    let _ = backend.write_register(UQ_UD_EXT, v).await;

                    if let Ok(actual) = backend.read_register(PID_TORQUE_FLUX_ACTUAL).await {
                        let current = (actual & 0xFFFF) as i16;
                        let delta = (current as i32 - *prev_current as i32).unsigned_abs();
                        data.ind_sum_delta[coil_idx] += delta as u64;
                        data.ind_count[coil_idx] += 1;
                        *prev_current = current;
                    }
                    *remaining_cycles -= 1;
                    cycles_this_step += 1;
                }

                if *remaining_cycles == 0 {
                    // All cycles done. Record elapsed time.
                    // Duration is bounded by DIDT_CYCLES * cycle time
                    // (~200ms max), well within u32 us range.
                    let end = backend.elapsed_us();
                    data.ind_elapsed_us[coil_idx] = (end - *start_us) as u32;

                    // Disable motor.
                    let _ = backend.write_register(UQ_UD_EXT, 0).await;
                    let _ = backend.write_register(MODE_RAMP_MODE_MOTION, 0).await;
                    backend.set_enable_pin(false);

                    let deadline = backend.elapsed_us().wrapping_add(ELEC_COOLDOWN_US);
                    *elec_id_state = ElectricalIdState::IndDisable {
                        coil: *coil,
                        deadline_us: deadline,
                    };
                }
                StepResult::Continue
            }

            ElectricalIdState::IndDisable { coil, deadline_us } => {
                if backend.elapsed_us().wrapping_sub(*deadline_us) >= 0x8000_0000 {
                    return StepResult::Continue; // Not yet.
                }
                let coil_idx = *coil;
                if coil_idx == 0 {
                    // Move to coil B inductance.
                    *elec_id_state = ElectricalIdState::IndSetup { coil: 1 };
                } else {
                    // Both inductance measurements done. Start delay capture.
                    *elec_id_state = ElectricalIdState::DelaySetup;
                }
                StepResult::Continue
            }

            // ---------------------------------------------------------------
            // Delay identification (transient capture)
            // ---------------------------------------------------------------
            ElectricalIdState::DelaySetup => {
                // Configure for step response on coil A (D-axis).
                if backend.write_register(PHI_E_SELECTION, 1).await.is_err()
                    || backend.write_register(PHI_E_EXT, 0).await.is_err()
                    || backend.write_register(UQ_UD_EXT, 0).await.is_err()
                    || backend
                        .write_register(MODE_RAMP_MODE_MOTION, 8)
                        .await
                        .is_err()
                {
                    return Self::fail_inner(phase, CommissionError::Spi);
                }
                backend.set_enable_pin(true);

                // Reset transient buffer for 160us sampling.
                buf.reset(ELEC_SAMPLE_PERIOD_US as u16);

                let deadline = backend.elapsed_us().wrapping_add(DIDT_PRE_SETTLE_US);
                *elec_id_state = ElectricalIdState::DelaySettle {
                    deadline_us: deadline,
                };
                StepResult::Continue
            }

            ElectricalIdState::DelaySettle { deadline_us } => {
                if backend.elapsed_us().wrapping_sub(*deadline_us) >= 0x8000_0000 {
                    return StepResult::Continue; // Not yet.
                }
                *elec_id_state = ElectricalIdState::DelayStep;
                StepResult::Continue
            }

            ElectricalIdState::DelayStep => {
                // Read baseline current (should be ~0 with UQ_UD_EXT=0).
                let baseline = match backend.read_register(PID_TORQUE_FLUX_ACTUAL).await {
                    Ok(actual) => (actual & 0xFFFF) as i16,
                    Err(_) => return Self::fail_inner(phase, CommissionError::Spi),
                };

                // Apply voltage step.
                if backend
                    .write_register(UQ_UD_EXT, data.voltage & 0xFFFF)
                    .await
                    .is_err()
                {
                    return Self::fail_inner(phase, CommissionError::Spi);
                }

                let next = backend.elapsed_us().wrapping_add(ELEC_SAMPLE_PERIOD_US);
                *elec_id_state = ElectricalIdState::DelaySample {
                    baseline,
                    next_sample_us: next,
                };
                StepResult::Continue
            }

            ElectricalIdState::DelaySample {
                baseline,
                next_sample_us,
            } => {
                // Capture samples into TransientBuffer at 160us intervals.
                loop {
                    if buf.len() >= DELAY_SAMPLE_COUNT || buf.is_full() {
                        // Capture complete. Disable motor.
                        let _ = backend.write_register(MODE_RAMP_MODE_MOTION, 0).await;
                        let _ = backend.write_register(UQ_UD_EXT, 0).await;
                        backend.set_enable_pin(false);
                        let deadline = backend.elapsed_us().wrapping_add(ELEC_COOLDOWN_US);
                        *elec_id_state = ElectricalIdState::DelayDisable {
                            deadline_us: deadline,
                        };
                        return StepResult::Continue;
                    }
                    if backend.elapsed_us().wrapping_sub(*next_sample_us) >= 0x8000_0000 {
                        break; // Not yet time, yield.
                    }
                    match backend.read_register(PID_TORQUE_FLUX_ACTUAL).await {
                        Ok(actual) => {
                            // Store absolute current relative to baseline.
                            let current = (actual & 0xFFFF) as i16;
                            let relative = (current as i32 - *baseline as i32).unsigned_abs();
                            buf.push(relative);
                        }
                        Err(_) => {
                            return Self::fail_inner(phase, CommissionError::Spi);
                        }
                    }
                    *next_sample_us = next_sample_us.wrapping_add(ELEC_SAMPLE_PERIOD_US);
                }
                StepResult::Continue
            }

            ElectricalIdState::DelayDisable { deadline_us } => {
                if backend.elapsed_us().wrapping_sub(*deadline_us) >= 0x8000_0000 {
                    return StepResult::Continue; // Not yet.
                }
                *elec_id_state = ElectricalIdState::Validate;
                StepResult::Continue
            }

            // ---------------------------------------------------------------
            // Model assembly and validation
            // ---------------------------------------------------------------
            ElectricalIdState::Validate => {
                let voltage = data.voltage;

                // --- Resistance ---
                // Per-coil: R_int = V / |I_avg|
                let mut r_int_per_coil = [0u32; 2];
                let mut r_mohm_per_coil = [0u32; 2];
                for coil_idx in 0..2 {
                    let count = data.res_sample_count[coil_idx] as i32;
                    if count == 0 {
                        return Self::fail_inner(phase, CommissionError::ElectricalIdFailed);
                    }
                    let avg_current = (data.res_current_sum[coil_idx] / count) as i16;
                    let abs_current = avg_current.unsigned_abs() as u32;
                    if abs_current == 0 {
                        return Self::fail_inner(phase, CommissionError::ElectricalIdFailed);
                    }
                    r_int_per_coil[coil_idx] = voltage / abs_current;
                    r_mohm_per_coil[coil_idx] = (voltage as u64 * 1000 / abs_current as u64) as u32;
                }
                // Average of both coils.
                let r_int = (r_int_per_coil[0] + r_int_per_coil[1]) / 2;
                let r_mohm = (r_mohm_per_coil[0] + r_mohm_per_coil[1]) / 2;
                if r_int == 0 {
                    return Self::fail_inner(phase, CommissionError::ElectricalIdFailed);
                }

                // --- Inductance (from di/dt) ---
                // L_int = V * total_time_s / sum_delta
                // tau_e_us = L_int / R_int * 1e6
                // Since L_int = V * elapsed_us * 1e-6 / (sum_delta / count):
                //   L_int = V * elapsed_us * count / (sum_delta * 1e6)
                // And tau_e_us = L_int / R_int * 1e6:
                //   tau_e_us = V * elapsed_us * count / (sum_delta * R_int)
                let mut tau_per_coil = [0u32; 2];
                for (tau_result, ((count, sum_delta), elapsed)) in tau_per_coil
                    .iter_mut()
                    .zip(data.ind_count.iter().zip(data.ind_sum_delta.iter()))
                    .zip(data.ind_elapsed_us.iter())
                    .map(|((t, (c, s)), e)| (t, ((c, s), e)))
                {
                    if *count == 0 || *sum_delta == 0 || *elapsed == 0 {
                        continue;
                    }
                    let avg_delta = (*sum_delta / *count as u64) as u32;
                    if avg_delta < DIDT_MIN_DELTA {
                        continue;
                    }
                    // dt_per_cycle_us = elapsed / count
                    // L_int = V * dt_per_cycle_us * 1e-6 / avg_delta
                    // tau_us = L_int / R_int * 1e6 = V * dt_per_cycle_us / (avg_delta * R_int)
                    let dt_per_cycle_us = *elapsed / *count;
                    *tau_result = (voltage as u64 * dt_per_cycle_us as u64
                        / (avg_delta as u64 * r_int as u64))
                        as u32;
                }

                // Use the valid coil(s) for tau.
                let tau_e_us = match (tau_per_coil[0] > 0, tau_per_coil[1] > 0) {
                    (true, true) => (tau_per_coil[0] + tau_per_coil[1]) / 2,
                    (true, false) => tau_per_coil[0],
                    (false, true) => tau_per_coil[1],
                    (false, false) => {
                        return Self::fail_inner(phase, CommissionError::ElectricalIdFailed);
                    }
                };

                // L_int from tau: L_int = R_int * tau_e_us (internal units,
                // such that tau_e_us = L_int / R_int holds directly).
                let l_int = r_int * tau_e_us;

                // Physical L: L_uh = R_mohm * tau_e_us / 1000
                // (since L = R*tau; mohm*us = nH; nH/1000 = uH)
                let l_uh = r_mohm as u64 * tau_e_us as u64 / 1000;

                if l_int == 0 || l_uh == 0 {
                    return Self::fail_inner(phase, CommissionError::ElectricalIdFailed);
                }

                // --- Delay identification (from transient buffer) ---
                let samples = buf.samples();
                let sample_count = samples.len();

                // Find steady-state from last 10 samples.
                let steady_state = if sample_count >= 20 {
                    let tail_start = sample_count - 10;
                    let tail_sum: u32 = samples[tail_start..].iter().sum();
                    tail_sum / 10
                } else if !samples.is_empty() {
                    // Short capture: use last sample as approximation.
                    samples[sample_count - 1]
                } else {
                    return Self::fail_inner(phase, CommissionError::ElectricalIdFailed);
                };

                // Onset: first sample above noise threshold.
                // Result values are bounded by samples.len() * sample period
                // (< 256 * 160us = 41ms), well within u32 us range.
                let onset_threshold = DELAY_ONSET_THRESHOLD as u32;
                let mut theta_e_us = 0u32;
                for (i, &sample) in samples.iter().enumerate() {
                    if sample > onset_threshold {
                        theta_e_us = (i as u64 * ELEC_SAMPLE_PERIOD_US) as u32;
                        break;
                    }
                }

                // 63% rise: first sample >= 0.632 * steady_state.
                let rise_threshold = (steady_state as u64 * RISE_63_PCT_Q10 as u64 / 1024) as u32;
                let mut tau_crosscheck_us = 0u32;
                for (i, &sample) in samples.iter().enumerate() {
                    if sample >= rise_threshold {
                        tau_crosscheck_us = (i as u64 * ELEC_SAMPLE_PERIOD_US) as u32;
                        break;
                    }
                }

                // Default theta_e to at least one sample period if zero.
                if theta_e_us == 0 {
                    theta_e_us = ELEC_SAMPLE_PERIOD_US as u32;
                }

                // --- Assemble ElectricalModel ---
                let model = ElectricalModel {
                    r_int,
                    l_int,
                    tau_e_us,
                    theta_e_us,
                    r_mohm,
                    l_uh: l_uh as u32,
                    tau_e_crosscheck_us: tau_crosscheck_us,
                };

                match sequence {
                    InnerSequence::Selftest => {
                        selftest_results.set(
                            7,
                            crate::selftest::SelftestStageValue {
                                status: 0,
                                value: r_mohm,
                            },
                        );
                        selftest_results.set(
                            8,
                            crate::selftest::SelftestStageValue {
                                status: 0,
                                value: l_uh as u32,
                            },
                        );
                        // Full safe-state restore matching restore_safe_state().
                        backend.set_enable_pin(false);
                        let _ = backend.write_register(MODE_RAMP_MODE_MOTION, 0).await;
                        let _ = backend.write_register(PID_TORQUE_FLUX_TARGET, 0).await;
                        let _ = backend.write_register(UQ_UD_EXT, 0).await;
                        let _ = backend.write_register(PWM_SV_CHOP, 0).await;
                        let _ = backend
                            .write_register(foci_tmc4671::registers::PID_VELOCITY_OFFSET, 0)
                            .await;
                        StepResult::SelftestDone
                    }
                    InnerSequence::Commission => {
                        // Transition to CurrentTune carrying the model.
                        *phase = InnerPhase::CurrentTune { model };
                        StepResult::PhaseChanged(PhaseId::CurrentTune)
                    }
                    InnerSequence::Calibrate => {
                        // Calibrate never reaches electrical ID.
                        unreachable!()
                    }
                }
            }
        }
    }

    // -------------------------------------------------------------------
    // Phase 6: Current-Loop SIMC Synthesis
    // -------------------------------------------------------------------

    /// Thin phase: read the electrical model, compute SIMC gains, and
    /// transition immediately to validation.
    async fn step_current_tune<B: CommissioningBackend>(
        phase: &mut InnerPhase,
        profile: CommissionProfile,
        backend: &mut B,
    ) -> StepResult {
        let model = match phase {
            InnerPhase::CurrentTune { model } => *model,
            _ => return StepResult::Continue,
        };

        let config = backend.tmc_config();
        let gains = crate::autotune::simc_current_gains(&model, profile, config);

        defmt::info!(
            "simc: R={} L={} tau_e={}us theta_e={}us lambda={}us → P={} I={} bw={}Hz",
            model.r_int,
            model.l_int,
            model.tau_e_us,
            model.theta_e_us,
            gains.lambda_us,
            gains.flux_p,
            gains.flux_i,
            gains.effective_bandwidth_hz,
        );

        // Compute step amplitude: 15% of run_current.
        let current_limit = backend.current_limit() & 0xFFFF;
        let step_amplitude = if current_limit > 0 {
            (current_limit * VALIDATION_STEP_PERCENT / 100).max(1) as u16
        } else {
            // No current limit configured — cannot validate.
            return Self::fail_inner(phase, CommissionError::CurrentValidationFailed);
        };

        *phase = InnerPhase::CurrentValidate {
            state: CurrentValidateState::WriteGains,
            data: CurrentValidateData {
                model,
                gains,
                attempt: 0,
                step_amplitude,
                forward_metrics: None,
                reverse_metrics: None,
                final_status: CommissionStatus::Accepted,
                saturation_count: 0,
                pidout_limit: {
                    // Read PIDOUT_UQ_UD_LIMITS for saturation detection.
                    backend
                        .read_register(foci_tmc4671::registers::PIDOUT_UQ_UD_LIMITS)
                        .await
                        .unwrap_or(0)
                        & 0xFFFF
                },
            },
        };
        StepResult::PhaseChanged(PhaseId::CurrentValidate)
    }

    // -------------------------------------------------------------------
    // Phase 7: Current-Loop Validation
    // -------------------------------------------------------------------

    /// Advance current-loop validation sub-state.
    ///
    /// Writes SIMC gains, runs bidirectional step-response captures,
    /// classifies the response, retries once if marginal, and falls
    /// back to conservative defaults on failure.
    async fn step_current_validate<B: CommissioningBackend>(
        phase: &mut InnerPhase,
        buf: &mut TransientBuffer,
        _profile: CommissionProfile,
        sequence: InnerSequence,
        backend: &mut B,
    ) -> StepResult {
        use foci_tmc4671::registers::*;

        let (validate_state, data) = match phase {
            InnerPhase::CurrentValidate { state, data } => (state, data),
            _ => return StepResult::Continue,
        };

        match validate_state {
            CurrentValidateState::WriteGains => {
                // Write gains to TMC4671 PI registers.
                // PID_FLUX_P_FLUX_I: P in [31:16], I in [15:0]
                let flux_reg = ((data.gains.flux_p as u32) << 16) | data.gains.flux_i as u32;
                let torque_reg = ((data.gains.torque_p as u32) << 16) | data.gains.torque_i as u32;

                if backend
                    .write_register(PID_FLUX_P_FLUX_I, flux_reg)
                    .await
                    .is_err()
                    || backend
                        .write_register(PID_TORQUE_P_TORQUE_I, torque_reg)
                        .await
                        .is_err()
                    || backend
                        .write_register(PIDOUT_UQ_UD_LIMITS, CURRENT_VALIDATE_PIDOUT_LIMIT)
                        .await
                        .is_err()
                    || Self::clear_current_integrators(backend).await.is_err()
                {
                    return Self::fail_inner(phase, CommissionError::Spi);
                }
                data.pidout_limit = CURRENT_VALIDATE_PIDOUT_LIMIT;

                // Clear any previous metrics for this attempt.
                data.forward_metrics = None;
                data.reverse_metrics = None;
                data.saturation_count = 0;

                *validate_state = CurrentValidateState::StepSetup {
                    direction: CaptureDirection::Forward,
                };
                StepResult::Continue
            }

            CurrentValidateState::StepSetup { direction } => {
                let dir = *direction;
                // Switch to torque mode using the same advanced PI
                // controller structure as the tuned current loop.
                if backend
                    .write_register(MODE_RAMP_MODE_MOTION, CURRENT_TORQUE_MODE)
                    .await
                    .is_err()
                {
                    return Self::fail_inner(phase, CommissionError::Spi);
                }
                backend.set_enable_pin(true);

                if Self::clear_current_integrators(backend).await.is_err() {
                    return Self::fail_inner(phase, CommissionError::Spi);
                }

                // Apply current step via PID_TORQUE_FLUX_TARGET.
                // Torque target in [31:16], flux target in [15:0].
                let torque_val = match dir {
                    CaptureDirection::Forward => data.step_amplitude as u32,
                    CaptureDirection::Reverse => (-(data.step_amplitude as i16) as u16) as u32,
                };
                let target = torque_val << 16;
                if backend
                    .write_register(PID_TORQUE_FLUX_TARGET, target)
                    .await
                    .is_err()
                {
                    return Self::fail_inner(phase, CommissionError::Spi);
                }

                // Reset buffer for this capture.
                buf.reset(ELEC_SAMPLE_PERIOD_US as u16);

                let next = backend.elapsed_us().wrapping_add(ELEC_SAMPLE_PERIOD_US);
                *validate_state = CurrentValidateState::StepCapture {
                    direction: dir,
                    next_sample_us: next,
                };
                StepResult::Continue
            }

            CurrentValidateState::StepSettle {
                direction,
                deadline_us,
            } => {
                if backend.elapsed_us().wrapping_sub(*deadline_us) >= 0x8000_0000 {
                    return StepResult::Continue;
                }
                // Settling done. Continue based on direction.
                let dir = *direction;
                match dir {
                    CaptureDirection::Forward => {
                        // Move to reverse capture.
                        *validate_state = CurrentValidateState::StepSetup {
                            direction: CaptureDirection::Reverse,
                        };
                    }
                    CaptureDirection::Reverse => {
                        // Both captures done. Classify.
                        *validate_state = CurrentValidateState::Classify;
                    }
                }
                StepResult::Continue
            }

            CurrentValidateState::StepCapture {
                direction,
                next_sample_us,
            } => {
                let dir = *direction;
                // Capture samples at 160us intervals.
                loop {
                    if buf.is_full() || buf.len() >= DELAY_SAMPLE_COUNT {
                        // Capture complete. Extract metrics.
                        // Saturation is counted from the PID output (voltage),
                        // not from measured current, so disable the helper's
                        // sample-level saturation check.
                        let target = data.step_amplitude as u32;
                        let metrics = crate::autotune::extract_step_metrics(buf, target, 0);

                        match dir {
                            CaptureDirection::Forward => {
                                data.forward_metrics = Some(metrics);
                            }
                            CaptureDirection::Reverse => {
                                data.reverse_metrics = Some(metrics);
                            }
                        }

                        // Return to zero target and clear accumulated current
                        // error so the next capture starts from a defined
                        // controller state.
                        if backend
                            .write_register(PID_TORQUE_FLUX_TARGET, 0)
                            .await
                            .is_err()
                            || Self::clear_current_integrators(backend).await.is_err()
                        {
                            return Self::fail_inner(phase, CommissionError::Spi);
                        }
                        let deadline = backend.elapsed_us().wrapping_add(VALIDATION_SETTLE_US);
                        *validate_state = CurrentValidateState::StepSettle {
                            direction: dir,
                            deadline_us: deadline,
                        };
                        return StepResult::Continue;
                    }
                    if backend.elapsed_us().wrapping_sub(*next_sample_us) >= 0x8000_0000 {
                        break; // Not yet time, yield.
                    }
                    match backend.read_register(PID_TORQUE_FLUX_ACTUAL).await {
                        Ok(actual) => {
                            // PID_TORQUE_ACTUAL is in [31:16], PID_FLUX_ACTUAL in [15:0].
                            // We stepped the torque axis, so read the upper half.
                            // Normalize by direction so both captures produce
                            // positive values when the motor responds correctly.
                            // Wrong-sign current clamps to zero.
                            let torque_actual = ((actual >> 16) & 0xFFFF) as i16;
                            let sample =
                                crate::autotune::normalize_capture_sample(torque_actual, dir);
                            buf.push(sample);
                        }
                        Err(_) => {
                            return Self::fail_inner(phase, CommissionError::Spi);
                        }
                    }
                    // Detect current-PID output saturation: read the limited
                    // FOC voltage output (FOC_UQ_UD_LIMITED) and count when
                    // it's near the PIDOUT_UQ_UD_LIMITS voltage ceiling.
                    if data.pidout_limit > 0 {
                        let sub = foci_tmc4671::registers::INTERIM_FOC_UQ_UD_LIMITED;
                        let _ = backend.write_register(sub.addr, sub.sub_addr as u32).await;
                        if let Ok(foc_limited) = backend.read_register(sub.data).await {
                            // FOC_UQ_UD_LIMITED packs UQ in [31:16], UD in [15:0].
                            // Check the torque-axis voltage (UQ) since we stepped torque.
                            let uq = ((foc_limited >> 16) & 0xFFFF) as i16;
                            let abs_uq = uq.unsigned_abs() as u32;
                            if abs_uq >= data.pidout_limit * 9 / 10 {
                                data.saturation_count += 1;
                            }
                        }
                    }
                    *next_sample_us = next_sample_us.wrapping_add(ELEC_SAMPLE_PERIOD_US);
                }
                StepResult::Continue
            }

            CurrentValidateState::Classify => {
                let forward = match data.forward_metrics {
                    Some(m) => m,
                    None => {
                        return Self::fail_inner(phase, CommissionError::CurrentValidationFailed);
                    }
                };
                let reverse = match data.reverse_metrics {
                    Some(m) => m,
                    None => {
                        return Self::fail_inner(phase, CommissionError::CurrentValidationFailed);
                    }
                };

                // Compute symmetry from forward and reverse tail levels
                // (steady-state current magnitude).
                let symmetry = crate::autotune::compute_tail_symmetry_pct(
                    forward.tail_level,
                    reverse.tail_level,
                );

                // Use worst-case tail_level (lower = further from target).
                let worst_tail = forward.tail_level.min(reverse.tail_level);
                let target = data.step_amplitude as u32;
                let target_error = crate::autotune::compute_target_error_pct(worst_tail, target);

                let combined = CurrentValidation {
                    symmetry_pct: symmetry,
                    // Use worst-case of forward and reverse for other metrics.
                    overshoot_pct: forward.overshoot_pct.max(reverse.overshoot_pct),
                    ringing_count: forward.ringing_count.max(reverse.ringing_count),
                    settling_time_us: forward.settling_time_us.max(reverse.settling_time_us),
                    // Use firmware-counted PID-output saturation, not the
                    // helper's current-level saturation.
                    saturation_samples: data.saturation_count,
                    rise_time_us: forward.rise_time_us,
                    tail_level: worst_tail,
                    target_error_pct: target_error,
                };

                let total_samples = crate::autotune::TRANSIENT_LEN as u16 * 2;
                let settle_reference_us = crate::autotune::current_settling_reference_us(
                    &data.gains,
                    backend.tmc_config(),
                );
                let result = crate::autotune::classify_current_validation(
                    &combined,
                    settle_reference_us,
                    total_samples,
                );

                defmt::info!(
                    "current validate: overshoot={}‰ ringing={} settle={}us sym={}‰ target_err={}‰ sat={}/{} tau_e={}us settle_ref={}us → {:?} (attempt={})",
                    combined.overshoot_pct,
                    combined.ringing_count,
                    combined.settling_time_us,
                    combined.symmetry_pct,
                    combined.target_error_pct,
                    combined.saturation_samples,
                    total_samples,
                    data.model.tau_e_us,
                    settle_reference_us,
                    result,
                    data.attempt,
                );

                match result {
                    ValidationResult::Pass => {
                        // Gains accepted.
                        data.final_status = if data.attempt > 0 {
                            CommissionStatus::AcceptedWithWarnings
                        } else {
                            CommissionStatus::Accepted
                        };
                    }
                    retry_result
                        if crate::autotune::should_retry_current_validation(
                            retry_result,
                            data.attempt,
                        ) =>
                    {
                        // Retry: keep P damping and adjust I based on the
                        // measured tail response.
                        let target = data.step_amplitude as u32;
                        let settle_band = (target * 5 / 100).max(1);
                        let retry_gains = crate::autotune::current_validation_retry_gains(
                            data.gains,
                            target,
                            settle_band,
                            forward.tail_level,
                            reverse.tail_level,
                            combined.overshoot_pct,
                        );
                        if crate::autotune::should_accept_current_validation_without_retry(
                            &combined,
                            settle_reference_us,
                            total_samples,
                            data.gains,
                            retry_gains,
                        ) {
                            defmt::info!(
                                "current validate accept: marginal symmetry only, retry unchanged"
                            );
                            data.final_status = CommissionStatus::Accepted;
                        } else {
                            data.attempt = 1;
                            defmt::info!(
                                "current validate retry: previous={:?} lambda={}us → P={} I={} tails={}/{}",
                                retry_result,
                                retry_gains.lambda_us,
                                retry_gains.flux_p,
                                retry_gains.flux_i,
                                forward.tail_level,
                                reverse.tail_level,
                            );
                            data.gains = retry_gains;
                            *validate_state = CurrentValidateState::WriteGains;
                            return StepResult::Continue;
                        }
                    }
                    ValidationResult::Marginal | ValidationResult::Fail => {
                        // Fall back to conservative defaults.
                        data.gains = CurrentGains {
                            flux_p: crate::pid::DEFAULT_INNER_GAINS.flux_p,
                            flux_i: crate::pid::DEFAULT_INNER_GAINS.flux_i,
                            torque_p: crate::pid::DEFAULT_INNER_GAINS.torque_p,
                            torque_i: crate::pid::DEFAULT_INNER_GAINS.torque_i,
                            effective_bandwidth_hz: 0,
                            lambda_us: 0,
                        };
                        data.final_status = CommissionStatus::AcceptedWithWarnings;

                        // Write the default gains to TMC4671 so outer
                        // commissioning uses the reported conservative gains.
                        let flux_reg =
                            ((data.gains.flux_p as u32) << 16) | data.gains.flux_i as u32;
                        let torque_reg =
                            ((data.gains.torque_p as u32) << 16) | data.gains.torque_i as u32;
                        let _ = backend.write_register(PID_FLUX_P_FLUX_I, flux_reg).await;
                        let _ = backend
                            .write_register(PID_TORQUE_P_TORQUE_I, torque_reg)
                            .await;
                    }
                }

                // Restore safe state.
                let _ = backend.write_register(PID_TORQUE_FLUX_TARGET, 0).await;
                let _ = backend.write_register(MODE_RAMP_MODE_MOTION, 0).await;
                backend.set_enable_pin(false);

                let deadline = backend.elapsed_us().wrapping_add(VALIDATION_SETTLE_US);
                *validate_state = CurrentValidateState::SafeState {
                    deadline_us: deadline,
                };
                StepResult::Continue
            }

            CurrentValidateState::SafeState { deadline_us } => {
                if backend.elapsed_us().wrapping_sub(*deadline_us) >= 0x8000_0000 {
                    return StepResult::Continue;
                }
                *validate_state = CurrentValidateState::Finalize;
                StepResult::Continue
            }

            CurrentValidateState::Finalize => {
                // Compute max ringing count from forward/reverse metrics.
                let ringing_count = match (data.forward_metrics, data.reverse_metrics) {
                    (Some(fwd), Some(rev)) => fwd.ringing_count.max(rev.ringing_count),
                    (Some(fwd), None) => fwd.ringing_count,
                    (None, Some(rev)) => rev.ringing_count,
                    (None, None) => 0,
                };

                let result = InnerResult {
                    status: data.final_status,
                    r_mohm: data.model.r_mohm,
                    l_uh: data.model.l_uh,
                    flux_p: data.gains.flux_p,
                    flux_i: data.gains.flux_i,
                    torque_p: data.gains.torque_p,
                    torque_i: data.gains.torque_i,
                    lambda_us: data.gains.lambda_us,
                    theta_e_us: data.model.theta_e_us,
                    ringing_count,
                    bandwidth_hz: data.gains.effective_bandwidth_hz,
                };

                match sequence {
                    InnerSequence::Commission => {
                        // Transition to ClosedLoopEntry with computed
                        // current-loop gains and fallback outer gains.
                        let vel_limit = backend.operational_velocity_limit();
                        *phase = InnerPhase::ClosedLoopEntry {
                            state: ClosedLoopEntryState::WriteGains,
                            flux_p: result.flux_p,
                            flux_i: result.flux_i,
                            torque_p: result.torque_p,
                            torque_i: result.torque_i,
                            // Conservative fallback outer gains.
                            velocity_p: 1152, // 4.5 in Q8.8
                            velocity_i: 0,
                            position_p: 640, // 2.5 in Q8.8
                            position_i: 0,
                            velocity_limit: vel_limit,
                            velocity_filter_hz: 0,
                            torque_filter_hz: 0,
                            position_filter_hz: 0,
                            flux_filter_hz: 0,
                            inner_result: Some(result),
                        };
                        StepResult::PhaseChanged(PhaseId::ClosedLoopEntry)
                    }
                    InnerSequence::Calibrate | InnerSequence::Selftest => {
                        unreachable!("Calibrate/Selftest never reach CurrentValidate")
                    }
                }
            }
        }
    }

    // -------------------------------------------------------------------
    // Phase 8: Closed-Loop Entry
    // -------------------------------------------------------------------

    /// Advance closed-loop entry sub-state.
    ///
    /// Mirrors the register write sequence from `tmc_enter_closedloop`
    /// (gains, limits, PHI_E verification, position zeroing, integrator
    /// clear, STEP_WIDTH, biquad filters, position mode entry, stability
    /// check) but uses the engine's per-tick state machine pattern.
    async fn step_closed_loop_entry<B: CommissioningBackend>(
        phase: &mut InnerPhase,
        sequence: InnerSequence,
        adc_cal: &mut Option<AdcCalResult>,
        backend: &mut B,
    ) -> StepResult {
        use foci_tmc4671::registers::*;

        let (
            cle_state,
            flux_p,
            flux_i,
            torque_p,
            torque_i,
            velocity_p,
            velocity_i,
            position_p,
            position_i,
            velocity_limit,
            velocity_filter_hz,
            torque_filter_hz,
            position_filter_hz,
            flux_filter_hz,
            inner_result,
        ) = match phase {
            InnerPhase::ClosedLoopEntry {
                state,
                flux_p,
                flux_i,
                torque_p,
                torque_i,
                velocity_p,
                velocity_i,
                position_p,
                position_i,
                velocity_limit,
                velocity_filter_hz,
                torque_filter_hz,
                position_filter_hz,
                flux_filter_hz,
                inner_result,
            } => (
                state,
                *flux_p,
                *flux_i,
                *torque_p,
                *torque_i,
                *velocity_p,
                *velocity_i,
                *position_p,
                *position_i,
                *velocity_limit,
                *velocity_filter_hz,
                *torque_filter_hz,
                *position_filter_hz,
                *flux_filter_hz,
                inner_result,
            ),
            _ => return StepResult::Continue,
        };

        match cle_state {
            ClosedLoopEntryState::WriteGains => {
                // 1. Flux and torque PID gains.
                let flux_reg = (flux_p as u32) << 16 | flux_i as u32;
                let torque_reg = (torque_p as u32) << 16 | torque_i as u32;
                if backend
                    .write_register(PID_FLUX_P_FLUX_I, flux_reg)
                    .await
                    .is_err()
                    || backend
                        .write_register(PID_TORQUE_P_TORQUE_I, torque_reg)
                        .await
                        .is_err()
                {
                    return Self::fail_inner(phase, CommissionError::Spi);
                }

                // 2. Velocity PID gains.
                let velocity_reg = (velocity_p as u32) << 16 | velocity_i as u32;
                if backend
                    .write_register(PID_VELOCITY_P_VELOCITY_I, velocity_reg)
                    .await
                    .is_err()
                {
                    return Self::fail_inner(phase, CommissionError::Spi);
                }

                // 3. Position PID gains.
                let position_reg = (position_p as u32) << 16 | position_i as u32;
                if backend
                    .write_register(PID_POSITION_P_POSITION_I, position_reg)
                    .await
                    .is_err()
                {
                    return Self::fail_inner(phase, CommissionError::Spi);
                }

                *cle_state = ClosedLoopEntryState::WriteLimits;
                StepResult::Continue
            }

            ClosedLoopEntryState::WriteLimits => {
                // 4. Velocity limit.
                if backend
                    .write_register(PID_VELOCITY_LIMIT, velocity_limit)
                    .await
                    .is_err()
                {
                    return Self::fail_inner(phase, CommissionError::Spi);
                }

                // 5. Raise voltage limit for actual operation.
                // ADC cal sets it to 1024 as a safety measure.
                if backend
                    .write_register(PIDOUT_UQ_UD_LIMITS, 16000)
                    .await
                    .is_err()
                {
                    return Self::fail_inner(phase, CommissionError::Spi);
                }
                let voltage_limit_readback = backend.read_register(PIDOUT_UQ_UD_LIMITS).await;
                match voltage_limit_readback {
                    Ok(16000) => {}
                    _ => {
                        defmt::error!("closed-loop: PIDOUT_UQ_UD_LIMITS readback mismatch");
                        return Self::fail_inner(phase, CommissionError::Spi);
                    }
                }

                *cle_state = ClosedLoopEntryState::VerifyPhiE;
                StepResult::Continue
            }

            ClosedLoopEntryState::VerifyPhiE => {
                // Set PHI_E_SELECTION to ABN encoder (3). For Calibrate this
                // is already 3 from EncoderAlign::Commit; for Commission the
                // intervening ElectricalId/CurrentTune phases reset it to 1
                // (external phi_e for voltage-mode sweeps).
                if backend.write_register(PHI_E_SELECTION, 3).await.is_err() {
                    return Self::fail_inner(phase, CommissionError::Spi);
                }
                match backend.read_register(PHI_E_SELECTION).await {
                    Ok(3) => {}
                    Ok(val) => {
                        defmt::error!("closed-loop: PHI_E_SELECTION={}, expected 3", val);
                        return Self::fail_inner(phase, CommissionError::Spi);
                    }
                    Err(_) => {
                        return Self::fail_inner(phase, CommissionError::Spi);
                    }
                }

                *cle_state = ClosedLoopEntryState::ZeroAndClear;
                StepResult::Continue
            }

            ClosedLoopEntryState::ZeroAndClear => {
                // 6. Zero position (also zeros target per datasheet 0x6B).
                if backend
                    .write_register(PID_POSITION_ACTUAL, 0)
                    .await
                    .is_err()
                {
                    return Self::fail_inner(phase, CommissionError::Spi);
                }

                // Clear PID integrators. These must succeed — entering
                // position mode with stale error sums causes transients.
                let integrators_ok = async {
                    Self::write_sub_register_checked(backend, PID_ERROR_PID_TORQUE_ERROR_SUM, 0)
                        .await?;
                    Self::write_sub_register_checked(backend, PID_ERROR_PID_FLUX_ERROR_SUM, 0)
                        .await?;
                    Self::write_sub_register_checked(backend, PID_ERROR_PID_VELOCITY_ERROR_SUM, 0)
                        .await?;
                    Self::write_sub_register_checked(backend, PID_ERROR_PID_POSITION_ERROR_SUM, 0)
                        .await?;
                    backend
                        .write_register(STATUS_FLAGS, 0)
                        .await
                        .map_err(|_| ())
                };
                if integrators_ok.await.is_err() {
                    return Self::fail_inner(phase, CommissionError::Spi);
                }

                *cle_state = ClosedLoopEntryState::WritePositionConfig;
                StepResult::Continue
            }

            ClosedLoopEntryState::WritePositionConfig => {
                // 7. Explicit POSITION_SELECTION=0 (phi_e).
                if backend.write_register(POSITION_SELECTION, 0).await.is_err() {
                    return Self::fail_inner(phase, CommissionError::Spi);
                }

                // 8. Write STEP_WIDTH for hardware step/dir input.
                let step_width = backend.step_width();
                if step_width == 0 {
                    defmt::error!("STEP_WIDTH=0, aborting closed-loop entry");
                    return Self::fail_inner(phase, CommissionError::Spi);
                }
                if backend
                    .write_register(STEP_WIDTH, step_width)
                    .await
                    .is_err()
                {
                    return Self::fail_inner(phase, CommissionError::Spi);
                }
                // Readback verification.
                match backend.read_register(STEP_WIDTH).await {
                    Ok(val) if val == step_width => {}
                    _ => {
                        defmt::error!("closed-loop: STEP_WIDTH readback mismatch");
                        return Self::fail_inner(phase, CommissionError::Spi);
                    }
                }
                defmt::info!("STEP_WIDTH(0x78) = {} written", step_width);

                *cle_state = ClosedLoopEntryState::WriteFilters;
                StepResult::Continue
            }

            ClosedLoopEntryState::WriteFilters => {
                let pwm_maxcnt = backend.pwm_maxcnt();

                // Best-effort filter writes (warn on failure, don't abort).
                if velocity_filter_hz > 0 {
                    Self::write_biquad_velocity(backend, velocity_filter_hz, pwm_maxcnt).await;
                } else {
                    Self::write_sub_register(backend, CONFIG_BIQUAD_V_ENABLE, 0).await;
                }

                if torque_filter_hz > 0 {
                    Self::write_biquad_torque(backend, torque_filter_hz, pwm_maxcnt).await;
                } else {
                    Self::write_sub_register(backend, CONFIG_BIQUAD_T_ENABLE, 0).await;
                }

                if position_filter_hz > 0 {
                    Self::write_biquad_position(backend, position_filter_hz, pwm_maxcnt).await;
                } else {
                    Self::write_sub_register(backend, CONFIG_BIQUAD_X_ENABLE, 0).await;
                }

                if flux_filter_hz > 0 {
                    Self::write_biquad_flux(backend, flux_filter_hz, pwm_maxcnt).await;
                } else {
                    Self::write_sub_register(backend, CONFIG_BIQUAD_F_ENABLE, 0).await;
                }

                *cle_state = ClosedLoopEntryState::EnterPositionMode;
                StepResult::Continue
            }

            ClosedLoopEntryState::EnterPositionMode => {
                // MODE_PID_TYPE=1 (bit 31), MODE_MOTION=3 (position).
                if backend
                    .write_register(MODE_RAMP_MODE_MOTION, 0x8000_0003)
                    .await
                    .is_err()
                {
                    return Self::fail_inner(phase, CommissionError::Spi);
                }
                // Readback verification.
                match backend.read_register(MODE_RAMP_MODE_MOTION).await {
                    Ok(0x8000_0003) => {}
                    _ => {
                        defmt::error!("closed-loop: MODE_RAMP_MODE_MOTION readback mismatch");
                        return Self::fail_inner(phase, CommissionError::Spi);
                    }
                }

                // Schedule stability check after 10ms.
                let now = backend.elapsed_us();
                *cle_state = ClosedLoopEntryState::StabilityCheck1 {
                    position_1: 0,
                    deadline_us: now + 10_000,
                };
                StepResult::Continue
            }

            ClosedLoopEntryState::StabilityCheck1 {
                position_1: _,
                deadline_us,
            } => {
                if backend.elapsed_us() < *deadline_us {
                    return StepResult::Continue;
                }

                // Read first position sample.
                match backend.read_register(PID_POSITION_ACTUAL).await {
                    Ok(val) => {
                        let pos1 = val as i32;
                        let now = backend.elapsed_us();
                        *cle_state = ClosedLoopEntryState::StabilityCheck2 {
                            position_1: pos1,
                            deadline_us: now + 20_000,
                        };
                    }
                    Err(_) => {
                        return Self::fail_inner(phase, CommissionError::Spi);
                    }
                }
                StepResult::Continue
            }

            ClosedLoopEntryState::StabilityCheck2 {
                position_1,
                deadline_us,
            } => {
                if backend.elapsed_us() < *deadline_us {
                    return StepResult::Continue;
                }

                // Read second position sample and check for runaway.
                let pos2 = match backend.read_register(PID_POSITION_ACTUAL).await {
                    Ok(val) => val as i32,
                    Err(_) => {
                        return Self::fail_inner(phase, CommissionError::Spi);
                    }
                };

                // Runaway detection: if position error is large and growing,
                // the feedback sign is wrong.
                const RUNAWAY_THRESHOLD: u32 = 32760; // ~40 steps
                if pos2.unsigned_abs() > RUNAWAY_THRESHOLD
                    && pos2.unsigned_abs() > position_1.unsigned_abs()
                {
                    defmt::error!(
                        "closed-loop: runaway detected: pos1={}, pos2={}",
                        position_1,
                        pos2
                    );
                    let _ = backend.write_register(MODE_RAMP_MODE_MOTION, 0).await;
                    return Self::fail_inner(phase, CommissionError::Spi);
                }

                // Stability check: position must not be drifting.
                // Threshold of 2 encoder counts between 10ms and 30ms
                // readings indicates the motor is holding position.
                const STABILITY_THRESHOLD: u32 = 2;
                let drift = (pos2.wrapping_sub(*position_1)).unsigned_abs();
                if drift > STABILITY_THRESHOLD {
                    defmt::error!(
                        "closed-loop: unstable: pos1={}, pos2={}, drift={}",
                        position_1,
                        pos2,
                        drift
                    );
                    let _ = backend.write_register(MODE_RAMP_MODE_MOTION, 0).await;
                    return Self::fail_inner(phase, CommissionError::Spi);
                }

                defmt::info!("closed-loop position mode active (engine)");

                match sequence {
                    InnerSequence::Calibrate => {
                        let adc_result = adc_cal.clone().unwrap_or(AdcCalResult {
                            adc_i0: 0,
                            adc_i1: 0,
                            encoder_count: 0,
                        });
                        StepResult::CalibrateDone(adc_result)
                    }
                    InnerSequence::Commission => {
                        let inner = inner_result.take().unwrap_or(InnerResult {
                            status: CommissionStatus::Accepted,
                            r_mohm: 0,
                            l_uh: 0,
                            flux_p: 0,
                            flux_i: 0,
                            torque_p: 0,
                            torque_i: 0,
                            lambda_us: 0,
                            theta_e_us: 0,
                            ringing_count: 0,
                            bandwidth_hz: 0,
                        });
                        StepResult::CommissionDone(CommissionResult {
                            inner,
                            fallback_velocity_p: velocity_p,
                            fallback_velocity_i: velocity_i,
                            fallback_position_p: position_p,
                            fallback_position_i: position_i,
                            fallback_velocity_limit: velocity_limit,
                        })
                    }
                    InnerSequence::Selftest => {
                        // Selftest never reaches ClosedLoopEntry.
                        unreachable!()
                    }
                }
            }
        }
    }

    // -------------------------------------------------------------------
    // Helpers
    // -------------------------------------------------------------------

    /// Transition inner phase to Failed and restore safe state.
    fn fail_inner(phase: &mut InnerPhase, err: CommissionError) -> StepResult {
        *phase = InnerPhase::Failed(err);
        StepResult::InnerFailed(err)
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
            Self::write_biquad_velocity(backend, cutoff_hz as u16, pwm_maxcnt).await;
        } else {
            Self::write_sub_register(backend, foci_tmc4671::registers::CONFIG_BIQUAD_V_ENABLE, 0)
                .await;
        }

        if let Some(cutoff_hz) = filter_config.position_filter_hz {
            Self::write_biquad_position(backend, cutoff_hz as u16, pwm_maxcnt).await;
        } else {
            Self::write_sub_register(backend, foci_tmc4671::registers::CONFIG_BIQUAD_X_ENABLE, 0)
                .await;
        }

        if let Some(cutoff_hz) = filter_config.flux_filter_hz {
            Self::write_biquad_flux(backend, cutoff_hz as u16, pwm_maxcnt).await;
        } else {
            Self::write_sub_register(backend, foci_tmc4671::registers::CONFIG_BIQUAD_F_ENABLE, 0)
                .await;
        }

        if let Some(cutoff_hz) = filter_config.torque_filter_hz {
            Self::write_biquad_torque(backend, cutoff_hz as u16, pwm_maxcnt).await;
        } else {
            Self::write_sub_register(backend, foci_tmc4671::registers::CONFIG_BIQUAD_T_ENABLE, 0)
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
    // Biquad filter write helpers
    // -------------------------------------------------------------------

    /// Write a sub-register via the backend's regular register writes.
    ///
    /// TMC4671 sub-registers use address register 0x4D and data register 0x4E.
    async fn write_sub_register<B: CommissioningBackend>(
        backend: &mut B,
        sub: foci_tmc4671::registers::SubRegister,
        value: u32,
    ) {
        let _ = backend.write_register(sub.addr, sub.sub_addr as u32).await;
        let _ = backend.write_register(sub.data, value).await;
    }

    /// Write a sub-register, returning `Err(())` if either SPI transfer fails.
    async fn write_sub_register_checked<B: CommissioningBackend>(
        backend: &mut B,
        sub: foci_tmc4671::registers::SubRegister,
        value: u32,
    ) -> Result<(), ()> {
        backend
            .write_register(sub.addr, sub.sub_addr as u32)
            .await
            .map_err(|_| ())?;
        backend
            .write_register(sub.data, value)
            .await
            .map_err(|_| ())
    }

    /// Clear torque/flux PI error sums for current-loop captures.
    async fn clear_current_integrators<B: CommissioningBackend>(backend: &mut B) -> Result<(), ()> {
        use foci_tmc4671::registers::{
            PID_ERROR_PID_FLUX_ERROR_SUM, PID_ERROR_PID_TORQUE_ERROR_SUM,
        };

        Self::write_sub_register_checked(backend, PID_ERROR_PID_TORQUE_ERROR_SUM, 0).await?;
        Self::write_sub_register_checked(backend, PID_ERROR_PID_FLUX_ERROR_SUM, 0).await
    }

    /// Write velocity biquad filter coefficients.
    async fn write_biquad_velocity<B: CommissioningBackend>(
        backend: &mut B,
        cutoff_hz: u16,
        pwm_maxcnt: u16,
    ) {
        use foci_tmc4671::registers::*;
        if let Some(coeffs) = crate::biquad::butterworth_lowpass(cutoff_hz, pwm_maxcnt) {
            Self::write_sub_register(backend, CONFIG_BIQUAD_V_A_1, coeffs.a_1 as u32).await;
            Self::write_sub_register(backend, CONFIG_BIQUAD_V_A_2, coeffs.a_2 as u32).await;
            Self::write_sub_register(backend, CONFIG_BIQUAD_V_B_0, coeffs.b_0 as u32).await;
            Self::write_sub_register(backend, CONFIG_BIQUAD_V_B_1, coeffs.b_1 as u32).await;
            Self::write_sub_register(backend, CONFIG_BIQUAD_V_B_2, coeffs.b_2 as u32).await;
            Self::write_sub_register(backend, CONFIG_BIQUAD_V_ENABLE, 1).await;
            defmt::info!("biquad velocity: {}Hz enabled", cutoff_hz);
        }
    }

    /// Write position biquad filter coefficients.
    async fn write_biquad_position<B: CommissioningBackend>(
        backend: &mut B,
        cutoff_hz: u16,
        pwm_maxcnt: u16,
    ) {
        use foci_tmc4671::registers::*;
        if let Some(coeffs) = crate::biquad::butterworth_lowpass(cutoff_hz, pwm_maxcnt) {
            Self::write_sub_register(backend, CONFIG_BIQUAD_X_A_1, coeffs.a_1 as u32).await;
            Self::write_sub_register(backend, CONFIG_BIQUAD_X_A_2, coeffs.a_2 as u32).await;
            Self::write_sub_register(backend, CONFIG_BIQUAD_X_B_0, coeffs.b_0 as u32).await;
            Self::write_sub_register(backend, CONFIG_BIQUAD_X_B_1, coeffs.b_1 as u32).await;
            Self::write_sub_register(backend, CONFIG_BIQUAD_X_B_2, coeffs.b_2 as u32).await;
            Self::write_sub_register(backend, CONFIG_BIQUAD_X_ENABLE, 1).await;
            defmt::info!("biquad position: {}Hz enabled", cutoff_hz);
        }
    }

    /// Write flux biquad filter coefficients.
    async fn write_biquad_flux<B: CommissioningBackend>(
        backend: &mut B,
        cutoff_hz: u16,
        pwm_maxcnt: u16,
    ) {
        use foci_tmc4671::registers::*;
        if let Some(coeffs) = crate::biquad::butterworth_lowpass(cutoff_hz, pwm_maxcnt) {
            Self::write_sub_register(backend, CONFIG_BIQUAD_F_A_1, coeffs.a_1 as u32).await;
            Self::write_sub_register(backend, CONFIG_BIQUAD_F_A_2, coeffs.a_2 as u32).await;
            Self::write_sub_register(backend, CONFIG_BIQUAD_F_B_0, coeffs.b_0 as u32).await;
            Self::write_sub_register(backend, CONFIG_BIQUAD_F_B_1, coeffs.b_1 as u32).await;
            Self::write_sub_register(backend, CONFIG_BIQUAD_F_B_2, coeffs.b_2 as u32).await;
            Self::write_sub_register(backend, CONFIG_BIQUAD_F_ENABLE, 1).await;
            defmt::info!("biquad flux: {}Hz enabled", cutoff_hz);
        }
    }

    /// Write torque biquad filter coefficients.
    async fn write_biquad_torque<B: CommissioningBackend>(
        backend: &mut B,
        cutoff_hz: u16,
        pwm_maxcnt: u16,
    ) {
        use foci_tmc4671::registers::*;
        if let Some(coeffs) = crate::biquad::butterworth_lowpass(cutoff_hz, pwm_maxcnt) {
            Self::write_sub_register(backend, CONFIG_BIQUAD_T_A_1, coeffs.a_1 as u32).await;
            Self::write_sub_register(backend, CONFIG_BIQUAD_T_A_2, coeffs.a_2 as u32).await;
            Self::write_sub_register(backend, CONFIG_BIQUAD_T_B_0, coeffs.b_0 as u32).await;
            Self::write_sub_register(backend, CONFIG_BIQUAD_T_B_1, coeffs.b_1 as u32).await;
            Self::write_sub_register(backend, CONFIG_BIQUAD_T_B_2, coeffs.b_2 as u32).await;
            Self::write_sub_register(backend, CONFIG_BIQUAD_T_ENABLE, 1).await;
            defmt::info!("biquad torque: {}Hz enabled", cutoff_hz);
        }
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
