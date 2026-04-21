//! Commissioning state machine and hardware backend.
//!
//! The [`CommissioningEngine`] is a cooperative state machine invoked
//! every 1ms by the `tmc_control` task. Each `step()` call advances the
//! current phase by one chunk of work (max 1ms runtime).
//!
//! Implementation now lives in `commissioning/`; this module retains the
//! `CommissioningEngine` struct and constructors plus the inner-only
//! sub-state types until the final pass moves them out too.

use crate::autotune::{
    CommissionError, CommissionProfile, CommissionStatus, CurrentGains, CurrentValidation,
    ElectricalModel, InnerSequence, MechanicalModel, OperatingMode, TransientBuffer,
};

pub(crate) use crate::commissioning::inner::InnerPhase;
pub(crate) use crate::commissioning::outer::{
    MechanicalIdData, MechanicalIdState, OUTER_FALLBACK_POSITION_I, OUTER_FALLBACK_POSITION_P,
    OUTER_FALLBACK_VELOCITY_I, OUTER_FALLBACK_VELOCITY_P, OUTER_MAX_POSITION_I,
    OUTER_MAX_POSITION_P, OUTER_MAX_VELOCITY_I, OUTER_MAX_VELOCITY_P, OUTER_WARNING_NONE,
    OUTER_WARNING_UNSAFE_SYNTH_GAINS, OuterDiagnostics, OuterPhase, OuterSafetyLimits,
    POS_TUNE_INITIAL_P, POS_TUNE_STEP_COUNTS, PositionTuneData, PositionTuneState,
    VELOCITY_CAP_COUNTS_PER_MS, VelocityTuneStrategy, VelocityValidateData, VelocityValidateState,
};
pub use crate::commissioning::types::{
    AdcCalResult, CommissionResult, CommissioningBackend, InnerResult, OuterResult, StepResult,
};

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
pub(crate) enum EngineMode {
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
    /// Returns a [`StepResult`] indicating whether to continue, report
    /// a phase transition, or finalize.
    ///
    /// Destructures `engine_mode` to pass `phase` and `buf` separately,
    /// avoiding a double `&mut self` borrow.
    pub async fn step<B: CommissioningBackend>(&mut self, backend: &mut B) -> StepResult {
        // E-stop check at the top of every step.
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
                crate::commissioning::outer::step_outer(
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
}

// MechanicalModel is referenced inside EngineMode but currently not via a
// public path; explicit use to satisfy the import set.
#[allow(dead_code)]
const _: Option<MechanicalModel> = None;
