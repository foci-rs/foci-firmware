//! Cooperative commissioning state machine.
//!
//! [`CommissioningEngine`] is the entry point. Stage 1 (inner-loop)
//! diagnostics, electrical identification, current-loop tuning, and
//! closed-loop entry live in [`inner`]. Stage 2 (outer-loop) mechanical
//! identification, velocity / position / filter tuning live in [`outer`].
//! Shared low-level register helpers live in [`helpers`]; the public
//! data types (and the [`types::CommissioningBackend`] trait) live in
//! [`types`].

pub mod helpers;
pub mod inner;
pub mod outer;
pub mod types;

use crate::autotune::{
    CommissionError, CommissionProfile, InnerSequence, OperatingMode, TransientBuffer,
};
use crate::commissioning::inner::InnerPhase;
use crate::commissioning::outer::{
    MechanicalIdData, MechanicalIdState, OuterPhase, OuterSafetyLimits,
};
use crate::commissioning::types::{AdcCalResult, CommissioningBackend, StepResult};

// Re-export public API surface for downstream consumers.
pub use crate::commissioning::types::{
    AdcCalResult as AdcCalResultExport, CommissionResult, CommissioningBackend as Backend,
    InnerResult, OuterResult, StepResult as Step,
};

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
                phase: InnerPhase::AdcCalibrate(inner::AdcCalState::Setup),
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
                phase: InnerPhase::AdcCalibrate(inner::AdcCalState::Setup),
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
                phase: InnerPhase::AdcCalibrate(inner::AdcCalState::Setup),
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
                inner::step_inner(
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
                outer::step_outer(
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
