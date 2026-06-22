//! Cooperative commissioning state machine.
//!
//! [`CommissioningEngine`] is the entry point. Stage 1 (inner-loop)
//! diagnostics, electrical identification, current-loop tuning, and
//! closed-loop entry live in [`inner`]. Stage 2 (outer-loop) mechanical
//! identification, velocity / position / filter tuning live in [`outer`].
//! Shared low-level register helpers live in [`helpers`]. The public data
//! types live in [`types`], while hardware capability traits live in
//! [`backend`].

pub mod backend;
pub mod helpers;
pub mod inner;
pub mod outer;
pub mod types;

use crate::autotune::{
    CommissionError, CommissionProfile, CommissionStatus, InnerSequence, OperatingMode, PhaseId,
    TransientBuffer,
};
use crate::commissioning::inner::InnerPhase;
use crate::commissioning::outer::{
    MechanicalIdData, MechanicalIdState, OuterPhase, OuterSafetyLimits, TraceSummarySeqs,
};
use crate::commissioning::types::{AdcCalResult, EngineKind, StepResult};
use crate::pid::InnerLoopGains;
use crate::state::tmc::TmcChannelState;
use crate::trace::events::{
    CommissionTraceEvent, EVENT_COMMISSION_ENGINE_START, EVENT_COMMISSION_ENGINE_TERMINAL,
    EVENT_COMMISSION_PHASE_END, EVENT_COMMISSION_PHASE_START, TraceEventLevel, u32_arg,
};

// Re-export public API surface for downstream consumers.
pub use crate::commissioning::backend::{
    CommissionBoardConfig, CommissionEncoderRead, CommissionMotorControl, CommissionRegisterIo,
    CommissionTiming, CommissionTrace, CommissioningBackend, CommissioningBackend as Backend,
};
pub use crate::commissioning::types::{
    AdcCalResult as AdcCalResultExport, CommissionResult, InnerResult, OuterResult,
    StepResult as Step,
};

/// Trace status code for a phase that ended without an error.
const PHASE_END_OK: u8 = 0;

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
    /// Phase 5 tau_e_us measurement (or host-defaulted).
    pub tau_e_us: u32,
    /// Phase 5 tau crosscheck (or host-defaulted).
    pub tau_e_crosscheck_us: u32,
    /// Phase 5 tau residual in permille.
    pub tau_residual_permille: u16,
    /// Inner-confidence warning bitfield carried from Stage 1.
    pub inner_warning_flags: u8,
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

/// Gain snapshot used by the calibrate-on-enable closed-loop entry.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) struct CalibrateEntryGains {
    /// Inner current-loop gains to use during closed-loop entry.
    pub inner: InnerLoopGains,
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
    /// Velocity biquad filter cutoff (Hz), 0 = disabled.
    pub velocity_filter_hz: u16,
    /// Torque biquad filter cutoff (Hz), 0 = disabled.
    pub torque_filter_hz: u16,
    /// Position biquad filter cutoff (Hz), 0 = disabled.
    pub position_filter_hz: u16,
    /// Flux biquad filter cutoff (Hz), 0 = disabled.
    pub flux_filter_hz: u16,
}

impl CalibrateEntryGains {
    fn from_tmc_state(tmc_state: &'static TmcChannelState, inner: Option<InnerLoopGains>) -> Self {
        use core::sync::atomic::Ordering::Relaxed;

        Self {
            inner: inner.unwrap_or(crate::pid::DEFAULT_INNER_GAINS),
            velocity_p: tmc_state.velocity_p_value.load(Relaxed),
            velocity_i: tmc_state.velocity_i_value.load(Relaxed),
            position_p: tmc_state.position_p_value.load(Relaxed),
            position_i: tmc_state.position_i_value.load(Relaxed),
            velocity_limit: tmc_state.velocity_limit_cfg.load(Relaxed),
            velocity_filter_hz: tmc_state.velocity_filter_hz.load(Relaxed),
            torque_filter_hz: tmc_state.torque_filter_hz.load(Relaxed),
            position_filter_hz: tmc_state.position_filter_hz.load(Relaxed),
            flux_filter_hz: tmc_state.flux_filter_hz.load(Relaxed),
        }
    }
}

/// Which commissioning mode is active.
#[derive(Debug, defmt::Format)]
#[allow(clippy::large_enum_variant)]
// The Outer variant holds an OuterPhase whose MechanicalId state carries a
// ~2 KB timestamp array for time-aware metrics. Acceptable on STM32F407's
// 128 KB SRAM; boxing is not available in no_std without an allocator.
pub(crate) enum EngineMode {
    Inner {
        phase: InnerPhase,
        profile: CommissionProfile,
        sequence: InnerSequence,
        adc_cal: Option<AdcCalResult>,
        calibrate_entry_gains: Option<CalibrateEntryGains>,
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
        /// Phase 5 tau_e_us (or persisted/host default).
        tau_e_us: u32,
        /// Phase 5 tau crosscheck (or persisted/host default).
        tau_e_crosscheck_us: u32,
        /// Phase 5 tau residual in permille.
        tau_residual_permille: u16,
        /// Inner-confidence warning bitfield carried from Stage 1.
        inner_warning_flags: u8,
        /// Gains snapshot at entry for soft-failure restoration.
        entry_snapshot: EntrySnapshot,
    },
}

#[derive(Debug, Clone, Copy, defmt::Format)]
struct TraceRunState {
    engine_seq: u32,
    phase_seq: u32,
    started: bool,
    terminal: bool,
    current_phase: Option<PhaseId>,
}

impl TraceRunState {
    const fn new(engine_seq: u32) -> Self {
        Self {
            engine_seq,
            phase_seq: 0,
            started: false,
            terminal: false,
            current_phase: None,
        }
    }
}

/// Cooperative commissioning state machine.
///
/// Created when a `foci_commission`, `foci_tune`, or `foci_calibrate`
/// command arrives. Stored as `Option<CommissioningEngine>` in the
/// P1 `tmc_control` task. Each 1kHz tick calls `step()`.
pub struct CommissioningEngine {
    oid: u8,
    tmc_state: &'static TmcChannelState,
    engine_mode: EngineMode,
    buf: TransientBuffer,
    /// Per-stage results collected during a selftest run. Populated only
    /// when `sequence == InnerSequence::Selftest`; unused otherwise.
    selftest_results: crate::selftest::SelftestResults,
    /// Structured diagnostics retained for host-visible troubleshooting.
    diagnostics: crate::selftest::CommissionDiagnostics,
    trace_run: TraceRunState,
    trace_summary_seqs: TraceSummarySeqs,
}

impl CommissioningEngine {
    /// Create an engine for calibration (re-enable path).
    ///
    /// Runs ADC calibration, encoder alignment, and closed-loop entry
    /// with saved gains. Skips coil/wiring/encoder diagnostics.
    /// Returns `StepResult::CalibrateDone` on success.
    pub fn new_calibrate(oid: u8) -> Self {
        Self::new_calibrate_with_trace_seq(oid, 0)
    }

    /// Create an engine for calibration with a caller-assigned trace sequence.
    pub fn new_calibrate_with_trace_seq(oid: u8, trace_engine_seq: u32) -> Self {
        Self::new_calibrate_for_tmc_with_trace_seq(
            oid,
            crate::state::tmc::tmc_channel0(),
            trace_engine_seq,
        )
    }

    /// Create a calibration engine bound to a TMC channel state.
    pub fn new_calibrate_for_tmc(oid: u8, tmc_state: &'static TmcChannelState) -> Self {
        Self::new_calibrate_for_tmc_with_trace_seq(oid, tmc_state, 0)
    }

    /// Create a calibration engine bound to a TMC channel state with a trace sequence.
    pub fn new_calibrate_for_tmc_with_trace_seq(
        oid: u8,
        tmc_state: &'static TmcChannelState,
        trace_engine_seq: u32,
    ) -> Self {
        Self::new_calibrate_for_tmc_with_gains_and_trace_seq(oid, tmc_state, None, trace_engine_seq)
    }

    /// Create a calibration engine bound to a TMC channel state and stored gains.
    pub fn new_calibrate_for_tmc_with_gains(
        oid: u8,
        tmc_state: &'static TmcChannelState,
        inner_gains: Option<InnerLoopGains>,
    ) -> Self {
        Self::new_calibrate_for_tmc_with_gains_and_trace_seq(oid, tmc_state, inner_gains, 0)
    }

    /// Create a calibration engine bound to a TMC channel state, stored gains, and trace sequence.
    pub fn new_calibrate_for_tmc_with_gains_and_trace_seq(
        oid: u8,
        tmc_state: &'static TmcChannelState,
        inner_gains: Option<InnerLoopGains>,
        trace_engine_seq: u32,
    ) -> Self {
        let calibrate_entry_gains =
            Some(CalibrateEntryGains::from_tmc_state(tmc_state, inner_gains));
        Self {
            oid,
            tmc_state,
            engine_mode: EngineMode::Inner {
                phase: InnerPhase::AdcCalibrate(inner::AdcCalState::Setup),
                profile: CommissionProfile::Balanced,
                sequence: InnerSequence::Calibrate,
                adc_cal: None,
                calibrate_entry_gains,
            },
            buf: TransientBuffer::new(),
            selftest_results: crate::selftest::SelftestResults::default(),
            diagnostics: crate::selftest::CommissionDiagnostics::default(),
            trace_run: TraceRunState::new(trace_engine_seq),
            trace_summary_seqs: TraceSummarySeqs::new(),
        }
    }

    /// Create an engine for selftest (diagnostics + electrical ID).
    ///
    /// Runs ADC calibration, coil check, phase wiring, encoder check,
    /// encoder alignment, and electrical identification, then stops with
    /// `StepResult::SelftestDone` after restoring safe state.
    pub fn new_selftest(oid: u8) -> Self {
        Self::new_selftest_with_trace_seq(oid, 0)
    }

    /// Create a selftest engine with a caller-assigned trace sequence.
    pub fn new_selftest_with_trace_seq(oid: u8, trace_engine_seq: u32) -> Self {
        Self::new_selftest_for_tmc_with_trace_seq(
            oid,
            crate::state::tmc::tmc_channel0(),
            trace_engine_seq,
        )
    }

    /// Create a selftest engine bound to a TMC channel state.
    pub fn new_selftest_for_tmc(oid: u8, tmc_state: &'static TmcChannelState) -> Self {
        Self::new_selftest_for_tmc_with_trace_seq(oid, tmc_state, 0)
    }

    /// Create a selftest engine bound to a TMC channel state with a trace sequence.
    pub fn new_selftest_for_tmc_with_trace_seq(
        oid: u8,
        tmc_state: &'static TmcChannelState,
        trace_engine_seq: u32,
    ) -> Self {
        Self {
            oid,
            tmc_state,
            engine_mode: EngineMode::Inner {
                phase: InnerPhase::AdcCalibrate(inner::AdcCalState::Setup),
                profile: CommissionProfile::Balanced,
                sequence: InnerSequence::Selftest,
                adc_cal: None,
                calibrate_entry_gains: None,
            },
            buf: TransientBuffer::new(),
            selftest_results: crate::selftest::SelftestResults::default(),
            diagnostics: crate::selftest::CommissionDiagnostics::default(),
            trace_run: TraceRunState::new(trace_engine_seq),
            trace_summary_seqs: TraceSummarySeqs::new(),
        }
    }

    /// Create an engine for Stage 1 commissioning.
    ///
    /// Full diagnostics, electrical ID, current tune/validate, then
    /// closed-loop entry with fallback outer gains.
    /// Returns `StepResult::CommissionDone` on success.
    pub fn new_commission(oid: u8, profile: CommissionProfile) -> Self {
        Self::new_commission_with_trace_seq(oid, profile, 0)
    }

    /// Create a Stage 1 commissioning engine with a caller-assigned trace sequence.
    pub fn new_commission_with_trace_seq(
        oid: u8,
        profile: CommissionProfile,
        trace_engine_seq: u32,
    ) -> Self {
        Self::new_commission_for_tmc_with_trace_seq(
            oid,
            crate::state::tmc::tmc_channel0(),
            profile,
            trace_engine_seq,
        )
    }

    /// Create a Stage 1 commissioning engine bound to a TMC channel state.
    pub fn new_commission_for_tmc(
        oid: u8,
        tmc_state: &'static TmcChannelState,
        profile: CommissionProfile,
    ) -> Self {
        Self::new_commission_for_tmc_with_trace_seq(oid, tmc_state, profile, 0)
    }

    /// Create a Stage 1 commissioning engine bound to a TMC channel state and trace sequence.
    pub fn new_commission_for_tmc_with_trace_seq(
        oid: u8,
        tmc_state: &'static TmcChannelState,
        profile: CommissionProfile,
        trace_engine_seq: u32,
    ) -> Self {
        Self {
            oid,
            tmc_state,
            engine_mode: EngineMode::Inner {
                phase: InnerPhase::AdcCalibrate(inner::AdcCalState::Setup),
                profile,
                sequence: InnerSequence::Commission,
                adc_cal: None,
                calibrate_entry_gains: None,
            },
            buf: TransientBuffer::new(),
            selftest_results: crate::selftest::SelftestResults::default(),
            diagnostics: crate::selftest::CommissionDiagnostics::default(),
            trace_run: TraceRunState::new(trace_engine_seq),
            trace_summary_seqs: TraceSummarySeqs::new(),
        }
    }

    /// Create an engine for outer (mechanical) commissioning.
    ///
    /// `entry_snapshot` captures the active outer gains at entry for
    /// soft-failure restoration. Built by the caller from firmware atomics.
    pub fn new_outer(oid: u8, params: OuterTuneParams, entry_snapshot: EntrySnapshot) -> Self {
        Self::new_outer_with_trace_seq(oid, params, entry_snapshot, 0)
    }

    /// Create an outer commissioning engine with a caller-assigned trace sequence.
    pub fn new_outer_with_trace_seq(
        oid: u8,
        params: OuterTuneParams,
        entry_snapshot: EntrySnapshot,
        trace_engine_seq: u32,
    ) -> Self {
        Self::new_outer_for_tmc_with_trace_seq(
            oid,
            crate::state::tmc::tmc_channel0(),
            params,
            entry_snapshot,
            trace_engine_seq,
        )
    }

    /// Create an outer commissioning engine bound to a TMC channel state.
    pub fn new_outer_for_tmc(
        oid: u8,
        tmc_state: &'static TmcChannelState,
        params: OuterTuneParams,
        entry_snapshot: EntrySnapshot,
    ) -> Self {
        Self::new_outer_for_tmc_with_trace_seq(oid, tmc_state, params, entry_snapshot, 0)
    }

    /// Create an outer commissioning engine bound to a TMC channel state and trace sequence.
    pub fn new_outer_for_tmc_with_trace_seq(
        oid: u8,
        tmc_state: &'static TmcChannelState,
        params: OuterTuneParams,
        entry_snapshot: EntrySnapshot,
        trace_engine_seq: u32,
    ) -> Self {
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
            tau_e_us,
            tau_e_crosscheck_us,
            tau_residual_permille,
            inner_warning_flags,
        } = params;
        Self {
            oid,
            tmc_state,
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
                tau_e_us,
                tau_e_crosscheck_us,
                tau_residual_permille,
                inner_warning_flags,
                entry_snapshot,
            },
            buf: TransientBuffer::new(),
            selftest_results: crate::selftest::SelftestResults::default(),
            diagnostics: crate::selftest::CommissionDiagnostics::default(),
            trace_run: TraceRunState::new(trace_engine_seq),
            trace_summary_seqs: TraceSummarySeqs::new(),
        }
    }

    /// The OID this engine was started for.
    pub fn oid(&self) -> u8 {
        self.oid
    }

    /// Identify which constructor produced this engine.
    ///
    /// Used by the dispatch to choose the right reply payload at terminal arms,
    /// and at trace-start gating in PhaseChanged.
    pub fn kind(&self) -> EngineKind {
        match &self.engine_mode {
            EngineMode::Inner {
                sequence: InnerSequence::Commission,
                ..
            } => EngineKind::Commission,
            EngineMode::Inner {
                sequence: InnerSequence::Calibrate,
                ..
            } => EngineKind::Calibrate,
            EngineMode::Inner {
                sequence: InnerSequence::Selftest,
                ..
            } => EngineKind::Selftest,
            EngineMode::Outer { .. } => EngineKind::OuterTune,
        }
    }

    #[cfg(test)]
    pub(crate) fn calibrate_entry_inner_gains_for_test(
        &self,
    ) -> Option<crate::pid::InnerLoopGains> {
        match &self.engine_mode {
            EngineMode::Inner {
                sequence: InnerSequence::Calibrate,
                calibrate_entry_gains,
                ..
            } => calibrate_entry_gains.map(|gains| gains.inner),
            _ => None,
        }
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

    /// Mutably borrow the diagnostics collected during this run.
    pub fn diagnostics_mut(&mut self) -> &mut crate::selftest::CommissionDiagnostics {
        &mut self.diagnostics
    }

    /// Consume diagnostics collected during this run.
    pub fn take_diagnostics(&mut self) -> crate::selftest::CommissionDiagnostics {
        core::mem::take(&mut self.diagnostics)
    }

    pub(crate) fn trace_summary_seqs_mut(&mut self) -> &mut TraceSummarySeqs {
        &mut self.trace_summary_seqs
    }

    fn current_phase_id(&self) -> Option<PhaseId> {
        match &self.engine_mode {
            EngineMode::Inner { phase, .. } => phase.current_phase_id(),
            EngineMode::Outer { phase, .. } => phase.current_phase_id(),
        }
    }

    fn trace_profile_or_mode(&self) -> i32 {
        match &self.engine_mode {
            EngineMode::Inner {
                profile,
                sequence: InnerSequence::Commission,
                ..
            } => i32::from(profile.to_u8()),
            EngineMode::Outer { profile, .. } => i32::from(profile.to_u8()),
            EngineMode::Inner { .. } => 0,
        }
    }

    fn trace_event(
        &self,
        level: TraceEventLevel,
        event_id: u16,
        payload: [i32; 2],
        payload_count: u8,
    ) -> CommissionTraceEvent {
        CommissionTraceEvent {
            level,
            event_id,
            engine_kind: self.kind(),
            payload,
            payload_count,
        }
    }

    fn emit_trace_start<B: CommissioningBackend>(&mut self, backend: &mut B) {
        if self.trace_run.started {
            return;
        }

        self.trace_run.started = true;
        backend.record_commission_trace_event(self.trace_event(
            TraceEventLevel::Info,
            EVENT_COMMISSION_ENGINE_START,
            [
                self.trace_profile_or_mode(),
                u32_arg(self.trace_run.engine_seq),
            ],
            2,
        ));

        if let Some(phase) = self.current_phase_id() {
            self.trace_run.current_phase = Some(phase);
            backend.record_commission_trace_event(self.trace_event(
                TraceEventLevel::Info,
                EVENT_COMMISSION_PHASE_START,
                [i32::from(phase.to_u8()), u32_arg(self.trace_run.phase_seq)],
                2,
            ));
        }
    }

    fn emit_trace_phase_end<B: CommissioningBackend>(
        &mut self,
        backend: &mut B,
        status_code: u8,
        level: TraceEventLevel,
    ) {
        if let Some(phase) = self.trace_run.current_phase.take() {
            backend.record_commission_trace_event(self.trace_event(
                level,
                EVENT_COMMISSION_PHASE_END,
                [i32::from(phase.to_u8()), i32::from(status_code)],
                2,
            ));
        }
    }

    fn emit_trace_phase_start<B: CommissioningBackend>(&mut self, backend: &mut B, phase: PhaseId) {
        self.trace_run.current_phase = Some(phase);
        backend.record_commission_trace_event(self.trace_event(
            TraceEventLevel::Info,
            EVENT_COMMISSION_PHASE_START,
            [i32::from(phase.to_u8()), u32_arg(self.trace_run.phase_seq)],
            2,
        ));
    }

    fn emit_trace_terminal<B: CommissioningBackend>(
        &mut self,
        backend: &mut B,
        status_code: u8,
        level: TraceEventLevel,
    ) {
        if self.trace_run.terminal {
            return;
        }
        self.trace_run.terminal = true;
        backend.record_commission_trace_event(self.trace_event(
            level,
            EVENT_COMMISSION_ENGINE_TERMINAL,
            [i32::from(status_code), u32_arg(self.trace_run.engine_seq)],
            2,
        ));
    }

    fn status_level(status: CommissionStatus) -> TraceEventLevel {
        match status {
            CommissionStatus::Accepted => TraceEventLevel::Info,
            CommissionStatus::AcceptedWithWarnings | CommissionStatus::InnerOnly => {
                TraceEventLevel::Warn
            }
            CommissionStatus::Failed(_) => TraceEventLevel::Error,
        }
    }

    fn emit_trace_after_step<B: CommissioningBackend>(
        &mut self,
        backend: &mut B,
        result: &StepResult,
    ) {
        match result {
            StepResult::Continue => {}
            StepResult::PhaseChanged(phase) => {
                self.emit_trace_phase_end(backend, PHASE_END_OK, TraceEventLevel::Info);
                self.trace_run.phase_seq = self.trace_run.phase_seq.wrapping_add(1);
                self.emit_trace_phase_start(backend, *phase);
            }
            StepResult::CommissionDone(result) => {
                self.emit_trace_phase_end(backend, PHASE_END_OK, TraceEventLevel::Info);
                let level = Self::status_level(result.inner.status);
                self.emit_trace_terminal(backend, result.inner.status.to_u8(), level);
            }
            StepResult::OuterDone(result) => {
                self.emit_trace_phase_end(backend, PHASE_END_OK, TraceEventLevel::Info);
                let level = Self::status_level(result.status);
                self.emit_trace_terminal(backend, result.status.to_u8(), level);
            }
            StepResult::CalibrateDone(_) | StepResult::SelftestDone => {
                self.emit_trace_phase_end(backend, PHASE_END_OK, TraceEventLevel::Info);
                self.emit_trace_terminal(backend, PHASE_END_OK, TraceEventLevel::Info);
            }
            StepResult::InnerFailed(err) | StepResult::OuterFailed(err) => {
                let status = err.status_code();
                self.emit_trace_phase_end(backend, status, TraceEventLevel::Error);
                self.emit_trace_terminal(backend, status, TraceEventLevel::Error);
            }
        }
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
            // Defense in depth: dispatch already restores safe state on
            // terminal failures, but the shutdown abort path bypasses
            // step_inner / step_outer entirely. Drive the canonical helper
            // here so step processing is gated and the chip is de-energized
            // before the abort is reported.
            self.tmc_state
                .motor_armed
                .store(false, core::sync::atomic::Ordering::Relaxed);
            crate::tmc_safe_state::restore_safe_state_via_backend(backend).await;
            return self.abort(CommissionError::Shutdown);
        }

        self.emit_trace_start(backend);

        let selftest_results = &mut self.selftest_results;
        let diagnostics = &mut self.diagnostics;
        let trace_summary_seqs = &mut self.trace_summary_seqs;
        let result = match &mut self.engine_mode {
            EngineMode::Inner {
                phase,
                profile,
                sequence,
                adc_cal,
                calibrate_entry_gains,
            } => {
                inner::step_inner(
                    phase,
                    &mut self.buf,
                    *profile,
                    *sequence,
                    adc_cal,
                    *calibrate_entry_gains,
                    selftest_results,
                    diagnostics,
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
                tau_e_us,
                tau_e_crosscheck_us,
                tau_residual_permille,
                inner_warning_flags,
                ..
            } => {
                outer::step_outer_with_trace(
                    phase,
                    &mut self.buf,
                    *profile,
                    *mode,
                    *inner_lambda_us,
                    *theta_e_us,
                    *current_ringing,
                    *current_bandwidth_hz,
                    *tau_e_us,
                    *tau_e_crosscheck_us,
                    *tau_residual_permille,
                    *inner_warning_flags,
                    trace_summary_seqs,
                    backend,
                )
                .await
            }
        };
        self.emit_trace_after_step(backend, &result);
        result
    }

    /// Abort commissioning, restoring safe state.
    fn abort(&mut self, error: CommissionError) -> StepResult {
        match &self.engine_mode {
            EngineMode::Inner { .. } => StepResult::InnerFailed(error),
            EngineMode::Outer { .. } => StepResult::OuterFailed(error),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::commissioning::types::EngineKind;

    fn tmc_state() -> &'static crate::state::tmc::TmcChannelState {
        crate::state::tmc::tmc_channel0()
    }

    fn sample_outer_params() -> OuterTuneParams {
        OuterTuneParams {
            profile: crate::autotune::CommissionProfile::Balanced,
            mode: crate::autotune::OperatingMode::Nominal,
            inner_lambda_us: 100,
            theta_e_us: 50,
            current_ringing: false,
            current_bandwidth_hz: 1000,
            tau_e_us: 730,
            tau_e_crosscheck_us: 730,
            tau_residual_permille: 0,
            inner_warning_flags: 0,
        }
    }

    fn sample_entry_snapshot() -> EntrySnapshot {
        EntrySnapshot {
            velocity_p: 1152,
            velocity_i: 0,
            position_p: 640,
            position_i: 0,
            velocity_limit: 500_000,
            voltage_limit: crate::state::tmc::DEFAULT_OPERATIONAL_PIDOUT_UQ_UD_LIMIT,
            velocity_filter_hz: 0,
            torque_filter_hz: 0,
            position_filter_hz: 0,
            flux_filter_hz: 0,
        }
    }

    fn sample_inner_result(status: crate::autotune::CommissionStatus) -> InnerResult {
        InnerResult {
            status,
            r_int: 1706,
            l_int: 1245,
            r_mohm: 1700,
            l_uh: 3100,
            flux_p: 100,
            flux_i: 10,
            torque_p: 200,
            torque_i: 20,
            lambda_us: 100,
            theta_e_us: 50,
            ringing_count: 0,
            bandwidth_hz: 1000,
            tau_e_us: 730,
            tau_e_crosscheck_us: 730,
            tau_residual_permille: 0,
            inner_warning_flags: 0,
            resistance_profile_version: 1,
            resistance_selected_count_slope_milli: 1706,
            resistance_gain_path_count_slope_milli: 1706,
            resistance_axis0_count_slope_milli: 1700,
            resistance_axis1_count_slope_milli: 1712,
            resistance_axis0_intercept_count: 0,
            resistance_axis1_intercept_count: 0,
            resistance_axis0_rmse_permille: 0,
            resistance_axis1_rmse_permille: 0,
            resistance_selected_mask_axis0: 0b1111_1000,
            resistance_selected_mask_axis1: 0b1111_1000,
            resistance_axis0_signed_count_slope_milli: 1702,
            resistance_axis1_signed_count_slope_milli: 1710,
            resistance_axis0_signed_asymmetry_permille: 0,
            resistance_axis1_signed_asymmetry_permille: 0,
            resistance_axis0_drift_permille: 0,
            resistance_axis1_drift_permille: 0,
            resistance_status_flags_or: 0,
            resistance_warning_flags: 0,
        }
    }

    fn sample_commission_result(status: crate::autotune::CommissionStatus) -> CommissionResult {
        CommissionResult {
            inner: sample_inner_result(status),
            fallback_velocity_p: 1152,
            fallback_velocity_i: 0,
            fallback_position_p: 640,
            fallback_position_i: 0,
            fallback_velocity_limit: 500_000,
        }
    }

    fn sample_outer_result(status: crate::autotune::CommissionStatus) -> OuterResult {
        OuterResult {
            status,
            velocity_p: 1152,
            velocity_i: 0,
            velocity_limit: 500_000,
            position_p: 640,
            position_i: 0,
            velocity_filter_hz: 0,
            position_filter_hz: 0,
            flux_filter_hz: 0,
            torque_filter_hz: 0,
            j_eff: 0,
            b_eff: 0,
            warning_code: 0,
            mech_torque_step: 0,
            mech_fwd_accel: 0,
            mech_rev_accel: 0,
            mech_fwd_peak_velocity: 0,
            mech_rev_peak_velocity: 0,
            mech_fwd_travel: 0,
            mech_rev_travel: 0,
            synth_velocity_p: 0,
            synth_velocity_i: 0,
            synth_position_p: 0,
            synth_position_i: 0,
        }
    }

    fn sample_adc_cal_result() -> AdcCalResult {
        AdcCalResult {
            adc_i0: 11,
            adc_i1: 13,
            encoder_count: 17,
        }
    }

    fn assert_trace_event(
        event: &CommissionTraceEvent,
        event_id: u16,
        engine_kind: EngineKind,
        level: TraceEventLevel,
        payload: [i32; 2],
    ) {
        assert_eq!(event.event_id, event_id);
        assert_eq!(event.engine_kind, engine_kind);
        assert_eq!(event.level, level);
        assert_eq!(event.payload, payload);
        assert_eq!(event.payload_count, 2);
    }

    fn assert_terminal_trace(
        mut engine: CommissioningEngine,
        result: StepResult,
        engine_kind: EngineKind,
        phase_id: PhaseId,
        terminal_status_code: u8,
        phase_level: TraceEventLevel,
        terminal_level: TraceEventLevel,
    ) {
        let mut backend = crate::commission_dispatch::test_support::MockBackend::new();
        engine.emit_trace_start(&mut backend);
        engine.emit_trace_after_step(&mut backend, &result);

        let events = &backend.trace_events;
        assert_eq!(events.len(), 4);
        assert_trace_event(
            &events[2],
            crate::trace::events::EVENT_COMMISSION_PHASE_END,
            engine_kind,
            phase_level,
            [i32::from(phase_id.to_u8()), i32::from(PHASE_END_OK)],
        );
        assert_trace_event(
            &events[3],
            crate::trace::events::EVENT_COMMISSION_ENGINE_TERMINAL,
            engine_kind,
            terminal_level,
            [i32::from(terminal_status_code), 0],
        );
    }

    #[test]
    fn calibrate_constructor_uses_stored_gains_for_closed_loop_entry() {
        pollster::block_on(async {
            use core::sync::atomic::Ordering::Relaxed;

            let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
                .lock()
                .await;
            let tmc_state = tmc_state();
            tmc_state.velocity_p_value.store(1434, Relaxed);
            tmc_state.velocity_i_value.store(2, Relaxed);
            tmc_state.position_p_value.store(627, Relaxed);
            tmc_state.position_i_value.store(1, Relaxed);
            tmc_state.velocity_limit_cfg.store(500_000, Relaxed);
            tmc_state.velocity_filter_hz.store(11, Relaxed);
            tmc_state.torque_filter_hz.store(13, Relaxed);
            tmc_state.position_filter_hz.store(17, Relaxed);
            tmc_state.flux_filter_hz.store(19, Relaxed);

            let inner = crate::pid::InnerLoopGains {
                flux_p: 711,
                flux_i: 159,
                torque_p: 711,
                torque_i: 159,
            };

            let engine =
                CommissioningEngine::new_calibrate_for_tmc_with_gains(0, tmc_state, Some(inner));

            let EngineMode::Inner {
                calibrate_entry_gains: Some(gains),
                ..
            } = engine.engine_mode
            else {
                panic!("calibrate constructor should snapshot entry gains");
            };

            assert_eq!(gains.inner, inner);
            assert_eq!(gains.velocity_p, 1434);
            assert_eq!(gains.velocity_i, 2);
            assert_eq!(gains.position_p, 627);
            assert_eq!(gains.position_i, 1);
            assert_eq!(gains.velocity_limit, 500_000);
            assert_eq!(gains.velocity_filter_hz, 11);
            assert_eq!(gains.torque_filter_hz, 13);
            assert_eq!(gains.position_filter_hz, 17);
            assert_eq!(gains.flux_filter_hz, 19);
        });
    }

    #[test]
    fn engine_kind_matches_constructor() {
        assert_eq!(
            CommissioningEngine::new_commission(0, crate::autotune::CommissionProfile::Balanced)
                .kind(),
            EngineKind::Commission,
        );
        assert_eq!(
            CommissioningEngine::new_calibrate(0).kind(),
            EngineKind::Calibrate
        );
        assert_eq!(
            CommissioningEngine::new_selftest(0).kind(),
            EngineKind::Selftest
        );
        let params = sample_outer_params();
        let snapshot = sample_entry_snapshot();
        assert_eq!(
            CommissioningEngine::new_outer(0, params, snapshot).kind(),
            EngineKind::OuterTune,
        );
    }

    #[test]
    fn commission_step_emits_engine_start_and_initial_phase_start() {
        pollster::block_on(async {
            let mut engine = CommissioningEngine::new_commission_with_trace_seq(
                0,
                crate::autotune::CommissionProfile::Balanced,
                42,
            );
            let mut backend = crate::commission_dispatch::test_support::MockBackend::new();

            let _ = engine.step(&mut backend).await;

            let events = &backend.trace_events;
            assert_eq!(
                events[0].event_id,
                crate::trace::events::EVENT_COMMISSION_ENGINE_START
            );
            assert_eq!(events[0].engine_kind, EngineKind::Commission);
            assert_eq!(
                events[0].payload,
                [
                    crate::autotune::CommissionProfile::Balanced.to_u8() as i32,
                    42,
                ]
            );
            assert_eq!(
                events[1].event_id,
                crate::trace::events::EVENT_COMMISSION_PHASE_START
            );
            assert_eq!(
                events[1].payload[0],
                crate::autotune::PhaseId::AdcCalibrate.to_u8() as i32
            );
            assert_eq!(events[1].payload[1], 0);
        });
    }

    #[test]
    fn phase_changed_trace_ends_previous_phase_and_starts_next_sequence() {
        let mut engine = CommissioningEngine::new_commission_with_trace_seq(
            0,
            crate::autotune::CommissionProfile::Balanced,
            12,
        );
        let mut backend = crate::commission_dispatch::test_support::MockBackend::new();

        engine.emit_trace_start(&mut backend);
        engine.emit_trace_after_step(
            &mut backend,
            &StepResult::PhaseChanged(crate::autotune::PhaseId::CoilCheck),
        );

        let events = &backend.trace_events;
        assert_eq!(events.len(), 4);
        assert_trace_event(
            &events[2],
            crate::trace::events::EVENT_COMMISSION_PHASE_END,
            EngineKind::Commission,
            TraceEventLevel::Info,
            [crate::autotune::PhaseId::AdcCalibrate.to_u8() as i32, 0],
        );
        assert_trace_event(
            &events[3],
            crate::trace::events::EVENT_COMMISSION_PHASE_START,
            EngineKind::Commission,
            TraceEventLevel::Info,
            [crate::autotune::PhaseId::CoilCheck.to_u8() as i32, 1],
        );
    }

    #[test]
    fn terminal_success_trace_covers_all_engine_paths() {
        assert_terminal_trace(
            CommissioningEngine::new_commission(0, crate::autotune::CommissionProfile::Balanced),
            StepResult::CommissionDone(sample_commission_result(
                crate::autotune::CommissionStatus::Accepted,
            )),
            EngineKind::Commission,
            crate::autotune::PhaseId::AdcCalibrate,
            0,
            TraceEventLevel::Info,
            TraceEventLevel::Info,
        );
        assert_terminal_trace(
            CommissioningEngine::new_outer(0, sample_outer_params(), sample_entry_snapshot()),
            StepResult::OuterDone(sample_outer_result(
                crate::autotune::CommissionStatus::Accepted,
            )),
            EngineKind::OuterTune,
            crate::autotune::PhaseId::MechanicalId,
            0,
            TraceEventLevel::Info,
            TraceEventLevel::Info,
        );
        assert_terminal_trace(
            CommissioningEngine::new_calibrate(0),
            StepResult::CalibrateDone(sample_adc_cal_result()),
            EngineKind::Calibrate,
            crate::autotune::PhaseId::AdcCalibrate,
            0,
            TraceEventLevel::Info,
            TraceEventLevel::Info,
        );
        assert_terminal_trace(
            CommissioningEngine::new_selftest(0),
            StepResult::SelftestDone,
            EngineKind::Selftest,
            crate::autotune::PhaseId::AdcCalibrate,
            0,
            TraceEventLevel::Info,
            TraceEventLevel::Info,
        );
    }

    #[test]
    fn failure_trace_ends_active_inner_phase_and_is_idempotent() {
        let mut engine = CommissioningEngine::new_commission_with_trace_seq(
            0,
            crate::autotune::CommissionProfile::Balanced,
            21,
        );
        let mut backend = crate::commission_dispatch::test_support::MockBackend::new();
        let err = crate::autotune::CommissionError::ChipResetDetected;

        engine.emit_trace_start(&mut backend);
        engine.emit_trace_after_step(&mut backend, &StepResult::InnerFailed(err));

        let status = err.status_code();
        let events = &backend.trace_events;
        assert_eq!(events.len(), 4);
        assert_trace_event(
            &events[2],
            crate::trace::events::EVENT_COMMISSION_PHASE_END,
            EngineKind::Commission,
            TraceEventLevel::Error,
            [
                crate::autotune::PhaseId::AdcCalibrate.to_u8() as i32,
                i32::from(status),
            ],
        );
        assert_trace_event(
            &events[3],
            crate::trace::events::EVENT_COMMISSION_ENGINE_TERMINAL,
            EngineKind::Commission,
            TraceEventLevel::Error,
            [i32::from(status), 21],
        );

        engine.emit_trace_after_step(&mut backend, &StepResult::InnerFailed(err));
        assert_eq!(backend.trace_events.len(), 4);
    }

    #[test]
    fn failure_trace_ends_active_outer_phase_and_is_idempotent() {
        let mut engine = CommissioningEngine::new_outer_with_trace_seq(
            0,
            sample_outer_params(),
            sample_entry_snapshot(),
            22,
        );
        let mut backend = crate::commission_dispatch::test_support::MockBackend::new();
        let err = crate::autotune::CommissionError::VelocityValidationFailed;

        engine.emit_trace_start(&mut backend);
        engine.emit_trace_after_step(&mut backend, &StepResult::OuterFailed(err));

        let status = err.status_code();
        let events = &backend.trace_events;
        assert_eq!(events.len(), 4);
        assert_trace_event(
            &events[2],
            crate::trace::events::EVENT_COMMISSION_PHASE_END,
            EngineKind::OuterTune,
            TraceEventLevel::Error,
            [
                crate::autotune::PhaseId::MechanicalId.to_u8() as i32,
                i32::from(status),
            ],
        );
        assert_trace_event(
            &events[3],
            crate::trace::events::EVENT_COMMISSION_ENGINE_TERMINAL,
            EngineKind::OuterTune,
            TraceEventLevel::Error,
            [i32::from(status), 22],
        );

        engine.emit_trace_after_step(&mut backend, &StepResult::OuterFailed(err));
        assert_eq!(backend.trace_events.len(), 4);
    }

    #[test]
    fn calibrate_and_selftest_trace_start_use_zero_profile_or_mode() {
        let mut backend = crate::commission_dispatch::test_support::MockBackend::new();
        let mut calibrate = CommissioningEngine::new_calibrate_with_trace_seq(0, 31);
        calibrate.emit_trace_start(&mut backend);
        assert_trace_event(
            &backend.trace_events[0],
            crate::trace::events::EVENT_COMMISSION_ENGINE_START,
            EngineKind::Calibrate,
            TraceEventLevel::Info,
            [0, 31],
        );

        backend.trace_events.clear();
        let mut selftest = CommissioningEngine::new_selftest_with_trace_seq(0, 32);
        selftest.emit_trace_start(&mut backend);
        assert_trace_event(
            &backend.trace_events[0],
            crate::trace::events::EVENT_COMMISSION_ENGINE_START,
            EngineKind::Selftest,
            TraceEventLevel::Info,
            [0, 32],
        );
    }

    #[test]
    fn warning_terminal_statuses_emit_warn_level() {
        assert_terminal_trace(
            CommissioningEngine::new_commission(0, crate::autotune::CommissionProfile::Balanced),
            StepResult::CommissionDone(sample_commission_result(
                crate::autotune::CommissionStatus::AcceptedWithWarnings,
            )),
            EngineKind::Commission,
            crate::autotune::PhaseId::AdcCalibrate,
            crate::autotune::CommissionStatus::AcceptedWithWarnings.to_u8(),
            TraceEventLevel::Info,
            TraceEventLevel::Warn,
        );
        assert_terminal_trace(
            CommissioningEngine::new_outer(0, sample_outer_params(), sample_entry_snapshot()),
            StepResult::OuterDone(sample_outer_result(
                crate::autotune::CommissionStatus::InnerOnly,
            )),
            EngineKind::OuterTune,
            crate::autotune::PhaseId::MechanicalId,
            crate::autotune::CommissionStatus::InnerOnly.to_u8(),
            TraceEventLevel::Info,
            TraceEventLevel::Warn,
        );
    }

    #[test]
    fn step_restores_safe_state_when_shutdown_requested_mid_inner() {
        pollster::block_on(async {
            use core::sync::atomic::Ordering;
            let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
                .lock()
                .await;
            let mut engine = CommissioningEngine::new_commission(
                0,
                crate::autotune::CommissionProfile::Balanced,
            );
            let mut backend = crate::commission_dispatch::test_support::MockBackend::new();
            backend.shutdown_requested = true;
            backend.enable_pin = true;
            tmc_state().motor_armed.store(true, Ordering::Relaxed);

            let result = engine.step(&mut backend).await;

            assert!(matches!(
                result,
                crate::commissioning::types::StepResult::InnerFailed(
                    crate::autotune::CommissionError::Shutdown,
                )
            ));
            assert!(!backend.enable_pin, "shutdown abort must lower enable pin");
            assert!(
                !tmc_state().motor_armed.load(Ordering::Relaxed),
                "shutdown abort must gate step processing before reporting failure"
            );
        });
    }
}
