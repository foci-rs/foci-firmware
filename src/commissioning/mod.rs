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
pub mod capture_timing;
pub(crate) mod encoder_domain;
pub mod helpers;
pub mod inner;
pub mod outer;
pub mod schedule;
pub mod types;

use crate::autotune::{
    CommissionError, CommissionProfile, CommissionStatus, InnerSequence, OperatingMode, PhaseId,
    TransientBuffer,
};
use crate::commissioning::inner::InnerPhase;
#[cfg(not(feature = "trace-velocity-sweep"))]
use crate::commissioning::outer::{MechanicalIdData, MechanicalIdState};
use crate::commissioning::outer::{
    OuterMotionBudget, OuterPhase, OuterSafetyLimits, TraceSummarySeqs,
};
#[cfg(feature = "trace-velocity-sweep")]
use crate::commissioning::outer::{
    VelocityIntegralData, VelocityIntegralState, VelocitySweepData, VelocitySweepState,
};
use crate::commissioning::types::{AdcCalResult, EngineKind, StepResult};
use crate::pid::InnerLoopGains;
use crate::state::tmc::TmcChannelState;
use crate::trace::events::{
    CommissionTraceEvent, EVENT_COMMISSION_ENGINE_START, EVENT_COMMISSION_ENGINE_TERMINAL,
    EVENT_COMMISSION_PHASE_END, EVENT_COMMISSION_PHASE_START,
    EVENT_COMMISSION_VELOCITY_INTEGRAL_OBSERVATION, EVENT_COMMISSION_VELOCITY_INTEGRAL_PLAN,
    EVENT_COMMISSION_VELOCITY_INTEGRAL_RUNG, EVENT_COMMISSION_VELOCITY_SWEEP_COMPLETE,
    EVENT_COMMISSION_VELOCITY_SWEEP_DERIVED, EVENT_COMMISSION_VELOCITY_SWEEP_OBSERVATION,
    EVENT_COMMISSION_VELOCITY_SWEEP_PLAN, EVENT_COMMISSION_VELOCITY_SWEEP_RUNG,
    EVENT_COMMISSION_VELOCITY_SWEEP_TERMINAL, TraceEventLevel, u32_arg,
};
#[cfg(feature = "trace-velocity-sweep")]
use crate::trace::events::{
    EVENT_COMMISSION_OUTER_SAFETY_TIMING, EVENT_COMMISSION_RUNG_ORIGIN_RECOVERY,
    EVENT_COMMISSION_VELOCITY_INTEGRAL_COMPLETE, EVENT_COMMISSION_VELOCITY_INTEGRAL_DERIVED,
    EVENT_COMMISSION_VELOCITY_INTEGRAL_TERMINAL,
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
    /// Inner-loop time scale from Stage 1 (microseconds).
    pub inner_lambda_us: u32,
    /// Theta-E delay from Stage 1 (microseconds).
    pub theta_e_us: u32,
    /// Whether the Stage 1 current step response showed ringing.
    pub current_ringing: bool,
    /// Current-loop bandwidth from Stage 1 (Hz).
    pub current_bandwidth_hz: u32,
    /// Inner-confidence warning bitfield carried from Stage 1.
    pub inner_warning_flags: u8,
    /// Host-requested mechanical velocity amplitude in milli-revolutions/s.
    pub requested_velocity_mrev_s: u32,
    /// Kinematics-derived absolute machine velocity ceiling.
    pub machine_velocity_ceiling_mrev_s: u32,
    /// Whether the requested amplitude was explicit or host-defaulted.
    pub requested_velocity_source:
        crate::commissioning::outer::velocity::sweep_plan::RequestedVelocitySource,
    /// Full displacement window spanning a moving stroke and its settle.
    pub max_stroke_travel_mrev: u32,
    /// Host planning reserve for braking and settle drift.
    pub settle_travel_reserve_mrev: u32,
    /// Negative absolute-position headroom from the Stage-2 entry anchor.
    pub negative_position_headroom_mrev: u32,
    /// Positive absolute-position headroom from the Stage-2 entry anchor.
    pub positive_position_headroom_mrev: u32,
    /// Host-computed max primitive duration in milliseconds.
    pub max_duration_ms: u16,
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
    /// Validated outer-loop gains, or `None` when retained storage was corrupt.
    pub outer: Option<crate::pid::OuterLoopGains>,
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
    fn from_tmc_state(
        tmc_state: &'static TmcChannelState,
        inner: Option<InnerLoopGains>,
        current_filter_default_hz: u16,
    ) -> Self {
        use core::sync::atomic::Ordering::Relaxed;

        Self {
            inner: inner.unwrap_or(crate::pid::DEFAULT_INNER_GAINS),
            outer: tmc_state.load_outer_gains(Relaxed).ok(),
            velocity_limit: tmc_state.velocity_limit_cfg.load(Relaxed),
            velocity_filter_hz: tmc_state.velocity_filter_hz.load_or_zero(Relaxed),
            torque_filter_hz: tmc_state
                .torque_filter_hz
                .load(Relaxed)
                .unwrap_or(current_filter_default_hz),
            position_filter_hz: tmc_state.position_filter_hz.load_or_zero(Relaxed),
            flux_filter_hz: tmc_state
                .flux_filter_hz
                .load(Relaxed)
                .unwrap_or(current_filter_default_hz),
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
        /// Inner-loop time scale in microseconds.
        inner_lambda_us: u32,
        /// Electrical transport delay in microseconds.
        theta_e_us: u32,
        /// Whether current-loop ringing was detected during inner validation.
        current_ringing: bool,
        /// Current-loop effective bandwidth Hz from inner commissioning.
        current_bandwidth_hz: u32,
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
        Self::new_calibrate_for_tmc_with_gains_and_trace_seq(
            oid,
            tmc_state,
            None,
            0,
            trace_engine_seq,
        )
    }

    /// Create a calibration engine bound to a TMC channel state and stored gains.
    pub fn new_calibrate_for_tmc_with_gains(
        oid: u8,
        tmc_state: &'static TmcChannelState,
        inner_gains: Option<InnerLoopGains>,
    ) -> Self {
        Self::new_calibrate_for_tmc_with_gains_and_trace_seq(oid, tmc_state, inner_gains, 0, 0)
    }

    /// Create a calibration engine bound to a TMC channel state, stored gains, and trace sequence.
    pub fn new_calibrate_for_tmc_with_gains_and_trace_seq(
        oid: u8,
        tmc_state: &'static TmcChannelState,
        inner_gains: Option<InnerLoopGains>,
        current_filter_default_hz: u16,
        trace_engine_seq: u32,
    ) -> Self {
        let calibrate_entry_gains = Some(CalibrateEntryGains::from_tmc_state(
            tmc_state,
            inner_gains,
            current_filter_default_hz,
        ));
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
        let OuterTuneParams {
            profile,
            mode,
            inner_lambda_us,
            theta_e_us,
            current_ringing,
            current_bandwidth_hz,
            inner_warning_flags,
            requested_velocity_mrev_s,
            machine_velocity_ceiling_mrev_s,
            requested_velocity_source,
            max_stroke_travel_mrev,
            settle_travel_reserve_mrev,
            negative_position_headroom_mrev,
            positive_position_headroom_mrev,
            max_duration_ms,
        } = params;
        let budget = OuterMotionBudget {
            requested_velocity_mrev_s,
            machine_velocity_ceiling_mrev_s,
            requested_velocity_source,
            max_stroke_travel_mrev,
            settle_travel_reserve_mrev,
            negative_position_headroom_mrev,
            positive_position_headroom_mrev,
            max_duration_ms,
        };
        let safety = OuterSafetyLimits::new(0, budget);
        #[cfg(not(feature = "trace-velocity-sweep"))]
        let phase = OuterPhase::MechanicalId {
            state: MechanicalIdState::Setup,
            data: MechanicalIdData::new(0),
            safety,
        };
        #[cfg(feature = "trace-velocity-sweep")]
        let phase = OuterPhase::VelocitySweep {
            state: VelocitySweepState::Setup,
            data: VelocitySweepData::new(trace_engine_seq),
            safety,
        };
        Self {
            oid,
            tmc_state,
            engine_mode: EngineMode::Outer {
                phase,
                profile,
                mode,
                inner_lambda_us,
                theta_e_us,
                current_ringing,
                current_bandwidth_hz,
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

    /// Create the trace-gated Stage B engine from the sole pre-motion plan.
    #[cfg(feature = "trace-velocity-sweep")]
    pub fn new_outer_for_tmc_with_stage_b_preflight(
        oid: u8,
        tmc_state: &'static TmcChannelState,
        params: OuterTuneParams,
        entry_snapshot: EntrySnapshot,
        preflight: crate::commission_dispatch::stage_b::StageBPreflight,
    ) -> Self {
        let mut engine = Self::new_outer_for_tmc_with_trace_seq(
            oid,
            tmc_state,
            params,
            entry_snapshot,
            preflight.run_sequence,
        );
        let EngineMode::Outer { phase, .. } = &mut engine.engine_mode else {
            unreachable!("outer constructor must produce an outer engine")
        };
        let OuterPhase::VelocitySweep { data, .. } = phase else {
            unreachable!("trace Stage B constructor must enter the velocity sweep")
        };
        *data = VelocitySweepData::new_with_plan(preflight.run_sequence, preflight.plan);
        engine
    }

    /// Derive the exact Stage-C continuation plan without changing the live phase.
    #[cfg(feature = "trace-velocity-sweep")]
    pub(crate) fn plan_stage_c_continuation(
        &self,
        authority: crate::commissioning::outer::velocity::integral_plan::StageCAuthority,
        pwm_hz: u32,
    ) -> Result<
        crate::commissioning::outer::velocity::integral_plan::VelocityIntegralPlan,
        crate::commissioning::outer::velocity::integral_plan::VelocityIntegralPlanError,
    > {
        use crate::commissioning::outer::velocity::integral_plan::{
            VelocityIntegralPlanError, plan_velocity_integral_response,
        };

        let EngineMode::Outer { phase, .. } = &self.engine_mode else {
            return Err(VelocityIntegralPlanError::MissingStageBPlan);
        };
        let (stage_b_plan, budget) = match phase {
            OuterPhase::VelocitySweep { data, safety, .. } => (data.plan, safety.budget),
            _ => return Err(VelocityIntegralPlanError::MissingStageBPlan),
        };
        let counts_per_rev = self
            .tmc_state
            .encoder_counts_per_rev()
            .ok_or(VelocityIntegralPlanError::InvalidRequest)?;
        plan_velocity_integral_response(authority, stage_b_plan, budget, pwm_hz, counts_per_rev)
    }

    /// Transition a live reproduced Stage-B engine into a prederived Stage-C plan.
    #[cfg(feature = "trace-velocity-sweep")]
    pub(crate) fn begin_stage_c_continuation_with_plan(
        &mut self,
        plan: crate::commissioning::outer::velocity::integral_plan::VelocityIntegralPlan,
    ) -> Result<(), crate::commissioning::outer::velocity::integral_plan::VelocityIntegralPlanError>
    {
        use crate::commissioning::outer::velocity::integral_plan::VelocityIntegralPlanError;

        let EngineMode::Outer { phase, .. } = &mut self.engine_mode else {
            return Err(VelocityIntegralPlanError::MissingStageBPlan);
        };
        let execution_plan = plan.execution_plan();
        let old_phase =
            core::mem::replace(phase, OuterPhase::Failed(CommissionError::InvalidSchedule));
        let OuterPhase::VelocitySweep { safety, .. } = old_phase else {
            unreachable!("Stage-C continuation validated a Stage-B phase")
        };
        *phase = OuterPhase::VelocityIntegral {
            state: VelocityIntegralState::Setup,
            data: VelocityIntegralData::new(self.trace_run.engine_seq, execution_plan),
            safety,
        };
        Ok(())
    }

    /// Create a direct Stage-C resume engine from a preflighted exact plan.
    #[cfg(feature = "trace-velocity-sweep")]
    pub fn new_outer_for_tmc_with_stage_c_preflight(
        oid: u8,
        tmc_state: &'static TmcChannelState,
        params: OuterTuneParams,
        entry_snapshot: EntrySnapshot,
        preflight: crate::commission_dispatch::stage_c::StageCPreflight,
    ) -> Self {
        let mut engine = Self::new_outer_for_tmc_with_trace_seq(
            oid,
            tmc_state,
            params,
            entry_snapshot,
            preflight.run_sequence,
        );
        let EngineMode::Outer { phase, .. } = &mut engine.engine_mode else {
            unreachable!("outer constructor must produce an outer engine")
        };
        let old_phase =
            core::mem::replace(phase, OuterPhase::Failed(CommissionError::InvalidSchedule));
        let OuterPhase::VelocitySweep { safety, .. } = old_phase else {
            unreachable!("trace outer constructor must enter Stage B")
        };
        *phase = OuterPhase::VelocityIntegral {
            state: VelocityIntegralState::Setup,
            data: VelocityIntegralData::new(
                preflight.run_sequence,
                preflight.plan.execution_plan(),
            ),
            safety,
        };
        engine
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

    #[cfg(test)]
    pub(crate) fn set_inductance_capture_for_test(
        &mut self,
        capture: inner::InductanceCaptureState,
    ) {
        let EngineMode::Inner { phase, .. } = &mut self.engine_mode else {
            panic!("inductance capture fixture requires an inner engine");
        };
        *phase = InnerPhase::ElectricalId {
            state: inner::ElectricalIdState::InductanceSample { capture },
            data: inner::ElectricalIdData::new(1_024),
        };
    }

    #[cfg(test)]
    pub(crate) fn set_delay_capture_for_test(&mut self, capture: inner::DelayCaptureState) {
        let EngineMode::Inner { phase, .. } = &mut self.engine_mode else {
            panic!("delay capture fixture requires an inner engine");
        };
        self.buf
            .reset(inner::electrical_identification::ELECTRICAL_SAMPLE_PERIOD_US as u16);
        *phase = InnerPhase::ElectricalId {
            state: inner::ElectricalIdState::DelaySample { capture },
            data: inner::ElectricalIdData::new(1_024),
        };
    }

    /// Return the entry snapshot for the outer engine, if active.
    pub fn entry_snapshot(&self) -> Option<EntrySnapshot> {
        match &self.engine_mode {
            EngineMode::Outer { entry_snapshot, .. } => Some(*entry_snapshot),
            _ => None,
        }
    }

    /// Return retained Stage 2 safety-envelope evidence for a safety terminal state.
    pub fn outer_safety_fault(&self) -> Option<crate::commissioning::types::OuterSafetyFault> {
        match &self.engine_mode {
            EngineMode::Outer {
                phase: OuterPhase::FailedWithSafety { safety_fault, .. },
                ..
            } => Some(*safety_fault),
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

    #[cfg(feature = "trace-velocity-sweep")]
    pub(crate) fn emit_outer_safety_timing_trace<B: CommissioningBackend>(
        &mut self,
        backend: &mut B,
        run_sequence: u32,
    ) {
        let summary = self.trace_summary_seqs.take_outer_safety_timing();
        backend.record_velocity_sweep_trace_record(
            crate::ankyra::velocity_sweep_encoding::encode_outer_safety_timing_record(
                self.trace_run.engine_seq,
                run_sequence,
                summary,
            ),
        );
        backend.record_commission_trace_event(self.trace_event(
            TraceEventLevel::Info,
            EVENT_COMMISSION_OUTER_SAFETY_TIMING,
            [
                u32_arg(self.trace_run.engine_seq),
                u32_arg(summary.max_gap_us),
            ],
            2,
        ));
    }

    #[cfg(feature = "trace-velocity-sweep")]
    pub(crate) fn trace_velocity_run_sequence(&self) -> Option<u32> {
        match &self.engine_mode {
            EngineMode::Outer {
                phase: OuterPhase::VelocitySweep { data, .. },
                ..
            } => Some(data.integrity.run_sequence()),
            EngineMode::Outer {
                phase: OuterPhase::VelocityIntegral { data, .. },
                ..
            } => Some(data.integrity.run_sequence()),
            EngineMode::Inner { .. }
            | EngineMode::Outer {
                phase:
                    OuterPhase::MechanicalId { .. }
                    | OuterPhase::VelocityTune { .. }
                    | OuterPhase::VelocityValidate { .. }
                    | OuterPhase::PositionTune { .. }
                    | OuterPhase::FilterTune { .. }
                    | OuterPhase::Commit { .. }
                    | OuterPhase::Done(_)
                    | OuterPhase::Failed(_)
                    | OuterPhase::FailedWithSafety { .. },
                ..
            } => None,
        }
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

    fn emit_velocity_sweep_terminal_lifecycle<B: CommissioningBackend>(
        &mut self,
        backend: &mut B,
        terminal: crate::commissioning::types::VelocitySweepTerminal,
    ) {
        let level = match terminal.outcome {
            crate::commissioning::types::VelocitySweepOutcome::CompleteCandidate
            | crate::commissioning::types::VelocitySweepOutcome::Complete => TraceEventLevel::Info,
            crate::commissioning::types::VelocitySweepOutcome::Inconclusive
            | crate::commissioning::types::VelocitySweepOutcome::RejectedPlanMismatch => {
                TraceEventLevel::Warn
            }
            crate::commissioning::types::VelocitySweepOutcome::Fault => TraceEventLevel::Error,
        };
        self.emit_trace_phase_end(backend, PHASE_END_OK, TraceEventLevel::Info);
        backend.record_commission_trace_event(CommissionTraceEvent {
            level,
            event_id: EVENT_COMMISSION_VELOCITY_SWEEP_TERMINAL,
            engine_kind: EngineKind::OuterTune,
            payload: [terminal.cause.into(), terminal.digest as i32],
            payload_count: 2,
        });
        if terminal.outcome == crate::commissioning::types::VelocitySweepOutcome::Complete {
            backend.record_commission_trace_event(CommissionTraceEvent {
                level: TraceEventLevel::Info,
                event_id: EVENT_COMMISSION_VELOCITY_SWEEP_COMPLETE,
                engine_kind: EngineKind::OuterTune,
                payload: [terminal.digest as i32, (terminal.digest >> 32) as i32],
                payload_count: 2,
            });
        }
        // VelocitySweepOutcome and its phase-local cause deliberately do not share the
        // CommissionStatus/CommissionError namespace. The dedicated sweep event owns
        // this trace lifecycle for complete, inconclusive, and fault outcomes alike.
        self.trace_run.terminal = true;
    }

    /// Emit the board-authoritative Stage B comparison and terminal after candidate policy.
    #[cfg(feature = "trace-velocity-sweep")]
    #[inline(never)]
    pub(crate) fn emit_stage_b_terminal_trace<B: CommissioningBackend>(
        &mut self,
        backend: &mut B,
        terminal: crate::commissioning::types::VelocitySweepTerminal,
        reproduction: Option<crate::commission_dispatch::stage_b::StageBReproductionReport>,
    ) {
        if let Some(report) = reproduction {
            crate::commission_dispatch::stage_b::for_each_stage_b_reproduction_record(
                report,
                |record| backend.record_velocity_sweep_trace_record(record),
            );
        }
        self.emit_outer_safety_timing_trace(backend, terminal.run_sequence);
        backend.record_velocity_sweep_trace_record(
            crate::commissioning::outer::velocity::sweep_reporting::encode_velocity_sweep_terminal(
                terminal,
                backend.mechanical_encoder_quadrature_counts_per_rev(),
                backend.tmc_config().pwm_freq_hz(),
            ),
        );
        self.emit_velocity_sweep_terminal_lifecycle(backend, terminal);
    }

    /// Emit Stage-B terminal evidence without closing a live Stage-C continuation.
    #[cfg(feature = "trace-velocity-sweep")]
    pub(crate) fn emit_stage_b_terminal_evidence<B: CommissioningBackend>(
        &mut self,
        backend: &mut B,
        terminal: crate::commissioning::types::VelocitySweepTerminal,
        reproduction: Option<crate::commission_dispatch::stage_b::StageBReproductionReport>,
    ) {
        if let Some(report) = reproduction {
            crate::commission_dispatch::stage_b::for_each_stage_b_reproduction_record(
                report,
                |record| backend.record_velocity_sweep_trace_record(record),
            );
        }
        backend.record_velocity_sweep_trace_record(
            crate::commissioning::outer::velocity::sweep_reporting::encode_velocity_sweep_terminal(
                terminal,
                backend.mechanical_encoder_quadrature_counts_per_rev(),
                backend.tmc_config().pwm_freq_hz(),
            ),
        );
    }

    /// Emit the authoritative Stage-C terminal after retained-evidence policy.
    #[cfg(feature = "trace-velocity-sweep")]
    #[inline(never)]
    pub(crate) fn emit_stage_c_terminal_trace<B: CommissioningBackend>(
        &mut self,
        backend: &mut B,
        terminal: crate::commissioning::types::VelocityIntegralTerminal,
        reproduction: Option<crate::commission_dispatch::stage_c::StageCReproductionReport>,
    ) {
        self.emit_outer_safety_timing_trace(backend, terminal.run_sequence);
        backend.record_velocity_sweep_trace_record(
            crate::commissioning::outer::velocity::integral_reporting::encode_velocity_integral_terminal(
                terminal,
            ),
        );
        if reproduction.is_some() {
            backend.record_commission_trace_event(CommissionTraceEvent {
                level: TraceEventLevel::Info,
                event_id: EVENT_COMMISSION_VELOCITY_INTEGRAL_DERIVED,
                engine_kind: EngineKind::OuterTune,
                payload: [i32::from(terminal.evidence_sequence), 0],
                payload_count: 1,
            });
        }
        let level = match terminal.outcome {
            crate::commissioning::types::VelocityIntegralOutcome::CompleteCandidate
            | crate::commissioning::types::VelocityIntegralOutcome::Complete => {
                TraceEventLevel::Info
            }
            crate::commissioning::types::VelocityIntegralOutcome::Inconclusive
            | crate::commissioning::types::VelocityIntegralOutcome::RejectedPlanMismatch => {
                TraceEventLevel::Warn
            }
            crate::commissioning::types::VelocityIntegralOutcome::Fault => TraceEventLevel::Error,
        };
        self.emit_trace_phase_end(backend, PHASE_END_OK, TraceEventLevel::Info);
        backend.record_commission_trace_event(CommissionTraceEvent {
            level,
            event_id: EVENT_COMMISSION_VELOCITY_INTEGRAL_TERMINAL,
            engine_kind: EngineKind::OuterTune,
            payload: [terminal.cause.into(), terminal.digest as i32],
            payload_count: 2,
        });
        if terminal.outcome == crate::commissioning::types::VelocityIntegralOutcome::Complete {
            backend.record_commission_trace_event(CommissionTraceEvent {
                level: TraceEventLevel::Info,
                event_id: EVENT_COMMISSION_VELOCITY_INTEGRAL_COMPLETE,
                engine_kind: EngineKind::OuterTune,
                payload: [terminal.digest as i32, (terminal.digest >> 32) as i32],
                payload_count: 2,
            });
        }
        self.trace_run.terminal = true;
    }

    fn emit_trace_after_step<B: CommissioningBackend>(
        &mut self,
        backend: &mut B,
        result: &StepResult,
    ) {
        match result {
            StepResult::ReadyNow | StepResult::At(_) => {}
            StepResult::PhaseChanged(phase) => {
                self.emit_trace_phase_end(backend, PHASE_END_OK, TraceEventLevel::Info);
                self.trace_run.phase_seq = self.trace_run.phase_seq.wrapping_add(1);
                self.emit_trace_phase_start(backend, *phase);
            }
            StepResult::VelocitySweepEvent(event) => {
                #[cfg(all(feature = "trace", feature = "trace-velocity-sweep"))]
                backend.record_velocity_sweep_trace_record(
                    crate::commissioning::outer::velocity::sweep_reporting::encode_velocity_sweep_event(
                        *event,
                    ),
                );
                #[cfg(all(feature = "trace", feature = "trace-velocity-sweep"))]
                if let crate::commissioning::types::VelocitySweepEvent::Observation(report) = event
                    && let Some(record) = crate::commissioning::outer::velocity::sweep_reporting::encode_velocity_sweep_structured_boundary(*report)
                {
                    backend.record_velocity_sweep_trace_record(record);
                }
                let (event_id, sequence) = match event {
                    crate::commissioning::types::VelocitySweepEvent::Planned(report) => (
                        EVENT_COMMISSION_VELOCITY_SWEEP_PLAN,
                        report.evidence_sequence,
                    ),
                    crate::commissioning::types::VelocitySweepEvent::Observation(report) => (
                        EVENT_COMMISSION_VELOCITY_SWEEP_OBSERVATION,
                        report.evidence_sequence,
                    ),
                    crate::commissioning::types::VelocitySweepEvent::Rung(report) => (
                        EVENT_COMMISSION_VELOCITY_SWEEP_RUNG,
                        report.evidence_sequence,
                    ),
                    #[cfg(feature = "trace-velocity-sweep")]
                    crate::commissioning::types::VelocitySweepEvent::Recovery(report) => (
                        EVENT_COMMISSION_RUNG_ORIGIN_RECOVERY,
                        report.evidence_sequence,
                    ),
                    crate::commissioning::types::VelocitySweepEvent::DirectionalRegion(report) => (
                        EVENT_COMMISSION_VELOCITY_SWEEP_DERIVED,
                        report.evidence_sequence,
                    ),
                    crate::commissioning::types::VelocitySweepEvent::JointRegion(report) => (
                        EVENT_COMMISSION_VELOCITY_SWEEP_DERIVED,
                        report.evidence_sequence,
                    ),
                    crate::commissioning::types::VelocitySweepEvent::Handoff(report) => (
                        EVENT_COMMISSION_VELOCITY_SWEEP_DERIVED,
                        report.evidence_sequence,
                    ),
                };
                backend.record_commission_trace_event(CommissionTraceEvent {
                    level: TraceEventLevel::Info,
                    event_id,
                    engine_kind: EngineKind::OuterTune,
                    payload: [i32::from(sequence), 0],
                    payload_count: 1,
                });
                if matches!(
                    event,
                    crate::commissioning::types::VelocitySweepEvent::Observation(report)
                        if report.structured_boundary.is_some()
                ) {
                    backend.record_commission_trace_event(CommissionTraceEvent {
                        level: TraceEventLevel::Info,
                        event_id: EVENT_COMMISSION_VELOCITY_SWEEP_DERIVED,
                        engine_kind: EngineKind::OuterTune,
                        payload: [i32::from(sequence), 0],
                        payload_count: 1,
                    });
                }
            }
            StepResult::VelocitySweepTerminal(terminal) => {
                #[cfg(feature = "trace-velocity-sweep")]
                let _ = terminal;
                #[cfg(not(feature = "trace-velocity-sweep"))]
                self.emit_velocity_sweep_terminal_lifecycle(backend, *terminal);
            }
            StepResult::VelocityIntegralEvent(event) => {
                #[cfg(all(feature = "trace", feature = "trace-velocity-sweep"))]
                backend.record_velocity_sweep_trace_record(
                    crate::commissioning::outer::velocity::integral_reporting::encode_velocity_integral_event(
                        *event,
                    ),
                );
                let (event_id, sequence) = match event {
                    crate::commissioning::types::VelocityIntegralEvent::Planned(report) => (
                        EVENT_COMMISSION_VELOCITY_INTEGRAL_PLAN,
                        report.evidence_sequence,
                    ),
                    crate::commissioning::types::VelocityIntegralEvent::Observation(report) => (
                        EVENT_COMMISSION_VELOCITY_INTEGRAL_OBSERVATION,
                        report.evidence_sequence,
                    ),
                    crate::commissioning::types::VelocityIntegralEvent::Rung(report) => (
                        EVENT_COMMISSION_VELOCITY_INTEGRAL_RUNG,
                        report.evidence_sequence,
                    ),
                    #[cfg(feature = "trace-velocity-sweep")]
                    crate::commissioning::types::VelocityIntegralEvent::Recovery(report) => (
                        EVENT_COMMISSION_RUNG_ORIGIN_RECOVERY,
                        report.evidence_sequence,
                    ),
                };
                backend.record_commission_trace_event(CommissionTraceEvent {
                    level: TraceEventLevel::Info,
                    event_id,
                    engine_kind: EngineKind::OuterTune,
                    payload: [i32::from(sequence), 0],
                    payload_count: 1,
                });
            }
            StepResult::VelocityIntegralTerminal(_) => {}
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
            if let EngineMode::Inner { phase, .. } = &self.engine_mode {
                inner::electrical_identification::push_active_electrical_diagnostics(
                    phase,
                    &mut self.diagnostics,
                );
            }
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

    fn deadline_capture_active(&self) -> bool {
        matches!(
            &self.engine_mode,
            EngineMode::Inner {
                phase: InnerPhase::ElectricalId {
                    state: inner::ElectricalIdState::InductanceSample { .. }
                        | inner::ElectricalIdState::DelaySample { .. },
                    ..
                },
                ..
            }
        )
    }

    /// Drive one deadline capture without unwinding between sample slots.
    pub(crate) async fn step_scheduled<B: CommissioningBackend>(
        &mut self,
        backend: &mut B,
    ) -> StepResult {
        loop {
            let capture_result = match &mut self.engine_mode {
                EngineMode::Inner { phase, .. } => {
                    inner::electrical_identification::drive_deadline_capture(
                        phase,
                        &mut self.buf,
                        &mut self.diagnostics,
                        backend,
                    )
                    .await
                }
                EngineMode::Outer { .. } => None,
            };
            if let Some(result) = capture_result {
                self.emit_trace_after_step(backend, &result);
                return result;
            }

            let result = self.step(backend).await;
            let StepResult::At(_) = result else {
                return result;
            };
            if !self.deadline_capture_active() {
                return result;
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::commission_dispatch::test_support::{SpiOp, current_limit};
    use crate::commissioning::types::EngineKind;

    #[test]
    fn size_regression_gates() {
        use core::mem::{size_of, size_of_val};

        let mut backend = crate::commission_dispatch::test_support::MockBackend::new();

        let mut engine =
            CommissioningEngine::new_commission(0, crate::autotune::CommissionProfile::Balanced);
        let engine_step = size_of_val(&engine.step(&mut backend));

        let mut engine =
            CommissioningEngine::new_commission(0, crate::autotune::CommissionProfile::Balanced);
        let step_inner = match &mut engine.engine_mode {
            EngineMode::Inner {
                phase,
                profile,
                sequence,
                adc_cal,
                calibrate_entry_gains,
            } => size_of_val(&inner::step_inner(
                phase,
                &mut engine.buf,
                *profile,
                *sequence,
                adc_cal,
                *calibrate_entry_gains,
                &mut engine.selftest_results,
                &mut engine.diagnostics,
                &mut backend,
            )),
            EngineMode::Outer { .. } => unreachable!(),
        };

        let mut phase = inner::InnerPhase::ElectricalId {
            state: inner::ElectricalIdState::ResistanceStart,
            data: inner::ElectricalIdData::new(1024),
        };
        let mut buf = crate::autotune::TransientBuffer::new();
        let mut selftest_results = crate::selftest::SelftestResults::default();
        let mut diagnostics = crate::selftest::CommissionDiagnostics::default();
        let electrical_id = size_of_val(
            &inner::electrical_identification::step_electrical_identification(
                &mut phase,
                &mut buf,
                crate::autotune::InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            ),
        );

        let mut runtime = crate::tmc_control::TmcControlRuntime::new_for_channel(0);
        let scheduled_step = size_of_val(&runtime.step_commissioning_scheduled(&mut backend));

        // Outer safety retains independent local and asymmetric absolute travel
        // bounds in the largest engine variant.
        #[cfg(not(feature = "trace-velocity-sweep"))]
        assert_eq!(size_of::<CommissioningEngine>(), 3_840);
        // The feature-on immutable plan and engine-owned cooperative analysis
        // state make VelocitySweep the largest outer payload. Resumable
        // partial-block analysis reaches, but does not raise, the audited F407
        // ceiling.
        #[cfg(feature = "trace-velocity-sweep")]
        {
            assert_eq!(size_of::<TraceRunState>(), 12);
            assert_eq!(size_of::<TraceSummarySeqs>(), 16);
            assert_eq!(
                size_of::<crate::commissioning::outer::VelocityIntegralData>(),
                2_480
            );
            assert!(
                size_of::<
                    crate::commissioning::outer::velocity::sweep_plan::VelocitySweepExecutionPlan,
                >() == 32
            );
            assert_eq!(
                size_of::<
                    crate::commissioning::outer::velocity::operating_region::OperatingRegionRuntime,
                >(),
                352
            );
            assert_eq!(
                size_of::<crate::commissioning::outer::VelocitySweepData>(),
                2_864
            );
            assert_eq!(size_of::<CommissioningEngine>(), 4_512);
        }
        assert!(size_of::<CommissioningEngine>() <= 4_528);
        assert!(size_of::<CommissioningEngine>() <= 6_144);
        assert!(size_of::<capture_timing::CaptureTiming>() <= 64);
        assert!(size_of::<capture_timing::CaptureTimingEvidence>() <= 32);
        assert_eq!(size_of::<[u16; crate::autotune::TRANSIENT_LEN]>(), 512);
        assert_eq!(size_of::<crate::autotune::DelayTimestampOffsets>(), 514);
        // Production-semantic verified retries add loop state to the mock
        // backend future used by this host-side size gate.
        // Resistance current evidence adds 16 bytes to the terminal result
        // carried by the engine-step future.
        assert_eq!(engine_step, 4_320);
        assert!(engine_step <= 4_320);
        assert_eq!(step_inner, 4_272);
        assert!(step_inner <= 4_272);
        // Run-owned current evidence adds 32 bytes to electrical ID state.
        assert_eq!(electrical_id, 640);
        assert!(electrical_id <= 640);
        assert!(electrical_id <= 960);
        // The capture-owned deadline loop adds 48 bytes to the scheduled
        // future while removing per-sample reconstruction of that future.
        // Production-semantic mock retries add another 32 bytes in tests.
        assert_eq!(scheduled_step, 4_400);
        // The hot path keeps nonterminal classification outside the terminal
        // async dispatch future.
        assert!(scheduled_step <= 4_400);
    }

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
            inner_warning_flags: 0,
            requested_velocity_mrev_s: 6000,
            machine_velocity_ceiling_mrev_s: 10_000,
            requested_velocity_source:
                crate::commissioning::outer::velocity::sweep_plan::RequestedVelocitySource::Defaulted,
            max_stroke_travel_mrev: 1000,
            settle_travel_reserve_mrev: 100,
            negative_position_headroom_mrev: 2000,
            positive_position_headroom_mrev: 2000,
            max_duration_ms: 3000,
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
            r_count_milli: 1706,
            l_count_micro: 1245,
            flux_p: 100,
            flux_i: 10,
            torque_p: 200,
            torque_i: 20,
            candidate_gains_source: crate::autotune::CURRENT_GAINS_SOURCE_MEASURED,
            candidate_axis_split_source: crate::autotune::AXIS_SPLIT_SOURCE_NONE,
            candidate_gains_tier: crate::autotune::CURRENT_GAINS_TIER_MEASURED_SYMMETRIC,
            candidate_measured_axis_split_permille: 1000,
            candidate_applied_axis_split_permille: 1000,
            candidate_axis_split_clamped: 0,
            candidate_flux_p: 100,
            candidate_flux_i: 10,
            candidate_torque_p: 200,
            candidate_torque_i: 20,
            candidate_attempt: 0,
            lambda_us: 100,
            theta_e_us: 50,
            ringing_count: 0,
            bandwidth_hz: 1000,
            tau_e_us: 730,
            inductance: crate::commissioning::types::InductanceEvidence::EMPTY,
            inner_warning_flags: 0,
            timing: [crate::commissioning::types::CommissionTimingSummary::NOT_RUN; 3],
            timing_summary: 0,
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
            resistance_peak_abs_current_count: 1_440,
            resistance_max_abs_steady_mean_current_count: 1_280,
            resistance_current_ceiling_count: 1_600,
            resistance_power_stage_tripped: false,
            encoder_alignment_count: 0,
            encoder_alignment_electrical_residual_counts: 0,
            encoder_alignment_stability_counts: 0,
            encoder_alignment_movement_counts: 0,
            encoder_alignment_min_movement_counts: 0,
            encoder_alignment_counts_per_electrical_rev: 0,
            adc_residual_chop0: crate::commissioning::types::AdcResidualEvidence::EMPTY,
            adc_residual_chop7: crate::commissioning::types::AdcResidualEvidence::EMPTY,
            current_gains_source: crate::autotune::CURRENT_GAINS_SOURCE_MEASURED,
            axis_split_source: crate::autotune::AXIS_SPLIT_SOURCE_NONE,
            gains_tier: crate::autotune::CURRENT_GAINS_TIER_MEASURED_SYMMETRIC,
            measured_axis_split_permille: 1000,
            applied_axis_split_permille: 1000,
            axis_split_clamped: 0,
            current_validation_axes: crate::autotune::CURRENT_VALIDATION_AXIS_TORQUE,
            retry_budget_exhausted: 0,
            current_loop_failure_reason: crate::autotune::CURRENT_LOOP_FAILURE_NONE,
            closed_loop_entry_status: crate::autotune::CLOSED_LOOP_ENTRY_STATUS_NOT_RUN,
            closed_loop_entry_position_1: 0,
            closed_loop_entry_position_2: 0,
            closed_loop_entry_drift_count: 0,
            closed_loop_entry_threshold_count: 0,
            closed_loop_entry_runaway: 0,
            current_hold_status: crate::autotune::HOLD_GATE_STATUS_NOT_RUN,
            current_hold_warning_flags: 0,
            current_hold_elapsed_us: 0,
            current_hold_requested_sample_period_us: crate::autotune::HOLD_GATE_SAMPLE_PERIOD_US
                as u16,
            current_hold_sample_count: 0,
            current_hold_position_span_count: 0,
            current_hold_position_drift_count: 0,
            current_hold_torque_mean_count: 0,
            current_hold_torque_rms_count: 0,
            current_hold_torque_peak_to_peak_count: 0,
            current_hold_torque_crossing_count: 0,
            current_hold_flux_mean_count: 0,
            current_hold_flux_rms_count: 0,
            current_hold_flux_peak_to_peak_count: 0,
            current_hold_flux_crossing_count: 0,
            current_hold_status_flags_or: 0,
            current_hold_actionable_status_count: 0,
            current_validation_step_amplitude: 128,
            current_validation_flux_step_amplitude: 128,
            current_validation_torque_step_amplitude: 128,
            current_validation_pidout_limit: 2400,
            current_validation_current_limited: 422,
            current_validation_voltage_limited: 128,
            current_validation_max_p: 3474,
            current_settled_validation_count: 0,
            current_settled_validation:
                [crate::commissioning::types::CurrentValidationSettledSampleEvidence::EMPTY;
                    crate::autotune::CURRENT_VALIDATION_SETTLED_MAX_SAMPLES],
            current_flux_validation_count: 0,
            current_flux_validation:
                [crate::commissioning::types::CurrentValidationAxisEvidence::EMPTY;
                    crate::autotune::CURRENT_VALIDATION_MAX_SAMPLES_PER_AXIS],
            current_torque_validation_count: 0,
            current_torque_validation:
                [crate::commissioning::types::CurrentValidationAxisEvidence::EMPTY;
                    crate::autotune::CURRENT_VALIDATION_MAX_SAMPLES_PER_AXIS],
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
            velocity_filter_hz: 0,
            torque_filter_hz: 0,
            position_filter_hz: 0,
            flux_filter_hz: 0,
        }
    }

    #[test]
    fn stage1_terminal_path_uses_only_explicit_scheduling() {
        pollster::block_on(async {
            use foci_tmc4671::registers::{
                ABN_DECODER_PPR, ADC_I_SELECT, MOTOR_TYPE_N_POLE_PAIRS, PID_TORQUE_FLUX_LIMITS,
                PWM_SV_CHOP,
            };

            let mut engine = CommissioningEngine::new_commission(
                0,
                crate::autotune::CommissionProfile::Balanced,
            );
            // Focused phase tests cover the earlier Stage 1 schedules. Seed the
            // final phase here to exercise engine dispatch through a successful
            // terminal result without reproducing analog estimator fixtures.
            let EngineMode::Inner { phase, .. } = &mut engine.engine_mode else {
                unreachable!();
            };
            *phase = InnerPhase::ClosedLoopEntry {
                state: crate::autotune::ClosedLoopEntryState::WriteGains,
                flux_p: 100,
                flux_i: 10,
                torque_p: 200,
                torque_i: 20,
                velocity_p: 1_152,
                velocity_i: 0,
                position_p: 640,
                position_i: 0,
                velocity_limit: 500_000,
                velocity_filter_hz: 0,
                torque_filter_hz: 0,
                position_filter_hz: 0,
                flux_filter_hz: 0,
                inner_result: Some(sample_inner_result(
                    crate::autotune::CommissionStatus::Accepted,
                )),
            };

            let mut backend = crate::commission_dispatch::test_support::MockBackend::new();
            backend.current_limit_value = current_limit(2_816);
            backend.configured_physical_width = Some(16);
            let required_registers = [
                (PWM_SV_CHOP.addr(), 7),
                (MOTOR_TYPE_N_POLE_PAIRS.addr(), (2 << 16) | 50),
                (
                    ABN_DECODER_PPR.addr(),
                    backend.mechanical_encoder_quadrature_counts_per_rev_value,
                ),
                (ADC_I_SELECT.addr(), backend.tmc_config.adc_i_select),
                (
                    PID_TORQUE_FLUX_LIMITS.addr(),
                    backend.current_limit_value.register_value(),
                ),
            ];
            for (address, value) in required_registers {
                backend.registers.insert(address, value).ok();
            }

            for _ in 0..512 {
                match engine.step_scheduled(&mut backend).await {
                    StepResult::ReadyNow => {}
                    StepResult::At(deadline_us) => backend.elapsed_us_value = deadline_us,
                    StepResult::CommissionDone(result) => {
                        assert_eq!(
                            result.inner.status,
                            crate::autotune::CommissionStatus::Accepted
                        );
                        return;
                    }
                    other => panic!("unexpected Stage 1 terminal result: {other:?}"),
                }
            }
            panic!("Stage 1 terminal path exceeded its explicit schedule bound");
        });
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
            outer_evidence_flags: 0,
            stiffness_timebase_ms: 0,
            velocity_search_stop_reason: 0,
            motion_budget_mrev: 0,
        }
    }

    fn sample_adc_cal_result() -> AdcCalResult {
        AdcCalResult {
            adc_i0: 11,
            adc_i1: 13,
            encoder_count: 17,
            encoder_alignment_electrical_residual_counts: 0,
            encoder_alignment_stability_counts: 0,
            encoder_alignment_movement_counts: 0,
            encoder_alignment_min_movement_counts: 0,
            encoder_alignment_counts_per_electrical_rev: 0,
            adc_residual_chop0: crate::commissioning::types::AdcResidualEvidence::EMPTY,
            adc_residual_chop7: crate::commissioning::types::AdcResidualEvidence::EMPTY,
        }
    }

    fn sample_inconclusive_sweep_terminal() -> crate::commissioning::types::VelocitySweepTerminal {
        use crate::commissioning::types::{VelocitySweepOutcome, VelocitySweepTerminal};
        VelocitySweepTerminal {
            run_sequence: 9,
            evidence_sequence: 0,
            plan_digest: 0x1234,
            outcome: VelocitySweepOutcome::Inconclusive,
            cause: 4,
            recovery_unavailable: false,
            model_direction_mask: 0,
            selected_memberships: [0; 3],
            nominated_p: 0,
            selected_intervals: [[0; 2]; 2],
            selected_common_intervals: [[0; 2]; 2],
            p_start: 0,
            p_top: 0,
            rung_count: 0,
            coverage_mask: 0,
            directional_region_counts: [0; 2],
            directional_fragment_counts: [0; 2],
            expected_observations: 20,
            emitted_observations: 20,
            expected_rungs: 5,
            emitted_rungs: 5,
            digest: 0x0123_4567_89ab_cdef,
            run_started_us: 1_000,
            run_completed_us: 2_000,
            safety_fault: None,
        }
    }

    fn initial_outer_phase_id() -> PhaseId {
        #[cfg(not(feature = "trace-velocity-sweep"))]
        {
            PhaseId::MechanicalId
        }
        #[cfg(feature = "trace-velocity-sweep")]
        {
            PhaseId::VelocityTune
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
            let outer = crate::pid::test_outer_gains(627, 1, 1434, 2);
            tmc_state.store_outer_gains(outer, Relaxed);
            tmc_state.velocity_limit_cfg.store(500_000, Relaxed);
            tmc_state.velocity_filter_hz.store(Some(11), Relaxed);
            tmc_state.torque_filter_hz.store(Some(13), Relaxed);
            tmc_state.position_filter_hz.store(Some(17), Relaxed);
            tmc_state.flux_filter_hz.store(Some(19), Relaxed);

            let inner = crate::pid::InnerLoopGains {
                flux_p: crate::pid::test_gain(711),
                flux_i: crate::pid::test_gain(159),
                torque_p: crate::pid::test_gain(711),
                torque_i: crate::pid::test_gain(159),
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
            assert_eq!(gains.outer, Some(outer));
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
    fn new_outer_seeds_host_motion_budget() {
        let params = OuterTuneParams {
            requested_velocity_mrev_s: 4_500,
            max_stroke_travel_mrev: 750,
            settle_travel_reserve_mrev: 75,
            negative_position_headroom_mrev: 900,
            positive_position_headroom_mrev: 1_100,
            max_duration_ms: 2_500,
            ..sample_outer_params()
        };
        let engine = CommissioningEngine::new_outer(0, params, sample_entry_snapshot());

        let EngineMode::Outer { phase, .. } = engine.engine_mode else {
            panic!("new_outer should create an outer engine");
        };
        let safety = match phase {
            OuterPhase::MechanicalId { safety, .. } => safety,
            #[cfg(feature = "trace-velocity-sweep")]
            OuterPhase::VelocitySweep { safety, .. } => safety,
            _ => panic!("new_outer should enter its first outer phase"),
        };

        assert_eq!(safety.budget.max_stroke_travel_mrev, 750);
        assert_eq!(safety.budget.requested_velocity_mrev_s, 4_500);
        assert_eq!(safety.budget.settle_travel_reserve_mrev, 75);
        assert_eq!(safety.budget.negative_position_headroom_mrev, 900);
        assert_eq!(safety.budget.positive_position_headroom_mrev, 1_100);
        assert_eq!(safety.budget.max_duration_ms, 2_500);
    }

    #[cfg(not(feature = "trace-velocity-sweep"))]
    #[test]
    fn new_outer_enters_mechanical_id_without_trace_sweep() {
        let engine =
            CommissioningEngine::new_outer(0, sample_outer_params(), sample_entry_snapshot());

        assert!(matches!(
            engine.engine_mode,
            EngineMode::Outer {
                phase: OuterPhase::MechanicalId { .. },
                ..
            }
        ));
    }

    #[cfg(feature = "trace-velocity-sweep")]
    #[test]
    fn new_outer_enters_velocity_sweep_with_trace_feature() {
        let engine =
            CommissioningEngine::new_outer(0, sample_outer_params(), sample_entry_snapshot());

        assert!(matches!(
            engine.engine_mode,
            EngineMode::Outer {
                phase: OuterPhase::VelocitySweep { .. },
                ..
            }
        ));
    }

    #[cfg(feature = "trace-velocity-sweep")]
    #[test]
    fn stage_b_constructor_seeds_the_admitted_plan_once() {
        use crate::commission_dispatch::stage_b::preflight_stage_b_request;
        use crate::commissioning::outer::velocity::reproduction::StageBCandidateSlot;
        use crate::commissioning::outer::velocity::sweep_plan::VelocitySweepRequest;

        let params = sample_outer_params();
        let request = VelocitySweepRequest {
            requested_velocity_mrev_s: params.requested_velocity_mrev_s,
            machine_velocity_ceiling_mrev_s: params.machine_velocity_ceiling_mrev_s,
            requested_velocity_source: params.requested_velocity_source,
            max_stroke_travel_mrev: params.max_stroke_travel_mrev,
            settle_travel_reserve_mrev: params.settle_travel_reserve_mrev,
            negative_position_headroom_mrev: params.negative_position_headroom_mrev,
            positive_position_headroom_mrev: params.positive_position_headroom_mrev,
            max_duration_ms: params.max_duration_ms,
        };
        let mut slot = StageBCandidateSlot::new();
        let preflight = preflight_stage_b_request(request, 4_000, 2_816, 0, 42, &mut slot)
            .expect("valid Stage B preflight");
        let expected_digest = preflight.identity.plan_digest;
        let engine = CommissioningEngine::new_outer_for_tmc_with_stage_b_preflight(
            0,
            tmc_state(),
            params,
            sample_entry_snapshot(),
            preflight,
        );

        let EngineMode::Outer {
            phase: OuterPhase::VelocitySweep { data, .. },
            ..
        } = engine.engine_mode
        else {
            panic!("Stage B preflight must construct the velocity sweep")
        };
        assert_eq!(data.plan.plan_digest, expected_digest);
        assert_eq!(data.integrity.next_evidence_sequence(), 1);
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
            initial_outer_phase_id(),
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
    fn inconclusive_sweep_does_not_reuse_cause_as_generic_status() {
        let mut engine = CommissioningEngine::new_outer_with_trace_seq(
            0,
            sample_outer_params(),
            sample_entry_snapshot(),
            9,
        );
        let mut backend = crate::commission_dispatch::test_support::MockBackend::new();

        engine.emit_trace_start(&mut backend);
        let terminal = sample_inconclusive_sweep_terminal();
        engine.emit_trace_after_step(&mut backend, &StepResult::VelocitySweepTerminal(terminal));
        #[cfg(feature = "trace-velocity-sweep")]
        assert!(backend.velocity_sweep_trace_records.is_empty());
        #[cfg(feature = "trace-velocity-sweep")]
        engine.emit_stage_b_terminal_trace(&mut backend, terminal, None);

        let events = &backend.trace_events;
        #[cfg(not(feature = "trace-velocity-sweep"))]
        assert_eq!(events.len(), 4);
        #[cfg(feature = "trace-velocity-sweep")]
        assert_eq!(events.len(), 5);
        let phase_end_index = usize::from(cfg!(feature = "trace-velocity-sweep")) + 2;
        assert_trace_event(
            &events[phase_end_index],
            crate::trace::events::EVENT_COMMISSION_PHASE_END,
            EngineKind::OuterTune,
            TraceEventLevel::Info,
            [initial_outer_phase_id().to_u8() as i32, PHASE_END_OK.into()],
        );
        assert_trace_event(
            &events[phase_end_index + 1],
            crate::trace::events::EVENT_COMMISSION_VELOCITY_SWEEP_TERMINAL,
            EngineKind::OuterTune,
            TraceEventLevel::Warn,
            [4, 0x89ab_cdef_u32 as i32],
        );
        assert!(engine.trace_run.terminal);
    }

    #[test]
    fn sweep_integrity_fault_does_not_reuse_cause_as_generic_status() {
        let mut engine = CommissioningEngine::new_outer_with_trace_seq(
            0,
            sample_outer_params(),
            sample_entry_snapshot(),
            9,
        );
        let mut backend = crate::commission_dispatch::test_support::MockBackend::new();
        let mut terminal = sample_inconclusive_sweep_terminal();
        terminal.outcome = crate::commissioning::types::VelocitySweepOutcome::Fault;
        terminal.cause = 3;

        engine.emit_trace_start(&mut backend);
        engine.emit_trace_after_step(&mut backend, &StepResult::VelocitySweepTerminal(terminal));
        #[cfg(feature = "trace-velocity-sweep")]
        engine.emit_stage_b_terminal_trace(&mut backend, terminal, None);

        let events = &backend.trace_events;
        #[cfg(not(feature = "trace-velocity-sweep"))]
        assert_eq!(events.len(), 4);
        #[cfg(feature = "trace-velocity-sweep")]
        assert_eq!(events.len(), 5);
        let phase_end_index = usize::from(cfg!(feature = "trace-velocity-sweep")) + 2;
        assert_trace_event(
            &events[phase_end_index],
            crate::trace::events::EVENT_COMMISSION_PHASE_END,
            EngineKind::OuterTune,
            TraceEventLevel::Info,
            [initial_outer_phase_id().to_u8() as i32, PHASE_END_OK.into()],
        );
        assert_trace_event(
            &events[phase_end_index + 1],
            crate::trace::events::EVENT_COMMISSION_VELOCITY_SWEEP_TERMINAL,
            EngineKind::OuterTune,
            TraceEventLevel::Error,
            [3, 0x89ab_cdef_u32 as i32],
        );
        assert!(engine.trace_run.terminal);
    }

    #[cfg(feature = "trace-velocity-sweep")]
    #[test]
    fn complete_stage_c_uses_dedicated_terminal_and_complete_events() {
        use crate::commissioning::outer::velocity::integral_evidence::{
            ExactRateInterval, IntegralRunEvidence,
        };
        use crate::commissioning::types::{VelocityIntegralOutcome, VelocityIntegralTerminal};

        let mut engine = CommissioningEngine::new_outer_with_trace_seq(
            0,
            sample_outer_params(),
            sample_entry_snapshot(),
            9,
        );
        let mut backend = crate::commission_dispatch::test_support::MockBackend::new();
        let terminal = VelocityIntegralTerminal {
            run_sequence: 9,
            evidence_sequence: 57,
            plan_digest: 0x1111_2222_3333_4444,
            digest: 0x0123_4567_89ab_cdef,
            outcome: VelocityIntegralOutcome::Complete,
            cause: 0,
            recovery_unavailable: false,
            evidence: IntegralRunEvidence::new([
                ExactRateInterval::new(100, 120).unwrap(),
                ExactRateInterval::new(-120, -100).unwrap(),
            ]),
            expected_observations: 56,
            emitted_observations: 56,
            expected_rungs: 7,
            emitted_rungs: 7,
            run_started_us: 1_000,
            run_completed_us: 2_000,
            safety_fault: None,
        };

        engine.emit_trace_start(&mut backend);
        engine.emit_stage_c_terminal_trace(&mut backend, terminal, None);

        assert_eq!(backend.velocity_sweep_trace_records.len(), 2);
        let ids: std::vec::Vec<_> = backend
            .trace_events
            .iter()
            .map(|event| event.event_id)
            .collect();
        assert_eq!(
            &ids[2..],
            &[
                crate::trace::events::EVENT_COMMISSION_OUTER_SAFETY_TIMING,
                crate::trace::events::EVENT_COMMISSION_PHASE_END,
                crate::trace::events::EVENT_COMMISSION_VELOCITY_INTEGRAL_TERMINAL,
                crate::trace::events::EVENT_COMMISSION_VELOCITY_INTEGRAL_COMPLETE,
            ]
        );
        assert!(engine.trace_run.terminal);
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
            [initial_outer_phase_id().to_u8() as i32, i32::from(status)],
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
            initial_outer_phase_id(),
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
            if let EngineMode::Inner { phase, .. } = &mut engine.engine_mode {
                let mut data = inner::ElectricalIdData::new(512);
                data.timing_states = crate::commissioning::types::COMMISSION_TIMING_REJECTED << 4;
                *phase = InnerPhase::ElectricalId {
                    state: inner::ElectricalIdState::DelaySample {
                        capture: inner::DelayCaptureState {
                            baseline: 0,
                            origin_us: 1_000,
                            next_sample_us: 1_160,
                            offsets: crate::autotune::DelayTimestampOffsets::new(),
                            timing: capture_timing::CaptureTiming::new(
                                160,
                                capture_timing::TimingEnvelope::DELAY,
                            ),
                        },
                    },
                    data,
                };
            }
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
            let diagnostics = engine.take_diagnostics();
            let delay = diagnostics
                .iter()
                .find(|event| event.code == 38)
                .expect("shutdown timing diagnostic");
            assert_eq!((delay.value1 >> 24) & 0x03, 2);
        });
    }

    #[test]
    fn shutdown_during_resistance_emits_current_evidence() {
        pollster::block_on(async {
            let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
                .lock()
                .await;
            let mut engine = CommissioningEngine::new_commission(
                0,
                crate::autotune::CommissionProfile::Balanced,
            );
            let mut backend = crate::commission_dispatch::test_support::MockBackend::new();
            backend.tmc_config.resistance_max_expected_current_count = 321;
            let profile = backend
                .tmc_config
                .resistance_profile(backend.tmc_config.resistance_scale_metadata_validated);
            if let EngineMode::Inner { phase, .. } = &mut engine.engine_mode {
                *phase = InnerPhase::ElectricalId {
                    state: inner::ElectricalIdState::ResistanceIdentify {
                        runner: crate::resistance_identification::ResistanceRunner::new(
                            crate::resistance_identification::ResistanceRunMode::CommissioningManagedEnable,
                            profile,
                            0,
                        )
                        .unwrap(),
                    },
                    data: inner::ElectricalIdData::new(512),
                };
            }
            backend.shutdown_requested = true;
            backend.enable_pin = true;

            let result = engine.step(&mut backend).await;

            assert!(matches!(
                result,
                crate::commissioning::types::StepResult::InnerFailed(
                    crate::autotune::CommissionError::Shutdown,
                )
            ));
            let diagnostics = engine.take_diagnostics();
            let current = diagnostics
                .iter()
                .find(|event| event.code == 40)
                .expect("shutdown resistance current diagnostic");
            assert_eq!(current.value0, 0);
            assert_eq!(current.value1, 0);
            assert_eq!(current.value2, 321);
            assert_eq!(current.status, 0);
        });
    }

    #[test]
    fn shutdown_during_inductance_restore_retains_completed_timing() {
        pollster::block_on(async {
            let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
                .lock()
                .await;
            let mut engine = CommissioningEngine::new_commission(
                0,
                crate::autotune::CommissionProfile::Balanced,
            );
            let timing = capture_timing::CaptureTimingEvidence {
                requested_period_us: 100,
                valid_samples: 7,
                missed_samples: 0,
                max_consecutive_misses: 0,
                max_lateness_us: 10,
                max_interval_us: 110,
                max_poll_wall_us: 15,
                max_spi_wall_us: 8,
            };
            if let EngineMode::Inner { phase, .. } = &mut engine.engine_mode {
                let mut data = inner::ElectricalIdData::new(512);
                data.timing_states = crate::commissioning::types::COMMISSION_TIMING_REJECTED << 2;
                *phase = InnerPhase::ElectricalId {
                    state: inner::ElectricalIdState::InductanceRestore {
                        operation: inner::InductanceRestoreOperation::RetainedUqUdExt,
                        primary_error: None,
                        evidence: Some(crate::commissioning::types::InductanceEvidence::EMPTY),
                        timing: Some(timing),
                        timing_accepted: true,
                    },
                    data,
                };
            }
            let mut backend = crate::commission_dispatch::test_support::MockBackend::new();
            backend.shutdown_requested = true;
            backend.enable_pin = true;

            let result = engine.step(&mut backend).await;

            assert!(matches!(
                result,
                crate::commissioning::types::StepResult::InnerFailed(
                    crate::autotune::CommissionError::Shutdown,
                )
            ));
            assert!(!backend.enable_pin);
            assert!(matches!(
                backend.spi_log.first(),
                Some(SpiOp::EnablePin(false))
            ));
            assert!(
                backend.delay_us_log.is_empty(),
                "shutdown snapshot must not delay before GPIO-first restore"
            );
            let diagnostics = engine.take_diagnostics();
            let header = diagnostics
                .iter()
                .find(|event| event.code == 36)
                .expect("inductance restore timing diagnostic");
            assert_eq!(header.value0 >> 16, 7);
            assert_eq!((header.value1 >> 24) & 0x03, 2);
        });
    }
}
