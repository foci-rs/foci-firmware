//! Stage 2 (mechanical / velocity / position / filter) commissioning.
//!
//! `OuterPhase` is the cooperative state-machine type. `step_outer`
//! dispatches one tick to the per-phase submodule, prefixed by a
//! safety-envelope check that aborts on velocity- or position-window
//! violations. The phase variants and their sub-state types live here
//! so each per-phase submodule can pattern-match and construct them.

pub mod commit;
mod dispatch;
pub mod filter;
mod limits;
pub mod mechanical;
#[cfg(test)]
mod policy_tests;
pub mod position;
pub mod response;
mod safety;
#[cfg_attr(not(any(test, feature = "trace-velocity-sweep")), allow(dead_code))]
mod safety_timing;
#[cfg(test)]
mod safety_timing_tests;
pub mod slope_estimator;
mod state;
mod trace;
pub mod velocity;

#[allow(unused_imports)]
pub(crate) use dispatch::step_outer;
pub(crate) use dispatch::{fail_outer, fail_outer_with_safety, step_outer_with_trace};
#[allow(unused_imports)]
pub(crate) use limits::MECHANICAL_ID_VELOCITY_CAP_COUNTS_PER_MS;
pub(crate) use limits::{
    OUTER_FALLBACK_POSITION_P, OUTER_FALLBACK_VELOCITY_I, OUTER_MAX_POSITION_I,
    OUTER_MAX_POSITION_P, OUTER_MAX_VELOCITY_I, OUTER_MAX_VELOCITY_P, OUTER_WARNING_NONE,
    OUTER_WARNING_UNSAFE_SYNTH_GAINS, POS_TUNE_INITIAL_P, mechanical_id_velocity_cap_counts_per_ms,
    position_tune_step_counts, position_window_counts,
    response_fallback_velocity_limit_counts_per_ms,
};
#[cfg(feature = "trace-velocity-sweep")]
pub(crate) use safety::check_outer_safety_with_timing;
pub(crate) use safety::{
    OuterMotionBudget, OuterSafetyLimits, check_outer_safety, safety_error_for_phase,
};
pub(crate) use safety_timing::OuterSafetyTimingAccumulator;
#[cfg(any(test, feature = "trace-velocity-sweep"))]
pub(crate) use safety_timing::{
    OuterSafetyTimingContext, OuterSafetyTimingPhase, OuterSafetyTimingStage,
    OuterSafetyTimingSummary,
};
pub(crate) use state::{
    EffectiveVelocityPolicy, MechanicalIdData, MechanicalIdState,
    OUTER_EVIDENCE_CONSERVATIVE_REQUIRED, OUTER_EVIDENCE_LAMBDA_WIDENED,
    OUTER_EVIDENCE_LOW_CONFIDENCE_MECH, OUTER_EVIDENCE_MOTION_BUDGET_CLAMPED,
    OUTER_EVIDENCE_TAU_DEFAULTED, OUTER_EVIDENCE_UNDER_DRIVEN_PROBE, OUTER_POLICY_WARNING_MASK,
    OuterDiagnostics, OuterPhase, PositionTuneData, PositionTuneState, VELOCITY_SEARCH_STOP_CLEAN,
    VELOCITY_SEARCH_STOP_LOW_CONFIDENCE, VELOCITY_SEARCH_STOP_UNDER_DRIVEN_BUDGET,
    VELOCITY_SEARCH_STOP_UNSAFE_BACKOFF, VelocityTuneStrategy, VelocityValidateData,
    VelocityValidateState,
};
#[cfg(feature = "trace-velocity-sweep")]
pub(crate) use state::{
    VelocityIntegralData, VelocityIntegralState, VelocitySweepData, VelocitySweepState,
};
pub(crate) use trace::{
    TraceSummarySeqs, trace_diagnostic_warning, trace_pack_i16_pair, trace_pack_u16_pair,
    trace_summary_event,
};
