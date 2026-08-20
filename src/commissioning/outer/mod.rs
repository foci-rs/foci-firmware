//! Stage 2 (velocity-integral, breakaway, and robustness-reversal) commissioning.
//!
//! `OuterPhase` is the cooperative state-machine type. `step_outer`
//! dispatches one tick to the per-phase submodule, prefixed by a
//! safety-envelope check that aborts on velocity- or position-window
//! violations. The phase variants and their sub-state types live here
//! so each per-phase submodule can pattern-match and construct them.

mod dispatch;
mod limits;
mod safety;
mod safety_timing;
#[cfg(test)]
mod safety_timing_tests;
mod state;
mod trace;
pub mod velocity;

#[allow(unused_imports)]
pub(crate) use dispatch::step_outer;
pub(crate) use dispatch::{fail_outer, fail_outer_with_safety, step_outer_with_trace};
#[allow(unused_imports)]
pub(crate) use limits::MECHANICAL_ID_VELOCITY_CAP_COUNTS_PER_MS;
pub(crate) use limits::{
    OUTER_MAX_VELOCITY_I, mechanical_id_velocity_cap_counts_per_ms, position_window_counts,
};
pub(crate) use safety::check_outer_safety_with_timing;
pub(crate) use safety::{
    OuterMotionBudget, OuterSafetyLimits, check_outer_safety, safety_error_for_phase,
};
pub(crate) use safety_timing::OuterSafetyTimingAccumulator;
#[cfg(any(test, feature = "trace"))]
pub(crate) use safety_timing::OuterSafetyTimingSummary;
pub(crate) use safety_timing::{
    OuterSafetyTimingContext, OuterSafetyTimingPhase, OuterSafetyTimingStage,
};
pub(crate) use state::OuterPhase;
#[cfg(test)]
pub(crate) use state::{PositiveIRestBoundary, VelocitySettleMetrics};
pub(crate) use state::{VelocityIntegralData, VelocityIntegralState};
pub(crate) use trace::{TraceSummarySeqs, trace_pack_i16_pair, trace_summary_event};
