//! Installed tuning: velocity-integral, breakaway, and robustness-reversal phases.

mod dispatch;
mod limits;
pub(crate) mod position_tune;
mod safety;
mod safety_timing;
#[cfg(test)]
mod safety_timing_tests;
mod state;
mod trace;
pub mod velocity;

pub(crate) use dispatch::{
    fail_outer, fail_outer_position_tune, fail_outer_with_safety, step_outer_with_trace,
};
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
pub(crate) use state::{VelocityIntegralRuntime, VelocityIntegralState};
pub(crate) use trace::{TraceSummarySeqs, trace_pack_i16_pair, trace_summary_event};
