//! Velocity-loop commissioning phases.

pub mod acceptance_evidence;
#[cfg(test)]
mod acceptance_evidence_tests;
#[cfg(feature = "trace-velocity-sweep")]
pub mod acceptance_plan;
#[cfg(all(test, feature = "trace-velocity-sweep"))]
mod acceptance_plan_tests;
#[cfg(feature = "trace-velocity-sweep")]
pub(crate) mod acceptance_reporting;
pub mod band_confirmation;
#[cfg(feature = "trace-velocity-sweep")]
pub(crate) mod breakaway_campaign;
pub mod breakaway_plan;
pub mod breakaway_search;
pub mod combined_plan;
#[cfg(test)]
mod combined_plan_tests;
pub(crate) mod current_guard;
#[cfg(test)]
mod current_guard_tests;
pub mod integral_evidence;
#[cfg(test)]
mod integral_evidence_tests;
pub mod integral_plan;
#[cfg(test)]
mod integral_plan_tests;
#[cfg(feature = "trace-velocity-sweep")]
pub(crate) mod integral_reporting;
#[cfg(feature = "trace-velocity-sweep")]
pub mod integral_reproduction;
#[cfg(all(test, feature = "trace-velocity-sweep"))]
mod integral_reproduction_tests;
#[cfg(feature = "trace-velocity-sweep")]
mod integral_sweep;
#[cfg(all(test, feature = "trace-velocity-sweep"))]
mod integral_sweep_tests;
#[cfg(test)]
mod integral_test_point_contract_tests;
pub mod operating_region;
#[cfg(test)]
mod operating_region_tests;
pub(crate) mod recovery;
#[cfg(all(test, feature = "trace-velocity-sweep"))]
mod recovery_tests;
pub mod reproduction;
mod response_ramp;
#[cfg(feature = "trace-velocity-sweep")]
mod sweep;
#[cfg(feature = "trace-velocity-sweep")]
mod sweep_analysis;
pub mod sweep_consensus;
#[cfg(test)]
mod sweep_consensus_tests;
pub mod sweep_evidence;
pub mod sweep_plan;
#[cfg(feature = "trace-velocity-sweep")]
pub(crate) mod sweep_reporting;
mod tune;
mod validate;

/// Wire value declaring best-effort polling for velocity-limit evidence.
pub const VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT: u32 =
    current_guard::VelocityLimitEvidenceCapability::BestEffortPolling as u32;

/// Wire value declaring latched velocity-limit status evidence.
pub const VELOCITY_LIMIT_EVIDENCE_CAPABILITY_LATCHED_FLAG: u32 =
    current_guard::VelocityLimitEvidenceCapability::LatchedFlag as u32;

#[cfg(feature = "trace-velocity-sweep")]
pub(crate) use breakaway_campaign::step_breakaway_campaign_with_trace;
#[cfg(feature = "trace-velocity-sweep")]
pub(crate) use integral_sweep::{
    integral_fault_terminal, integral_i_at, step_velocity_integral_with_trace,
};
#[allow(unused_imports)]
pub(crate) use response_ramp::VELOCITY_RESPONSE_MAX_RETRIES;
#[cfg(feature = "trace-velocity-sweep")]
pub(crate) use sweep::{step_velocity_sweep_with_trace, sweep_fault_terminal};
#[allow(unused_imports)]
pub(crate) use tune::step_velocity_tune;
#[allow(unused_imports)]
pub(crate) use validate::{
    VELOCITY_VALIDATE_CHATTER_WINDOW, VELOCITY_VALIDATE_SAMPLE_PERIOD_US,
    VELOCITY_VALIDATE_SETTLE_US, step_velocity_validate, step_velocity_validate_with_trace,
};
