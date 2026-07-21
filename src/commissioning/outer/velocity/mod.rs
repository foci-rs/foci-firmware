//! Velocity-loop commissioning phases.

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
pub mod operating_region;
#[cfg(test)]
mod operating_region_tests;
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

#[cfg(feature = "trace-velocity-sweep")]
pub(crate) use integral_sweep::{integral_fault_terminal, step_velocity_integral_with_trace};
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
