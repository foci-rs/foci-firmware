//! Velocity-loop commissioning phases.

mod response_ramp;
#[cfg(feature = "trace-velocity-sweep")]
mod sweep;
#[cfg(feature = "trace-velocity-sweep")]
mod sweep_analysis;
pub mod sweep_evidence;
pub mod sweep_plan;
#[cfg(feature = "trace-velocity-sweep")]
pub(crate) mod sweep_reporting;
mod tune;
mod validate;

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
