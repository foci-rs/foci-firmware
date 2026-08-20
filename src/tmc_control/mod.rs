//! Shared TMC control runtime for board firmware.
//!
//! The runtime owns board-agnostic TMC control state. Board crates own RTIC
//! resources, SPI handles, GPIO pins, reply macros, shutdown macros, and task
//! spawning.

pub mod actions;
pub mod bootstrap;
pub mod commission_poll;
pub mod config;
pub mod current_limit;
pub mod dispatch;
pub mod filters;
pub mod helpers;
pub mod lifecycle;
pub mod pi_representation;
pub mod request;
pub mod request_writes;
pub mod settle;
pub mod signals;
pub mod state;

#[cfg(test)]
mod pi_representation_tests;
#[cfg(test)]
mod tests;

pub use actions::{
    BoardAction, BoardActions, DelegatedDiagnostic, ScheduledCommissionTickAction, ShutdownReason,
    StepperAction, TmcControlRequestOutcome, TmcControlRequestTail, push_action,
};
pub use bootstrap::{
    TmcSafetyBootstrapFailure, TmcSafetyBootstrapStage, bootstrap_tmc_safety,
    first_tmc_safety_bootstrap_failure,
};
pub use commission_poll::{
    AbsoluteWaitAction, COMMISSION_WAIT_SLICE_US, CommissionPollDriver,
    CommissionWaitRequestDisposition, FORCED_READY_YIELD_US, MAX_CONSECUTIVE_READY_POLLS,
    PollDriverAction, classify_absolute_wait, classify_wait_request, nonterminal_continuation,
};
pub use config::{TmcControlConfig, TmcControlHomingRefs, TmcControlStateRefs};
pub use request::{TmcPreRequestDisposition, TmcRequestBoard, handle_tmc_control_request};
pub use signals::TmcControlSignals;
pub use state::{
    HomingMaintenanceResult, TmcControlRuntime, TmcControlTraceState, TmcControlTraceTick,
};

/// Slow-control ticks to wait before clearing settled PID error sums.
pub const HOLD_SETTLE_DELAY_MS: u8 = 100;
