//! Shared TMC control runtime for board firmware.
//!
//! The runtime owns board-agnostic TMC control state. Board crates own RTIC
//! resources, SPI handles, GPIO pins, reply macros, shutdown macros, and task
//! spawning.

pub mod actions;
pub mod config;
pub mod dispatch;
pub mod filters;
pub mod helpers;
pub mod request_writes;
pub mod settle;
pub mod signals;
pub mod state;

#[cfg(test)]
mod tests;

pub use actions::{
    BoardAction, BoardActions, CommissionTickAction, DelegatedDiagnostic, ShutdownReason,
    StepperAction, TmcControlRequestOutcome, TmcControlRequestTail, push_action,
};
pub use config::{TmcControlConfig, TmcControlHomingRefs, TmcControlStateRefs};
pub use signals::TmcControlSignals;
pub use state::{
    HomingMaintenanceResult, TmcControlRuntime, TmcControlTraceState, TmcControlTraceTick,
};

/// Slow-control ticks to wait before clearing settled PID error sums.
pub const HOLD_SETTLE_DELAY_MS: u8 = 100;
