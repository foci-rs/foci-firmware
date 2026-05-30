//! Shared TMC control runtime for board firmware.
//!
//! The runtime owns board-agnostic TMC control state. Board crates own RTIC
//! resources, SPI handles, GPIO pins, reply macros, shutdown macros, and task
//! spawning.

pub mod actions;
pub mod signals;
pub mod state;

#[cfg(test)]
mod tests;

pub use actions::{
    BoardAction, BoardActions, DelegatedDiagnostic, ShutdownReason, StepperAction, push_action,
};
pub use signals::TmcControlSignals;
pub use state::{TmcControlRuntime, TmcControlTraceState};
