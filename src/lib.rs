//! Board-agnostic FOCI firmware library.
//!
//! Provides the [`tmc::TmcRequest`] communication type, register [`dump`]
//! protocol, autotune / commissioning logic, and the [`platform`] trait
//! surface that board binaries implement to plug in their hardware.

#![no_std]

// Host-side `cargo test` always links std; make it available in test modules
// even though the production crate is no_std.
#[cfg(test)]
extern crate std;

pub mod ankyra;
pub mod autotune;
pub mod biquad;
pub mod calibration;
pub mod commission_dispatch;
pub mod commissioning;
pub mod config;
pub mod context;
pub mod diagnostic;
pub mod dump;
pub mod helpers;
pub mod pid;
pub mod platform;
pub mod sample_timing;
pub mod selftest;
pub mod shutdown;
pub mod state;
pub mod stepper_runtime;
pub mod tmc;
pub mod tmc_ops;
pub mod tmc_safe_state;
pub mod trace;
pub mod trace_events;
#[cfg(feature = "trace")]
pub mod trace_stream;

/// Backwards-compatibility shim. The implementation lives in
/// [`commissioning`]; this module exposes the engine and result types
/// under the legacy `foci_firmware::commission::*` path so existing board
/// binaries continue to compile through the renaming transition.
pub mod commission {
    pub use crate::commissioning::types::{
        AdcCalResult, CommissionResult, CommissioningBackend, InnerResult, OuterResult, StepResult,
    };
    pub use crate::commissioning::{CommissioningEngine, EntrySnapshot, OuterTuneParams};
}
