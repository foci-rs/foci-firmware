//! Board-agnostic FOCI firmware library.
//!
//! Provides the [`tmc::TmcRequest`] communication type, register [`dump`]
//! protocol, autotune / commissioning logic, and the [`platform`] trait
//! surface that board binaries implement to plug in their hardware.

#![no_std]

pub mod ankyra;
pub mod autotune;
pub mod biquad;
pub mod calibration;
pub mod commissioning;
pub mod config;
pub mod context;
pub mod dump;
pub mod helpers;
pub mod pid;
pub mod platform;
pub mod selftest;
pub mod shutdown;
pub mod state;
pub mod tmc;
pub mod trace;

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
