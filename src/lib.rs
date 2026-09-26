//! Board-agnostic FOCI firmware library.
//!
//! Provides the [`tmc::TmcRequest`] communication type, register [`dump`]
//! protocol, and autotune / commissioning logic. Board binaries plug in
//! their hardware directly rather than through a shared trait surface.

#![no_std]

// Host-side `cargo test` always links std; make it available in test modules
// even though the production crate is no_std.
#[cfg(test)]
extern crate std;

// Lets code inside this crate use the crate-absolute path
// `foci_firmware::...` (needed by __commission_reply_manifest!'s entries,
// which must resolve the same way whether expanded here or in a board
// crate that depends on foci-firmware normally) instead of only `crate::...`.
extern crate self as foci_firmware;

pub mod adc_vm;
pub mod analog_in;
pub mod ankyra;
pub mod autotune;
pub mod biquad;
pub mod calibration;
pub mod commission_dispatch;
pub mod commissioning;
pub mod config;
pub mod context;
pub mod cpu_load;
pub mod diagnostic;
pub mod dump;
pub mod electrical_capture;
pub mod helpers;
pub mod mcu_temp;
pub mod motion_scale;
pub mod pid;
pub mod resistance_identification;
pub mod selftest;
pub mod shutdown;
pub mod stack_watermark;
pub mod state;
pub mod step_pulse;
pub mod stepper_runtime;
pub mod tmc;
pub mod tmc_control;
pub mod tmc_diagnostics;
pub mod tmc_safe_state;
pub mod trace;
pub mod transport;
pub mod usb_pump;

/// Backwards-compatibility shim. The implementation lives in
/// [`commissioning`]; this module exposes the engine and result types
/// under the legacy `foci_firmware::commission::*` path so existing board
/// binaries continue to compile through the renaming transition.
pub mod commission {
    pub use crate::commissioning::backend::CommissioningBackend;
    pub use crate::commissioning::types::{
        AdcCalResult, CommissionResult, InnerResult, OuterResult, StepResult,
    };
    pub use crate::commissioning::{
        CommissioningEngine, OuterAutotuneInputs, OuterGainRollbackSnapshot,
    };
}
