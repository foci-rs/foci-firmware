//! Board-agnostic FOCI firmware library.
//!
//! Provides the [`tmc::TmcRequest`] communication type, register [`dump`]
//! protocol, autotune / commissioning logic, and the [`platform`] trait
//! surface that board binaries implement to plug in their hardware.

#![no_std]

pub mod autotune;
pub mod biquad;
// Commission is target-only firmware: its runtime `defmt::info!` calls emit
// Mach-O custom sections that overflow Apple ld's per-binary limit when a
// host test binary also pulls in the defmt derives from the rest of the
// workspace.
#[cfg(target_os = "none")]
pub mod commission;
pub mod config;
pub mod dump;
pub mod pid;
pub mod platform;
pub mod selftest;
pub mod tmc;
pub mod trace;
