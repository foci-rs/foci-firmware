//! Board-agnostic FOCI firmware library.
//!
//! Provides the [`tmc::TmcRequest`] communication type, register [`dump`]
//! protocol, autotune / commissioning logic, and the [`platform`] trait
//! surface that board binaries implement to plug in their hardware.

#![no_std]

pub mod autotune;
pub mod biquad;
pub mod config;
pub mod dump;
pub mod pid;
pub mod platform;
pub mod selftest;
pub mod tmc;
pub mod trace;
