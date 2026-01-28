//! Shared FOCI firmware types and TMC4671 command logic.
//!
//! This crate provides the [`tmc::TmcRequest`] communication type and
//! register [`dump`] protocol used by all FOCI firmware binaries.

#![no_std]

pub mod dump;
pub mod tmc;
