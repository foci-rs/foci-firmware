//! Shared protocol-level `pub static` atomics, one submodule per subsystem.

pub mod lifecycle;
pub mod report_seq;
pub mod stepper;
pub mod tmc;
pub mod trsync;
