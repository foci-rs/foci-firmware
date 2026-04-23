//! Shared protocol-level state primitives.
//!
//! Each submodule holds the `pub static` atomics (and one embassy-sync
//! `Signal`) for one subsystem, exactly mirroring the pre-lift layout
//! in `openffboard-fw/src/state/*`. Populated in Stage 2 (Task 5).
//!
//! Distinct from `crate::tmc` (the TMC4671 driver layer) and
//! `crate::trace` (the trace ring-buffer implementation) — this module
//! holds only concurrency primitives read/written by ankyra handlers
//! and RTIC tasks.

pub mod lifecycle;
pub mod stepper;
pub mod tmc;
pub mod trsync;
