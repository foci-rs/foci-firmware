//! Board-agnostic TMC4671 operations extracted from the RTIC task body.
//!
//! Functions here are monomorphized over `B: CommissioningBackend` rather
//! than dyn-dispatched: the hot-path SPI access does not benefit from
//! vtable indirection, and `async fn` in traits is not viable in `no_std`.
//! The RTIC task shell (`#[task]` declaration, shared-resource lock,
//! `spawn()`, EXTI setup) stays board-local; the body calls into this
//! module.
//!
//! Populated progressively by Phase A-7 Tasks 26–27 of the internal-
//! restructure plan (`docs/plans/archive/2026-04-20-phase-a-internal-
//! restructure.md`). See the extraction blueprint in this session's
//! logbook entry for commit-by-commit scope.
