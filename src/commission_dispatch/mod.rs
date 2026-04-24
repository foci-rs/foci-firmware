//! Board-agnostic post-processing for `CommissioningEngine::step()` results.
//!
//! Lifted from `openffboard-fw::tmc_control` per the design at
//! `docs/specs/2026-04-23-commission-dispatch-extraction-design.md`.
//!
//! Public surface: a single `drive_commission_result` function plus the
//! types it returns. The board calls this once per engine.step() result
//! and matches `CommissionOutcome` to fire RTIC primitives + emit replies.

// pub(crate) so sibling test modules (tmc_ops, commissioning, …) can
// import MockBackend + helpers via crate::commission_dispatch::test_support::*.
#[cfg(test)]
pub(crate) mod test_support;

// Types and the entry function are added in Task 11+.
