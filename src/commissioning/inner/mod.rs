//! Stage 1 (current-loop) commissioning: per-phase step functions.
//!
//! Submodules implement individual phase step routines; the
//! `commission` module's `step_inner` dispatcher routes to them based on
//! the active `InnerPhase` variant.

use crate::autotune::CommissionError;
use crate::commission::InnerPhase;
use crate::commissioning::types::StepResult;

pub mod adc;

/// Transition the inner phase to `Failed` and emit an `InnerFailed` result.
///
/// Free-function counterpart of `CommissioningEngine::fail_inner`. As step
/// functions move out of the engine impl, they call this directly via
/// `super::fail_inner`; the impl-side method delegates here so the two
/// sites stay in sync until the dispatcher itself is extracted.
pub(crate) fn fail_inner(phase: &mut InnerPhase, err: CommissionError) -> StepResult {
    *phase = InnerPhase::Failed(err);
    StepResult::InnerFailed(err)
}
