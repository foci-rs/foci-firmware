//! Velocity-loop commissioning phases.

pub mod band_confirmation;
pub(crate) mod breakaway_campaign;
pub(crate) mod breakaway_excursion;
pub mod breakaway_plan;
pub(crate) mod breakaway_reporting;
pub mod breakaway_search;
#[cfg(feature = "trace")]
pub(crate) mod breakaway_trace_reporting;
pub mod integral_evidence;
#[cfg(test)]
mod integral_evidence_tests;
pub mod integral_plan;
#[cfg(test)]
mod integral_plan_tests;
pub(crate) mod integral_reporting;
pub mod integral_reproduction;
#[cfg(test)]
mod integral_reproduction_tests;
mod integral_sweep;
#[cfg(test)]
mod integral_sweep_tests;
#[cfg(test)]
mod integral_test_point_contract_tests;
#[cfg_attr(not(test), expect(dead_code))]
mod peak_ratio;
pub(crate) mod recovery;
#[cfg(test)]
mod recovery_tests;
pub mod reproduction;
pub(crate) mod robustness_cycles;
pub mod robustness_evidence;
pub mod robustness_metrics;
pub(crate) mod robustness_reporting;
pub(crate) mod robustness_reversal;
mod sweep_analysis;
pub mod sweep_consensus;
#[cfg(test)]
mod sweep_consensus_tests;
pub mod sweep_evidence;
pub mod sweep_plan;
pub(crate) mod sweep_reporting;
pub(crate) mod velocity_clamp;
#[cfg(test)]
mod velocity_clamp_tests;
pub mod workflow_plan;

/// Wire value declaring best-effort polling for velocity-limit evidence.
pub const VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT: u32 =
    velocity_clamp::VelocityLimitEvidenceCapability::BestEffortPolling as u32;

/// Wire value declaring latched velocity-limit status evidence.
pub const VELOCITY_LIMIT_EVIDENCE_CAPABILITY_LATCHED_FLAG: u32 =
    velocity_clamp::VelocityLimitEvidenceCapability::LatchedFlag as u32;

pub(crate) use breakaway_campaign::{
    breakaway_campaign_fault_terminal, breakaway_campaign_pending_terminal,
    step_breakaway_campaign_with_trace,
};
pub(crate) use integral_sweep::{
    integral_fault_terminal, integral_i_at, step_velocity_integral_with_trace,
};

/// Zero the TMC4671 velocity-loop integral error sum (`PID_ERROR_PID_VELOCITY_ERROR_SUM`).
///
/// Does not touch `PID_VELOCITY_OFFSET`; see [`clear_velocity_offset`].
///
/// # Errors
///
/// Returns [`CommissionError::Spi`] when the register write fails.
pub(crate) async fn clear_velocity_error_sum<B>(
    backend: &mut B,
) -> Result<(), crate::autotune::CommissionError>
where
    B: crate::commissioning::backend::CommissioningBackend,
{
    crate::commissioning::helpers::write_sub_register_checked(
        backend,
        foci_tmc4671::registers::PID_ERROR_PID_VELOCITY_ERROR_SUM,
        0,
    )
    .await
    .map_err(|_| crate::autotune::CommissionError::Spi)
}

/// Zero the TMC4671 velocity feedforward offset (`PID_VELOCITY_OFFSET`).
///
/// Does not clear the error sum; see [`clear_velocity_error_sum`].
///
/// # Errors
///
/// Returns [`CommissionError::Spi`] when the register write fails.
pub(crate) async fn clear_velocity_offset<B>(
    backend: &mut B,
) -> Result<(), crate::autotune::CommissionError>
where
    B: crate::commissioning::backend::CommissioningBackend,
{
    backend
        .write_register(foci_tmc4671::registers::PID_VELOCITY_OFFSET, 0)
        .await
        .map_err(|_| crate::autotune::CommissionError::Spi)
}
