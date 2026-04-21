//! Phase 12: Filter selection.
//!
//! Calls the pure recommendation helper, computes biquad coefficients
//! for whichever filters were chosen, and writes them to the TMC4671
//! sub-registers. Filters not selected are explicitly disabled to
//! avoid stale biquad state leaking from prior commissioning runs or
//! host configuration.

use crate::autotune::PhaseId;
use crate::commission::OuterPhase;
use crate::commissioning::helpers;
use crate::commissioning::types::{CommissioningBackend, StepResult};

/// Implement filter selection from prior validation metrics.
pub(crate) async fn step_filter_tune<B: CommissioningBackend>(
    phase: &mut OuterPhase,
    backend: &mut B,
) -> StepResult {
    let (
        velocity_gains,
        position_gains,
        position_oscillation_early,
        velocity_chatter,
        current_ringing,
        current_bandwidth_hz,
        j_eff,
        b_eff,
        diagnostics,
    ) = match phase {
        OuterPhase::FilterTune {
            velocity_gains,
            position_gains,
            position_oscillation_early,
            velocity_chatter,
            current_ringing,
            current_bandwidth_hz,
            j_eff,
            b_eff,
            diagnostics,
            ..
        } => (
            *velocity_gains,
            *position_gains,
            *position_oscillation_early,
            *velocity_chatter,
            *current_ringing,
            *current_bandwidth_hz,
            *j_eff,
            *b_eff,
            *diagnostics,
        ),
        _ => return StepResult::Continue,
    };

    // Estimate position bandwidth as velocity_bw / 4.
    let position_bandwidth_hz = velocity_gains.effective_bandwidth_hz / 4;

    let filter_config = crate::autotune::recommend_filters(
        velocity_chatter,
        velocity_gains.effective_bandwidth_hz,
        position_oscillation_early,
        position_bandwidth_hz,
        current_ringing,
        current_bandwidth_hz,
    );

    let pwm_maxcnt = backend.pwm_maxcnt();

    if let Some(cutoff_hz) = filter_config.velocity_filter_hz {
        helpers::write_biquad_velocity(backend, cutoff_hz as u16, pwm_maxcnt).await;
    } else {
        helpers::write_sub_register(backend, foci_tmc4671::registers::CONFIG_BIQUAD_V_ENABLE, 0)
            .await;
    }

    if let Some(cutoff_hz) = filter_config.position_filter_hz {
        helpers::write_biquad_position(backend, cutoff_hz as u16, pwm_maxcnt).await;
    } else {
        helpers::write_sub_register(backend, foci_tmc4671::registers::CONFIG_BIQUAD_X_ENABLE, 0)
            .await;
    }

    if let Some(cutoff_hz) = filter_config.flux_filter_hz {
        helpers::write_biquad_flux(backend, cutoff_hz as u16, pwm_maxcnt).await;
    } else {
        helpers::write_sub_register(backend, foci_tmc4671::registers::CONFIG_BIQUAD_F_ENABLE, 0)
            .await;
    }

    if let Some(cutoff_hz) = filter_config.torque_filter_hz {
        helpers::write_biquad_torque(backend, cutoff_hz as u16, pwm_maxcnt).await;
    } else {
        helpers::write_sub_register(backend, foci_tmc4671::registers::CONFIG_BIQUAD_T_ENABLE, 0)
            .await;
    }

    defmt::info!(
        "filter tune: vel={}, pos={}, flux={}, torque={}",
        filter_config.velocity_filter_hz.unwrap_or(0),
        filter_config.position_filter_hz.unwrap_or(0),
        filter_config.flux_filter_hz.unwrap_or(0),
        filter_config.torque_filter_hz.unwrap_or(0)
    );

    *phase = OuterPhase::Commit {
        velocity_gains,
        position_gains,
        filter_config,
        j_eff,
        b_eff,
        diagnostics,
    };
    StepResult::PhaseChanged(PhaseId::Commit)
}
