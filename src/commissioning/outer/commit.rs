//! Phase 13: Commit.
//!
//! Assemble the final outer commissioning result from the accepted
//! gains, filter selection, and diagnostics. Reject gains that exceed
//! the safety envelope by falling back to conservative runtime defaults
//! and flagging the result with `OUTER_WARNING_UNSAFE_SYNTH_GAINS`.

use crate::autotune::CommissionStatus;
use crate::commission::{
    OUTER_FALLBACK_POSITION_I, OUTER_FALLBACK_POSITION_P, OUTER_FALLBACK_VELOCITY_I,
    OUTER_FALLBACK_VELOCITY_P, OUTER_MAX_POSITION_I, OUTER_MAX_POSITION_P, OUTER_MAX_VELOCITY_I,
    OUTER_MAX_VELOCITY_P, OUTER_WARNING_NONE, OUTER_WARNING_UNSAFE_SYNTH_GAINS, OuterPhase,
};
use crate::commissioning::types::{OuterResult, StepResult};

/// Assemble and emit the final outer commissioning result.
pub(crate) fn step_commit(phase: &mut OuterPhase) -> StepResult {
    let (mut velocity_gains, mut position_gains, filter_config, j_eff, b_eff, mut diagnostics) =
        match phase {
            OuterPhase::Commit {
                velocity_gains,
                position_gains,
                filter_config,
                j_eff,
                b_eff,
                diagnostics,
            } => (
                *velocity_gains,
                *position_gains,
                *filter_config,
                *j_eff,
                *b_eff,
                *diagnostics,
            ),
            _ => return StepResult::Continue,
        };

    if diagnostics.synth_velocity_p == 0 && diagnostics.synth_velocity_i == 0 {
        diagnostics.synth_velocity_p = velocity_gains.velocity_p;
        diagnostics.synth_velocity_i = velocity_gains.velocity_i;
    }
    diagnostics.synth_position_p = position_gains.position_p;
    diagnostics.synth_position_i = position_gains.position_i;

    let mut status = if diagnostics.warning_code == OUTER_WARNING_NONE {
        CommissionStatus::Accepted
    } else {
        CommissionStatus::AcceptedWithWarnings
    };
    if velocity_gains.velocity_p > OUTER_MAX_VELOCITY_P
        || velocity_gains.velocity_i > OUTER_MAX_VELOCITY_I
        || position_gains.position_p > OUTER_MAX_POSITION_P
        || position_gains.position_i > OUTER_MAX_POSITION_I
    {
        defmt::warn!(
            "outer commit: rejecting unsafe gains vel_p={}, vel_i={}, pos_p={}, pos_i={}",
            velocity_gains.velocity_p,
            velocity_gains.velocity_i,
            position_gains.position_p,
            position_gains.position_i
        );
        velocity_gains.velocity_p = OUTER_FALLBACK_VELOCITY_P;
        velocity_gains.velocity_i = OUTER_FALLBACK_VELOCITY_I;
        position_gains.position_p = OUTER_FALLBACK_POSITION_P;
        position_gains.position_i = OUTER_FALLBACK_POSITION_I;
        diagnostics.warning_code = OUTER_WARNING_UNSAFE_SYNTH_GAINS;
        status = CommissionStatus::AcceptedWithWarnings;
    }

    let result = OuterResult {
        status,
        velocity_p: velocity_gains.velocity_p,
        velocity_i: velocity_gains.velocity_i,
        velocity_limit: velocity_gains.velocity_limit,
        position_p: position_gains.position_p,
        position_i: position_gains.position_i,
        velocity_filter_hz: filter_config.velocity_filter_hz.unwrap_or(0) as u16,
        position_filter_hz: filter_config.position_filter_hz.unwrap_or(0) as u16,
        flux_filter_hz: filter_config.flux_filter_hz.unwrap_or(0) as u16,
        torque_filter_hz: filter_config.torque_filter_hz.unwrap_or(0) as u16,
        j_eff,
        b_eff,
        warning_code: diagnostics.warning_code,
        mech_torque_step: diagnostics.mech_torque_step,
        mech_fwd_accel: diagnostics.mech_fwd_accel,
        mech_rev_accel: diagnostics.mech_rev_accel,
        mech_fwd_peak_velocity: diagnostics.mech_fwd_peak_velocity,
        mech_rev_peak_velocity: diagnostics.mech_rev_peak_velocity,
        mech_fwd_travel: diagnostics.mech_fwd_travel,
        mech_rev_travel: diagnostics.mech_rev_travel,
        synth_velocity_p: diagnostics.synth_velocity_p,
        synth_velocity_i: diagnostics.synth_velocity_i,
        synth_position_p: diagnostics.synth_position_p,
        synth_position_i: diagnostics.synth_position_i,
    };

    defmt::info!(
        "outer commit: vel_p={}, vel_i={}, pos_p={}, pos_i={}",
        result.velocity_p,
        result.velocity_i,
        result.position_p,
        result.position_i
    );

    *phase = OuterPhase::Done(result.clone());
    StepResult::OuterDone(result)
}
