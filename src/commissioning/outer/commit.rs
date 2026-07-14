//! Phase 13: Commit.
//!
//! Assemble the final outer commissioning result from the accepted
//! gains, filter selection, and diagnostics. Reject gains that exceed
//! the safety envelope by falling back to conservative runtime defaults
//! and flagging the result with `OUTER_WARNING_UNSAFE_SYNTH_GAINS`.

use super::{
    OUTER_FALLBACK_POSITION_I, OUTER_FALLBACK_POSITION_P, OUTER_FALLBACK_VELOCITY_I,
    OUTER_FALLBACK_VELOCITY_P, OUTER_MAX_POSITION_I, OUTER_MAX_POSITION_P, OUTER_MAX_VELOCITY_I,
    OUTER_MAX_VELOCITY_P, OUTER_WARNING_NONE, OUTER_WARNING_UNSAFE_SYNTH_GAINS, OuterPhase,
};
use crate::autotune::CommissionStatus;
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
            _ => return StepResult::ReadyNow,
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
        outer_evidence_flags: diagnostics.outer_evidence_flags,
        stiffness_timebase_ms: diagnostics.stiffness_timebase_ms,
        velocity_search_stop_reason: diagnostics.velocity_search_stop_reason,
        motion_budget_mrev: diagnostics.motion_budget_mrev,
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

#[cfg(test)]
mod tests {
    use super::step_commit;
    use crate::autotune::{FilterConfig, PositionGains, VelocityGains};
    use crate::commissioning::outer::{OuterDiagnostics, OuterPhase};
    use crate::commissioning::types::StepResult;

    #[test]
    fn commit_assembles_the_result_immediately() {
        let mut phase = OuterPhase::Commit {
            velocity_gains: VelocityGains {
                velocity_p: 350,
                velocity_i: 80,
                velocity_limit: 1_000,
                effective_bandwidth_hz: 100,
                lambda_vel_ms: 10,
            },
            position_gains: PositionGains {
                position_p: 100,
                position_i: 0,
            },
            filter_config: FilterConfig {
                velocity_filter_hz: None,
                position_filter_hz: None,
                flux_filter_hz: None,
                torque_filter_hz: None,
            },
            j_eff: 1,
            b_eff: 2,
            diagnostics: OuterDiagnostics {
                warning_code: 0,
                mech_torque_step: 0,
                mech_fwd_accel: 0,
                mech_rev_accel: 0,
                mech_fwd_peak_velocity: 0,
                mech_rev_peak_velocity: 0,
                mech_fwd_travel: 0,
                mech_rev_travel: 0,
                mech_tau_defaulted: false,
                synth_velocity_p: 0,
                synth_velocity_i: 0,
                synth_position_p: 0,
                synth_position_i: 0,
                outer_evidence_flags: 0,
                stiffness_timebase_ms: 0,
                velocity_search_stop_reason: 0,
                motion_budget_mrev: 0,
            },
        };

        let result = step_commit(&mut phase);

        assert!(matches!(result, StepResult::OuterDone(_)));
        assert!(matches!(phase, OuterPhase::Done(_)));
    }
}
