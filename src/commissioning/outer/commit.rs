//! Phase 13: Commit.
//!
//! Assemble the final outer commissioning result from the accepted
//! gains, filter selection, and diagnostics. Reject accepted gains that
//! violate the phase envelopes instead of publishing a different controller.

use super::{
    OUTER_MAX_POSITION_I, OUTER_MAX_POSITION_P, OUTER_MAX_VELOCITY_I, OUTER_MAX_VELOCITY_P,
    OUTER_POLICY_WARNING_MASK, OUTER_WARNING_NONE, OuterPhase, fail_outer,
};
use crate::autotune::{CommissionError, CommissionStatus};
use crate::commissioning::types::{OuterResult, StepResult};

/// Assemble and emit the final outer commissioning result.
pub(crate) fn step_commit(phase: &mut OuterPhase) -> StepResult {
    let (velocity_gains, position_gains, filter_config, j_eff, b_eff, mut diagnostics) = match phase
    {
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
        diagnostics.synth_velocity_p = velocity_gains.velocity_p.raw();
        diagnostics.synth_velocity_i = velocity_gains.velocity_i.raw();
    }
    diagnostics.synth_position_p = position_gains.position_p.raw();
    diagnostics.synth_position_i = position_gains.position_i.raw();

    if velocity_gains.velocity_p.raw() > OUTER_MAX_VELOCITY_P
        || velocity_gains.velocity_i.raw() > OUTER_MAX_VELOCITY_I
    {
        defmt::warn!(
            "outer commit: invalid accepted velocity gains P={}, I={}",
            velocity_gains.velocity_p,
            velocity_gains.velocity_i
        );
        return fail_outer(phase, CommissionError::VelocityValidationFailed);
    }

    if position_gains.position_p.raw() > OUTER_MAX_POSITION_P
        || position_gains.position_i.raw() > OUTER_MAX_POSITION_I
    {
        defmt::warn!(
            "outer commit: invalid accepted position gains P={}, I={}",
            position_gains.position_p,
            position_gains.position_i
        );
        return fail_outer(phase, CommissionError::PositionTuneFailed);
    }

    let has_policy_warning = diagnostics.outer_evidence_flags & OUTER_POLICY_WARNING_MASK != 0;
    let status = if diagnostics.warning_code == OUTER_WARNING_NONE && !has_policy_warning {
        CommissionStatus::Accepted
    } else {
        CommissionStatus::AcceptedWithWarnings
    };

    let result = OuterResult {
        status,
        velocity_p: velocity_gains.velocity_p.raw(),
        velocity_i: velocity_gains.velocity_i.raw(),
        velocity_limit: velocity_gains.velocity_limit,
        position_p: position_gains.position_p.raw(),
        position_i: position_gains.position_i.raw(),
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
        provenance: crate::commissioning::types::TuneProvenance::default(),
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
    use super::*;
    use crate::autotune::{CommissionError, FilterConfig, PositionGains, VelocityGains};
    use crate::commissioning::outer::{
        OUTER_EVIDENCE_CONSERVATIVE_REQUIRED, OUTER_EVIDENCE_LAMBDA_WIDENED,
        OUTER_EVIDENCE_LOW_CONFIDENCE_MECH, OUTER_EVIDENCE_MOTION_BUDGET_CLAMPED,
        OUTER_EVIDENCE_TAU_DEFAULTED, OUTER_EVIDENCE_UNDER_DRIVEN_PROBE, OuterDiagnostics,
        OuterPhase,
    };
    use crate::commissioning::types::StepResult;
    use foci_tmc4671::pid::{PidIGain, PidPGain};

    fn p_gain(raw: u16) -> PidPGain {
        PidPGain::new(raw).expect("test gain must be in range")
    }

    fn i_gain(raw: u16) -> PidIGain {
        PidIGain::new(raw).expect("test gain must be in range")
    }

    fn zero_diagnostics() -> OuterDiagnostics {
        OuterDiagnostics {
            warning_code: OUTER_WARNING_NONE,
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
        }
    }

    fn commit_phase(velocity_gains: VelocityGains, position_gains: PositionGains) -> OuterPhase {
        OuterPhase::Commit {
            velocity_gains,
            position_gains,
            filter_config: FilterConfig {
                velocity_filter_hz: None,
                position_filter_hz: None,
                flux_filter_hz: None,
                torque_filter_hz: None,
            },
            j_eff: 11,
            b_eff: 22,
            diagnostics: zero_diagnostics(),
        }
    }

    fn valid_velocity_gains() -> VelocityGains {
        VelocityGains {
            velocity_p: p_gain(1152),
            velocity_i: i_gain(341),
            velocity_limit: 16,
            effective_bandwidth_hz: 50,
            lambda_vel_ms: 20,
        }
    }

    fn valid_position_gains() -> PositionGains {
        PositionGains {
            position_p: p_gain(640),
            position_i: PidIGain::ZERO,
        }
    }

    fn commit_with_evidence(outer_evidence_flags: u8) -> OuterResult {
        let mut phase = commit_phase(valid_velocity_gains(), valid_position_gains());
        let OuterPhase::Commit { diagnostics, .. } = &mut phase else {
            unreachable!();
        };
        diagnostics.outer_evidence_flags = outer_evidence_flags;

        let StepResult::OuterDone(result) = step_commit(&mut phase) else {
            panic!("expected accepted outer result");
        };
        result
    }

    #[test]
    fn conservative_required_policy_is_accepted_with_warnings() {
        let result = commit_with_evidence(OUTER_EVIDENCE_CONSERVATIVE_REQUIRED);
        assert_eq!(result.status, CommissionStatus::AcceptedWithWarnings);
    }

    #[test]
    fn widened_lambda_policy_is_accepted_with_warnings() {
        let result = commit_with_evidence(OUTER_EVIDENCE_LAMBDA_WIDENED);
        assert_eq!(result.status, CommissionStatus::AcceptedWithWarnings);
    }

    #[test]
    fn unrelated_outer_evidence_does_not_change_status() {
        let result = commit_with_evidence(
            OUTER_EVIDENCE_UNDER_DRIVEN_PROBE
                | OUTER_EVIDENCE_LOW_CONFIDENCE_MECH
                | OUTER_EVIDENCE_TAU_DEFAULTED
                | OUTER_EVIDENCE_MOTION_BUDGET_CLAMPED,
        );
        assert_eq!(result.status, CommissionStatus::Accepted);
    }

    #[test]
    fn warning_code_remains_accepted_with_warnings() {
        let mut phase = commit_phase(valid_velocity_gains(), valid_position_gains());
        let OuterPhase::Commit { diagnostics, .. } = &mut phase else {
            unreachable!();
        };
        diagnostics.warning_code = 1;

        let StepResult::OuterDone(result) = step_commit(&mut phase) else {
            panic!("expected accepted outer result");
        };
        assert_eq!(result.status, CommissionStatus::AcceptedWithWarnings);
    }

    #[test]
    fn commit_copies_validated_gains_bit_for_bit() {
        let velocity = valid_velocity_gains();
        let position = valid_position_gains();
        let mut phase = commit_phase(velocity, position);

        let result = step_commit(&mut phase);

        let StepResult::OuterDone(result) = result else {
            panic!("expected OuterDone, got {result:?}");
        };
        assert_eq!(result.velocity_p, velocity.velocity_p.raw());
        assert_eq!(result.velocity_i, velocity.velocity_i.raw());
        assert_eq!(result.velocity_limit, velocity.velocity_limit);
        assert_eq!(result.position_p, position.position_p.raw());
        assert_eq!(result.position_i, position.position_i.raw());
        assert_eq!(result.synth_velocity_p, velocity.velocity_p.raw());
        assert_eq!(result.synth_velocity_i, velocity.velocity_i.raw());
        assert_eq!(result.synth_position_p, position.position_p.raw());
        assert_eq!(result.synth_position_i, position.position_i.raw());
    }

    #[test]
    fn commit_rejects_out_of_envelope_velocity_gains() {
        let mut velocity = valid_velocity_gains();
        velocity.velocity_i = i_gain(OUTER_MAX_VELOCITY_I + 1);
        let mut phase = commit_phase(velocity, valid_position_gains());

        let result = step_commit(&mut phase);

        assert!(matches!(
            result,
            StepResult::OuterFailed(CommissionError::VelocityValidationFailed)
        ));
        assert!(matches!(
            phase,
            OuterPhase::Failed(CommissionError::VelocityValidationFailed)
        ));
    }

    #[test]
    fn commit_rejects_out_of_envelope_position_gains() {
        let mut position = valid_position_gains();
        position.position_p = p_gain(OUTER_MAX_POSITION_P + 1);
        let mut phase = commit_phase(valid_velocity_gains(), position);

        let result = step_commit(&mut phase);

        assert!(matches!(
            result,
            StepResult::OuterFailed(CommissionError::PositionTuneFailed)
        ));
        assert!(matches!(
            phase,
            OuterPhase::Failed(CommissionError::PositionTuneFailed)
        ));
    }
}
