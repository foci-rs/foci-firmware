use crate::autotune::{
    CommissionProfile, CommissionStatus, MechanicalModel, OperatingMode, PhaseId, TransientBuffer,
};
use crate::commission_dispatch::test_support::MockBackend;
use crate::commissioning::types::StepResult;

use super::{
    OUTER_EVIDENCE_CONSERVATIVE_REQUIRED, OUTER_EVIDENCE_LAMBDA_WIDENED, OuterDiagnostics,
    OuterMotionBudget, OuterPhase, OuterSafetyLimits, POS_TUNE_INITIAL_P, PositionTuneState,
    VelocityValidateState, commit, filter, position, velocity,
};

fn zero_diagnostics() -> OuterDiagnostics {
    OuterDiagnostics {
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
    }
}

#[test]
fn policy_evidence_survives_every_outer_transition() {
    pollster::block_on(async {
        let model = MechanicalModel {
            j_eff: 1_000_000,
            b_eff: 1_000,
            tau_mech_ms: 100,
            theta_mech_us: 20_000,
            t_f: 0,
        };
        let mut safety = OuterSafetyLimits::new(2_448, OuterMotionBudget::firmware_caps());
        safety.configure_encoder_resolution(4_000).unwrap();
        let mut phase = OuterPhase::VelocityTune {
            model,
            peak_velocity: 1_000,
            diagnostics: zero_diagnostics(),
            safety,
        };
        let mut backend = MockBackend::new();

        let tuned = velocity::step_velocity_tune(
            &mut phase,
            CommissionProfile::Stiff,
            OperatingMode::Nominal,
            1_000,
            100,
            (1 << 0) | (1 << 5),
            1_000,
            &mut backend,
        )
        .await;
        assert!(matches!(
            tuned,
            StepResult::PhaseChanged(PhaseId::VelocityValidate)
        ));

        let OuterPhase::VelocityValidate { state, data, .. } = &mut phase else {
            panic!("expected velocity validation");
        };
        assert_eq!(
            data.policy.effective_profile,
            CommissionProfile::Conservative
        );
        assert!((data.policy.lambda_multiplier - 1.2).abs() < f32::EPSILON);
        let expected_flags = OUTER_EVIDENCE_CONSERVATIVE_REQUIRED | OUTER_EVIDENCE_LAMBDA_WIDENED;
        assert_eq!(
            data.diagnostics.outer_evidence_flags & expected_flags,
            expected_flags
        );
        *state = VelocityValidateState::Finalize;

        let mut buf = TransientBuffer::new();
        let validated = velocity::step_velocity_validate(
            &mut phase,
            &mut buf,
            OperatingMode::Nominal,
            1_000,
            100,
            false,
            1_000,
            &mut backend,
        )
        .await;
        assert!(matches!(
            validated,
            StepResult::PhaseChanged(PhaseId::PositionTune)
        ));

        let OuterPhase::PositionTune { state, data, .. } = &mut phase else {
            panic!("expected position tuning");
        };
        assert_eq!(
            data.diagnostics.outer_evidence_flags & expected_flags,
            expected_flags
        );
        data.stable_p = POS_TUNE_INITIAL_P;
        *state = PositionTuneState::Finalize;

        let positioned = position::step_position_tune(&mut phase, &mut buf, &mut backend).await;
        assert!(matches!(
            positioned,
            StepResult::PhaseChanged(PhaseId::FilterTune)
        ));
        let OuterPhase::FilterTune { diagnostics, .. } = &phase else {
            panic!("expected filter tuning");
        };
        assert_eq!(
            diagnostics.outer_evidence_flags & expected_flags,
            expected_flags
        );

        let filtered = filter::step_filter_tune(&mut phase, &mut backend).await;
        assert!(matches!(
            filtered,
            StepResult::PhaseChanged(PhaseId::Commit)
        ));
        let OuterPhase::Commit { diagnostics, .. } = &phase else {
            panic!("expected commit");
        };
        assert_eq!(
            diagnostics.outer_evidence_flags & expected_flags,
            expected_flags
        );

        let committed = commit::step_commit(&mut phase);
        let StepResult::OuterDone(result) = committed else {
            panic!("expected terminal outer result");
        };
        assert_eq!(result.status, CommissionStatus::AcceptedWithWarnings);
        assert_eq!(result.outer_evidence_flags & expected_flags, expected_flags);
    });
}
