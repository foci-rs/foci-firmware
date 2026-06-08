//! Stage 2 (mechanical / velocity / position / filter) commissioning.
//!
//! `OuterPhase` is the cooperative state-machine type. `step_outer`
//! dispatches one tick to the per-phase submodule, prefixed by a
//! safety-envelope check that aborts on velocity- or position-window
//! violations. The phase variants and their sub-state types live here
//! so each per-phase submodule can pattern-match and construct them.

pub mod commit;
mod dispatch;
pub mod filter;
mod limits;
pub mod mechanical;
pub mod position;
pub mod response;
mod safety;
pub mod slope_estimator;
mod state;
mod trace;
pub mod velocity;

pub(crate) use dispatch::{fail_outer, step_outer, step_outer_with_trace};
pub(crate) use limits::{
    MECHANICAL_ID_VELOCITY_CAP_COUNTS_PER_MS, OUTER_FALLBACK_POSITION_I, OUTER_FALLBACK_POSITION_P,
    OUTER_FALLBACK_VELOCITY_I, OUTER_FALLBACK_VELOCITY_P, OUTER_MAX_POSITION_I,
    OUTER_MAX_POSITION_P, OUTER_MAX_VELOCITY_I, OUTER_MAX_VELOCITY_P, OUTER_WARNING_NONE,
    OUTER_WARNING_UNSAFE_SYNTH_GAINS, POS_TUNE_INITIAL_P, REFERENCE_ENCODER_COUNTS_PER_REV,
    mechanical_id_velocity_cap_counts_per_ms, position_tune_step_counts, position_window_counts,
    response_fallback_velocity_limit_counts_per_ms,
};
pub(crate) use safety::{OuterSafetyLimits, check_outer_safety, safety_error_for_phase};
pub(crate) use state::{
    MechanicalIdData, MechanicalIdState, OuterDiagnostics, OuterPhase, PositionTuneData,
    PositionTuneState, VelocityTuneStrategy, VelocityValidateData, VelocityValidateState,
};
pub(crate) use trace::{
    TraceSummarySeqs, trace_diagnostic_warning, trace_pack_i16_pair, trace_pack_u16_pair,
    trace_summary_event,
};

#[cfg(test)]
mod tests {
    use super::{
        OuterDiagnostics, OuterPhase, OuterSafetyLimits, VelocityTuneStrategy,
        VelocityValidateData, VelocityValidateState, velocity,
    };
    use crate::autotune::{
        CommissionError, CommissionProfile, MechanicalModel, OperatingMode, TransientBuffer,
        VelocityGains, VelocityValidation,
    };
    use crate::commission_dispatch::test_support::MockBackend;
    use crate::commissioning::types::StepResult;

    #[test]
    fn response_ramp_fail_does_not_advance_to_position_tune() {
        pollster::block_on(async {
            let mut phase = OuterPhase::VelocityValidate {
                state: VelocityValidateState::Classify,
                data: VelocityValidateData {
                    gains: VelocityGains {
                        velocity_p: 1152,
                        velocity_i: 0,
                        velocity_limit: 16,
                        effective_bandwidth_hz: 0,
                        lambda_vel_ms: 0,
                    },
                    accepted_gains: VelocityGains {
                        velocity_p: 1152,
                        velocity_i: 0,
                        velocity_limit: 16,
                        effective_bandwidth_hz: 0,
                        lambda_vel_ms: 0,
                    },
                    has_accepted_gains: false,
                    response_lower_p: 0,
                    response_upper_p: 0,
                    model: MechanicalModel {
                        j_eff: 1,
                        b_eff: 1,
                        tau_mech_ms: 50,
                        theta_mech_us: 0,
                        t_f: 0,
                    },
                    strategy: VelocityTuneStrategy::ResponseRamp,
                    attempt: 0,
                    step_target: 4,
                    forward_metrics: Some(VelocityValidation {
                        rise_time_ms: 20,
                        overshoot_pct: 5250,
                        peak_velocity: 25,
                        settling_time_ms: 256,
                        zero_speed_chatter: false,
                        saturation_samples: 0,
                        symmetry_pct: 0,
                    }),
                    reverse_metrics: Some(VelocityValidation {
                        rise_time_ms: 0,
                        overshoot_pct: 5000,
                        peak_velocity: 24,
                        settling_time_ms: 256,
                        zero_speed_chatter: false,
                        saturation_samples: 0,
                        symmetry_pct: 0,
                    }),
                    max_chatter: 24,
                    peak_velocity: 18,
                    prev_encoder_for_velocity: 0,
                    saturation_count: 0,
                    torque_limit: 2448,
                    operational_velocity_limit: 500_000,
                    diagnostics: OuterDiagnostics {
                        warning_code: 1,
                        mech_torque_step: 367,
                        mech_fwd_accel: 0,
                        mech_rev_accel: 0,
                        mech_fwd_peak_velocity: 18,
                        mech_rev_peak_velocity: 18,
                        mech_fwd_travel: 3907,
                        mech_rev_travel: 3886,
                        synth_velocity_p: 0,
                        synth_velocity_i: 0,
                        synth_position_p: 0,
                        synth_position_i: 0,
                    },
                },
                safety: OuterSafetyLimits::new(0, 2448),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();

            let result = velocity::step_velocity_validate(
                &mut phase,
                &mut buf,
                CommissionProfile::Balanced,
                OperatingMode::Nominal,
                0,
                0,
                false,
                0,
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::OuterFailed(CommissionError::VelocityValidationFailed)
            ));
            assert!(matches!(
                phase,
                OuterPhase::Failed(CommissionError::VelocityValidationFailed)
            ));
        });
    }
}
