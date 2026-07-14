//! Phase 9 velocity-loop SIMC gain synthesis.

use super::super::{
    EffectiveVelocityPolicy, OUTER_MAX_VELOCITY_I, OUTER_MAX_VELOCITY_P,
    OUTER_WARNING_UNSAFE_SYNTH_GAINS, OuterPhase, VELOCITY_SEARCH_STOP_CLEAN, VelocityTuneStrategy,
    VelocityValidateData, VelocityValidateState,
};
use super::validate::VELOCITY_VALIDATE_STEP_PERCENT;
use crate::autotune::{CommissionError, CommissionProfile, OperatingMode, PhaseId};
use crate::commissioning::outer::{fail_outer, response};
use crate::commissioning::types::{CommissioningBackend, StepResult};

/// Phase 9: synthesize velocity gains and transition to validation.
///
/// If the mechanical-ID warning indicates low confidence, take the
/// response-ramp path with conservative defaults. Otherwise run the
/// SIMC synthesis; if it produces gains outside the safety envelope,
/// fall back to response-ramp tuning.
fn derive_effective_velocity_policy(
    requested_profile: CommissionProfile,
    inner_warning_flags: u8,
    current_bandwidth_hz: u32,
) -> EffectiveVelocityPolicy {
    let conservative_required =
        inner_warning_flags & ((1 << 5) | (1 << 6)) != 0 || current_bandwidth_hz == 0;
    let effective_profile = if conservative_required {
        CommissionProfile::Conservative
    } else {
        requested_profile
    };

    let mut lambda_multiplier = 1.0_f32;
    if inner_warning_flags & (1 << 0) != 0 {
        lambda_multiplier *= 1.2;
    }
    if inner_warning_flags & (1 << 1) != 0 {
        lambda_multiplier *= 1.5;
    }
    if inner_warning_flags & (1 << 3) != 0 {
        lambda_multiplier *= 1.5;
    }
    lambda_multiplier = lambda_multiplier.min(4.0);

    EffectiveVelocityPolicy {
        effective_profile,
        lambda_multiplier,
        conservative_required,
        lambda_widened: lambda_multiplier > 1.0 + f32::EPSILON,
    }
}

#[allow(clippy::too_many_arguments)]
pub(crate) async fn step_velocity_tune<B: CommissioningBackend>(
    phase: &mut OuterPhase,
    profile: CommissionProfile,
    mode: OperatingMode,
    inner_lambda_us: u32,
    theta_e_us: u32,
    inner_warning_flags: u8,
    current_bandwidth_hz: u32,
    backend: &mut B,
) -> StepResult {
    let (model, peak_velocity, mut diagnostics, safety) = match phase {
        OuterPhase::VelocityTune {
            model,
            peak_velocity,
            diagnostics,
            safety,
        } => (*model, *peak_velocity, *diagnostics, *safety),
        _ => return StepResult::ReadyNow,
    };

    let policy =
        derive_effective_velocity_policy(profile, inner_warning_flags, current_bandwidth_hz);
    if policy.lambda_widened {
        let widening_permille = (policy.lambda_multiplier * 1000.0) as u32;
        defmt::info!(
            "outer: widening velocity lambda by {} permille (flags=0x{:02x})",
            widening_permille,
            inner_warning_flags,
        );
    }

    let (gains, strategy, step_target) =
        if diagnostics.warning_code == response::OUTER_WARNING_LOW_CONFIDENCE_MECHANICAL_ID {
            let gains = response::response_velocity_gains(safety.counts_per_rev());
            (
                gains,
                VelocityTuneStrategy::ResponseRamp,
                response::VELOCITY_RESPONSE_STEP_TARGET,
            )
        } else {
            let config = backend.tmc_config();
            let encoder_counts_per_rev = backend.mechanical_encoder_quadrature_counts_per_rev();
            let gains = match crate::autotune::simc_velocity_gains_with_lambda_multiplier(
                &model,
                policy.effective_profile,
                mode,
                inner_lambda_us,
                theta_e_us,
                peak_velocity,
                encoder_counts_per_rev,
                config,
                policy.lambda_multiplier,
            ) {
                Some(gains) => gains,
                None => {
                    defmt::warn!("velocity tune: synthesis returned None");
                    return fail_outer(phase, CommissionError::MechanicalIdFailed);
                }
            };

            diagnostics.synth_velocity_p = gains.velocity_p;
            diagnostics.synth_velocity_i = gains.velocity_i;

            if gains.velocity_p > OUTER_MAX_VELOCITY_P || gains.velocity_i > OUTER_MAX_VELOCITY_I {
                diagnostics.warning_code = OUTER_WARNING_UNSAFE_SYNTH_GAINS;
                defmt::warn!(
                    "velocity tune: synthesized P={}, I={} outside envelope, using response tuning",
                    gains.velocity_p,
                    gains.velocity_i
                );
                (
                    response::response_velocity_gains(safety.counts_per_rev()),
                    VelocityTuneStrategy::ResponseRamp,
                    response::VELOCITY_RESPONSE_STEP_TARGET,
                )
            } else {
                let step_target = if gains.velocity_limit > 0 {
                    (gains.velocity_limit * VELOCITY_VALIDATE_STEP_PERCENT / 100).max(1)
                } else {
                    return fail_outer(phase, CommissionError::VelocityValidationFailed);
                };
                (gains, VelocityTuneStrategy::Model, step_target)
            }
        };

    defmt::info!(
        "velocity tune: P={}, I={}, limit={}, bw={}Hz, lambda={}ms, strategy={}",
        gains.velocity_p,
        gains.velocity_i,
        gains.velocity_limit,
        gains.effective_bandwidth_hz,
        gains.lambda_vel_ms,
        match strategy {
            VelocityTuneStrategy::Model => "model",
            VelocityTuneStrategy::ResponseRamp => "response",
        }
    );

    let low_confidence_mech =
        diagnostics.warning_code == response::OUTER_WARNING_LOW_CONFIDENCE_MECHANICAL_ID;
    let mech_tau_trusted = !low_confidence_mech && !diagnostics.mech_tau_defaulted;
    let stiffness_timebase_ms = if mech_tau_trusted {
        model.tau_mech_ms.max(1)
    } else {
        response::VELOCITY_RESPONSE_VALIDATE_TAU_MS
    };

    let data = VelocityValidateData {
        gains,
        accepted_gains: gains,
        has_accepted_gains: false,
        response_lower_p: 0,
        response_upper_p: 0,
        model,
        policy,
        strategy,
        stiffness_timebase_ms,
        mech_tau_trusted,
        velocity_search_stop_reason: VELOCITY_SEARCH_STOP_CLEAN,
        attempt: 0,
        step_target,
        forward_metrics: None,
        reverse_metrics: None,
        max_chatter: 0,
        peak_velocity,
        prev_encoder_for_velocity: None,
        saturation_count: 0,
        torque_limit: backend.current_limit() & 0xFFFF,
        operational_velocity_limit: backend.operational_velocity_limit(),
        diagnostics,
    };

    *phase = OuterPhase::VelocityValidate {
        state: VelocityValidateState::WriteGains,
        data,
        safety,
    };
    StepResult::PhaseChanged(PhaseId::VelocityValidate)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::autotune::MechanicalModel;
    use crate::commission_dispatch::test_support::MockBackend;
    use crate::commissioning::outer::{
        OuterDiagnostics, OuterPhase, OuterSafetyLimits, VelocityTuneStrategy,
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
    fn advanced_pi_i_above_envelope_uses_response_tuning() {
        pollster::block_on(async {
            let mut safety = OuterSafetyLimits::new(
                2_448,
                crate::commissioning::outer::OuterMotionBudget::firmware_caps(),
            );
            safety.configure_encoder_resolution(4_000).unwrap();
            let mut phase = OuterPhase::VelocityTune {
                model: MechanicalModel {
                    j_eff: 400,
                    b_eff: 100,
                    tau_mech_ms: 4,
                    theta_mech_us: 500,
                    t_f: 0,
                },
                peak_velocity: 1_000,
                diagnostics: zero_diagnostics(),
                safety,
            };
            let mut backend = MockBackend::new();
            backend.current_limit_value = 2_448;

            let result = step_velocity_tune(
                &mut phase,
                CommissionProfile::Balanced,
                OperatingMode::Nominal,
                100,
                20,
                0,
                1_000,
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::PhaseChanged(PhaseId::VelocityValidate)
            ));
            match phase {
                OuterPhase::VelocityValidate { data, .. } => {
                    assert_eq!(data.strategy, VelocityTuneStrategy::ResponseRamp);
                    assert!(data.diagnostics.synth_velocity_i > OUTER_MAX_VELOCITY_I);
                    assert_eq!(
                        data.diagnostics.warning_code,
                        OUTER_WARNING_UNSAFE_SYNTH_GAINS
                    );
                    assert_eq!(data.gains.velocity_i, 0);
                }
                other => panic!("expected VelocityValidate, got {other:?}"),
            }
        });
    }

    #[test]
    fn velocity_tune_model_warning_widens_final_lambda_when_mechanical_lambda_dominates() {
        pollster::block_on(async {
            let model = MechanicalModel {
                j_eff: 1_000_000,
                b_eff: 1_000,
                tau_mech_ms: 100,
                theta_mech_us: 20_000,
                t_f: 0,
            };
            let mut clean_phase = OuterPhase::VelocityTune {
                model,
                peak_velocity: 1_000,
                diagnostics: zero_diagnostics(),
                safety: OuterSafetyLimits::new(
                    2_448,
                    crate::commissioning::outer::OuterMotionBudget::firmware_caps(),
                ),
            };
            let mut warned_phase = OuterPhase::VelocityTune {
                model,
                peak_velocity: 1_000,
                diagnostics: zero_diagnostics(),
                safety: OuterSafetyLimits::new(
                    2_448,
                    crate::commissioning::outer::OuterMotionBudget::firmware_caps(),
                ),
            };
            let mut backend = MockBackend::new();
            backend.current_limit_value = 2_448;

            let clean_result = step_velocity_tune(
                &mut clean_phase,
                CommissionProfile::Balanced,
                OperatingMode::Nominal,
                1_000,
                100,
                0,
                1_000,
                &mut backend,
            )
            .await;
            assert!(matches!(
                clean_result,
                StepResult::PhaseChanged(crate::autotune::PhaseId::VelocityValidate)
            ));

            let clean_lambda_ms = match clean_phase {
                OuterPhase::VelocityValidate { data, .. } => {
                    assert_eq!(data.strategy, VelocityTuneStrategy::Model);
                    data.gains.lambda_vel_ms
                }
                other => panic!("expected VelocityValidate, got {:?}", other),
            };

            // Planner resolution must not change encoder-domain gain synthesis.
            let warned_result = step_velocity_tune(
                &mut warned_phase,
                CommissionProfile::Balanced,
                OperatingMode::Nominal,
                1_000,
                100,
                1 << 3,
                1_000,
                &mut backend,
            )
            .await;
            assert!(matches!(
                warned_result,
                StepResult::PhaseChanged(crate::autotune::PhaseId::VelocityValidate)
            ));

            let warned_lambda_ms = match warned_phase {
                OuterPhase::VelocityValidate { data, .. } => {
                    assert_eq!(data.strategy, VelocityTuneStrategy::Model);
                    data.gains.lambda_vel_ms
                }
                other => panic!("expected VelocityValidate, got {:?}", other),
            };

            assert_eq!(clean_lambda_ms, 60);
            assert_eq!(warned_lambda_ms, 90);
        });
    }

    #[test]
    fn velocity_tune_defaulted_tau_uses_conservative_timebase() {
        pollster::block_on(async {
            let model = MechanicalModel {
                j_eff: 1_000_000,
                b_eff: 1_000,
                tau_mech_ms: 1000,
                theta_mech_us: 20_000,
                t_f: 0,
            };
            let mut diagnostics = zero_diagnostics();
            diagnostics.mech_tau_defaulted = true;
            let mut phase = OuterPhase::VelocityTune {
                model,
                peak_velocity: 1_000,
                diagnostics,
                safety: OuterSafetyLimits::new(
                    2_448,
                    crate::commissioning::outer::OuterMotionBudget::firmware_caps(),
                ),
            };
            let mut backend = MockBackend::new();
            backend.current_limit_value = 2_448;

            let result = step_velocity_tune(
                &mut phase,
                CommissionProfile::Balanced,
                OperatingMode::Nominal,
                1_000,
                100,
                0,
                1_000,
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::PhaseChanged(crate::autotune::PhaseId::VelocityValidate)
            ));
            match phase {
                OuterPhase::VelocityValidate { data, .. } => {
                    assert_eq!(data.strategy, VelocityTuneStrategy::Model);
                    assert!(!data.mech_tau_trusted);
                    assert_eq!(
                        data.stiffness_timebase_ms,
                        response::VELOCITY_RESPONSE_VALIDATE_TAU_MS
                    );
                }
                other => panic!("expected VelocityValidate, got {:?}", other),
            }
        });
    }

    // ---------------------------------------------------------------
    // Phase 1 inner-confidence outer-loop policy
    // (spec docs/specs/2026-04-30-inner-commissioning-stability.md §4)
    // ---------------------------------------------------------------

    #[test]
    fn clean_policy_preserves_every_requested_profile() {
        for profile in [
            CommissionProfile::Conservative,
            CommissionProfile::Balanced,
            CommissionProfile::Stiff,
        ] {
            let policy = derive_effective_velocity_policy(profile, 0, 1_000);
            assert_eq!(policy.effective_profile, profile);
            assert!(!policy.conservative_required);
            assert!(!policy.lambda_widened);
            assert!((policy.lambda_multiplier - 1.0).abs() < 1e-6);
        }
    }

    #[test]
    fn degraded_confidence_requires_conservative_even_when_already_requested() {
        for (profile, flags, bandwidth_hz) in [
            (CommissionProfile::Stiff, 1 << 5, 1_000),
            (CommissionProfile::Balanced, 1 << 6, 1_000),
            (CommissionProfile::Stiff, 0, 0),
            (CommissionProfile::Conservative, 1 << 5, 1_000),
        ] {
            let policy = derive_effective_velocity_policy(profile, flags, bandwidth_hz);
            assert_eq!(policy.effective_profile, CommissionProfile::Conservative);
            assert!(policy.conservative_required);
        }
    }

    #[test]
    fn warning_bits_retain_existing_lambda_widening() {
        for (flags, expected) in [
            (1 << 0, 1.2),
            (1 << 1, 1.5),
            (1 << 3, 1.5),
            ((1 << 0) | (1 << 1) | (1 << 3), 2.7),
        ] {
            let policy =
                derive_effective_velocity_policy(CommissionProfile::Balanced, flags, 1_000);
            assert!(policy.lambda_widened);
            assert!((policy.lambda_multiplier - expected).abs() < 1e-6);
        }
    }

    #[test]
    fn non_derating_warning_bits_do_not_widen_lambda() {
        let policy = derive_effective_velocity_policy(
            CommissionProfile::Balanced,
            (1 << 2) | (1 << 5) | (1 << 6),
            1_000,
        );
        assert!(!policy.lambda_widened);
        assert!((policy.lambda_multiplier - 1.0).abs() < 1e-6);
    }
}
