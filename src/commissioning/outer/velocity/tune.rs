//! Phase 9 velocity-loop SIMC gain synthesis.

use super::super::{
    OUTER_MAX_VELOCITY_I, OUTER_MAX_VELOCITY_P, OUTER_WARNING_UNSAFE_SYNTH_GAINS, OuterPhase,
    VELOCITY_SEARCH_STOP_CLEAN, VelocityTuneStrategy, VelocityValidateData, VelocityValidateState,
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
/// Compose per-bit lambda widening multipliers for the inner-confidence
/// warning bitfield. Multipliers compose multiplicatively and clamp at 4.0x.
///
/// - bit 0 (per-coil R mismatch) → 1.2x
/// - bit 1 (per-coil tau/L mismatch) → 1.5x
/// - bit 3 (theta/tau ratio) → 1.5x
pub(crate) fn outer_lambda_widening(inner_warning_flags: u8) -> f32 {
    let mut multiplier: f32 = 1.0;
    if inner_warning_flags & (1 << 0) != 0 {
        multiplier *= 1.2;
    }
    if inner_warning_flags & (1 << 1) != 0 {
        multiplier *= 1.5;
    }
    if inner_warning_flags & (1 << 3) != 0 {
        multiplier *= 1.5;
    }
    multiplier.min(4.0)
}

/// Force the outer profile to Conservative when the inner contract reports
/// fallback gains (bit 5), host-default confidence (bit 6), or zero
/// current-loop bandwidth.
pub(crate) fn force_conservative_profile(
    profile: CommissionProfile,
    inner_warning_flags: u8,
    current_bandwidth_hz: u32,
) -> CommissionProfile {
    if inner_warning_flags & (1 << 5) != 0
        || inner_warning_flags & (1 << 6) != 0
        || current_bandwidth_hz == 0
    {
        CommissionProfile::Conservative
    } else {
        profile
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
        _ => return StepResult::Continue,
    };

    let effective_profile =
        force_conservative_profile(profile, inner_warning_flags, current_bandwidth_hz);
    let widening = outer_lambda_widening(inner_warning_flags);
    if widening > 1.0 + f32::EPSILON {
        let widening_permille = (widening * 1000.0) as u32;
        defmt::info!(
            "outer: widening velocity lambda by {} permille (flags=0x{:02x})",
            widening_permille,
            inner_warning_flags,
        );
    }

    let (gains, strategy, step_target) =
        if diagnostics.warning_code == response::OUTER_WARNING_LOW_CONFIDENCE_MECHANICAL_ID {
            let gains = response::response_velocity_gains(safety.counts_per_rev);
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
                effective_profile,
                mode,
                inner_lambda_us,
                theta_e_us,
                peak_velocity,
                encoder_counts_per_rev,
                config,
                widening,
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
                    response::response_velocity_gains(safety.counts_per_rev),
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
        prev_encoder_for_velocity: 0,
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
                    0,
                    2_448,
                    crate::commissioning::outer::OuterMotionBudget::firmware_caps(),
                ),
            };
            let mut warned_phase = OuterPhase::VelocityTune {
                model,
                peak_velocity: 1_000,
                diagnostics: zero_diagnostics(),
                safety: OuterSafetyLimits::new(
                    0,
                    2_448,
                    crate::commissioning::outer::OuterMotionBudget::firmware_caps(),
                ),
            };
            let mut backend = MockBackend::new();
            backend.step_width_value = 16;
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
            backend.step_width_value = 8;
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
                    0,
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
    fn outer_widening_zero_when_clean() {
        let multiplier = outer_lambda_widening(0);
        assert!((multiplier - 1.0).abs() < 1e-6);
    }

    #[test]
    fn outer_widening_bit_0_uses_1_2x() {
        let multiplier = outer_lambda_widening(1 << 0);
        assert!((multiplier - 1.2).abs() < 1e-6);
    }

    #[test]
    fn outer_widening_bit_1_uses_1_5x() {
        let multiplier = outer_lambda_widening(1 << 1);
        assert!((multiplier - 1.5).abs() < 1e-6);
    }

    #[test]
    fn outer_widening_bit_2_is_telemetry_only() {
        let lo = outer_lambda_widening(1 << 2);
        assert!((lo - 1.0).abs() < 1e-6);
        let hi = outer_lambda_widening(1 << 2);
        assert!((hi - 1.0).abs() < 1e-6);
    }

    #[test]
    fn outer_widening_bit_3_uses_1_5x() {
        let multiplier = outer_lambda_widening(1 << 3);
        assert!((multiplier - 1.5).abs() < 1e-6);
    }

    #[test]
    fn outer_widening_clamps_at_four() {
        // Bits 0, 1, and 3 widen; bit 2 is unused.
        let multiplier = outer_lambda_widening(0x0F);
        assert!((multiplier - 2.7).abs() < 1e-6);
    }

    #[test]
    fn outer_widening_ignores_bits_5_and_6() {
        // Bits 5 (fallback gains) and 6 (host-default confidence) drive
        // profile override, NOT lambda widening (spec §4).
        let multiplier = outer_lambda_widening((1 << 5) | (1 << 6));
        assert!((multiplier - 1.0).abs() < 1e-6);
    }

    #[test]
    fn outer_widening_composes_two_bits_multiplicatively() {
        // bit 0 (1.2x) + bit 3 (1.5x) = 1.8x.
        let multiplier = outer_lambda_widening((1 << 0) | (1 << 3));
        assert!((multiplier - 1.8).abs() < 1e-6);
    }

    #[test]
    fn force_conservative_profile_clean_inputs_pass_through() {
        assert_eq!(
            force_conservative_profile(CommissionProfile::Stiff, 0, 1000),
            CommissionProfile::Stiff
        );
        assert_eq!(
            force_conservative_profile(CommissionProfile::Balanced, 0, 1000),
            CommissionProfile::Balanced
        );
    }

    #[test]
    fn force_conservative_profile_when_fallback_bit_set() {
        assert_eq!(
            force_conservative_profile(CommissionProfile::Stiff, 1 << 5, 1000),
            CommissionProfile::Conservative
        );
    }

    #[test]
    fn force_conservative_profile_when_host_default_bit_set() {
        assert_eq!(
            force_conservative_profile(CommissionProfile::Stiff, 1 << 6, 1000),
            CommissionProfile::Conservative
        );
    }

    #[test]
    fn force_conservative_profile_when_bandwidth_zero() {
        assert_eq!(
            force_conservative_profile(CommissionProfile::Stiff, 0, 0),
            CommissionProfile::Conservative
        );
    }

    #[test]
    fn force_conservative_profile_ignores_model_warning_bits() {
        // Bits 0, 1, and 3 widen lambda but do not force conservative;
        // bit 2 is unused.
        assert_eq!(
            force_conservative_profile(CommissionProfile::Balanced, 0x0F, 1000),
            CommissionProfile::Balanced
        );
    }
}
