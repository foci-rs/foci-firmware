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
        MechanicalIdData, MechanicalIdState, OuterDiagnostics, OuterPhase, OuterSafetyLimits,
        VelocityTuneStrategy, VelocityValidateData, VelocityValidateState, mechanical, step_outer,
        velocity,
    };
    use crate::autotune::{
        CommissionError, CommissionProfile, MechanicalModel, OperatingMode, PhaseId,
        TransientBuffer, VelocityGains, VelocityValidation,
    };
    use crate::commission_dispatch::test_support::{MockBackend, SpiOp};
    use crate::commissioning::types::StepResult;
    use foci_tmc4671::registers::{ABN_DECODER_PPR, MODE_RAMP_MODE_MOTION, PID_TORQUE_FLUX_TARGET};

    #[test]
    fn mechanical_id_setup_zeroes_torque_before_switching_modes() {
        pollster::block_on(async {
            let mut phase = OuterPhase::MechanicalId {
                state: MechanicalIdState::Setup,
                data: MechanicalIdData::new(0, 0),
                safety: OuterSafetyLimits::new(0, 0),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();
            backend.current_limit_value = 2448;
            backend.encoder_count_value = 1234;
            let _ = backend.registers.insert(ABN_DECODER_PPR.addr(), 4000);

            let result = step_outer(
                &mut phase,
                &mut buf,
                CommissionProfile::Balanced,
                OperatingMode::Nominal,
                0,
                0,
                false,
                0,
                0,
                0,
                0,
                0,
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::PhaseChanged(PhaseId::MechanicalId)
            ));

            let zero_torque_idx = backend
                .spi_log
                .iter()
                .position(|op| {
                    matches!(
                        op,
                        SpiOp::Write(addr, 0) if *addr == PID_TORQUE_FLUX_TARGET.addr()
                    )
                })
                .expect("setup should zero PID_TORQUE_FLUX_TARGET");
            let mode_switch_idx = backend
                .spi_log
                .iter()
                .position(|op| {
                    matches!(
                        op,
                        SpiOp::Write(addr, _) if *addr == MODE_RAMP_MODE_MOTION.addr()
                    )
                })
                .expect("setup should switch MODE_RAMP_MODE_MOTION");

            assert!(
                zero_torque_idx < mode_switch_idx,
                "setup must zero torque before entering torque mode"
            );
        });
    }

    #[test]
    fn mechanical_id_ignores_immediate_post_switch_velocity_spike() {
        pollster::block_on(async {
            let mut phase = OuterPhase::MechanicalId {
                state: MechanicalIdState::Setup,
                data: MechanicalIdData::new(0, 0),
                safety: OuterSafetyLimits::new(0, 0),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();
            backend.current_limit_value = 2448;
            backend.encoder_count_value = 1000;
            let _ = backend.registers.insert(ABN_DECODER_PPR.addr(), 4000);

            let first = step_outer(
                &mut phase,
                &mut buf,
                CommissionProfile::Balanced,
                OperatingMode::Nominal,
                0,
                0,
                false,
                0,
                0,
                0,
                0,
                0,
                &mut backend,
            )
            .await;
            assert!(matches!(
                first,
                StepResult::PhaseChanged(PhaseId::MechanicalId)
            ));

            // Reproduce the field failure: the first sample after the position->torque
            // mode switch jumps by 25 counts/ms before the intended torque step starts.
            backend.encoder_count_value = 1025;

            let second = step_outer(
                &mut phase,
                &mut buf,
                CommissionProfile::Balanced,
                OperatingMode::Nominal,
                0,
                0,
                false,
                0,
                0,
                0,
                0,
                0,
                &mut backend,
            )
            .await;

            assert!(
                !matches!(
                    second,
                    StepResult::OuterFailed(CommissionError::SafetyEnvelopeViolation)
                ),
                "mechanical-id should not abort on the immediate post-switch spike"
            );
            assert!(
                !matches!(
                    phase,
                    OuterPhase::Failed(CommissionError::SafetyEnvelopeViolation)
                ),
                "phase should survive the immediate post-switch spike"
            );
        });
    }

    #[test]
    fn mechanical_id_waits_for_post_switch_settle_before_arming_safety() {
        pollster::block_on(async {
            let mut phase = OuterPhase::MechanicalId {
                state: MechanicalIdState::Setup,
                data: MechanicalIdData::new(0, 0),
                safety: OuterSafetyLimits::new(0, 0),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();
            backend.current_limit_value = 2448;
            backend.encoder_count_value = 1000;
            let _ = backend.registers.insert(ABN_DECODER_PPR.addr(), 4000);

            let first = step_outer(
                &mut phase,
                &mut buf,
                CommissionProfile::Balanced,
                OperatingMode::Nominal,
                0,
                0,
                false,
                0,
                0,
                0,
                0,
                0,
                &mut backend,
            )
            .await;
            assert!(matches!(
                first,
                StepResult::PhaseChanged(PhaseId::MechanicalId)
            ));

            backend.elapsed_us_value = 10_000;
            backend.encoder_count_value = 1025;
            let second = step_outer(
                &mut phase,
                &mut buf,
                CommissionProfile::Balanced,
                OperatingMode::Nominal,
                0,
                0,
                false,
                0,
                0,
                0,
                0,
                0,
                &mut backend,
            )
            .await;
            assert!(matches!(second, StepResult::Continue));
            match &phase {
                OuterPhase::MechanicalId { state, safety, .. } => {
                    assert!(matches!(state, MechanicalIdState::PostSwitchSettle { .. }));
                    assert_eq!(
                        safety.torque_cap, 0,
                        "safety should remain disarmed while drifting"
                    );
                }
                _ => panic!("expected MechanicalId phase"),
            }

            for (us, pos) in [(11_000, 1026), (12_000, 1027), (13_000, 1028)] {
                backend.elapsed_us_value = us;
                backend.encoder_count_value = pos;
                let result = step_outer(
                    &mut phase,
                    &mut buf,
                    CommissionProfile::Balanced,
                    OperatingMode::Nominal,
                    0,
                    0,
                    false,
                    0,
                    0,
                    0,
                    0,
                    0,
                    &mut backend,
                )
                .await;
                assert!(matches!(result, StepResult::Continue));
            }

            match &phase {
                OuterPhase::MechanicalId {
                    state,
                    safety,
                    data,
                } => {
                    assert!(matches!(state, MechanicalIdState::ForwardCapture { .. }));
                    assert_eq!(safety.torque_cap, 2448);
                    assert_eq!(safety.prev_encoder_count, 1028);
                    assert_eq!(data.start_position, 1027);
                }
                _ => panic!("expected MechanicalId phase"),
            }

            assert!(
                backend.spi_log.iter().any(|op| matches!(
                    op,
                    SpiOp::Write(addr, val)
                        if *addr == PID_TORQUE_FLUX_TARGET.addr() && (*val >> 16) != 0
                )),
                "settle completion should apply the first torque step immediately"
            );
        });
    }

    #[test]
    fn mechanical_id_fails_if_post_switch_motion_never_settles() {
        pollster::block_on(async {
            let mut phase = OuterPhase::MechanicalId {
                state: MechanicalIdState::Setup,
                data: MechanicalIdData::new(0, 0),
                safety: OuterSafetyLimits::new(0, 0),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();
            backend.current_limit_value = 2448;
            backend.encoder_count_value = 1000;
            let _ = backend.registers.insert(ABN_DECODER_PPR.addr(), 4000);

            let first = step_outer(
                &mut phase,
                &mut buf,
                CommissionProfile::Balanced,
                OperatingMode::Nominal,
                0,
                0,
                false,
                0,
                0,
                0,
                0,
                0,
                &mut backend,
            )
            .await;
            assert!(matches!(
                first,
                StepResult::PhaseChanged(PhaseId::MechanicalId)
            ));

            for (us, pos) in [
                (10_000, 1050),
                (20_000, 1100),
                (30_000, 1150),
                (40_000, 1200),
            ] {
                backend.elapsed_us_value = us;
                backend.encoder_count_value = pos;
                let _ = step_outer(
                    &mut phase,
                    &mut buf,
                    CommissionProfile::Balanced,
                    OperatingMode::Nominal,
                    0,
                    0,
                    false,
                    0,
                    0,
                    0,
                    0,
                    0,
                    &mut backend,
                )
                .await;
            }

            backend.elapsed_us_value = 150_000;
            backend.encoder_count_value = 1250;
            let result = step_outer(
                &mut phase,
                &mut buf,
                CommissionProfile::Balanced,
                OperatingMode::Nominal,
                0,
                0,
                false,
                0,
                0,
                0,
                0,
                0,
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::OuterFailed(CommissionError::SafetyEnvelopeViolation)
            ));
            assert!(matches!(
                phase,
                OuterPhase::Failed(CommissionError::SafetyEnvelopeViolation)
            ));
        });
    }

    #[test]
    fn low_confidence_slope_falls_back_instead_of_retrying_more_torque() {
        pollster::block_on(async {
            use crate::commissioning::outer::slope_estimator::SlopeEstimate;
            let low_conf = SlopeEstimate::default(); // fit_sample_count = 0 → low-conf
            let no_response =
                crate::commissioning::outer::response::MechanicalResponseEstimate::default();
            let mut phase = OuterPhase::MechanicalId {
                state: MechanicalIdState::Finalize,
                data: MechanicalIdData {
                    start_position: 0,
                    attempt: 0,
                    torque_step: 367,
                    fwd_slope: low_conf,
                    fwd_response: no_response,
                    forward_peak_velocity: 28,
                    forward_delay_us: 1000,
                    forward_travel: 3830,
                    rev_slope: low_conf,
                    rev_response: no_response,
                    reverse_peak_velocity: 29,
                    reverse_delay_us: 1000,
                    reverse_travel: 3888,
                    sample_timestamps_us: [0; crate::autotune::TRANSIENT_LEN],
                    sample_timestamp_len: 0,
                },
                safety: OuterSafetyLimits::new(0, 2448),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();

            let result = mechanical::step_mechanical_id(&mut phase, &mut buf, &mut backend).await;

            assert!(matches!(
                result,
                StepResult::PhaseChanged(PhaseId::VelocityTune)
            ));
            assert!(
                matches!(phase, OuterPhase::VelocityTune { .. }),
                "low-confidence should fall back to VelocityTune (response ramp), not retry"
            );
        });
    }

    #[test]
    fn full_response_model_takes_precedence_over_low_confidence_slope() {
        pollster::block_on(async {
            use crate::commissioning::outer::response::MechanicalResponseEstimate;
            use crate::commissioning::outer::slope_estimator::SlopeEstimate;
            let low_conf = SlopeEstimate::default();
            let fwd_response = MechanicalResponseEstimate {
                j_eff: 100_000,
                b_eff: 20_000,
                tau_mech_ms: 5,
                theta_mech_us: 10_000,
                steady_velocity_cpms: 18,
                travel: 3000,
                residual_mcpms: 0,
                fit_sample_count: 4,
                sign_match: true,
            };
            let rev_response = MechanicalResponseEstimate {
                steady_velocity_cpms: -18,
                ..fwd_response
            };
            let mut phase = OuterPhase::MechanicalId {
                state: MechanicalIdState::Finalize,
                data: MechanicalIdData {
                    start_position: 0,
                    attempt: 0,
                    torque_step: 122,
                    fwd_slope: low_conf,
                    fwd_response,
                    forward_peak_velocity: 18,
                    forward_delay_us: 10_000,
                    forward_travel: 3000,
                    rev_slope: low_conf,
                    rev_response,
                    reverse_peak_velocity: 18,
                    reverse_delay_us: 10_000,
                    reverse_travel: 3000,
                    sample_timestamps_us: [0; crate::autotune::TRANSIENT_LEN],
                    sample_timestamp_len: 0,
                },
                safety: OuterSafetyLimits::new(0, 2448),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();

            let result = mechanical::step_mechanical_id(&mut phase, &mut buf, &mut backend).await;

            assert!(matches!(
                result,
                StepResult::PhaseChanged(PhaseId::VelocityTune)
            ));
            match phase {
                OuterPhase::VelocityTune {
                    model, diagnostics, ..
                } => {
                    assert_eq!(diagnostics.warning_code, super::OUTER_WARNING_NONE);
                    assert_eq!(model.j_eff, 100_000);
                    assert_eq!(model.b_eff, 20_000);
                    assert_eq!(model.tau_mech_ms, 5);
                }
                other => panic!("expected VelocityTune phase, got {:?}", other),
            }
        });
    }

    #[test]
    fn asymmetric_full_response_retries_with_more_torque() {
        pollster::block_on(async {
            use crate::commissioning::outer::response::MechanicalResponseEstimate;
            let fwd_response = MechanicalResponseEstimate {
                j_eff: 100_000,
                b_eff: 20_000,
                tau_mech_ms: 5,
                theta_mech_us: 10_000,
                steady_velocity_cpms: 18,
                travel: 3000,
                residual_mcpms: 0,
                fit_sample_count: 4,
                sign_match: true,
            };
            let rev_response = MechanicalResponseEstimate {
                j_eff: 200_000,
                steady_velocity_cpms: -18,
                ..fwd_response
            };
            let mut phase = OuterPhase::MechanicalId {
                state: MechanicalIdState::Finalize,
                data: MechanicalIdData {
                    start_position: 0,
                    attempt: 0,
                    torque_step: 367,
                    fwd_slope: Default::default(),
                    fwd_response,
                    forward_peak_velocity: 18,
                    forward_delay_us: 10_000,
                    forward_travel: 3000,
                    rev_slope: Default::default(),
                    rev_response,
                    reverse_peak_velocity: 18,
                    reverse_delay_us: 10_000,
                    reverse_travel: 3000,
                    sample_timestamps_us: [0; crate::autotune::TRANSIENT_LEN],
                    sample_timestamp_len: 0,
                },
                safety: OuterSafetyLimits::new(0, 2448),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();

            let result = mechanical::step_mechanical_id(&mut phase, &mut buf, &mut backend).await;

            assert!(matches!(result, StepResult::Continue));
            match phase {
                OuterPhase::MechanicalId { state, data, .. } => {
                    assert_eq!(data.attempt, 1);
                    assert!(matches!(state, MechanicalIdState::Setup));
                }
                other => panic!("expected MechanicalId retry, got {:?}", other),
            }
        });
    }

    #[test]
    fn high_confidence_asymmetry_still_retries_with_more_torque() {
        pollster::block_on(async {
            use crate::commissioning::outer::slope_estimator::SlopeEstimate;
            // Both slopes high-confidence, asymmetric by >30%.
            let high_conf_fwd = SlopeEstimate {
                accel: 20,
                residual_counts: 1,
                velocity_rise_mcpms: 20_000,
                fit_sample_count: 6,
                sign_match: true,
            };
            let high_conf_rev = SlopeEstimate {
                accel: -50,
                residual_counts: 1,
                velocity_rise_mcpms: -50_000,
                fit_sample_count: 6,
                sign_match: true,
            };
            let no_response =
                crate::commissioning::outer::response::MechanicalResponseEstimate::default();
            let mut phase = OuterPhase::MechanicalId {
                state: MechanicalIdState::Finalize,
                data: MechanicalIdData {
                    start_position: 0,
                    attempt: 0,
                    torque_step: 367,
                    fwd_slope: high_conf_fwd,
                    fwd_response: no_response,
                    forward_peak_velocity: 20,
                    forward_delay_us: 1000,
                    forward_travel: 3000,
                    rev_slope: high_conf_rev,
                    rev_response: no_response,
                    reverse_peak_velocity: 50,
                    reverse_delay_us: 1000,
                    reverse_travel: 5000,
                    sample_timestamps_us: [0; crate::autotune::TRANSIENT_LEN],
                    sample_timestamp_len: 0,
                },
                safety: OuterSafetyLimits::new(0, 2448),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();

            let _ = mechanical::step_mechanical_id(&mut phase, &mut buf, &mut backend).await;

            // Expect retry: attempt increments, state returns to Setup.
            match &phase {
                OuterPhase::MechanicalId { state, data, .. } => {
                    assert_eq!(data.attempt, 1, "expected attempt increment on retry");
                    assert!(
                        matches!(state, MechanicalIdState::Setup),
                        "expected return to Setup for retry, got {:?}",
                        state,
                    );
                }
                other => panic!("expected MechanicalId phase after retry, got {:?}", other),
            }
        });
    }

    #[test]
    fn high_confidence_zero_accel_still_retries() {
        pollster::block_on(async {
            use crate::commissioning::outer::slope_estimator::SlopeEstimate;
            // Forward slope "high-confidence" but accel is zero: a
            // high-confidence fit could still produce 0 if the rotor
            // genuinely didn't move during the fit window. The zero-accel
            // retry path is gated on confidence, not on magnitude, so
            // this retries.
            let zero_accel = SlopeEstimate {
                accel: 0,
                residual_counts: 1,
                velocity_rise_mcpms: 10_000,
                fit_sample_count: 6,
                sign_match: true,
            };
            let no_response =
                crate::commissioning::outer::response::MechanicalResponseEstimate::default();
            let mut phase = OuterPhase::MechanicalId {
                state: MechanicalIdState::Finalize,
                data: MechanicalIdData {
                    start_position: 0,
                    attempt: 0,
                    torque_step: 367,
                    fwd_slope: zero_accel,
                    fwd_response: no_response,
                    forward_peak_velocity: 10,
                    forward_delay_us: 1000,
                    forward_travel: 100,
                    rev_slope: zero_accel,
                    rev_response: no_response,
                    reverse_peak_velocity: 10,
                    reverse_delay_us: 1000,
                    reverse_travel: 100,
                    sample_timestamps_us: [0; crate::autotune::TRANSIENT_LEN],
                    sample_timestamp_len: 0,
                },
                safety: OuterSafetyLimits::new(0, 2448),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();

            let _ = mechanical::step_mechanical_id(&mut phase, &mut buf, &mut backend).await;

            match &phase {
                OuterPhase::MechanicalId { state, data, .. } => {
                    assert_eq!(data.attempt, 1, "high-confidence zero-accel should retry");
                    assert!(matches!(state, MechanicalIdState::Setup));
                }
                other => panic!("expected retry, got {:?}", other),
            }
        });
    }

    #[test]
    fn high_signal_asymmetry_still_retries_with_more_torque() {
        pollster::block_on(async {
            use crate::commissioning::outer::slope_estimator::SlopeEstimate;
            // Both slopes high-confidence, asymmetric by >30%.
            let high_conf_fwd = SlopeEstimate {
                accel: 20,
                residual_counts: 1,
                velocity_rise_mcpms: 20_000,
                fit_sample_count: 6,
                sign_match: true,
            };
            let high_conf_rev = SlopeEstimate {
                accel: -50,
                residual_counts: 1,
                velocity_rise_mcpms: -50_000,
                fit_sample_count: 6,
                sign_match: true,
            };
            let no_response =
                crate::commissioning::outer::response::MechanicalResponseEstimate::default();
            let mut phase = OuterPhase::MechanicalId {
                state: MechanicalIdState::Finalize,
                data: MechanicalIdData {
                    start_position: 0,
                    attempt: 0,
                    torque_step: 367,
                    fwd_slope: high_conf_fwd,
                    fwd_response: no_response,
                    forward_peak_velocity: 28,
                    forward_delay_us: 1000,
                    forward_travel: 3830,
                    rev_slope: high_conf_rev,
                    rev_response: no_response,
                    reverse_peak_velocity: 29,
                    reverse_delay_us: 1000,
                    reverse_travel: 3888,
                    sample_timestamps_us: [0; crate::autotune::TRANSIENT_LEN],
                    sample_timestamp_len: 0,
                },
                safety: OuterSafetyLimits::new(0, 2448),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();

            let _ = mechanical::step_mechanical_id(&mut phase, &mut buf, &mut backend).await;

            match &phase {
                OuterPhase::MechanicalId { state, data, .. } => {
                    assert_eq!(data.attempt, 1, "expected attempt increment on retry");
                    assert!(
                        matches!(state, MechanicalIdState::Setup),
                        "expected return to Setup for retry, got {:?}",
                        state,
                    );
                }
                other => panic!("expected MechanicalId phase after retry, got {:?}", other),
            }
        });
    }

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
