//! Phase 12: Filter selection.
//!
//! Calls the pure recommendation helper, computes biquad coefficients
//! for whichever filters were chosen, and writes them to the TMC4671
//! sub-registers. Filters not selected are explicitly disabled to
//! avoid stale biquad state leaking from prior commissioning runs or
//! host configuration.

use super::OuterPhase;
use crate::autotune::{CommissionError, PhaseId};
use crate::commissioning::helpers;
use crate::commissioning::outer::fail_outer;
use crate::commissioning::types::{CommissioningBackend, StepResult};

/// Maximum register operations performed by filter selection.
#[allow(dead_code)]
pub(super) const FILTER_TUNE_MAX_REGISTER_OPS_PER_ARM: usize = 28;

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
        _ => return StepResult::ReadyNow,
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
        if helpers::write_biquad_velocity_checked(backend, cutoff_hz as u16, pwm_maxcnt)
            .await
            .is_err()
        {
            return fail_outer(phase, CommissionError::Spi);
        }
    } else {
        if helpers::write_sub_register_checked(
            backend,
            foci_tmc4671::registers::CONFIG_BIQUAD_V_ENABLE,
            0,
        )
        .await
        .is_err()
        {
            return fail_outer(phase, CommissionError::Spi);
        }
    }

    if let Some(cutoff_hz) = filter_config.position_filter_hz {
        if helpers::write_biquad_position_checked(backend, cutoff_hz as u16, pwm_maxcnt)
            .await
            .is_err()
        {
            return fail_outer(phase, CommissionError::Spi);
        }
    } else {
        if helpers::write_sub_register_checked(
            backend,
            foci_tmc4671::registers::CONFIG_BIQUAD_X_ENABLE,
            0,
        )
        .await
        .is_err()
        {
            return fail_outer(phase, CommissionError::Spi);
        }
    }

    if let Some(cutoff_hz) = filter_config.flux_filter_hz {
        if helpers::write_biquad_flux_checked(backend, cutoff_hz as u16, pwm_maxcnt)
            .await
            .is_err()
        {
            return fail_outer(phase, CommissionError::Spi);
        }
    } else {
        if helpers::write_sub_register_checked(
            backend,
            foci_tmc4671::registers::CONFIG_BIQUAD_F_ENABLE,
            0,
        )
        .await
        .is_err()
        {
            return fail_outer(phase, CommissionError::Spi);
        }
    }

    if let Some(cutoff_hz) = filter_config.torque_filter_hz {
        if helpers::write_biquad_torque_checked(backend, cutoff_hz as u16, pwm_maxcnt)
            .await
            .is_err()
        {
            return fail_outer(phase, CommissionError::Spi);
        }
    } else {
        if helpers::write_sub_register_checked(
            backend,
            foci_tmc4671::registers::CONFIG_BIQUAD_T_ENABLE,
            0,
        )
        .await
        .is_err()
        {
            return fail_outer(phase, CommissionError::Spi);
        }
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

#[cfg(test)]
mod tests {
    use super::{FILTER_TUNE_MAX_REGISTER_OPS_PER_ARM, step_filter_tune};
    use crate::autotune::{CommissionError, PositionGains, VelocityGains};
    use crate::commission_dispatch::test_support::{MockBackend, SpiOp};
    use crate::commissioning::outer::{OuterDiagnostics, OuterPhase, OuterSafetyLimits, commit};
    use crate::commissioning::types::StepResult;
    use foci_tmc4671::pid::PidGain;
    use foci_tmc4671::registers::SubRegister;

    fn gain(raw: u16) -> PidGain {
        PidGain::new(raw).expect("test gain must be in range")
    }

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

    fn has_sub_write(backend: &MockBackend, sub: SubRegister, value: u32) -> bool {
        backend.spi_log.windows(2).any(|pair| {
            matches!(
                pair,
                [SpiOp::Write(selector_addr, selector_value), SpiOp::Write(data_addr, data_value)]
                    if *selector_addr == sub.addr.addr()
                        && *selector_value == sub.sub_addr as u32
                        && *data_addr == sub.data.addr()
                        && *data_value == value
            )
        })
    }

    #[test]
    fn motion_filter_disable_failure_fails_outer_with_spi() {
        pollster::block_on(async {
            let mut phase = OuterPhase::FilterTune {
                velocity_gains: VelocityGains {
                    velocity_p: gain(1152),
                    velocity_i: PidGain::ZERO,
                    velocity_limit: 16,
                    effective_bandwidth_hz: 100,
                    lambda_vel_ms: 10,
                },
                position_gains: PositionGains {
                    position_p: gain(640),
                    position_i: PidGain::ZERO,
                },
                position_oscillation_early: false,
                velocity_chatter: true,
                current_ringing: false,
                current_bandwidth_hz: 0,
                j_eff: 1,
                b_eff: 1,
                diagnostics: zero_diagnostics(),
                safety: OuterSafetyLimits::new(
                    2448,
                    crate::commissioning::outer::OuterMotionBudget::firmware_caps(),
                ),
            };
            let mut backend = MockBackend::new();
            backend.fail_on_write =
                Some(foci_tmc4671::registers::CONFIG_BIQUAD_V_ENABLE.data.addr());

            let result = step_filter_tune(&mut phase, &mut backend).await;

            assert!(matches!(
                result,
                StepResult::OuterFailed(CommissionError::Spi)
            ));
            assert!(matches!(phase, OuterPhase::Failed(CommissionError::Spi)));
        });
    }

    #[test]
    fn motion_filters_are_disabled_while_current_filters_reach_commit() {
        pollster::block_on(async {
            let mut phase = OuterPhase::FilterTune {
                velocity_gains: VelocityGains {
                    velocity_p: gain(1_152),
                    velocity_i: PidGain::ZERO,
                    velocity_limit: 16,
                    effective_bandwidth_hz: 100,
                    lambda_vel_ms: 10,
                },
                position_gains: PositionGains {
                    position_p: gain(640),
                    position_i: PidGain::ZERO,
                },
                position_oscillation_early: true,
                velocity_chatter: true,
                current_ringing: true,
                current_bandwidth_hz: 1_600,
                j_eff: 1,
                b_eff: 1,
                diagnostics: zero_diagnostics(),
                safety: OuterSafetyLimits::new(
                    2_448,
                    crate::commissioning::outer::OuterMotionBudget::firmware_caps(),
                ),
            };
            let mut backend = MockBackend::new();

            let result = step_filter_tune(&mut phase, &mut backend).await;

            assert!(matches!(
                result,
                StepResult::PhaseChanged(crate::autotune::PhaseId::Commit)
            ));

            use foci_tmc4671::registers::{
                CONFIG_BIQUAD_F_ENABLE, CONFIG_BIQUAD_T_ENABLE, CONFIG_BIQUAD_V_ENABLE,
                CONFIG_BIQUAD_X_ENABLE,
            };
            assert!(has_sub_write(&backend, CONFIG_BIQUAD_V_ENABLE, 0));
            assert!(has_sub_write(&backend, CONFIG_BIQUAD_X_ENABLE, 0));
            assert!(has_sub_write(&backend, CONFIG_BIQUAD_F_ENABLE, 1));
            assert!(has_sub_write(&backend, CONFIG_BIQUAD_T_ENABLE, 1));
            assert_eq!(backend.spi_log.len(), FILTER_TUNE_MAX_REGISTER_OPS_PER_ARM);

            let OuterPhase::Commit { filter_config, .. } = &phase else {
                panic!("expected commit");
            };
            assert_eq!(filter_config.velocity_filter_hz, None);
            assert_eq!(filter_config.position_filter_hz, None);
            assert_eq!(filter_config.flux_filter_hz, Some(1000));
            assert_eq!(filter_config.torque_filter_hz, Some(1000));

            let StepResult::OuterDone(result) = commit::step_commit(&mut phase) else {
                panic!("expected accepted outer result");
            };
            assert_eq!(result.velocity_filter_hz, 0);
            assert_eq!(result.position_filter_hz, 0);
            assert_eq!(result.flux_filter_hz, 1000);
            assert_eq!(result.torque_filter_hz, 1000);
        });
    }
}
