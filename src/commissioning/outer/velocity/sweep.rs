//! Trace-gated disturbance-identification velocity sweep.

use crate::autotune::CommissionError;
use crate::commissioning::backend::CommissioningBackend;
use crate::commissioning::outer::{OuterPhase, VelocitySweepState};
use crate::commissioning::types::StepResult;

use super::super::dispatch::fail_outer;

/// Advance the trace-gated velocity sweep by one cooperative state-machine step.
pub(crate) async fn step_velocity_sweep_with_trace<B: CommissioningBackend>(
    phase: &mut OuterPhase,
    backend: &mut B,
) -> StepResult {
    let (state, data, safety) = match phase {
        OuterPhase::VelocitySweep {
            state,
            data,
            safety,
        } => (state, data, safety),
        _ => return StepResult::ReadyNow,
    };

    match state {
        VelocitySweepState::Setup => {
            if backend.is_shutdown_requested() {
                return fail_outer(phase, CommissionError::Shutdown);
            }
            let encoder_counts_per_rev = backend.mechanical_encoder_quadrature_counts_per_rev();
            let torque_limit = u16::from(backend.current_limit());
            let request = super::sweep_plan::VelocitySweepRequest {
                requested_velocity_mrev_s: safety.budget.requested_velocity_mrev_s,
                machine_velocity_ceiling_mrev_s: safety.budget.machine_velocity_ceiling_mrev_s,
                requested_velocity_source: safety.budget.requested_velocity_source,
                max_stroke_travel_mrev: safety.budget.max_stroke_travel_mrev,
                settle_travel_reserve_mrev: safety.budget.settle_travel_reserve_mrev,
                negative_position_headroom_mrev: safety.budget.negative_position_headroom_mrev,
                positive_position_headroom_mrev: safety.budget.positive_position_headroom_mrev,
                max_duration_ms: safety.budget.max_duration_ms,
            };
            let plan = match super::sweep_plan::plan_velocity_sweep(
                request,
                encoder_counts_per_rev,
                backend.tmc_config().pwm_freq_hz(),
                torque_limit,
            ) {
                Ok(plan) => plan,
                Err(_) => return fail_outer(phase, CommissionError::InvalidSchedule),
            };
            let encoder_raw = match backend.try_read_mechanical_encoder_position_counts().await {
                Ok(value) => value,
                Err(_) => return fail_outer(phase, CommissionError::Spi),
            };
            if safety
                .configure_encoder_resolution(encoder_counts_per_rev)
                .and_then(|_| safety.set_encoder_baseline(encoder_raw))
                .is_err()
                || safety
                    .configure_velocity_envelope(plan.used_velocity_mrev_s)
                    .is_err()
            {
                return fail_outer(phase, CommissionError::EncoderFault);
            }
            safety.torque_cap = torque_limit;
            if safety.arm_safety_observation(backend.elapsed_us()).is_err() {
                return fail_outer(phase, CommissionError::InvalidSchedule);
            }
            data.plan = Some(plan);
            *state = VelocitySweepState::EmitPlan;
            StepResult::ReadyNow
        }
        VelocitySweepState::EmitPlan => {
            *state = VelocitySweepState::WriteGain;
            StepResult::ReadyNow
        }
        VelocitySweepState::WriteGain => StepResult::ReadyNow,
    }
}

#[cfg(test)]
mod tests {
    use crate::commission_dispatch::test_support::{MockBackend, SpiOp, current_limit};
    use crate::commissioning::outer::velocity::sweep_plan::RequestedVelocitySource;
    use crate::commissioning::outer::{OuterMotionBudget, OuterSafetyLimits, VelocitySweepData};

    use super::*;

    fn phase() -> OuterPhase {
        let budget = OuterMotionBudget {
            requested_velocity_mrev_s: 6_000,
            machine_velocity_ceiling_mrev_s: 10_000,
            requested_velocity_source: RequestedVelocitySource::Defaulted,
            max_stroke_travel_mrev: 2_000,
            settle_travel_reserve_mrev: 100,
            negative_position_headroom_mrev: 2_000,
            positive_position_headroom_mrev: 2_000,
            max_duration_ms: 60_000,
        };
        OuterPhase::VelocitySweep {
            state: VelocitySweepState::Setup,
            data: VelocitySweepData::new(),
            safety: OuterSafetyLimits::new(0, budget),
        }
    }

    #[test]
    fn plan_is_frozen_and_exposed_before_any_motion_write() {
        let mut phase = phase();
        let mut backend = MockBackend::new();
        backend.current_limit_value = current_limit(2_448);

        assert!(matches!(
            pollster::block_on(step_velocity_sweep_with_trace(&mut phase, &mut backend)),
            StepResult::ReadyNow
        ));
        let OuterPhase::VelocitySweep { state, data, .. } = &phase else {
            panic!("velocity sweep should remain active");
        };
        assert!(matches!(state, VelocitySweepState::EmitPlan));
        assert!(data.plan.is_some());
        assert!(!backend.spi_log.iter().any(|op| {
            matches!(op, SpiOp::Write(address, value)
                if *address == foci_tmc4671::registers::PID_VELOCITY_TARGET.addr()
                    && *value != 0)
        }));

        assert!(matches!(
            pollster::block_on(step_velocity_sweep_with_trace(&mut phase, &mut backend)),
            StepResult::ReadyNow
        ));
        assert!(matches!(
            phase,
            OuterPhase::VelocitySweep {
                state: VelocitySweepState::WriteGain,
                ..
            }
        ));
    }

    #[test]
    fn invalid_plan_fails_before_motion() {
        let mut phase = phase();
        let mut backend = MockBackend::new();

        assert!(matches!(
            pollster::block_on(step_velocity_sweep_with_trace(&mut phase, &mut backend)),
            StepResult::OuterFailed(CommissionError::InvalidSchedule)
        ));
        assert!(
            !backend
                .spi_log
                .iter()
                .any(|op| matches!(op, SpiOp::Write(_, _)))
        );
    }
}
