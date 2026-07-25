//! Cooperative state machine for rung-origin recovery.

use crate::autotune::CommissionError;
use crate::commissioning::backend::CommissioningBackend;
use crate::commissioning::outer::check_outer_safety;
use crate::commissioning::types::{OUTER_SAFETY_FAULT_CURRENT, OuterSafetyFault};
use foci_tmc4671::pid::{PidGainPair, PidIGain};
use foci_tmc4671::registers::{PID_VELOCITY_P_VELOCITY_I, PID_VELOCITY_TARGET};

use super::super::current_guard::{
    VelocityCurrentContact, VelocityCurrentGuardError, VelocityCurrentZeroPhase,
    finish_velocity_current_moving_phase, finish_velocity_current_zero_phase,
    read_velocity_pid_input_abs, read_velocity_pid_output_abs, restart_velocity_current_primitive,
};
use super::super::sweep::clear_velocity_integrator;
use super::super::sweep_analysis::poll_settle_confirmation_parts;
use super::super::sweep_plan::{SETTLE_ANALYSIS_BUDGET_US, ZERO_SETTLE_US};
use super::{
    ExactU64Parts, RecoveryBindingSource, RecoveryState, RungOriginRecoveryOutcome,
    RungOriginRecoveryResources, RungOriginRecoveryRuntime, RungOriginRecoveryStep,
};

/// Advance one bounded unit of shared rung-origin recovery work.
pub(crate) async fn step_rung_origin_recovery<B: CommissioningBackend>(
    runtime: &mut RungOriginRecoveryRuntime,
    resources: &mut RungOriginRecoveryResources<'_>,
    backend: &mut B,
) -> RungOriginRecoveryStep {
    if backend.is_shutdown_requested() {
        return recovery_fault(CommissionError::Shutdown, None);
    }
    match runtime.state {
        RecoveryState::Terminal {
            outcome,
            binding_source,
        } => {
            let report = runtime.report(
                outcome,
                binding_source,
                resources.safety.unwrapped_position,
                backend.elapsed_us(),
                resources.current_observer.peak_pid_output_abs(),
            );
            if matches!(
                outcome,
                RungOriginRecoveryOutcome::AlreadyCentered | RungOriginRecoveryOutcome::Recovered
            ) {
                RungOriginRecoveryStep::Finished(report)
            } else {
                RungOriginRecoveryStep::Contained(report)
            }
        }
        RecoveryState::WriteGains => {
            if runtime.start_offset_counts.unsigned_abs() > resources.safety.position_window {
                runtime.state = RecoveryState::Terminal {
                    outcome: RungOriginRecoveryOutcome::ContainedDeadline,
                    binding_source: RecoveryBindingSource::LocalTravel,
                };
                return RungOriginRecoveryStep::Contained(runtime.report(
                    RungOriginRecoveryOutcome::ContainedDeadline,
                    RecoveryBindingSource::LocalTravel,
                    resources.safety.unwrapped_position,
                    backend.elapsed_us(),
                    resources.current_observer.peak_pid_output_abs(),
                ));
            }
            if resources.safety.unwrapped_position != runtime.start_offset_counts {
                return recovery_fault(CommissionError::InvalidSchedule, None);
            }
            if backend
                .write_register(
                    PID_VELOCITY_P_VELOCITY_I,
                    PidGainPair::new(runtime.p, PidIGain::ZERO).register_value(),
                )
                .await
                .is_err()
            {
                return recovery_fault(CommissionError::Spi, None);
            }
            runtime.state = RecoveryState::ClearMovingIntegrator;
            RungOriginRecoveryStep::ReadyNow
        }
        RecoveryState::ClearMovingIntegrator => {
            if clear_velocity_integrator(backend).await.is_err() {
                return recovery_fault(CommissionError::Spi, None);
            }
            runtime.state = RecoveryState::CommandTowardOrigin;
            RungOriginRecoveryStep::ReadyNow
        }
        RecoveryState::CommandTowardOrigin => {
            let direction = if runtime.start_offset_counts > 0 {
                -1
            } else {
                1
            };
            let target = match runtime.target_velocity_rpm.with_direction(direction) {
                Ok(target) => target.raw(),
                Err(_) => return recovery_fault(CommissionError::InvalidSchedule, None),
            };
            let started_us = backend.elapsed_us();
            if resources.safety.begin_primitive(started_us).is_err() {
                return recovery_fault(CommissionError::InvalidSchedule, None);
            }
            if let Err(error) =
                restart_velocity_current_primitive(resources.current_observer, backend).await
            {
                resources.safety.end_primitive();
                return current_guard_fault(runtime, resources, backend, error);
            }
            if backend
                .write_register(PID_VELOCITY_TARGET, target as u32)
                .await
                .is_err()
            {
                resources.safety.end_primitive();
                return recovery_fault(CommissionError::Spi, None);
            }
            let moving_budget_us = runtime
                .runtime_us
                .checked_sub(ZERO_SETTLE_US)
                .and_then(|value| value.checked_sub(SETTLE_ANALYSIS_BUDGET_US));
            let Some(moving_deadline_us) =
                moving_budget_us.and_then(|value| started_us.checked_add(u64::from(value)))
            else {
                resources.safety.end_primitive();
                return recovery_fault(CommissionError::InvalidSchedule, None);
            };
            runtime.recovery_started_us = ExactU64Parts::from_u64(started_us);
            runtime.motion_started = true;
            runtime.state = RecoveryState::Observe {
                moving_deadline_us: ExactU64Parts::from_u64(moving_deadline_us),
            };
            RungOriginRecoveryStep::At(
                resources
                    .safety
                    .next_safety_deadline()
                    .unwrap_or(started_us),
            )
        }
        RecoveryState::Observe { moving_deadline_us } => {
            let moving_deadline_us = moving_deadline_us.to_u64();
            if let Some(step) = recovery_safety_sample(runtime, resources, backend, false).await {
                return step;
            }
            let offset = resources.safety.unwrapped_position;
            update_closest(runtime, offset);
            if wrong_way(runtime.start_offset_counts, offset) {
                let elapsed_us = backend
                    .elapsed_us()
                    .saturating_sub(runtime.recovery_started_us.to_u64());
                let fault = resources
                    .safety
                    .recovery_wrong_way_fault(runtime.start_offset_counts, elapsed_us);
                return recovery_fault(CommissionError::SafetyEnvelopeViolation, Some(fault));
            }
            let in_band = offset.unsigned_abs() <= runtime.plan.origin_band_counts;
            let deadline_reached = backend.elapsed_us() >= moving_deadline_us;
            if in_band || deadline_reached {
                let current_result =
                    finish_velocity_current_moving_phase(resources.current_observer, backend).await;
                let zero_result = backend.write_register(PID_VELOCITY_TARGET, 0).await;
                if zero_result.is_err() {
                    return recovery_fault(CommissionError::Spi, None);
                }
                if let Err(error) = current_result {
                    return current_guard_fault(runtime, resources, backend, error);
                }
                if recovery_current_contact(resources.current_observer) {
                    return recovery_current_headroom(runtime, resources, backend).await;
                }
                runtime.zero_commanded_us = ExactU64Parts::from_u64(backend.elapsed_us());
                runtime.state = RecoveryState::ClearSettleIntegrator {
                    settle_started_us: runtime.zero_commanded_us,
                    contained: deadline_reached && !in_band,
                };
                RungOriginRecoveryStep::ReadyNow
            } else {
                RungOriginRecoveryStep::At(
                    resources
                        .safety
                        .next_safety_deadline()
                        .unwrap_or(backend.elapsed_us()),
                )
            }
        }
        RecoveryState::ClearSettleIntegrator {
            settle_started_us,
            contained,
        } => {
            if clear_velocity_integrator(backend).await.is_err() {
                return recovery_fault(CommissionError::Spi, None);
            }
            let Some(deadline_us) = settle_started_us
                .to_u64()
                .checked_add(u64::from(ZERO_SETTLE_US))
            else {
                return recovery_fault(CommissionError::InvalidSchedule, None);
            };
            resources.observations.clear();
            resources.analysis.reset();
            *resources.settle_displacement_counts = 0;
            *resources.settle_min_displacement_counts = 0;
            *resources.settle_max_displacement_counts = 0;
            let next_sample_us = resources
                .safety
                .next_safety_deadline()
                .unwrap_or(backend.elapsed_us());
            runtime.state = RecoveryState::CaptureSettle {
                next_sample_us: ExactU64Parts::from_u64(next_sample_us),
                deadline_us: ExactU64Parts::from_u64(deadline_us),
                contained,
            };
            RungOriginRecoveryStep::At(next_sample_us)
        }
        RecoveryState::CaptureSettle {
            next_sample_us,
            deadline_us,
            contained,
        } => {
            let next_sample_us = next_sample_us.to_u64();
            let deadline_us = deadline_us.to_u64();
            if backend.elapsed_us() < next_sample_us {
                return RungOriginRecoveryStep::At(next_sample_us);
            }
            if let Some(step) = recovery_safety_sample(runtime, resources, backend, true).await {
                return step;
            }
            let offset = resources.safety.unwrapped_position;
            update_closest(runtime, offset);
            if wrong_way(runtime.start_offset_counts, offset) {
                let elapsed_us = backend
                    .elapsed_us()
                    .saturating_sub(runtime.recovery_started_us.to_u64());
                let fault = resources
                    .safety
                    .recovery_wrong_way_fault(runtime.start_offset_counts, elapsed_us);
                return recovery_fault(CommissionError::SafetyEnvelopeViolation, Some(fault));
            }
            let Some((delta, elapsed_us)) =
                resources
                    .safety
                    .last_accepted_observation()
                    .and_then(|(delta, elapsed_us)| {
                        u16::try_from(elapsed_us)
                            .ok()
                            .map(|elapsed_us| (delta, elapsed_us))
                    })
            else {
                return recovery_fault(CommissionError::InvalidSchedule, None);
            };
            *resources.settle_displacement_counts = i32::from(*resources.settle_displacement_counts)
                .saturating_add(delta)
                .clamp(-2, 2) as i8;
            *resources.settle_min_displacement_counts = (*resources.settle_min_displacement_counts)
                .min(*resources.settle_displacement_counts);
            *resources.settle_max_displacement_counts = (*resources.settle_max_displacement_counts)
                .max(*resources.settle_displacement_counts);
            if resources.observations.push(delta, elapsed_us).is_err() {
                return recovery_fault(CommissionError::InvalidSchedule, None);
            }
            let read_us = backend.elapsed_us();
            if read_us >= deadline_us {
                let Some(analysis_deadline_us) =
                    read_us.checked_add(u64::from(SETTLE_ANALYSIS_BUDGET_US))
                else {
                    return recovery_fault(CommissionError::InvalidSchedule, None);
                };
                resources.analysis.reset();
                runtime.state = RecoveryState::AnalyzeSettle {
                    analysis_deadline_us: ExactU64Parts::from_u64(analysis_deadline_us),
                    contained,
                };
                RungOriginRecoveryStep::ReadyNow
            } else {
                let next = resources.safety.next_safety_deadline().unwrap_or(read_us);
                runtime.state = RecoveryState::CaptureSettle {
                    next_sample_us: ExactU64Parts::from_u64(next),
                    deadline_us: ExactU64Parts::from_u64(deadline_us),
                    contained,
                };
                RungOriginRecoveryStep::At(next)
            }
        }
        RecoveryState::AnalyzeSettle {
            analysis_deadline_us,
            contained,
        } => {
            let analysis_deadline_us = analysis_deadline_us.to_u64();
            if backend.elapsed_us() >= analysis_deadline_us {
                return recovery_fault(CommissionError::AnalysisOverrun, None);
            }
            match poll_settle_confirmation_parts(
                resources.analysis,
                resources.observations,
                resources.blocker,
                *resources.settle_min_displacement_counts,
                *resources.settle_max_displacement_counts,
            ) {
                Ok(None) => return RungOriginRecoveryStep::ReadyNow,
                Ok(Some(result)) if !result.confirmed_rest => {
                    return recovery_fault(CommissionError::VelocityValidationFailed, None);
                }
                Err(error) => return recovery_fault(error, None),
                Ok(Some(_)) => {}
            }
            if let Err(error) =
                finish_velocity_current_zero_phase(resources.current_observer, backend).await
            {
                return current_guard_fault(runtime, resources, backend, error);
            }
            if recovery_current_contact(resources.current_observer) {
                return recovery_current_headroom(runtime, resources, backend).await;
            }
            resources.analysis.reset();
            resources.observations.clear();
            let final_offset = resources.safety.unwrapped_position;
            let in_band = final_offset.unsigned_abs() <= runtime.plan.origin_band_counts;
            let (outcome, binding_source) = if contained || !in_band {
                (
                    RungOriginRecoveryOutcome::ContainedDeadline,
                    RecoveryBindingSource::Deadline,
                )
            } else {
                resources.safety.end_primitive();
                (
                    RungOriginRecoveryOutcome::Recovered,
                    runtime.authority_source,
                )
            };
            runtime.state = RecoveryState::Terminal {
                outcome,
                binding_source,
            };
            let report = runtime.report(
                outcome,
                binding_source,
                final_offset,
                backend.elapsed_us(),
                resources.current_observer.peak_pid_output_abs(),
            );
            if outcome == RungOriginRecoveryOutcome::Recovered {
                RungOriginRecoveryStep::Finished(report)
            } else {
                RungOriginRecoveryStep::Contained(report)
            }
        }
    }
}

async fn recovery_safety_sample<B: CommissioningBackend>(
    runtime: &mut RungOriginRecoveryRuntime,
    resources: &mut RungOriginRecoveryResources<'_>,
    backend: &mut B,
    zero_phase: bool,
) -> Option<RungOriginRecoveryStep> {
    match check_outer_safety(resources.safety, backend).await {
        Ok(Some(fault)) => {
            return Some(recovery_fault(
                CommissionError::SafetyEnvelopeViolation,
                Some(fault),
            ));
        }
        Err(error) => return Some(recovery_fault(error, None)),
        Ok(None) => {}
    }
    let torque_demand = match read_velocity_pid_input_abs(backend).await {
        Ok(value) => value,
        Err(error) => return Some(current_guard_fault(runtime, resources, backend, error)),
    };
    if torque_demand > resources.current_observer.clamp_limit() {
        return Some(current_limit_fault(runtime, resources, backend));
    }
    let torque_target = match read_velocity_pid_output_abs(backend).await {
        Ok(value) => value,
        Err(error) => return Some(current_guard_fault(runtime, resources, backend, error)),
    };
    if zero_phase {
        resources
            .current_observer
            .observe_zero_output(torque_target);
    } else {
        resources
            .current_observer
            .observe_moving_output(torque_target);
    }
    if recovery_current_contact(resources.current_observer) {
        return Some(recovery_current_headroom(runtime, resources, backend).await);
    }
    None
}

async fn recovery_current_headroom<B: CommissioningBackend>(
    runtime: &mut RungOriginRecoveryRuntime,
    resources: &mut RungOriginRecoveryResources<'_>,
    backend: &mut B,
) -> RungOriginRecoveryStep {
    if backend
        .write_register(PID_VELOCITY_TARGET, 0)
        .await
        .is_err()
        || clear_velocity_integrator(backend).await.is_err()
    {
        return recovery_fault(CommissionError::Spi, None);
    }
    resources.safety.end_primitive();
    let outcome = RungOriginRecoveryOutcome::RecoveryCurrentHeadroom;
    let binding_source = RecoveryBindingSource::CurrentHeadroom;
    runtime.state = RecoveryState::Terminal {
        outcome,
        binding_source,
    };
    RungOriginRecoveryStep::Contained(runtime.report(
        outcome,
        binding_source,
        resources.safety.unwrapped_position,
        backend.elapsed_us(),
        resources.current_observer.peak_pid_output_abs(),
    ))
}

fn recovery_current_contact(
    observer: &super::super::current_guard::VelocityCurrentObserver,
) -> bool {
    observer
        .evidence(VelocityCurrentZeroPhase::Recovery)
        .map(|evidence| evidence.contact != VelocityCurrentContact::None)
        .unwrap_or(true)
}

fn current_guard_fault<B: CommissioningBackend>(
    runtime: &RungOriginRecoveryRuntime,
    resources: &mut RungOriginRecoveryResources<'_>,
    backend: &B,
    error: VelocityCurrentGuardError,
) -> RungOriginRecoveryStep {
    match error {
        VelocityCurrentGuardError::RegisterIo => recovery_fault(CommissionError::Spi, None),
        VelocityCurrentGuardError::InvalidLimit
        | VelocityCurrentGuardError::ActionableStatus(_) => {
            current_limit_fault(runtime, resources, backend)
        }
    }
}

fn current_limit_fault<B: CommissioningBackend>(
    runtime: &RungOriginRecoveryRuntime,
    resources: &mut RungOriginRecoveryResources<'_>,
    backend: &B,
) -> RungOriginRecoveryStep {
    let elapsed_us = backend
        .elapsed_us()
        .saturating_sub(runtime.recovery_started_us.to_u64());
    let fault = resources
        .safety
        .fault(OUTER_SAFETY_FAULT_CURRENT, 0, 0, 0, elapsed_us);
    recovery_fault(CommissionError::SafetyEnvelopeViolation, Some(fault))
}

fn update_closest(runtime: &mut RungOriginRecoveryRuntime, offset: i32) {
    if offset.unsigned_abs() < runtime.closest_offset_counts.unsigned_abs() {
        runtime.closest_offset_counts = offset;
    }
}

fn wrong_way(start_offset: i32, current_offset: i32) -> bool {
    current_offset.unsigned_abs() > start_offset.unsigned_abs().saturating_add(1)
}

fn recovery_fault(
    error: CommissionError,
    safety: Option<OuterSafetyFault>,
) -> RungOriginRecoveryStep {
    RungOriginRecoveryStep::Fault { error, safety }
}
