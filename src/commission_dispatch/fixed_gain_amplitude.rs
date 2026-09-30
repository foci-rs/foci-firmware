//! Firmware-owned admission and retained lifecycle for fixed-gain amplitude validation runs.

use foci_tmc4671::current_limit::PidTorqueFluxLimit;

use crate::commission_dispatch::integral::IntegralPreflightConfig;
use crate::commissioning::outer::OuterMotionBudget;
use crate::commissioning::outer::velocity::combined_plan::BREAKAWAY_INTEGRAL_SCHEMA_REVISION;
use crate::commissioning::outer::velocity::fixed_gain_amplitude_plan::{
    FixedGainAmplitudeOrder, FixedGainAmplitudePlan, FixedGainAmplitudePlanError,
    plan_fixed_gain_amplitude,
};
use crate::commissioning::outer::velocity::integral_plan::{
    AcquisitionContext, BreakawayIntegralEvidence, IntegralPlanPhysics, VelocityIntegralPlan,
    VelocityIntegralPlanError, plan_retained_breakaway_velocity_integral_response,
};
use crate::commissioning::outer::velocity::integral_reproduction::{
    AcceptancePoint, FixedGainAmplitudeProgress, IntegralRetentionSlot,
};
use crate::commissioning::outer::velocity::sweep_plan::{
    VelocitySweepRequest, plan_proportional_velocity_sweep,
};
use crate::commissioning::outer::velocity::velocity_clamp::derive_probe_limit;
use crate::commissioning::types::{
    FixedGainAmplitudeCause, FixedGainAmplitudeOutcome, FixedGainAmplitudeTerminal,
};

pub use crate::tmc::AutotuneAction;

use crate::commissioning::outer::velocity::integral_plan::IntegralSlotOrder;

impl AutotuneAction {
    /// Return the immutable amplitude order for an amplitude action.
    pub const fn amplitude_order(self) -> Option<FixedGainAmplitudeOrder> {
        match self {
            Self::BreakawaySeeded
            | Self::IntegralResume
            | Self::RobustnessReversal
            | Self::PositionTune => None,
            Self::FixedGainAmplitudeAscending => Some(FixedGainAmplitudeOrder::Ascending),
            Self::FixedGainAmplitudeDescending => Some(FixedGainAmplitudeOrder::Descending),
        }
    }

    /// Return the immutable observation slot order for an amplitude action.
    pub const fn amplitude_slot_order(self) -> Option<IntegralSlotOrder> {
        match self {
            Self::BreakawaySeeded
            | Self::IntegralResume
            | Self::RobustnessReversal
            | Self::PositionTune => None,
            Self::FixedGainAmplitudeAscending | Self::FixedGainAmplitudeDescending => {
                Some(IntegralSlotOrder::ForwardFirst)
            }
        }
    }
}

/// Exact amplitude plan accepted before any SPI write or enable assertion.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FixedGainAmplitudePreflight {
    pub plan: FixedGainAmplitudePlan,
    pub run_sequence: u32,
}

/// Definitive zero-motion amplitude admission rejection.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum FixedGainAmplitudePreflightRejection {
    MissingAcceptancePoint,
    AcceptancePointIdentityMismatch,
    AmplitudeOrderMismatch,
    InvalidPlan,
}

/// Reconstruct the breakaway plan whose confirmed token installed the retained point.
///
/// Rejects points without a token and any digest, schema, final P, or target velocity
/// divergence from the retained identity.
pub fn reconstruct_breakaway_plan(
    point: AcceptancePoint,
    request: VelocitySweepRequest,
    encoder_counts_per_rev: u32,
    torque_limit: u16,
    limit_evidence_capability: u32,
    pwm_hz: u32,
) -> Result<VelocityIntegralPlan, FixedGainAmplitudePreflightRejection> {
    let token = point
        .token
        .ok_or(FixedGainAmplitudePreflightRejection::AcceptancePointIdentityMismatch)?;
    let proportional = plan_proportional_velocity_sweep(
        request,
        encoder_counts_per_rev,
        torque_limit,
        limit_evidence_capability,
    )
    .map_err(|_| FixedGainAmplitudePreflightRejection::InvalidPlan)?;
    let mut context = AcquisitionContext::from(proportional.execution_plan(proportional.digest()));
    context.channel = point.identity.channel;
    context.plan_time_pwm_hz = pwm_hz;
    context.plan_time_encoder_counts_per_rev = encoder_counts_per_rev;
    let operational_limit = PidTorqueFluxLimit::new(u32::from(torque_limit))
        .map_err(|_| FixedGainAmplitudePreflightRejection::InvalidPlan)?;
    context.usable_torque_limit = derive_probe_limit(operational_limit)
        .map_err(|_| FixedGainAmplitudePreflightRejection::InvalidPlan)?
        .raw();
    let motion_budget = OuterMotionBudget {
        requested_velocity_mrev_s: request.requested_velocity_mrev_s,
        machine_velocity_ceiling_mrev_s: request.machine_velocity_ceiling_mrev_s,
        requested_velocity_source: request.requested_velocity_source,
        max_stroke_travel_mrev: request.max_stroke_travel_mrev,
        settle_travel_reserve_mrev: request.settle_travel_reserve_mrev,
        negative_position_headroom_mrev: request.negative_position_headroom_mrev,
        positive_position_headroom_mrev: request.positive_position_headroom_mrev,
        max_duration_ms: request.max_duration_ms,
        homing_speed_mrev_s: 0,
        max_accel_mrev_s2: 0,
    };
    let current = plan_retained_breakaway_velocity_integral_response(
        token,
        point.selected_p,
        context,
        BreakawayIntegralEvidence {
            held_out_response_intervals_q: point.selected_response_intervals_q,
            confidence_q: point.confidence_q,
            band_position_q: point.band_position_q,
        },
        IntegralPlanPhysics {
            motion_budget,
            pwm_hz,
            encoder_counts_per_rev,
        },
    )
    .map_err(|error| match error {
        VelocityIntegralPlanError::Arithmetic
        | VelocityIntegralPlanError::InvalidRequest
        | VelocityIntegralPlanError::ConfidenceFamilyTooLarge
        | VelocityIntegralPlanError::InsufficientStructuralSpan => {
            FixedGainAmplitudePreflightRejection::InvalidPlan
        }
        _ => FixedGainAmplitudePreflightRejection::AcceptancePointIdentityMismatch,
    })?;
    if current.digest() != point.identity.plan_digest
        || current.schema_revision != BREAKAWAY_INTEGRAL_SCHEMA_REVISION
        || current.fixed_p != point.selected_p
        || current.target_velocity_rpm != point.target_velocity_rpm
    {
        return Err(FixedGainAmplitudePreflightRejection::AcceptancePointIdentityMismatch);
    }
    Ok(current)
}

/// Validate a retained point and derive one immutable amplitude plan without mutating retention.
pub fn preflight_fixed_gain_amplitude(
    action: AutotuneAction,
    request: VelocitySweepRequest,
    config: &IntegralPreflightConfig,
    run_sequence: u32,
    integral_candidate: &IntegralRetentionSlot,
) -> Result<FixedGainAmplitudePreflight, FixedGainAmplitudePreflightRejection> {
    let point = integral_candidate
        .acceptance_point()
        .ok_or(FixedGainAmplitudePreflightRejection::MissingAcceptancePoint)?;
    let order = action
        .amplitude_order()
        .ok_or(FixedGainAmplitudePreflightRejection::AmplitudeOrderMismatch)?;
    let slot_order = action
        .amplitude_slot_order()
        .ok_or(FixedGainAmplitudePreflightRejection::AmplitudeOrderMismatch)?;
    let expected_progress = match order {
        FixedGainAmplitudeOrder::Ascending => FixedGainAmplitudeProgress::ReadyAscending,
        FixedGainAmplitudeOrder::Descending => FixedGainAmplitudeProgress::ReadyDescending,
    };
    if point.progress != expected_progress {
        return Err(FixedGainAmplitudePreflightRejection::AmplitudeOrderMismatch);
    }
    let current_config = if point.identity.schema_revision == BREAKAWAY_INTEGRAL_SCHEMA_REVISION {
        reconstruct_breakaway_plan(
            point,
            request,
            config.encoder_counts_per_rev,
            config.torque_limit,
            config.limit_evidence_capability,
            config.pwm_hz,
        )?
    } else {
        return Err(FixedGainAmplitudePreflightRejection::AcceptancePointIdentityMismatch);
    };
    let plan = plan_fixed_gain_amplitude(point, request, order, slot_order, current_config)
        .map_err(|error| match error {
            FixedGainAmplitudePlanError::IdentityMismatch => {
                FixedGainAmplitudePreflightRejection::AcceptancePointIdentityMismatch
            }
            FixedGainAmplitudePlanError::UnsupportedTargets
            | FixedGainAmplitudePlanError::Arithmetic => {
                FixedGainAmplitudePreflightRejection::InvalidPlan
            }
        })?;
    Ok(FixedGainAmplitudePreflight { plan, run_sequence })
}

/// Apply one active amplitude terminal after disarm-first restoration.
pub fn apply_fixed_gain_amplitude_terminal(
    mut terminal: FixedGainAmplitudeTerminal,
    order: FixedGainAmplitudeOrder,
    expected_plan_digest: u64,
    integral_candidate: &mut IntegralRetentionSlot,
) -> FixedGainAmplitudeTerminal {
    let Some(point) = integral_candidate.acceptance_point() else {
        terminal.outcome = FixedGainAmplitudeOutcome::Fault;
        terminal.cause = FixedGainAmplitudeCause::MissingAcceptancePoint;
        return terminal;
    };
    let expected_progress = match order {
        FixedGainAmplitudeOrder::Ascending => FixedGainAmplitudeProgress::ReadyAscending,
        FixedGainAmplitudeOrder::Descending => FixedGainAmplitudeProgress::ReadyDescending,
    };
    if point.identity.plan_digest != terminal.acceptance_plan_digest
        || terminal.plan_digest != expected_plan_digest
    {
        integral_candidate.observe_fault();
        terminal.outcome = FixedGainAmplitudeOutcome::Fault;
        terminal.cause = FixedGainAmplitudeCause::AcceptancePointIdentityMismatch;
        return terminal;
    }
    if point.progress != expected_progress {
        integral_candidate.observe_fault();
        terminal.outcome = FixedGainAmplitudeOutcome::Fault;
        terminal.cause = FixedGainAmplitudeCause::AmplitudeOrderMismatch;
        return terminal;
    }

    let transition_ok = match terminal.outcome {
        FixedGainAmplitudeOutcome::Complete => match order {
            FixedGainAmplitudeOrder::Ascending => integral_candidate.advance_matrix_progress(),
            FixedGainAmplitudeOrder::Descending => integral_candidate.close_matrix_progress(),
        },
        FixedGainAmplitudeOutcome::Inconclusive
            if terminal.cause == FixedGainAmplitudeCause::VelocityRestNotConfirmed =>
        {
            true
        }
        FixedGainAmplitudeOutcome::Inconclusive => integral_candidate.close_matrix_progress(),
        FixedGainAmplitudeOutcome::Fault => {
            integral_candidate.observe_fault();
            true
        }
        FixedGainAmplitudeOutcome::Failed => true,
    };
    if !transition_ok {
        integral_candidate.observe_fault();
        terminal.outcome = FixedGainAmplitudeOutcome::Fault;
        terminal.cause = FixedGainAmplitudeCause::ProgressTransitionRejected;
    }
    terminal
}

/// Build one definitive zero-motion amplitude terminal from admission policy.
pub fn failed_fixed_gain_amplitude_terminal(
    run_sequence: u32,
    rejection: FixedGainAmplitudePreflightRejection,
    integral_candidate: &IntegralRetentionSlot,
) -> FixedGainAmplitudeTerminal {
    let cause = match rejection {
        FixedGainAmplitudePreflightRejection::MissingAcceptancePoint => {
            FixedGainAmplitudeCause::MissingAcceptancePoint
        }
        FixedGainAmplitudePreflightRejection::AcceptancePointIdentityMismatch => {
            FixedGainAmplitudeCause::AcceptancePointIdentityMismatch
        }
        FixedGainAmplitudePreflightRejection::AmplitudeOrderMismatch => {
            FixedGainAmplitudeCause::AmplitudeOrderMismatch
        }
        FixedGainAmplitudePreflightRejection::InvalidPlan => {
            FixedGainAmplitudeCause::EvidenceIntegrity
        }
    };
    FixedGainAmplitudeTerminal {
        schema_revision:
            crate::commissioning::outer::velocity::fixed_gain_amplitude_plan::FIXED_GAIN_AMPLITUDE_SCHEMA_REVISION,
        run_sequence,
        evidence_sequence: 0,
        acceptance_plan_digest: integral_candidate
            .acceptance_point()
            .map_or(0, |point| point.identity.plan_digest),
        plan_digest: 0,
        digest: 0,
        outcome: FixedGainAmplitudeOutcome::Failed,
        cause,
        attempted_masks: [0; 2],
        eligible_masks: [0; 2],
        current_terminus_masks: [0; 2],
        unattempted_masks: [0; 2],
        expected_observations: 0,
        emitted_observations: 0,
        expected_amplitudes: 0,
        emitted_amplitudes: 0,
        qualifier_bits: integral_candidate
            .acceptance_point()
            .map_or(0, |point| point.qualifier_bits),
        run_started_us: 0,
        run_completed_us: 0,
        safety_fault: None,
    }
}
