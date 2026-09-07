//! Firmware-owned admission and retained lifecycle for fixed-gain amplitude validation runs.

use foci_tmc4671::current_limit::PidTorqueFluxLimit;

use crate::commission_dispatch::stage_c::StageCPreflightConfig;
use crate::commissioning::outer::OuterMotionBudget;
use crate::commissioning::outer::velocity::combined_plan::BREAKAWAY_STAGE_C_SCHEMA_REVISION;
use crate::commissioning::outer::velocity::fixed_gain_amplitude_plan::{
    FixedGainAmplitudeOrder, FixedGainAmplitudePlan, FixedGainAmplitudePlanError,
    plan_fixed_gain_amplitude,
};
use crate::commissioning::outer::velocity::integral_plan::{
    AcquisitionContext, VelocityIntegralPlan, VelocityIntegralPlanError,
    plan_retained_breakaway_velocity_integral_response,
};
use crate::commissioning::outer::velocity::integral_reproduction::{
    AcceptanceMatrixProgress, AcceptancePoint, StageCCandidateSlot,
};
use crate::commissioning::outer::velocity::sweep_plan::{
    VelocitySweepRequest, plan_stage_b_velocity_sweep,
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
            Self::BreakawaySeeded | Self::StageCResume | Self::RobustnessReversal => None,
            Self::FixedGainAmplitudeAscending => Some(FixedGainAmplitudeOrder::Ascending),
            Self::FixedGainAmplitudeDescending => Some(FixedGainAmplitudeOrder::Descending),
        }
    }

    /// Return the immutable observation slot order for an amplitude action.
    ///
    /// Amplitude order and slot order are independent dimensions that share one
    /// wire byte; this keeps them separate in the type system.
    pub const fn amplitude_slot_order(self) -> Option<IntegralSlotOrder> {
        match self {
            Self::BreakawaySeeded | Self::StageCResume | Self::RobustnessReversal => None,
            Self::FixedGainAmplitudeAscending | Self::FixedGainAmplitudeDescending => {
                Some(IntegralSlotOrder::ForwardFirst)
            }
        }
    }
}

/// Exact amplitude plan accepted before any SPI write or enable assertion.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FixedGainAmplitudePreflight {
    /// Sole immutable amplitude plan used for disclosure and execution.
    pub plan: FixedGainAmplitudePlan,
    /// Caller-assigned trace sequence.
    pub run_sequence: u32,
}

/// Definitive zero-motion amplitude admission rejection.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum FixedGainAmplitudePreflightRejection {
    /// No target-reaching combined operating point is retained.
    MissingAcceptancePoint,
    /// Current controller geometry does not reconstruct the retained identity.
    AcceptancePointIdentityMismatch,
    /// The action does not match the retained two-run progress.
    AmplitudeOrderMismatch,
    /// Exact plan arithmetic or geometry was invalid.
    InvalidPlan,
}

/// Reconstruct the breakaway plan whose confirmed token installed the retained point.
///
/// Breakaway carries no retained combined-selected-response plan, and its
/// I-ladder is deterministic (PWM frequency plus fixed constants), so the
/// plan is rebuilt directly from the point's carried `token`, `selected_p`,
/// and `selected_response_intervals_q` via
/// [`plan_retained_breakaway_velocity_integral_response`] -- the same builder
/// the Stage-C resume path already uses -- rather than from any retained full
/// plan. `point.token` is `Some` only for a genuine breakaway acceptance
/// point (see [`AcceptancePoint::token`]), so a point carrying none is
/// refused before any reconstruction is attempted. Digest, schema, final P,
/// and target velocity are all re-checked against the retained identity
/// before the reconstruction is trusted, with any divergence rejected.
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
    // Breakaway has no Stage-B phase of its own; the campaign that produced
    // this point derived its planned velocity, target RPM, hard torque limit,
    // and limit-evidence capability from exactly this same
    // `plan_stage_b_velocity_sweep` reconstruction
    // (`breakaway_campaign::plan_breakaway_campaign_command`), so this
    // reproduces those fields identically. The plan-time physical context
    // guard inside the breakaway builder compares against the live
    // `pwm_hz`/`encoder_counts_per_rev` passed below; there is no separately
    // retained plan-time value here, so any live drift is instead caught by
    // the digest check at the end of this function.
    let stage_b = plan_stage_b_velocity_sweep(
        request,
        encoder_counts_per_rev,
        torque_limit,
        limit_evidence_capability,
    )
    .map_err(|_| FixedGainAmplitudePreflightRejection::InvalidPlan)?;
    let mut context = AcquisitionContext::from(stage_b.execution_plan(stage_b.digest()));
    context.channel = point.identity.channel;
    context.plan_time_pwm_hz = pwm_hz;
    context.plan_time_encoder_counts_per_rev = encoder_counts_per_rev;
    // `usable_torque_limit` is the one field the generic Stage-B sweep gets
    // wrong for this path: `plan_stage_b_velocity_sweep` plans with
    // `use_operational_limit = true` and so stamps the UNSCALED hard limit.
    // The campaign never uses that value for its gain span -- it derives
    // `usable_torque_limit` from `current_gain_span`, which applies the
    // deterministic nine-tenths probe headroom via `derive_probe_limit`
    // (`sweep_plan::current_gain_span` -> `velocity_clamp::derive_probe_limit`).
    // That headroom depends only on `hard_torque_limit`, not on the target
    // velocity or any live measurement, so it reproduces exactly here.
    // `VelocityIntegralPlan::digest()` hashes `usable_torque_limit`, so
    // leaving the Stage-B sweep's stamped value in place would make a
    // genuine breakaway point's digest never match.
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
    };
    let current = plan_retained_breakaway_velocity_integral_response(
        token,
        point.selected_p,
        context,
        point.selected_response_intervals_q,
        point.confidence_q,
        point.band_position_q,
        motion_budget,
        pwm_hz,
        encoder_counts_per_rev,
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
        || current.schema_revision != BREAKAWAY_STAGE_C_SCHEMA_REVISION
        || current.final_p != point.selected_p
        || current.target_velocity_rpm != point.target_velocity_rpm
    {
        return Err(FixedGainAmplitudePreflightRejection::AcceptancePointIdentityMismatch);
    }
    Ok(current)
}

/// Validate a retained point and derive one immutable amplitude plan without mutating retention.
///
/// Reconstructs the plan the retained point identifies before checking it:
/// breakaway points rebuild theirs deterministically from the carried
/// confirmation token via [`reconstruct_breakaway_plan`]; every other
/// identity is rejected. No full plan is retained -- only the compact
/// [`AcceptancePoint`] the caller reads from `stage_c_candidate`.
pub fn preflight_fixed_gain_amplitude(
    action: AutotuneAction,
    request: VelocitySweepRequest,
    config: &StageCPreflightConfig,
    run_sequence: u32,
    stage_c_candidate: &StageCCandidateSlot,
) -> Result<FixedGainAmplitudePreflight, FixedGainAmplitudePreflightRejection> {
    let point = stage_c_candidate
        .acceptance_point()
        .ok_or(FixedGainAmplitudePreflightRejection::MissingAcceptancePoint)?;
    let order = action
        .amplitude_order()
        .ok_or(FixedGainAmplitudePreflightRejection::AmplitudeOrderMismatch)?;
    let slot_order = action
        .amplitude_slot_order()
        .ok_or(FixedGainAmplitudePreflightRejection::AmplitudeOrderMismatch)?;
    let expected_progress = match order {
        FixedGainAmplitudeOrder::Ascending => AcceptanceMatrixProgress::ReadyAscending,
        FixedGainAmplitudeOrder::Descending => AcceptanceMatrixProgress::ReadyDescending,
    };
    if point.progress != expected_progress {
        return Err(FixedGainAmplitudePreflightRejection::AmplitudeOrderMismatch);
    }
    let current_config = if point.identity.schema_revision == BREAKAWAY_STAGE_C_SCHEMA_REVISION {
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
    stage_c_candidate: &mut StageCCandidateSlot,
) -> FixedGainAmplitudeTerminal {
    let Some(point) = stage_c_candidate.acceptance_point() else {
        terminal.outcome = FixedGainAmplitudeOutcome::Fault;
        terminal.cause = FixedGainAmplitudeCause::EvidenceIntegrity;
        return terminal;
    };
    let expected_progress = match order {
        FixedGainAmplitudeOrder::Ascending => AcceptanceMatrixProgress::ReadyAscending,
        FixedGainAmplitudeOrder::Descending => AcceptanceMatrixProgress::ReadyDescending,
    };
    if point.identity.plan_digest != terminal.acceptance_plan_digest
        || terminal.plan_digest != expected_plan_digest
        || point.progress != expected_progress
    {
        stage_c_candidate.observe_fault();
        terminal.outcome = FixedGainAmplitudeOutcome::Fault;
        terminal.cause = FixedGainAmplitudeCause::EvidenceIntegrity;
        return terminal;
    }

    let transition_ok = match terminal.outcome {
        FixedGainAmplitudeOutcome::Complete => match order {
            FixedGainAmplitudeOrder::Ascending => stage_c_candidate.advance_matrix_progress(),
            FixedGainAmplitudeOrder::Descending => stage_c_candidate.close_matrix_progress(),
        },
        FixedGainAmplitudeOutcome::Inconclusive
            if terminal.cause == FixedGainAmplitudeCause::VelocityRestNotConfirmed =>
        {
            true
        }
        FixedGainAmplitudeOutcome::Inconclusive => stage_c_candidate.close_matrix_progress(),
        FixedGainAmplitudeOutcome::Fault => {
            stage_c_candidate.observe_fault();
            true
        }
        FixedGainAmplitudeOutcome::Failed => true,
    };
    if !transition_ok {
        stage_c_candidate.observe_fault();
        terminal.outcome = FixedGainAmplitudeOutcome::Fault;
        terminal.cause = FixedGainAmplitudeCause::EvidenceIntegrity;
    }
    terminal
}

/// Build one definitive zero-motion amplitude terminal from admission policy.
pub fn failed_fixed_gain_amplitude_terminal(
    run_sequence: u32,
    rejection: FixedGainAmplitudePreflightRejection,
    stage_c_candidate: &StageCCandidateSlot,
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
        acceptance_plan_digest: stage_c_candidate
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
        qualifier_bits: stage_c_candidate
            .acceptance_point()
            .map_or(0, |point| point.qualifier_bits),
        run_started_us: 0,
        run_completed_us: 0,
        safety_fault: None,
    }
}

#[cfg(test)]
pub(crate) fn test_acceptance_point(
    request: VelocitySweepRequest,
    progress: AcceptanceMatrixProgress,
) -> crate::commissioning::outer::velocity::integral_reproduction::AcceptancePoint {
    use crate::autotune::velocity_measurement::VelocityDirection;
    use crate::commissioning::outer::velocity::band_confirmation::{
        ConfirmationDirectionalInput, DiscoveryObservationAtom, evaluate_band_confirmation,
        plan_band_confirmation,
    };
    use crate::commissioning::outer::velocity::integral_plan::bind_breakaway_stage_c_authority;
    use crate::commissioning::outer::velocity::reproduction::BUILD_FEATURES;
    use crate::commissioning::outer::velocity::sweep_consensus::{
        CONSENSUS_OBSERVATIONS_PER_DIRECTION, DirectionalConsensus, DirectionalConsensusClass,
    };
    use crate::commissioning::outer::velocity::sweep_evidence::VelocityObservationClass;
    use foci_tmc4671::pid::PidPGain;

    const ENCODER_COUNTS_PER_REV: u32 = 4_000;
    const TORQUE_LIMIT: u16 = 2_816;
    const PWM_HZ: u32 = 25_000;

    let stage_b = plan_stage_b_velocity_sweep(
        request,
        ENCODER_COUNTS_PER_REV,
        TORQUE_LIMIT,
        crate::commissioning::outer::velocity::VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT,
    )
    .unwrap();
    let mut context = AcquisitionContext::from(stage_b.execution_plan(stage_b.digest()));
    context.channel = 0;
    context.plan_time_pwm_hz = PWM_HZ;
    context.plan_time_encoder_counts_per_rev = ENCODER_COUNTS_PER_REV;
    // Mirrors `reconstruct_breakaway_plan`'s own derivation exactly: the
    // deterministic nine-tenths probe headroom depends only on the hard
    // torque limit, not on the target velocity or any live measurement.
    let operational_limit = PidTorqueFluxLimit::new(u32::from(TORQUE_LIMIT)).unwrap();
    context.usable_torque_limit = derive_probe_limit(operational_limit).unwrap().raw();

    let confirmation_plan =
        plan_band_confirmation(0x1111_2222_3333_4444, PidPGain::new(512).unwrap()).unwrap();
    let consensus = DirectionalConsensus {
        classification: DirectionalConsensusClass::EligibleConsensus,
        components: [[0, 0]; 2],
        component_count: 1,
        collected_mask: 0b1111,
        eligible_mask: 0b1111,
        included_mask: 0b1111,
    };
    let mean_rate_q: i64 = 750_000;
    let variance_of_mean_q2: u128 = 58_752_225;
    let target_rate_q: i64 = 1_000_000;
    let forward = ConfirmationDirectionalInput {
        consensus,
        classes: [VelocityObservationClass::Eligible; CONSENSUS_OBSERVATIONS_PER_DIRECTION],
        observations: [Some(DiscoveryObservationAtom {
            mean_rate_q,
            variance_of_mean_q2,
            target_rate_q,
            direction: VelocityDirection::Forward,
        }); CONSENSUS_OBSERVATIONS_PER_DIRECTION],
    };
    let reverse = ConfirmationDirectionalInput {
        consensus,
        classes: [VelocityObservationClass::Eligible; CONSENSUS_OBSERVATIONS_PER_DIRECTION],
        observations: [Some(DiscoveryObservationAtom {
            mean_rate_q: -mean_rate_q,
            variance_of_mean_q2,
            target_rate_q,
            direction: VelocityDirection::Reverse,
        }); CONSENSUS_OBSERVATIONS_PER_DIRECTION],
    };
    let acceptance =
        evaluate_band_confirmation(confirmation_plan.candidate_p, forward, reverse).unwrap();
    let held_out_response_intervals_q = acceptance.held_out_response_intervals_q;
    let token = bind_breakaway_stage_c_authority(
        confirmation_plan.plan_digest,
        confirmation_plan,
        Ok(acceptance),
    )
    .unwrap();

    let motion_budget = OuterMotionBudget {
        requested_velocity_mrev_s: request.requested_velocity_mrev_s,
        machine_velocity_ceiling_mrev_s: request.machine_velocity_ceiling_mrev_s,
        requested_velocity_source: request.requested_velocity_source,
        max_stroke_travel_mrev: request.max_stroke_travel_mrev,
        settle_travel_reserve_mrev: request.settle_travel_reserve_mrev,
        negative_position_headroom_mrev: request.negative_position_headroom_mrev,
        positive_position_headroom_mrev: request.positive_position_headroom_mrev,
        max_duration_ms: request.max_duration_ms,
    };
    let plan = plan_retained_breakaway_velocity_integral_response(
        token,
        confirmation_plan.candidate_p.raw(),
        context,
        held_out_response_intervals_q,
        acceptance.confidence_q,
        acceptance.band_position_q,
        motion_budget,
        PWM_HZ,
        ENCODER_COUNTS_PER_REV,
    )
    .unwrap();

    crate::commissioning::outer::velocity::integral_reproduction::AcceptancePoint {
        identity:
            crate::commissioning::outer::velocity::integral_reproduction::StageCPlanIdentity {
                channel: 0,
                build_features: BUILD_FEATURES,
                schema_revision: plan.schema_revision,
                plan_digest: plan.digest(),
            },
        selected_p: plan.final_p,
        selected_i: plan.positive_rungs()[0],
        target_velocity_rpm: plan.target_velocity_rpm,
        selected_response_intervals_q: plan.authority.directional_pooled_intervals,
        confidence_q: plan.authority.confidence_q,
        band_position_q: plan.authority.band_position_q,
        qualifier_bits: 0,
        token: Some(token),
        progress,
    }
}
