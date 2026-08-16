use super::acceptance::{
    AcceptanceMatrixPreflightRejection, AutotuneAction, apply_acceptance_matrix_terminal,
    preflight_acceptance_matrix, reconstruct_breakaway_plan,
};
use super::stage_c::StageCPreflightConfig;
use crate::autotune::velocity_measurement::VelocityDirection;
use crate::commissioning::outer::OuterMotionBudget;
use crate::commissioning::outer::velocity::VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT;
use crate::commissioning::outer::velocity::acceptance_plan::AcceptanceMatrixOrder;
use crate::commissioning::outer::velocity::band_confirmation::{
    BandConfirmationPlan, ConfirmationDirectionalInput, DiscoveryObservationAtom,
    evaluate_band_confirmation, plan_band_confirmation,
};
use crate::commissioning::outer::velocity::combined_plan::BREAKAWAY_STAGE_C_SCHEMA_REVISION;
use crate::commissioning::outer::velocity::integral_plan::IntegralSlotOrder;
use crate::commissioning::outer::velocity::integral_plan::{
    AcquisitionContext, CommissioningWorkflowPlan, ConfirmedAcquisitionDigest, StageCPlanShape,
    VelocityIntegralPlan, bind_breakaway_stage_c_authority,
    plan_breakaway_velocity_integral_response,
};
use crate::commissioning::outer::velocity::integral_reproduction::{
    AcceptanceMatrixProgress, AcceptancePoint, StageCCandidateSlot, StageCPlanIdentity,
};
use crate::commissioning::outer::velocity::reproduction::BUILD_FEATURES;
use crate::commissioning::outer::velocity::sweep_consensus::{
    CONSENSUS_OBSERVATIONS_PER_DIRECTION, DirectionalConsensus, DirectionalConsensusClass,
};
use crate::commissioning::outer::velocity::sweep_evidence::VelocityObservationClass;
use crate::commissioning::outer::velocity::sweep_plan::{
    RequestedVelocitySource, VelocitySweepRequest, current_gain_span, plan_stage_b_velocity_sweep,
};
use crate::commissioning::types::{
    AcceptanceMatrixCause, AcceptanceMatrixOutcome, AcceptanceMatrixTerminal,
};
use foci_tmc4671::pid::PidPGain;

const CHANNEL: u8 = 0;
const COUNTS_PER_REV: u32 = 4_000;
const PWM_HZ: u32 = 25_000;
const TORQUE_LIMIT: u16 = 2_816;

fn matrix_config() -> StageCPreflightConfig {
    StageCPreflightConfig {
        encoder_counts_per_rev: COUNTS_PER_REV,
        torque_limit: TORQUE_LIMIT,
        limit_evidence_capability: VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT,
        channel: CHANNEL,
        pwm_hz: PWM_HZ,
    }
}

// --- Breakaway matrix-authority fixtures ------------------------------------
//
// A schema-16 breakaway plan can only be minted through the real confirmation
// gate (`ConfirmedAcquisitionDigest`'s inner field is private outside
// `integral_plan.rs`), so these fixtures walk the same
// discovery-confirmation-token path `breakaway_campaign.rs` does, rather than
// fabricating a plan. `breakaway_request` freezes its own velocity (6_666
// mrev/s, distinct from `request()`'s 4_394) because gain 512 is only known
// transition-capable under that target.

fn breakaway_request() -> VelocitySweepRequest {
    VelocitySweepRequest {
        requested_velocity_mrev_s: 6_666,
        machine_velocity_ceiling_mrev_s: 10_000,
        requested_velocity_source: RequestedVelocitySource::Defaulted,
        max_stroke_travel_mrev: 2_500,
        settle_travel_reserve_mrev: 250,
        negative_position_headroom_mrev: 7_025,
        positive_position_headroom_mrev: 7_025,
        max_duration_ms: 3_000,
    }
}

fn full_directional_consensus() -> DirectionalConsensus {
    DirectionalConsensus {
        classification: DirectionalConsensusClass::EligibleConsensus,
        components: [[0, 0]; 2],
        component_count: 1,
        collected_mask: 0b1111,
        eligible_mask: 0b1111,
        included_mask: 0b1111,
    }
}

/// A held-out family-eight block that `evaluate_band_confirmation` accepts:
/// both directions fully collected, well inside the 70-80% response band.
fn accepted_confirmation_evidence() -> (ConfirmationDirectionalInput, ConfirmationDirectionalInput)
{
    let mean_rate_q: i64 = 750_000;
    let variance_of_mean_q2: u128 = 58_752_225;
    let target_rate_q: i64 = 1_000_000;
    let forward = ConfirmationDirectionalInput {
        consensus: full_directional_consensus(),
        classes: [VelocityObservationClass::Eligible; CONSENSUS_OBSERVATIONS_PER_DIRECTION],
        observations: [Some(DiscoveryObservationAtom {
            mean_rate_q,
            variance_of_mean_q2,
            target_rate_q,
            direction: VelocityDirection::Forward,
        }); CONSENSUS_OBSERVATIONS_PER_DIRECTION],
    };
    let reverse = ConfirmationDirectionalInput {
        consensus: full_directional_consensus(),
        classes: [VelocityObservationClass::Eligible; CONSENSUS_OBSERVATIONS_PER_DIRECTION],
        observations: [Some(DiscoveryObservationAtom {
            mean_rate_q: -mean_rate_q,
            variance_of_mean_q2,
            target_rate_q,
            direction: VelocityDirection::Reverse,
        }); CONSENSUS_OBSERVATIONS_PER_DIRECTION],
    };
    (forward, reverse)
}

fn breakaway_confirmation_plan(candidate_p: PidPGain) -> BandConfirmationPlan {
    plan_band_confirmation(0x1111_2222_3333_4444, candidate_p).unwrap()
}

/// Mint the confirmation token through the real gate, exactly as a live
/// campaign would, rather than fabricating the digest.
fn breakaway_token_for(confirmation_plan: BandConfirmationPlan) -> ConfirmedAcquisitionDigest {
    let (forward, reverse) = accepted_confirmation_evidence();
    let acceptance =
        evaluate_band_confirmation(confirmation_plan.candidate_p, forward, reverse).unwrap();
    bind_breakaway_stage_c_authority(
        confirmation_plan.plan_digest,
        confirmation_plan,
        Ok(acceptance),
    )
    .unwrap()
}

fn breakaway_intervals_for(confirmation_plan: BandConfirmationPlan) -> [[i32; 2]; 2] {
    let (forward, reverse) = accepted_confirmation_evidence();
    evaluate_band_confirmation(confirmation_plan.candidate_p, forward, reverse)
        .unwrap()
        .held_out_response_intervals_q
}

/// One genuine schema-16 breakaway plan, built through the real
/// discovery-confirmation-token path (see the module doc above) via the
/// CAMPAIGN builder (`plan_breakaway_velocity_integral_response`), plus the
/// token that authorizes it.
///
/// Deliberately independent of `reconstruct_breakaway_plan`'s own logic: this
/// fixture derives `usable_torque_limit` via `current_gain_span` the same way
/// `BreakawayCampaign::new` does, not via any helper `reconstruct_breakaway_plan`
/// shares, so a digest match against this fixture is a genuine check against
/// production rather than the reconstruction checking itself.
fn breakaway_plan_and_token() -> (VelocityIntegralPlan, ConfirmedAcquisitionDigest) {
    let request = breakaway_request();
    let stage_b = plan_stage_b_velocity_sweep(
        request,
        COUNTS_PER_REV,
        TORQUE_LIMIT,
        VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT,
    )
    .unwrap();
    let execution_plan = stage_b.execution_plan(stage_b.digest());
    let mut context = AcquisitionContext::from(execution_plan);
    context.channel = CHANNEL;
    context.plan_time_pwm_hz = PWM_HZ;
    context.plan_time_encoder_counts_per_rev = COUNTS_PER_REV;
    let (_, _, usable_torque_limit, _) =
        current_gain_span(execution_plan.target_velocity_rpm, TORQUE_LIMIT).unwrap();
    context.usable_torque_limit = usable_torque_limit;

    let confirmation_plan = breakaway_confirmation_plan(PidPGain::new(512).unwrap());
    let token = breakaway_token_for(confirmation_plan);
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
    let plan = plan_breakaway_velocity_integral_response(
        token,
        confirmation_plan,
        context,
        breakaway_intervals_for(confirmation_plan),
        motion_budget,
        PWM_HZ,
        COUNTS_PER_REV,
    )
    .expect("the campaign's own Stage-C plan");
    (plan, token)
}

fn breakaway_point(
    plan: VelocityIntegralPlan,
    token: ConfirmedAcquisitionDigest,
    progress: AcceptanceMatrixProgress,
) -> AcceptancePoint {
    AcceptancePoint {
        identity: StageCPlanIdentity {
            channel: CHANNEL,
            build_features: BUILD_FEATURES,
            schema_revision: plan.schema_revision,
            plan_digest: plan.digest(),
        },
        selected_p: plan.final_p,
        selected_i: plan.positive_rungs()[0],
        target_velocity_rpm: plan.target_velocity_rpm,
        selected_response_intervals_q: plan.authority.directional_pooled_intervals,
        qualifier_bits: 0x0123,
        token: Some(token),
        progress,
    }
}

/// The end-to-end proof the cutover depends on: a genuine breakaway point's
/// carried token, `selected_p`, and intervals rebuild the exact retained
/// plan, so `plan_digest` matches. If any reconstruction input were wrong or
/// missing this would fail, not merely warn.
#[test]
fn breakaway_reconstructs_and_verifies_from_token() {
    let (plan, token) = breakaway_plan_and_token();
    let point = breakaway_point(plan, token, AcceptanceMatrixProgress::ReadyAscending);

    let rebuilt = reconstruct_breakaway_plan(
        point,
        breakaway_request(),
        COUNTS_PER_REV,
        TORQUE_LIMIT,
        VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT,
        PWM_HZ,
    )
    .expect("a genuine breakaway point reconstructs");

    assert_eq!(rebuilt.digest(), point.identity.plan_digest);
    assert_eq!(rebuilt.schema_revision, BREAKAWAY_STAGE_C_SCHEMA_REVISION);
    assert_eq!(rebuilt.final_p, point.selected_p);
    assert_eq!(rebuilt.target_velocity_rpm, point.target_velocity_rpm);
}

/// Altering the carried intervals still lets the deterministic I-ladder
/// rebuild a plan, but a different one -- the digest check must catch the
/// divergence rather than accepting the tampered point.
#[test]
fn breakaway_reconstruct_rejects_tampered_intervals() {
    let (plan, token) = breakaway_plan_and_token();
    let mut point = breakaway_point(plan, token, AcceptanceMatrixProgress::ReadyAscending);
    point.selected_response_intervals_q[0][0] += 2;

    assert_eq!(
        reconstruct_breakaway_plan(
            point,
            breakaway_request(),
            COUNTS_PER_REV,
            TORQUE_LIMIT,
            VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT,
            PWM_HZ,
        ),
        Err(AcceptanceMatrixPreflightRejection::AcceptancePointIdentityMismatch)
    );
}

/// `selected_p` is folded into the confirmed gain used to rebuild the plan;
/// tampering it must also be refused, not silently rebuild a different plan.
#[test]
fn breakaway_reconstruct_rejects_tampered_selected_p() {
    let (plan, token) = breakaway_plan_and_token();
    let mut point = breakaway_point(plan, token, AcceptanceMatrixProgress::ReadyAscending);
    point.selected_p += 1;

    assert_eq!(
        reconstruct_breakaway_plan(
            point,
            breakaway_request(),
            COUNTS_PER_REV,
            TORQUE_LIMIT,
            VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT,
            PWM_HZ,
        ),
        Err(AcceptanceMatrixPreflightRejection::AcceptancePointIdentityMismatch)
    );
}

/// A point with no token (only ever the combined case) must be refused before
/// any reconstruction is attempted.
#[test]
fn breakaway_reconstruct_rejects_point_without_token() {
    let (plan, token) = breakaway_plan_and_token();
    let mut point = breakaway_point(plan, token, AcceptanceMatrixProgress::ReadyAscending);
    point.token = None;

    assert_eq!(
        reconstruct_breakaway_plan(
            point,
            breakaway_request(),
            COUNTS_PER_REV,
            TORQUE_LIMIT,
            VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT,
            PWM_HZ,
        ),
        Err(AcceptanceMatrixPreflightRejection::AcceptancePointIdentityMismatch)
    );
}

fn terminal(plan_digest: u64, outcome: AcceptanceMatrixOutcome) -> AcceptanceMatrixTerminal {
    AcceptanceMatrixTerminal {
        schema_revision:
            crate::commissioning::outer::velocity::acceptance_plan::ACCEPTANCE_MATRIX_SCHEMA_REVISION,
        run_sequence: 9,
        evidence_sequence: 1,
        acceptance_plan_digest: breakaway_plan_and_token().0.digest(),
        plan_digest,
        digest: 0x1234,
        outcome,
        cause: AcceptanceMatrixCause::None,
        attempted_masks: [0b1_1111; 2],
        eligible_masks: [0b1_1111; 2],
        current_terminus_masks: [0; 2],
        unattempted_masks: [0; 2],
        expected_observations: 40,
        emitted_observations: 40,
        expected_amplitudes: 5,
        emitted_amplitudes: 5,
        qualifier_bits: 0x0123,
        run_started_us: 1,
        run_completed_us: 2,
        safety_fault: None,
    }
}

fn rest_unconfirmed_terminal(plan_digest: u64) -> AcceptanceMatrixTerminal {
    let mut terminal = terminal(plan_digest, AcceptanceMatrixOutcome::Inconclusive);
    terminal.cause = AcceptanceMatrixCause::VelocityRestNotConfirmed;
    terminal
}

#[test]
fn action_codes_and_invalid_values_are_exact() {
    assert_eq!(AutotuneAction::MatrixAscending as u8, 1);
    assert_eq!(AutotuneAction::MatrixDescending as u8, 2);
    assert_eq!(AutotuneAction::BreakawaySeeded as u8, 7);
    assert_eq!(AutotuneAction::StageCResume as u8, 8);
    for byte in [0u8, 3, 4, 5, 6, 10] {
        assert_eq!(AutotuneAction::from_u8(byte), None);
    }
    assert_eq!(
        AutotuneAction::from_u8(1),
        Some(AutotuneAction::MatrixAscending)
    );
    assert_eq!(
        AutotuneAction::from_u8(2),
        Some(AutotuneAction::MatrixDescending)
    );
    assert_eq!(
        AutotuneAction::from_u8(7),
        Some(AutotuneAction::BreakawaySeeded)
    );
    assert_eq!(
        AutotuneAction::from_u8(8),
        Some(AutotuneAction::StageCResume)
    );
    assert_eq!(
        AutotuneAction::from_u8(9),
        Some(AutotuneAction::RobustnessReversal)
    );
    assert_eq!(AutotuneAction::RobustnessReversal as u8, 9);
    assert_eq!(
        AutotuneAction::RobustnessReversal.slot_order(),
        IntegralSlotOrder::ForwardFirst
    );
    assert_eq!(AutotuneAction::RobustnessReversal.matrix_order(), None);
    assert_eq!(AutotuneAction::RobustnessReversal.matrix_slot_order(), None);
    // The breakaway campaign runs its own schedule; it is neither a matrix
    // acquisition nor a classic forward/reverse slot order.
    assert_eq!(
        AutotuneAction::BreakawaySeeded.slot_order(),
        IntegralSlotOrder::ForwardFirst
    );
    assert_eq!(AutotuneAction::BreakawaySeeded.matrix_order(), None);
    assert_eq!(AutotuneAction::BreakawaySeeded.matrix_slot_order(), None);

    // Slot order is defined for every surviving acquisition.
    for (action, slot_order) in [
        (
            AutotuneAction::MatrixAscending,
            IntegralSlotOrder::ForwardFirst,
        ),
        (
            AutotuneAction::MatrixDescending,
            IntegralSlotOrder::ForwardFirst,
        ),
    ] {
        assert_eq!(action.slot_order(), slot_order, "{action:?}");
    }

    // Amplitude order and slot order are independent dimensions of one action.
    for (action, order, slot_order) in [
        (
            AutotuneAction::MatrixAscending,
            AcceptanceMatrixOrder::Ascending,
            IntegralSlotOrder::ForwardFirst,
        ),
        (
            AutotuneAction::MatrixDescending,
            AcceptanceMatrixOrder::Descending,
            IntegralSlotOrder::ForwardFirst,
        ),
    ] {
        assert_eq!(action.matrix_order(), Some(order), "{action:?} order");
        assert_eq!(
            action.matrix_slot_order(),
            Some(slot_order),
            "{action:?} slot order"
        );
    }
    assert_eq!(AutotuneAction::StageCResume.matrix_order(), None);
    assert_eq!(AutotuneAction::StageCResume.matrix_slot_order(), None);
    let ascending = CommissioningWorkflowPlan::new_acceptance_matrix(
        1,
        AcceptanceMatrixOrder::Ascending,
        63_041,
        66_456,
    )
    .unwrap();
    let descending = CommissioningWorkflowPlan::new_acceptance_matrix(
        2,
        AcceptanceMatrixOrder::Descending,
        63_041,
        66_456,
    )
    .unwrap();
    assert_eq!(ascending.shape, StageCPlanShape::AcceptanceMatrixAscending);
    assert_eq!(
        descending.shape,
        StageCPlanShape::AcceptanceMatrixDescending
    );
}

#[test]
fn preflight_requires_matching_point_and_order_without_mutation() {
    let (current, token) = breakaway_plan_and_token();
    let config = matrix_config();
    let mut empty = StageCCandidateSlot::new();
    assert_eq!(
        preflight_acceptance_matrix(
            AutotuneAction::MatrixAscending,
            breakaway_request(),
            &config,
            7,
            &empty,
        ),
        Err(AcceptanceMatrixPreflightRejection::MissingAcceptancePoint)
    );

    let mut stale = breakaway_point(current, token, AcceptanceMatrixProgress::ReadyAscending);
    stale.identity.plan_digest ^= 1;
    empty.install_acceptance_point(stale);
    assert_eq!(
        preflight_acceptance_matrix(
            AutotuneAction::MatrixAscending,
            breakaway_request(),
            &config,
            7,
            &empty,
        ),
        Err(AcceptanceMatrixPreflightRejection::AcceptancePointIdentityMismatch)
    );
    assert_eq!(empty.acceptance_point(), Some(stale));

    let ready_descending =
        breakaway_point(current, token, AcceptanceMatrixProgress::ReadyDescending);
    empty.install_acceptance_point(ready_descending);
    assert_eq!(
        preflight_acceptance_matrix(
            AutotuneAction::MatrixAscending,
            breakaway_request(),
            &config,
            7,
            &empty,
        ),
        Err(AcceptanceMatrixPreflightRejection::MatrixOrderMismatch)
    );
    assert_eq!(empty.acceptance_point(), Some(ready_descending));
}

#[test]
fn preflight_admits_only_the_expected_matrix_direction() {
    let (current, token) = breakaway_plan_and_token();
    let config = matrix_config();
    let mut slot = StageCCandidateSlot::new();
    slot.install_acceptance_point(breakaway_point(
        current,
        token,
        AcceptanceMatrixProgress::ReadyAscending,
    ));
    let ascending = preflight_acceptance_matrix(
        AutotuneAction::MatrixAscending,
        breakaway_request(),
        &config,
        7,
        &slot,
    )
    .unwrap();
    assert_eq!(ascending.plan.order, AcceptanceMatrixOrder::Ascending);

    slot.advance_matrix_progress();
    let descending = preflight_acceptance_matrix(
        AutotuneAction::MatrixDescending,
        breakaway_request(),
        &config,
        8,
        &slot,
    )
    .unwrap();
    assert_eq!(descending.plan.order, AcceptanceMatrixOrder::Descending);
}

#[test]
fn safe_terminals_advance_or_close_and_fault_clears() {
    let (current, token) = breakaway_plan_and_token();
    let config = matrix_config();
    let mut slot = StageCCandidateSlot::new();
    slot.install_acceptance_point(breakaway_point(
        current,
        token,
        AcceptanceMatrixProgress::ReadyAscending,
    ));
    let ascending = preflight_acceptance_matrix(
        AutotuneAction::MatrixAscending,
        breakaway_request(),
        &config,
        7,
        &slot,
    )
    .unwrap();
    apply_acceptance_matrix_terminal(
        terminal(ascending.plan.digest(), AcceptanceMatrixOutcome::Complete),
        AcceptanceMatrixOrder::Ascending,
        ascending.plan.digest(),
        &mut slot,
    );
    assert_eq!(
        slot.acceptance_point().unwrap().progress,
        AcceptanceMatrixProgress::ReadyDescending
    );

    let descending = preflight_acceptance_matrix(
        AutotuneAction::MatrixDescending,
        breakaway_request(),
        &config,
        8,
        &slot,
    )
    .unwrap();
    apply_acceptance_matrix_terminal(
        terminal(
            descending.plan.digest(),
            AcceptanceMatrixOutcome::Inconclusive,
        ),
        AcceptanceMatrixOrder::Descending,
        descending.plan.digest(),
        &mut slot,
    );
    assert_eq!(
        slot.acceptance_point().unwrap().progress,
        AcceptanceMatrixProgress::Closed
    );

    slot.install_acceptance_point(breakaway_point(
        current,
        token,
        AcceptanceMatrixProgress::ReadyAscending,
    ));
    apply_acceptance_matrix_terminal(
        terminal(ascending.plan.digest(), AcceptanceMatrixOutcome::Fault),
        AcceptanceMatrixOrder::Ascending,
        ascending.plan.digest(),
        &mut slot,
    );
    assert_eq!(slot.acceptance_point(), None);
}

#[test]
fn rest_unconfirmed_preserves_the_exact_point_and_matrix_order() {
    let (current, token) = breakaway_plan_and_token();
    let config = matrix_config();

    for (action, order, progress) in [
        (
            AutotuneAction::MatrixAscending,
            AcceptanceMatrixOrder::Ascending,
            AcceptanceMatrixProgress::ReadyAscending,
        ),
        (
            AutotuneAction::MatrixDescending,
            AcceptanceMatrixOrder::Descending,
            AcceptanceMatrixProgress::ReadyDescending,
        ),
    ] {
        let mut slot = StageCCandidateSlot::new();
        slot.install_acceptance_point(breakaway_point(current, token, progress));
        let before = slot.acceptance_point().unwrap();
        let preflight =
            preflight_acceptance_matrix(action, breakaway_request(), &config, 9, &slot).unwrap();

        let terminal = apply_acceptance_matrix_terminal(
            rest_unconfirmed_terminal(preflight.plan.digest()),
            order,
            preflight.plan.digest(),
            &mut slot,
        );

        assert_eq!(terminal.outcome, AcceptanceMatrixOutcome::Inconclusive);
        assert_eq!(
            terminal.cause,
            AcceptanceMatrixCause::VelocityRestNotConfirmed
        );
        assert_eq!(slot.acceptance_point(), Some(before));
        assert!(
            preflight_acceptance_matrix(action, breakaway_request(), &config, 10, &slot).is_ok(),
            "the same matrix order must remain admissible after a rest rejection"
        );
    }
}
