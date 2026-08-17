use super::current_guard::{
    VelocityCurrentContact, VelocityCurrentObserver, VelocityCurrentZeroPhase,
    VelocityLimitEvidenceCapability,
};
use super::recovery::{
    RecoveryBindingSource, RecoveryCurrentContactPolicy, RecoveryRestSchedule, RecoveryStage,
    RungOriginRecoveryConfig, RungOriginRecoveryOutcome, RungOriginRecoveryPlanError,
    RungOriginRecoveryResources, RungOriginRecoveryRuntime, RungOriginRecoveryStep,
    commanded_side_lower_rate_q, plan_rung_origin_recovery, recovery_moving_target_counts,
    recovery_return_distance_counts, recovery_runtime_us, recovery_runtime_us_with_schedule,
    step_rung_origin_recovery,
};
use super::sweep_plan::{
    MAX_SAFETY_OBSERVATION_GAP_US, RequestedVelocitySource, SETTLE_ANALYSIS_BUDGET_US,
    ZERO_SETTLE_US,
};
use crate::autotune::TmcMechanicalRpm;
use crate::autotune::velocity_measurement::{
    DyadicBlocker, RestSelectionDisposition, StationarityWork, VelocityDirection,
};
use crate::commission_dispatch::test_support::{MockBackend, SpiOp, current_limit};
use crate::commissioning::outer::check_outer_safety;
use crate::commissioning::outer::state::{VELOCITY_SWEEP_BLOCK_LEVELS, VelocityObservationRing};
use crate::commissioning::outer::{OuterMotionBudget, OuterSafetyLimits, OuterSafetyTimingPhase};
use crate::commissioning::types::OUTER_SAFETY_FAULT_RECOVERY_WRONG_WAY;
use foci_tmc4671::current_limit::PidTorqueFluxLimit;
use foci_tmc4671::pid::PidPGain;
use foci_tmc4671::registers::{PID_VELOCITY_P_VELOCITY_I, PID_VELOCITY_TARGET};

const RATE_Q: u128 = 1_000;

fn live_budget() -> OuterMotionBudget {
    OuterMotionBudget {
        requested_velocity_mrev_s: 4_394,
        machine_velocity_ceiling_mrev_s: 10_000,
        requested_velocity_source: RequestedVelocitySource::Defaulted,
        max_stroke_travel_mrev: 2_500,
        settle_travel_reserve_mrev: 250,
        negative_position_headroom_mrev: 7_025,
        positive_position_headroom_mrev: 7_025,
        max_duration_ms: 3_000,
    }
}

#[test]
fn live_geometry_derives_band_and_slots_from_the_budget() {
    let budget = live_budget();
    let primitive_cap_us = u32::from(budget.max_duration_ms) * 1_000;

    let plan = plan_rung_origin_recovery(budget, 4_000, 4_394).unwrap();

    assert_eq!(plan.origin_band_counts, 1_000);
    assert_eq!(plan.nominal_slot_us, 1_816_958);
    assert_eq!(plan.maximum_slot_us, primitive_cap_us);
}

#[test]
fn matrix_lowest_target_cannot_author_recovery_geometry() {
    let legacy_low_target_mrev_s = 16_u32 * 1_000 / 60;

    assert_eq!(legacy_low_target_mrev_s, 266);
    assert_eq!(
        plan_rung_origin_recovery(live_budget(), 4_000, legacy_low_target_mrev_s),
        Err(RungOriginRecoveryPlanError::ExceedsPrimitiveCap)
    );

    let corrected = plan_rung_origin_recovery(live_budget(), 4_000, 4_394).unwrap();
    assert_eq!(corrected.nominal_slot_us, 1_816_958);
    assert_eq!(corrected.maximum_slot_us, 3_000_000);
}

#[test]
fn recovered_with_current_headroom_code_is_append_only() {
    assert_eq!(RungOriginRecoveryOutcome::RecoveryCurrentHeadroom as u8, 4);
    assert_eq!(
        RungOriginRecoveryOutcome::RecoveredWithCurrentHeadroom as u8,
        5
    );
}

#[test]
fn return_distance_stops_at_the_inclusive_origin_band() {
    assert_eq!(recovery_return_distance_counts(8_458, 1_000), Some(7_458));
    assert_eq!(recovery_return_distance_counts(-8_458, 1_000), Some(7_458));
    assert_eq!(recovery_return_distance_counts(1_000, 1_000), Some(0));
    assert_eq!(recovery_return_distance_counts(-1_000, 1_000), Some(0));
    assert_eq!(recovery_return_distance_counts(i32::MIN, 1_000), None);
}

#[test]
fn maximum_gap_blind_travel_remains_inside_the_recovery_reserve() {
    let blind_travel_counts =
        (6_591_u128 * 4_000 * u128::from(MAX_SAFETY_OBSERVATION_GAP_US)).div_ceil(1_000_000_000);

    assert_eq!(blind_travel_counts, 528);
    assert_eq!(1_001 - i32::try_from(blind_travel_counts).unwrap(), 473);
    assert_eq!(473 - 1_000, -527);
    assert_eq!(-1_001 + i32::try_from(blind_travel_counts).unwrap(), -473);
    assert_eq!(-473 + 1_000, 527);
}

#[test]
fn direction_authority_uses_the_smallest_commanded_side_magnitude() {
    assert_eq!(
        commanded_side_lower_rate_q(VelocityDirection::Forward, 12_000, 18_000),
        Some(12_000)
    );
    assert_eq!(
        commanded_side_lower_rate_q(VelocityDirection::Reverse, -18_000, -11_000),
        Some(11_000)
    );
    assert_eq!(
        commanded_side_lower_rate_q(VelocityDirection::Forward, 0, 18_000),
        None
    );
    assert_eq!(
        commanded_side_lower_rate_q(VelocityDirection::Reverse, -18_000, 0),
        None
    );
}

#[test]
fn runtime_uses_outward_rate_rounding_and_fixed_allowances() {
    let runtime = recovery_runtime_us(7_458, 14_000_000, 3_000_000).unwrap();
    let measured = (7_458_u128 * 1_000_000 * RATE_Q).div_ceil(14_000_000);

    assert_eq!(runtime, u32::try_from(measured).unwrap() + 1_248_000);
    assert!(recovery_runtime_us(7_458, 3_150_000, 3_000_000).is_err());
    assert!(recovery_runtime_us(1, 0, 3_000_000).is_err());
}

#[test]
fn recovery_slot_includes_settle_tail() {
    let runtime_us = recovery_runtime_us(834, 9_174_528, 3_000_000).unwrap();
    let reserved_settle_analysis_us = ZERO_SETTLE_US + SETTLE_ANALYSIS_BUDGET_US;
    let moving_budget_us = runtime_us.checked_sub(reserved_settle_analysis_us).unwrap();
    let maximum_moving_budget_us = 3_000_000_u32
        .checked_sub(reserved_settle_analysis_us)
        .unwrap();

    assert_eq!(runtime_us, 1_338_904);
    assert_eq!(reserved_settle_analysis_us, 736_000);
    assert_eq!(moving_budget_us, 602_904);
    assert_eq!(maximum_moving_budget_us, 2_264_000);
    assert_eq!(
        maximum_moving_budget_us + reserved_settle_analysis_us,
        3_000_000
    );
}

#[test]
fn selected_recovery_reserves_one_fixed_lead_in() {
    assert_eq!(
        recovery_runtime_us_with_schedule(
            2_500,
            4_394_000,
            3_000_000,
            RecoveryRestSchedule::Stabilized,
        ),
        Ok(2_316_958)
    );
    assert_eq!(
        recovery_runtime_us_with_schedule(
            9_000,
            13_257_728,
            3_000_000,
            RecoveryRestSchedule::Stabilized,
        ),
        Ok(2_426_850)
    );
    assert_eq!(
        recovery_runtime_us_with_schedule(
            9_000,
            12_901_376,
            3_000_000,
            RecoveryRestSchedule::Stabilized,
        ),
        Ok(2_445_601)
    );
}

#[test]
fn proportional_decay_stalls_outside_the_declared_band() {
    let disturbance_rate_q = 3_150_000_u128;
    let proportional_horizon_us = 512_000_u128;
    let stalled_offset =
        (disturbance_rate_q * proportional_horizon_us).div_ceil(1_000_000 * RATE_Q);

    assert_eq!(stalled_offset, 1_613);
    assert!(stalled_offset > 1_000);
}

#[test]
fn invalid_geometry_rejects_without_saturation() {
    let mut budget = live_budget();
    budget.settle_travel_reserve_mrev = budget.max_stroke_travel_mrev;
    assert!(plan_rung_origin_recovery(budget, 4_000, 4_394).is_err());

    let budget = live_budget();
    assert!(plan_rung_origin_recovery(budget, 0, 4_394).is_err());
    assert!(plan_rung_origin_recovery(budget, 4_000, 0).is_err());

    let mut too_short = live_budget();
    too_short.max_duration_ms = 1_816;
    assert!(plan_rung_origin_recovery(too_short, 4_000, 4_394).is_err());
}

fn configured_safety(start_offset_counts: i32) -> OuterSafetyLimits {
    let mut safety = OuterSafetyLimits::new(2_816, live_budget());
    safety
        .configure_velocity_sweep_encoder_resolution(4_000)
        .unwrap();
    safety.configure_velocity_envelope(4_394).unwrap();
    let raw = start_offset_counts.rem_euclid(4_000) as u32;
    safety.set_encoder_baseline(raw).unwrap();
    safety.unwrapped_position = start_offset_counts;
    safety.arm_safety_observation(0).unwrap();
    safety
}

fn runtime(start_offset_counts: i32, lower_rate_q: Option<u64>) -> RungOriginRecoveryRuntime {
    let authority_source = if lower_rate_q.is_some() {
        RecoveryBindingSource::SettledRate
    } else {
        RecoveryBindingSource::MissingRate
    };
    runtime_with_source(start_offset_counts, lower_rate_q, authority_source)
}

fn runtime_with_source(
    start_offset_counts: i32,
    lower_rate_q: Option<u64>,
    authority_source: RecoveryBindingSource,
) -> RungOriginRecoveryRuntime {
    runtime_with_policy(
        start_offset_counts,
        lower_rate_q,
        authority_source,
        RecoveryCurrentContactPolicy::StopImmediately,
    )
}

fn runtime_with_policy(
    start_offset_counts: i32,
    lower_rate_q: Option<u64>,
    authority_source: RecoveryBindingSource,
    current_contact_policy: RecoveryCurrentContactPolicy,
) -> RungOriginRecoveryRuntime {
    runtime_with_schedule(
        start_offset_counts,
        lower_rate_q,
        authority_source,
        current_contact_policy,
        RecoveryRestSchedule::ImmediateScored,
    )
}

fn runtime_with_schedule(
    start_offset_counts: i32,
    lower_rate_q: Option<u64>,
    authority_source: RecoveryBindingSource,
    current_contact_policy: RecoveryCurrentContactPolicy,
    rest_schedule: RecoveryRestSchedule,
) -> RungOriginRecoveryRuntime {
    RungOriginRecoveryRuntime::new(
        RungOriginRecoveryConfig {
            stage: RecoveryStage::Proportional,
            rung_index: 17,
            p: PidPGain::new(1_448).unwrap(),
            target_velocity_rpm: TmcMechanicalRpm::from_register(264),
            planned_velocity_mrev_s: 4_394,
            start_offset_counts,
            lower_rate_q,
            authority_source,
            current_contact_policy,
            rest_schedule,
        },
        plan_rung_origin_recovery(live_budget(), 4_000, 4_394).unwrap(),
    )
    .unwrap()
}

struct RecoveryFixture {
    runtime: RungOriginRecoveryRuntime,
    safety: OuterSafetyLimits,
    observations: VelocityObservationRing,
    blocker: DyadicBlocker<VELOCITY_SWEEP_BLOCK_LEVELS>,
    analysis: StationarityWork,
    displacement: i8,
    minimum: i8,
    maximum: i8,
    current_observer: VelocityCurrentObserver,
    backend: MockBackend,
}

impl RecoveryFixture {
    fn new(start_offset_counts: i32, lower_rate_q: Option<u64>) -> Self {
        let authority_source = if lower_rate_q.is_some() {
            RecoveryBindingSource::SettledRate
        } else {
            RecoveryBindingSource::MissingRate
        };
        Self::new_with_source(start_offset_counts, lower_rate_q, authority_source)
    }

    fn new_with_source(
        start_offset_counts: i32,
        lower_rate_q: Option<u64>,
        authority_source: RecoveryBindingSource,
    ) -> Self {
        Self::new_with_policy(
            start_offset_counts,
            lower_rate_q,
            authority_source,
            RecoveryCurrentContactPolicy::StopImmediately,
        )
    }

    fn new_with_policy(
        start_offset_counts: i32,
        lower_rate_q: Option<u64>,
        authority_source: RecoveryBindingSource,
        current_contact_policy: RecoveryCurrentContactPolicy,
    ) -> Self {
        Self::new_with_schedule(
            start_offset_counts,
            lower_rate_q,
            authority_source,
            current_contact_policy,
            RecoveryRestSchedule::ImmediateScored,
        )
    }

    fn new_with_schedule(
        start_offset_counts: i32,
        lower_rate_q: Option<u64>,
        authority_source: RecoveryBindingSource,
        current_contact_policy: RecoveryCurrentContactPolicy,
        rest_schedule: RecoveryRestSchedule,
    ) -> Self {
        let mut backend = MockBackend::new();
        backend.current_limit_value = current_limit(2_816);
        backend.mechanical_encoder_position_counts_value =
            start_offset_counts.rem_euclid(4_000) as u32;
        let _ = backend.registers.insert(
            foci_tmc4671::registers::INTERIM_PIDOUT_TARGET_TORQUE
                .data
                .addr(),
            100,
        );
        Self {
            runtime: runtime_with_schedule(
                start_offset_counts,
                lower_rate_q,
                authority_source,
                current_contact_policy,
                rest_schedule,
            ),
            safety: configured_safety(start_offset_counts),
            observations: VelocityObservationRing::new(),
            blocker: DyadicBlocker::new(),
            analysis: StationarityWork::new(),
            displacement: 0,
            minimum: 0,
            maximum: 0,
            current_observer: operational_current_observer(),
            backend,
        }
    }

    /// Drive to a terminal step, counting the recovery-owned diagnostics
    /// (`RestSelection`, `RestAnalysis`) returned along the way.
    ///
    /// The caller must already have driven every time-based `At(_)` wait
    /// (the moving deadline, the settle capture window) with `sample`/`step`,
    /// exactly as the other fixture-driven tests do. From that point on, the
    /// remaining steps are `ReadyNow`, at most one `RestSelection`, at most
    /// one `RestAnalysis`, and finally a genuine terminal step (`Finished`,
    /// `Contained`, `Fault`, or `RestUnconfirmed`).
    fn drive_to_terminal(&mut self) -> (RungOriginRecoveryStep, u32, u32) {
        let mut selection_count = 0u32;
        let mut analysis_count = 0u32;
        loop {
            match self.step() {
                RungOriginRecoveryStep::ReadyNow => {}
                RungOriginRecoveryStep::RestSelection { .. } => selection_count += 1,
                RungOriginRecoveryStep::RestAnalysis(_) => analysis_count += 1,
                terminal => return (terminal, selection_count, analysis_count),
            }
        }
    }

    fn step(&mut self) -> RungOriginRecoveryStep {
        let mut resources = RungOriginRecoveryResources {
            safety: &mut self.safety,
            observations: &mut self.observations,
            blocker: &mut self.blocker,
            analysis: &mut self.analysis,
            settle_displacement_counts: &mut self.displacement,
            settle_min_displacement_counts: &mut self.minimum,
            settle_max_displacement_counts: &mut self.maximum,
            current_observer: &mut self.current_observer,
        };
        pollster::block_on(step_rung_origin_recovery(
            &mut self.runtime,
            &mut resources,
            &mut self.backend,
        ))
    }

    fn command_motion(&mut self) {
        assert_eq!(self.step(), RungOriginRecoveryStep::ReadyNow);
        assert_eq!(self.step(), RungOriginRecoveryStep::ReadyNow);
        assert!(matches!(self.step(), RungOriginRecoveryStep::At(1_000)));
    }

    fn sample(&mut self, elapsed_us: u64, absolute_offset_counts: i32) -> RungOriginRecoveryStep {
        self.backend.elapsed_us_value = elapsed_us;
        self.backend.mechanical_encoder_position_counts_value =
            absolute_offset_counts.rem_euclid(4_000) as u32;
        self.step()
    }

    /// Drive to one step short of the moving target, returning the elapsed time.
    ///
    /// Leaves the final observation to the caller, which is how a dispatcher-taken
    /// sample completes the moving phase.
    fn approach_moving_target(&mut self, start_offset_counts: i32) -> u64 {
        const STEP_COUNTS: i32 = 20;
        let target = moving_arrival_counts();
        let sign = if start_offset_counts < 0 { -1 } else { 1 };
        let mut magnitude = start_offset_counts.abs();
        let mut elapsed_us = 0;
        while magnitude - STEP_COUNTS > target {
            magnitude -= STEP_COUNTS;
            elapsed_us += 1_000;
            assert!(matches!(
                self.sample(elapsed_us, sign * magnitude),
                RungOriginRecoveryStep::At(_)
            ));
        }
        elapsed_us
    }

    /// Drive the axis in to the moving target and return the handover time.
    ///
    /// The moving phase ends inside the origin band rather than on its edge, so
    /// arriving takes several samples: the outer envelope caps how far the axis
    /// may travel per millisecond, and a single jump to the target trips it.
    /// Walking at a legal rate is also what the axis actually does.
    fn arrive_at_moving_target(&mut self, start_offset_counts: i32) -> u64 {
        const STEP_COUNTS: i32 = 20;
        let target = moving_arrival_counts();
        let sign = if start_offset_counts < 0 { -1 } else { 1 };
        let mut magnitude = start_offset_counts.abs();
        let mut elapsed_us = 0;
        loop {
            magnitude = (magnitude - STEP_COUNTS).max(target);
            elapsed_us += 1_000;
            match self.sample(elapsed_us, sign * magnitude) {
                RungOriginRecoveryStep::ReadyNow => return elapsed_us,
                RungOriginRecoveryStep::At(_) => assert!(
                    magnitude > target,
                    "the moving phase must hand over once the target is reached"
                ),
                other => panic!("driving toward the origin must not fault: {other:?}"),
            }
        }
    }
}

#[test]
fn selected_recovery_discards_lead_in_before_scoring_rest() {
    let mut fixture = RecoveryFixture::new_with_schedule(
        1_040,
        Some(14_000_000),
        RecoveryBindingSource::SettledRate,
        RecoveryCurrentContactPolicy::StopImmediately,
        RecoveryRestSchedule::Stabilized,
    );
    fixture.command_motion();
    let handover_us = fixture.arrive_at_moving_target(1_040);
    let first = handover_us / 1_000 + 1;
    assert!(matches!(fixture.step(), RungOriginRecoveryStep::At(next) if next == first * 1_000));

    for sample in first..=first + 499 {
        let offset = if sample % 2 == 0 {
            moving_arrival_counts() - 1
        } else {
            moving_arrival_counts()
        };
        let result = fixture.sample(sample * 1_000, offset);
        if sample < first + 499 {
            assert!(matches!(result, RungOriginRecoveryStep::At(_)));
        } else {
            assert_eq!(result, RungOriginRecoveryStep::At((first + 500) * 1_000));
        }
    }
    for sample in first + 500..=first + 999 {
        let offset = if sample % 2 == 0 {
            moving_arrival_counts() - 1
        } else {
            moving_arrival_counts()
        };
        let result = fixture.sample(sample * 1_000, offset);
        if sample < first + 999 {
            assert!(matches!(result, RungOriginRecoveryStep::At(_)));
        } else {
            assert_eq!(result, RungOriginRecoveryStep::ReadyNow);
        }
    }
    let decision = loop {
        match fixture.step() {
            RungOriginRecoveryStep::ReadyNow => {}
            RungOriginRecoveryStep::RestSelection {
                decision,
                stage,
                rung_index,
            } => {
                assert_eq!(stage, RecoveryStage::Proportional);
                assert_eq!(rung_index, 17);
                break decision;
            }
            other => panic!("selected recovery should yield its rest decision: {other:?}"),
        }
    };
    assert_eq!(decision.selected_suffix_len, 256);
    let analysis = loop {
        match fixture.step() {
            RungOriginRecoveryStep::ReadyNow => {}
            RungOriginRecoveryStep::RestSelection { decision, .. } => {
                assert_eq!(decision.disposition, RestSelectionDisposition::Rejected);
            }
            RungOriginRecoveryStep::RestAnalysis(analysis) => break analysis,
            other => panic!("selected recovery should produce rest analysis: {other:?}"),
        }
    };

    // The moving phase now hands over at the target inside the band, so entry is
    // recorded there rather than on the band edge.
    assert_eq!(
        analysis.first_in_band_offset_counts,
        moving_arrival_counts()
    );
    assert_eq!(u64::from(analysis.entry_to_band_duration_us), handover_us);
    assert_eq!(analysis.lead_in.net_displacement_counts, 0);
    assert_eq!(analysis.lead_in.displacement_span_counts, 1);
    assert_eq!(analysis.scored_net_displacement_counts, 0);
    assert_eq!(analysis.disposition, 0);
    assert_eq!(analysis.interval_present, 1);
    assert!(matches!(
        fixture.step(),
        RungOriginRecoveryStep::Finished(_)
    ));
}

#[test]
fn selected_recovery_reports_completed_unconfirmed_rest_without_a_fault() {
    let mut fixture = RecoveryFixture::new_with_schedule(
        1_040,
        Some(14_000_000),
        RecoveryBindingSource::SettledRate,
        RecoveryCurrentContactPolicy::StopImmediately,
        RecoveryRestSchedule::Stabilized,
    );
    fixture.command_motion();
    let handover_us = fixture.arrive_at_moving_target(1_040);
    let first = handover_us / 1_000 + 1;
    assert!(matches!(fixture.step(), RungOriginRecoveryStep::At(next) if next == first * 1_000));
    for sample in first..=first + 499 {
        let result = fixture.sample(sample * 1_000, moving_arrival_counts());
        if sample < first + 499 {
            assert!(matches!(result, RungOriginRecoveryStep::At(_)));
        } else {
            assert_eq!(result, RungOriginRecoveryStep::At((first + 500) * 1_000));
        }
    }
    for sample in first + 500..=first + 999 {
        let offset = moving_arrival_counts() - (sample as i32 - (first as i32 + 499));
        let result = fixture.sample(sample * 1_000, offset);
        if sample < first + 999 {
            assert!(matches!(result, RungOriginRecoveryStep::At(_)));
        } else {
            assert_eq!(result, RungOriginRecoveryStep::ReadyNow);
        }
    }
    let analysis = loop {
        match fixture.step() {
            RungOriginRecoveryStep::ReadyNow => {}
            RungOriginRecoveryStep::RestSelection { decision, .. } => {
                assert_eq!(decision.disposition, RestSelectionDisposition::Rejected);
            }
            RungOriginRecoveryStep::RestAnalysis(analysis) => break analysis,
            other => panic!("weak rest should still produce analysis: {other:?}"),
        }
    };

    assert_eq!(analysis.disposition, 1);
    assert_eq!(analysis.scored_min_displacement_counts, -2);
    assert_eq!(fixture.step(), RungOriginRecoveryStep::RestUnconfirmed);
}

fn operational_current_observer() -> VelocityCurrentObserver {
    VelocityCurrentObserver::new(
        PidTorqueFluxLimit::new(2_816).unwrap(),
        VelocityLimitEvidenceCapability::BestEffortPolling,
    )
}

#[test]
fn recovery_binding_source_codes_are_unique_and_stable() {
    assert_eq!(RecoveryBindingSource::AlreadyCentered as u8, 0);
    assert_eq!(RecoveryBindingSource::SettledRate as u8, 1);
    assert_eq!(RecoveryBindingSource::MissingRate as u8, 2);
    assert_eq!(RecoveryBindingSource::Deadline as u8, 3);
    assert_eq!(RecoveryBindingSource::LocalTravel as u8, 4);
    assert_eq!(RecoveryBindingSource::CurrentHeadroom as u8, 6);
    assert_eq!(RungOriginRecoveryOutcome::RecoveryCurrentHeadroom as u8, 4);
}

#[test]
fn recovery_controller_contained_paths_never_write_a_nonzero_target() {
    for (start, lower_rate, expected) in [
        (1_000, None, RungOriginRecoveryOutcome::AlreadyCentered),
        (8_458, None, RungOriginRecoveryOutcome::ContainedNoRate),
        (
            -8_458,
            Some(3_150_000),
            RungOriginRecoveryOutcome::ContainedDeadline,
        ),
    ] {
        let mut runtime = runtime(start, lower_rate);
        let mut safety = configured_safety(start);
        let mut observations = VelocityObservationRing::new();
        let mut blocker = DyadicBlocker::<VELOCITY_SWEEP_BLOCK_LEVELS>::new();
        let mut analysis = StationarityWork::new();
        let mut displacement = 0;
        let mut minimum = 0;
        let mut maximum = 0;
        let mut current_observer = operational_current_observer();
        let mut resources = RungOriginRecoveryResources {
            safety: &mut safety,
            observations: &mut observations,
            blocker: &mut blocker,
            analysis: &mut analysis,
            settle_displacement_counts: &mut displacement,
            settle_min_displacement_counts: &mut minimum,
            settle_max_displacement_counts: &mut maximum,
            current_observer: &mut current_observer,
        };
        let mut backend = MockBackend::new();
        backend.current_limit_value = current_limit(2_816);
        backend.elapsed_us_value = 123_456;

        let result = pollster::block_on(step_rung_origin_recovery(
            &mut runtime,
            &mut resources,
            &mut backend,
        ));
        let report = match result {
            RungOriginRecoveryStep::Finished(report)
            | RungOriginRecoveryStep::Contained(report) => report,
            other => panic!("contained setup must finish without motion: {other:?}"),
        };
        assert_eq!(report.outcome, expected);
        assert_eq!(report.moving_duration_us, 0);
        assert_eq!(report.settle_duration_us, 0);
        assert_eq!(report.total_duration_us, 0);
        assert!(!backend.spi_log.iter().any(|operation| {
            matches!(operation, SpiOp::Write(address, value)
                if *address == PID_VELOCITY_TARGET.addr() && *value != 0)
        }));
    }
}

#[test]
fn moving_phase_drives_inside_the_band_before_settling() {
    // The moving phase used to end on the first sample inside the band, so
    // settle began at the edge and any outward movement before the final
    // in-band test pushed the rung out of it. Measured across 29 moving
    // recoveries on 2026-08-11: the final position is outward of the closest
    // approach every time, by 11 to 50 counts, leaving a final margin of 0 to
    // 62. Two rungs finished 2 and 5 counts outside and ended Stage C.
    //
    // The moving phase therefore drives to a target inside the band. The band
    // itself is unchanged: it is the settle travel reserve in counts, and the
    // acceptance test, the return-distance plan and the already-centered
    // short-circuit all still use it.
    let band = i32::try_from(origin_band_counts()).unwrap();
    let target = i32::try_from(recovery_moving_target_counts(origin_band_counts())).unwrap();
    assert!(target < band, "the moving target sits inside the band");

    let mut fixture = RecoveryFixture::new(1_400, Some(14_000_000));
    fixture.command_motion();

    // Walk in toward the origin at the cadence the envelope allows, and record
    // the offset at which the moving phase hands over to settle.
    let mut handover = None;
    for sample in 1..=40 {
        let offset = 1_400 - sample as i32 * 20;
        match fixture.sample(sample * 1_000, offset) {
            RungOriginRecoveryStep::At(_) => {}
            RungOriginRecoveryStep::ReadyNow => {
                handover = Some(offset);
                break;
            }
            other => panic!("recovery must drive toward the origin: {other:?}"),
        }
    }
    let handover = handover.expect("the moving phase completes");

    assert!(
        handover <= target,
        "settle must begin inside the band, not on its edge: handed over at \
         {handover} with band {band} and target {target}"
    );
}

#[test]
fn recovery_controller_releases_contained_primitive() {
    let mut fixture = RecoveryFixture::new(1_040, Some(14_000_000));
    fixture.command_motion();

    let mut moving_end_sample = None;
    for sample in 1..=600 {
        match fixture.sample(sample * 1_000, 1_001) {
            RungOriginRecoveryStep::At(_) => {}
            RungOriginRecoveryStep::ReadyNow => {
                moving_end_sample = Some(sample);
                break;
            }
            other => panic!("out-of-band motion must reach zero-target settle: {other:?}"),
        }
    }
    let settle_first_sample = moving_end_sample.expect("moving deadline must be bounded") + 1;
    assert!(matches!(
        fixture.step(),
        RungOriginRecoveryStep::At(deadline) if deadline == settle_first_sample * 1_000
    ));

    let settle_last_sample = settle_first_sample + 499;
    for sample in settle_first_sample..=settle_last_sample {
        let result = fixture.sample(sample * 1_000, 1_001);
        if sample < settle_last_sample {
            assert!(matches!(result, RungOriginRecoveryStep::At(_)));
        } else {
            assert_eq!(result, RungOriginRecoveryStep::ReadyNow);
        }
    }

    let report = loop {
        match fixture.step() {
            RungOriginRecoveryStep::ReadyNow => {}
            RungOriginRecoveryStep::RestSelection { .. } => {}
            RungOriginRecoveryStep::Contained(report) => break report,
            other => panic!("out-of-band rest must remain contained: {other:?}"),
        }
    };
    assert_eq!(report.outcome, RungOriginRecoveryOutcome::ContainedDeadline);
    assert!(
        fixture
            .safety
            .begin_primitive(fixture.backend.elapsed_us_value)
            .is_ok()
    );
    fixture.safety.end_primitive();
}

#[test]
fn recovery_controller_uses_constant_signed_target_and_bounds_wrong_way_growth() {
    let mut runtime = runtime(8_458, Some(14_000_000));
    let mut safety = configured_safety(8_458);
    let mut observations = VelocityObservationRing::new();
    let mut blocker = DyadicBlocker::<VELOCITY_SWEEP_BLOCK_LEVELS>::new();
    let mut analysis = StationarityWork::new();
    let mut displacement = 0;
    let mut minimum = 0;
    let mut maximum = 0;
    let mut current_observer = operational_current_observer();
    let mut resources = RungOriginRecoveryResources {
        safety: &mut safety,
        observations: &mut observations,
        blocker: &mut blocker,
        analysis: &mut analysis,
        settle_displacement_counts: &mut displacement,
        settle_min_displacement_counts: &mut minimum,
        settle_max_displacement_counts: &mut maximum,
        current_observer: &mut current_observer,
    };
    let mut backend = MockBackend::new();
    backend.current_limit_value = current_limit(2_816);
    backend.mechanical_encoder_position_counts_value = 458;

    for _ in 0..3 {
        let _ = pollster::block_on(step_rung_origin_recovery(
            &mut runtime,
            &mut resources,
            &mut backend,
        ));
    }
    assert_eq!(
        backend.registers.get(&PID_VELOCITY_TARGET.addr()),
        Some(&((-264_i32) as u32))
    );
    assert!(backend.spi_log.iter().any(|operation| {
        matches!(operation, SpiOp::Write(address, _)
            if *address == PID_VELOCITY_P_VELOCITY_I.addr())
    }));
    assert_eq!(backend.pid_velocity_error_sum_clears, 1);

    backend.elapsed_us_value = 1_000;
    backend.mechanical_encoder_position_counts_value = 459;
    assert!(matches!(
        pollster::block_on(step_rung_origin_recovery(
            &mut runtime,
            &mut resources,
            &mut backend,
        )),
        RungOriginRecoveryStep::At(_)
    ));

    // Two counts of wrong-way growth is jitter, not a wrong-direction drive.
    // Recovery's own success criterion accepts anything within
    // `origin_band_counts`, so a fatal gate an order of magnitude tighter than
    // the band it aims for discards a whole campaign for nothing.
    backend.elapsed_us_value = 2_000;
    backend.mechanical_encoder_position_counts_value = 460;
    assert!(
        !matches!(
            pollster::block_on(step_rung_origin_recovery(
                &mut runtime,
                &mut resources,
                &mut backend,
            )),
            RungOriginRecoveryStep::Fault { .. }
        ),
        "two counts of wrong-way growth is inside the band and must be tolerated"
    );

    // Beyond the band it is a genuine wrong-direction drive and still faults.
    // Stepped at 12 counts/ms, just under the 13 counts/ms envelope cap, so the
    // velocity gate does not fire first and mask the reason under test.
    let band = origin_band_counts();
    let mut elapsed = 2_000_u64;
    let mut position = 460_u32;
    let mut step_fault = None;
    let mut last_step = None;
    while position < 460 + band + 200 {
        elapsed += 1_000;
        position += 12;
        backend.elapsed_us_value = elapsed;
        backend.mechanical_encoder_position_counts_value = position;
        let step = pollster::block_on(step_rung_origin_recovery(
            &mut runtime,
            &mut resources,
            &mut backend,
        ));
        if let RungOriginRecoveryStep::Fault {
            safety: Some(fault),
            ..
        } = step
        {
            step_fault = Some(fault);
            break;
        }
        last_step = Some(step);
    }
    let Some(fault) = step_fault else {
        panic!("wrong-way growth beyond the band must fault, last step: {last_step:?}");
    };
    assert_eq!(fault.reason, OUTER_SAFETY_FAULT_RECOVERY_WRONG_WAY);
}

#[test]
fn recovery_safety_reason_is_unique_and_stable() {
    assert_eq!(OUTER_SAFETY_FAULT_RECOVERY_WRONG_WAY, 10);
}

#[test]
fn recovery_controller_confirms_fixed_settle_and_reports_complete_geometry() {
    let mut fixture = RecoveryFixture::new(1_040, Some(14_000_000));
    fixture.command_motion();
    let handover_us = fixture.arrive_at_moving_target(1_040);
    assert_eq!(
        fixture.backend.registers.get(&PID_VELOCITY_TARGET.addr()),
        Some(&0)
    );
    let first = handover_us / 1_000 + 1;
    assert!(matches!(fixture.step(), RungOriginRecoveryStep::At(next) if next == first * 1_000));
    assert_eq!(fixture.backend.pid_velocity_error_sum_clears, 2);

    for sample in first..=first + 499 {
        let result = fixture.sample(sample * 1_000, moving_arrival_counts());
        if sample < first + 499 {
            assert!(matches!(result, RungOriginRecoveryStep::At(_)));
        } else {
            assert_eq!(result, RungOriginRecoveryStep::ReadyNow);
        }
    }
    let report = loop {
        match fixture.step() {
            RungOriginRecoveryStep::ReadyNow => {}
            RungOriginRecoveryStep::RestSelection { .. } => {}
            RungOriginRecoveryStep::Finished(report) => break report,
            other => panic!("fixed rest should recover: {other:?}"),
        }
    };
    assert_eq!(report.outcome, RungOriginRecoveryOutcome::Recovered);
    assert_eq!(report.start_offset_counts, 1_040);
    assert_eq!(report.closest_offset_counts, moving_arrival_counts());
    assert_eq!(report.final_offset_counts, moving_arrival_counts());
    assert_eq!(report.origin_band_counts, 1_000);
    assert_eq!(report.lower_rate_q.to_u64(), 14_000_000);
    assert_eq!(u64::from(report.moving_duration_us), handover_us);
    assert_eq!(report.settle_duration_us, 500_000);
    assert_eq!(u64::from(report.total_duration_us), handover_us + 500_000);
    assert_eq!(report.peak_torque_target_abs, 100);
    assert!(fixture.safety.primitive_start_us.is_none());
    assert_eq!(fixture.observations.len(), 0);
}

#[test]
fn recovery_current_observer_covers_motion_and_zero_target_settle() {
    let mut fixture = RecoveryFixture::new(1_040, Some(14_000_000));
    fixture.command_motion();
    let handover_us = fixture.arrive_at_moving_target(1_040);
    let first = handover_us / 1_000 + 1;
    assert!(matches!(fixture.step(), RungOriginRecoveryStep::At(next) if next == first * 1_000));

    let _ = fixture.backend.registers.insert(
        foci_tmc4671::registers::INTERIM_PIDOUT_TARGET_TORQUE
            .data
            .addr(),
        2_816,
    );
    let RungOriginRecoveryStep::Contained(report) =
        fixture.sample(handover_us + 1_000, moving_arrival_counts())
    else {
        panic!("recovery clamp contact must be contained")
    };
    assert_eq!(
        report.outcome,
        RungOriginRecoveryOutcome::RecoveryCurrentHeadroom
    );
    assert_eq!(
        report.binding_source,
        RecoveryBindingSource::CurrentHeadroom
    );
    assert_eq!(fixture.current_observer.moving_pid_output_peak(), 100);
    assert_eq!(fixture.current_observer.zero_pid_output_peak(), 2_816);
    assert_eq!(
        fixture
            .current_observer
            .evidence(VelocityCurrentZeroPhase::Recovery)
            .unwrap()
            .contact,
        VelocityCurrentContact::Recovery
    );
}

#[test]
fn recovery_contact_passive_settle_proves_an_in_band_origin() {
    let mut fixture = RecoveryFixture::new_with_policy(
        1_040,
        Some(14_000_000),
        RecoveryBindingSource::SettledRate,
        RecoveryCurrentContactPolicy::ProvePassiveSettle,
    );
    fixture.command_motion();
    let _ = fixture.backend.registers.insert(
        foci_tmc4671::registers::INTERIM_PIDOUT_TARGET_TORQUE
            .data
            .addr(),
        2_816,
    );

    let handover_us = fixture.arrive_at_moving_target(1_040);
    assert_eq!(
        fixture.backend.registers.get(&PID_VELOCITY_TARGET.addr()),
        Some(&0)
    );
    assert_eq!(fixture.backend.pid_velocity_error_sum_clears, 2);
    let first = handover_us / 1_000 + 1;
    assert!(matches!(fixture.step(), RungOriginRecoveryStep::At(next) if next == first * 1_000));

    let _ = fixture.backend.registers.insert(
        foci_tmc4671::registers::INTERIM_PIDOUT_TARGET_TORQUE
            .data
            .addr(),
        100,
    );
    for sample in first..=first + 499 {
        let result = fixture.sample(sample * 1_000, 998);
        if sample < first + 499 {
            assert!(matches!(result, RungOriginRecoveryStep::At(_)));
        } else {
            assert_eq!(result, RungOriginRecoveryStep::ReadyNow);
        }
    }
    let report = loop {
        match fixture.step() {
            RungOriginRecoveryStep::ReadyNow => {}
            RungOriginRecoveryStep::RestSelection { .. } => {}
            RungOriginRecoveryStep::Finished(report) => break report,
            other => panic!("passive in-band rest should recover: {other:?}"),
        }
    };

    assert_eq!(
        report.outcome,
        RungOriginRecoveryOutcome::RecoveredWithCurrentHeadroom
    );
    assert_eq!(
        report.binding_source,
        RecoveryBindingSource::CurrentHeadroom
    );
    assert_eq!(report.final_offset_counts, 998);
    assert_eq!(fixture.backend.pid_velocity_error_sum_clears, 2);
    assert_eq!(
        fixture
            .backend
            .spi_log
            .iter()
            .filter(|operation| {
                matches!(
                    operation,
                    SpiOp::Write(address, value)
                        if *address == PID_VELOCITY_TARGET.addr() && *value != 0
                )
            })
            .count(),
        1
    );
}

#[test]
fn recovery_contact_passive_settle_rejects_an_out_of_band_origin() {
    let mut fixture = RecoveryFixture::new_with_policy(
        1_040,
        Some(14_000_000),
        RecoveryBindingSource::SettledRate,
        RecoveryCurrentContactPolicy::ProvePassiveSettle,
    );
    fixture.command_motion();
    let _ = fixture.backend.registers.insert(
        foci_tmc4671::registers::INTERIM_PIDOUT_TARGET_TORQUE
            .data
            .addr(),
        2_816,
    );

    assert_eq!(
        fixture.sample(1_000, 1_001),
        RungOriginRecoveryStep::ReadyNow
    );
    let _ = fixture.backend.registers.insert(
        foci_tmc4671::registers::INTERIM_PIDOUT_TARGET_TORQUE
            .data
            .addr(),
        100,
    );
    for sample in 2..=501 {
        let result = fixture.sample(sample * 1_000, 1_001);
        if sample < 501 {
            assert!(matches!(result, RungOriginRecoveryStep::At(_)));
        } else {
            assert_eq!(result, RungOriginRecoveryStep::ReadyNow);
        }
    }

    let report = loop {
        match fixture.step() {
            RungOriginRecoveryStep::ReadyNow => {}
            RungOriginRecoveryStep::RestSelection { .. } => {}
            RungOriginRecoveryStep::Contained(report) => break report,
            other => panic!("out-of-band rest must remain contained: {other:?}"),
        }
    };

    assert_eq!(
        report.outcome,
        RungOriginRecoveryOutcome::RecoveryCurrentHeadroom
    );
    assert_eq!(
        report.binding_source,
        RecoveryBindingSource::CurrentHeadroom
    );
    assert_eq!(report.final_offset_counts, 1_001);
}

#[test]
fn recovery_consumes_the_safety_observation_taken_by_the_outer_dispatcher() {
    let mut fixture = RecoveryFixture::new(1_040, Some(14_000_000));
    fixture.command_motion();
    // Drive to one step short of the target so the dispatcher's own observation
    // is the one that completes the moving phase.
    let elapsed_us = fixture.approach_moving_target(1_040);
    fixture.backend.elapsed_us_value = elapsed_us + 1_000;
    fixture.backend.mechanical_encoder_position_counts_value =
        moving_arrival_counts().rem_euclid(4_000) as u32;

    assert_eq!(
        pollster::block_on(check_outer_safety(
            &mut fixture.safety,
            &mut fixture.backend,
        )),
        Ok(None)
    );
    assert_eq!(fixture.safety.unwrapped_position, moving_arrival_counts());

    assert_eq!(fixture.step(), RungOriginRecoveryStep::ReadyNow);
    assert_eq!(
        fixture.backend.registers.get(&PID_VELOCITY_TARGET.addr()),
        Some(&0)
    );
}

#[test]
fn recovery_retains_dispatcher_timing_for_the_zero_target_settle() {
    let mut fixture = RecoveryFixture::new(1_040, Some(14_000_000));
    fixture.command_motion();
    // Drive to one step short of the target, then let the dispatcher take the
    // observation that completes the moving phase.
    let elapsed_us = fixture.approach_moving_target(1_040);
    fixture.backend.elapsed_us_value = elapsed_us + 1_000;
    fixture.backend.mechanical_encoder_position_counts_value =
        moving_arrival_counts().rem_euclid(4_000) as u32;
    assert_eq!(
        pollster::block_on(check_outer_safety(
            &mut fixture.safety,
            &mut fixture.backend,
        )),
        Ok(None)
    );
    assert_eq!(fixture.step(), RungOriginRecoveryStep::ReadyNow);
    assert!(
        matches!(fixture.step(), RungOriginRecoveryStep::At(next) if next == elapsed_us + 2_000)
    );

    fixture.backend.elapsed_us_value = elapsed_us + 2_000;
    assert_eq!(
        pollster::block_on(check_outer_safety(
            &mut fixture.safety,
            &mut fixture.backend,
        )),
        Ok(None)
    );

    assert!(
        matches!(fixture.step(), RungOriginRecoveryStep::At(next) if next == elapsed_us + 3_000)
    );
    assert_eq!(fixture.observations.len(), 1);
    let observation = fixture.observations.iter().next().unwrap();
    assert_eq!(observation.delta_counts, 0);
    assert_eq!(observation.elapsed_us, 1_000);
}

#[test]
fn recovery_controller_preserves_reverse_command_sign() {
    let mut fixture = RecoveryFixture::new(-8_458, Some(14_000_000));
    fixture.command_motion();
    assert_eq!(
        fixture.backend.registers.get(&PID_VELOCITY_TARGET.addr()),
        Some(&264_u32)
    );
}

#[test]
fn recovery_timing_context_tracks_setup_motion_and_settle() {
    let mut fixture = RecoveryFixture::new(1_040, Some(14_000_000));
    assert_eq!(
        fixture.runtime.outer_safety_timing_phase(),
        (OuterSafetyTimingPhase::RecoverySetup, true)
    );

    fixture.command_motion();
    assert_eq!(
        fixture.runtime.outer_safety_timing_phase(),
        (OuterSafetyTimingPhase::RecoveryMove, false)
    );

    let _ = fixture.arrive_at_moving_target(1_040);
    assert_eq!(
        fixture.runtime.outer_safety_timing_phase(),
        (OuterSafetyTimingPhase::RecoverySettle, true)
    );
}

fn safety_fault(result: RungOriginRecoveryStep) -> u8 {
    let RungOriginRecoveryStep::Fault {
        safety: Some(fault),
        ..
    } = result
    else {
        panic!("expected an inherited safety fault, got {result:?}")
    };
    fault.reason
}

#[test]
fn recovery_safety_inherits_gap_quarter_turn_duration_current_and_spi_faults() {
    let mut gap = RecoveryFixture::new(8_458, Some(14_000_000));
    gap.command_motion();
    assert_eq!(
        safety_fault(gap.sample(MAX_SAFETY_OBSERVATION_GAP_US + 1, 8_458)),
        7
    );

    let mut quarter = RecoveryFixture::new(8_458, Some(14_000_000));
    quarter.command_motion();
    assert_eq!(safety_fault(quarter.sample(1_000, 9_458)), 8);

    let mut duration = RecoveryFixture::new(8_458, Some(14_000_000));
    duration.command_motion();
    assert_eq!(safety_fault(duration.sample(3_000_001, 8_458)), 3);

    let mut current = RecoveryFixture::new(8_458, Some(14_000_000));
    let _ = current.backend.registers.insert(
        foci_tmc4671::registers::INTERIM_PIDOUT_TARGET_TORQUE
            .data
            .addr(),
        2_817,
    );
    current.command_motion();
    assert_eq!(safety_fault(current.sample(1_000, 8_458)), 9);

    let mut spi = RecoveryFixture::new(8_458, Some(14_000_000));
    crate::diagnostic::begin_breakaway_campaign();
    spi.command_motion();
    spi.backend.fail_on_read = Some(foci_tmc4671::registers::ABN_DECODER_COUNT.addr());
    assert!(matches!(
        spi.sample(1_000, 8_458),
        RungOriginRecoveryStep::Fault {
            error: crate::autotune::CommissionError::Spi,
            safety: None,
        }
    ));
    assert_eq!(
        crate::diagnostic::current_breakaway_breadcrumb_for_test(),
        Some(crate::diagnostic::BreakawayBreadcrumb {
            checkpoint: crate::diagnostic::BreakawayCheckpoint::RecoveryMovingSafetySample,
            interrupted: true,
        })
    );
}

#[test]
fn shared_recovery_does_not_create_a_breakaway_breadcrumb() {
    crate::diagnostic::set_breakaway_breadcrumb_raw_for_test(0);
    let mut fixture = RecoveryFixture::new(8_458, Some(14_000_000));

    fixture.command_motion();

    assert_eq!(
        crate::diagnostic::current_breakaway_breadcrumb_for_test(),
        None
    );
}

#[test]
fn recovery_safety_inherits_velocity_local_and_absolute_position_faults() {
    let mut velocity = RecoveryFixture::new(8_458, Some(14_000_000));
    velocity.command_motion();
    assert!(matches!(
        velocity.sample(1_000, 8_428),
        RungOriginRecoveryStep::At(_)
    ));
    assert_eq!(safety_fault(velocity.sample(2_000, 8_398)), 4);

    let mut local = RecoveryFixture::new(90, Some(14_000_000));
    local.runtime = RungOriginRecoveryRuntime::new(
        RungOriginRecoveryConfig {
            stage: RecoveryStage::Proportional,
            rung_index: 17,
            p: PidPGain::new(1_448).unwrap(),
            target_velocity_rpm: TmcMechanicalRpm::from_register(264),
            planned_velocity_mrev_s: 4_394,
            start_offset_counts: 90,
            lower_rate_q: Some(14_000_000),
            authority_source: RecoveryBindingSource::SettledRate,
            current_contact_policy: RecoveryCurrentContactPolicy::StopImmediately,
            rest_schedule: RecoveryRestSchedule::ImmediateScored,
        },
        super::recovery::RungOriginRecoveryPlan {
            origin_band_counts: 10,
            nominal_slot_us: 1_816_958,
            maximum_slot_us: 3_000_000,
        },
    )
    .unwrap();
    local.safety.position_window = 100;
    local.command_motion();
    // Local (per-stroke) branch now reports its own code (POSITION_LOCAL = 11).
    assert_eq!(safety_fault(local.sample(1_000, -20)), 11);

    let mut absolute = RecoveryFixture::new(8_458, Some(14_000_000));
    absolute.safety.positive_position_window = 8_000;
    absolute.command_motion();
    // Absolute (machine-headroom) branch keeps POSITION = 5.
    assert_eq!(safety_fault(absolute.sample(1_000, 8_450)), 5);
}

#[test]
fn recovery_safety_rejects_unconfirmed_rest() {
    let mut fixture = RecoveryFixture::new(1_020, Some(14_000_000));
    fixture.command_motion();
    let handover_us = fixture.arrive_at_moving_target(1_040);
    let first = handover_us / 1_000 + 1;
    assert!(matches!(fixture.step(), RungOriginRecoveryStep::At(next) if next == first * 1_000));
    for sample in first..=first + 499 {
        let offset = moving_arrival_counts() - (sample as i32 - first as i32);
        let result = fixture.sample(sample * 1_000, offset);
        if sample < first + 499 {
            assert!(matches!(result, RungOriginRecoveryStep::At(_)));
        } else {
            assert_eq!(result, RungOriginRecoveryStep::ReadyNow);
        }
    }
    loop {
        match fixture.step() {
            RungOriginRecoveryStep::ReadyNow => {}
            RungOriginRecoveryStep::RestSelection { decision, .. } => {
                assert_eq!(decision.disposition, RestSelectionDisposition::Rejected);
            }
            RungOriginRecoveryStep::Fault {
                error: crate::autotune::CommissionError::VelocityRestNotConfirmed,
                safety: None,
            } => break,
            other => panic!("creeping settle must fail confirmation: {other:?}"),
        }
    }
}

/// The origin having drifted by a single encoder sample must not end the
/// campaign.
///
/// The captured start offset feeds only the planned return distance; every
/// verdict reads the live position for direction. Exact
/// equality here discarded whole campaigns for one count of drift out of a
/// thousand-count target band.
#[test]
fn recovery_tolerates_origin_drift_within_the_target_band() {
    let mut fixture = RecoveryFixture::new(1_040, Some(14_000_000));
    // One outer-safety encoder sample landing between the tick that captured
    // the origin and the tick recovery begins.
    fixture.safety.unwrapped_position = 1_041;

    assert_eq!(fixture.step(), RungOriginRecoveryStep::ReadyNow);
}

/// The tolerance is the band recovery aims for, and is applied in both
/// directions.
#[test]
fn recovery_tolerates_origin_drift_up_to_the_whole_band() {
    let band = i32::try_from(origin_band_counts()).unwrap();

    for drift in [band, -band] {
        let mut fixture = RecoveryFixture::new(1_040, Some(14_000_000));
        fixture.safety.unwrapped_position = 1_040 + drift;
        assert_eq!(
            fixture.step(),
            RungOriginRecoveryStep::ReadyNow,
            "drift of {drift} counts is inside the band and must be tolerated"
        );
    }
}

/// Drift beyond the band still faults, and reports the measurement first.
#[test]
fn recovery_reports_and_faults_on_drift_beyond_the_band() {
    let band = i32::try_from(origin_band_counts()).unwrap();
    let mut fixture = RecoveryFixture::new(1_040, Some(14_000_000));
    fixture.safety.unwrapped_position = 1_040 + band + 1;

    let RungOriginRecoveryStep::OriginMismatch(evidence) = fixture.step() else {
        panic!("drift beyond the band must report its measurement");
    };
    assert_eq!(evidence.expected_counts, 1_040);
    assert_eq!(evidence.observed_counts, 1_040 + band + 1);

    // The fault itself is unchanged; it simply arrives one tick later.
    assert_eq!(
        fixture.step(),
        RungOriginRecoveryStep::Fault {
            error: crate::autotune::CommissionError::RecoveryOriginMismatch,
            safety: None,
        }
    );
}

/// A drift that crosses zero is necessarily already inside the band.
///
/// This is why tolerating band-sized drift is safe with respect to direction.
/// Commanding on a stale sign would drive the motor away from the origin, so
/// the question is whether a tolerated drift can flip the sign while motion is
/// still required. It cannot: crossing zero within a drift of at most `band`
/// forces `|captured| + |live| <= band`, hence `|live| < band`, and a position
/// inside the band is `AlreadyCentered` with no motion commanded at all.
///
/// The controller reads the live position for its direction regardless, so the
/// hazard stays closed if this tolerance is ever widened.
#[test]
fn a_sign_flip_within_the_drift_tolerance_needs_no_motion() {
    let band = i32::try_from(origin_band_counts()).unwrap();

    for (captured, live) in [(band - 1, -1), (1, -(band - 1)), (-(band - 1), 1)] {
        assert!(
            (live - captured).abs() <= band,
            "fixture must stay inside the tolerance"
        );
        let mut fixture = RecoveryFixture::new(captured, Some(14_000_000));
        fixture.safety.unwrapped_position = live;

        let RungOriginRecoveryStep::Finished(report) = fixture.step() else {
            panic!("a position inside the band needs no recovery motion");
        };
        assert_eq!(
            report.outcome,
            RungOriginRecoveryOutcome::AlreadyCentered,
            "captured {captured} live {live} should be already centered"
        );
    }
}

/// Offset at which the moving phase completes: inside the band by the
/// stopping margin, so settle begins with room for outward movement.
fn moving_arrival_counts() -> i32 {
    i32::try_from(recovery_moving_target_counts(origin_band_counts())).unwrap()
}

fn origin_band_counts() -> u32 {
    plan_rung_origin_recovery(live_budget(), 4_000, 4_394)
        .expect("live geometry plans")
        .origin_band_counts
}

// The Klipper host assembler accepts a hidden trace-only evidence run of 0 to
// 2 positions before a rung-origin recovery, and refuses 0 for outcomes 1
// (`Recovered`) and 5 (`RecoveredWithCurrentHeadroom`). Both bounds are
// properties of this controller, not of the host, and they follow from a
// structural fact rather than an exhaustive search:
//
// `step_rung_origin_recovery` returns a recovery-owned diagnostic at exactly
// two sites, both inside `RecoveryState::AnalyzeSettle`
// (`recovery/controller.rs`): `RestSelection` when `poll_scored_rest_confirmation`
// yields `Diagnostic`, and `RestAnalysis`, guarded by `runtime.uses_stabilization()`,
// immediately after. Every other return site yields `ReadyNow`, `At`,
// `OriginMismatch`, `Finished`, `Contained`, `RestUnconfirmed`, or a
// `recovery_fault`, none of which commits a recovery-owned trace record.
//
// The count is bounded by 2 because each site fires at most once per
// recovery:
//
// - `AnalyzeSettle` is entered at most once. Nothing in the controller ever
//   transitions back into `CaptureSettle` with `lead_in: false` (the only
//   predecessor that calls `analysis.reset_for_scored_rest` and advances into
//   `AnalyzeSettle`) after `AnalyzeSettle` has been reached.
// - Within one `AnalyzeSettle` visit, `poll_scored_rest_confirmation` can
//   yield `Diagnostic` at most once. `StationarityWork::poll` only returns
//   `RestDiagnostic` from its `Idle { mode: RestDiagnostic { .. } }` stage,
//   and that stage transitions to `RestDecisionReady` in the same call that
//   produces the diagnostic (`stationarity.rs`, `advance_rest_diagnostic` and
//   the empty-suffix branch of `poll`). Every subsequent poll dispatches
//   through `RestDecisionReady` (`resume_rest_decision`) or `Blocking`
//   (`RestWitness` reconstruction), never back through `Idle`, so the next
//   non-`Pending` result is always `RestComplete`. This is also covered
//   independently, at the `StationarityWork` level, by
//   `sweep_analysis::tests::scored_rest_contract_yields_diagnostic_before_conditioned_witness`,
//   which fails via `Option::replace` if `Diagnostic` is ever produced twice.
// - `RestAnalysis` is returned once, from the tail of that same
//   `AnalyzeSettle` visit, and immediately moves `runtime.state` to
//   `RecoveryState::RestTerminal`, which never returns to `AnalyzeSettle`.
//
// The tests below drive every legal outcome through `RecoveryFixture` and
// assert the exact diagnostic count each one commits, reusing the existing
// scenario setups rather than rebuilding them.

#[test]
fn recovery_diagnostic_count_already_centered() {
    let mut fixture = RecoveryFixture::new(1_000, None);
    let (terminal, selections, analyses) = fixture.drive_to_terminal();

    let RungOriginRecoveryStep::Finished(report) = terminal else {
        panic!("already-centered recovery must finish immediately: {terminal:?}");
    };
    assert_eq!(report.outcome, RungOriginRecoveryOutcome::AlreadyCentered);
    assert_eq!(selections, 0);
    assert_eq!(analyses, 0);
    assert!(selections + analyses <= 2);
}

#[test]
fn recovery_diagnostic_count_no_commanded_side_rate() {
    let mut fixture = RecoveryFixture::new(8_458, None);
    let (terminal, selections, analyses) = fixture.drive_to_terminal();

    let RungOriginRecoveryStep::Contained(report) = terminal else {
        panic!("missing commanded-side rate must remain contained: {terminal:?}");
    };
    assert_eq!(report.outcome, RungOriginRecoveryOutcome::ContainedNoRate);
    assert_eq!(selections, 0);
    assert_eq!(analyses, 0);
    assert!(selections + analyses <= 2);
}

#[test]
fn recovery_diagnostic_count_start_offset_outside_position_window() {
    // `position_window` under `configured_safety` is `max_stroke_travel_mrev`
    // (2_500) converted at 4_000 counts/rev, i.e. 10_000 counts. 11_040 trips
    // the `WriteGains` guard while still fitting the recovery deadline
    // (~1.97 ms of 3.0 ms), so the runtime reaches `WriteGains` instead of
    // being turned away at construction with `RecoveryBindingSource::Deadline`.
    let mut fixture = RecoveryFixture::new_with_source(
        11_040,
        Some(14_000_000),
        RecoveryBindingSource::SettledRate,
    );
    assert_eq!(fixture.safety.position_window, 10_000);
    let (terminal, selections, analyses) = fixture.drive_to_terminal();

    let RungOriginRecoveryStep::Contained(report) = terminal else {
        panic!("start offset outside position_window must remain contained: {terminal:?}");
    };
    assert_eq!(report.outcome, RungOriginRecoveryOutcome::ContainedDeadline);
    assert_eq!(report.binding_source, RecoveryBindingSource::LocalTravel);
    assert_eq!(selections, 0);
    assert_eq!(analyses, 0);
    assert!(selections + analyses <= 2);
}

#[test]
fn recovery_diagnostic_count_current_contact_before_settle_poll() {
    // Same setup as `recovery_current_observer_covers_motion_and_zero_target_settle`:
    // clamp contact is detected while finishing the moving phase, inside
    // `RecoveryState::Observe`, well before `AnalyzeSettle` is ever entered.
    let mut fixture = RecoveryFixture::new(1_040, Some(14_000_000));
    fixture.command_motion();
    let handover_us = fixture.arrive_at_moving_target(1_040);
    let first = handover_us / 1_000 + 1;
    assert!(matches!(fixture.step(), RungOriginRecoveryStep::At(next) if next == first * 1_000));

    let _ = fixture.backend.registers.insert(
        foci_tmc4671::registers::INTERIM_PIDOUT_TARGET_TORQUE
            .data
            .addr(),
        2_816,
    );
    fixture.backend.elapsed_us_value = handover_us + 1_000;
    fixture.backend.mechanical_encoder_position_counts_value =
        moving_arrival_counts().rem_euclid(4_000) as u32;
    let (terminal, selections, analyses) = fixture.drive_to_terminal();

    let RungOriginRecoveryStep::Contained(report) = terminal else {
        panic!("contact during the moving phase must remain contained: {terminal:?}");
    };
    assert_eq!(
        report.outcome,
        RungOriginRecoveryOutcome::RecoveryCurrentHeadroom
    );
    assert_eq!(selections, 0);
    assert_eq!(analyses, 0);
    assert!(selections + analyses <= 2);
}

#[test]
fn recovery_diagnostic_count_current_contact_after_settle_poll_immediate() {
    // Same timeline as `recovery_controller_confirms_fixed_settle_and_reports_complete_geometry`
    // (a normal, immediate-schedule completion), but the velocity-output
    // clamp status flag is only raised once the scored settle capture has
    // finished. `recovery_safety_sample` never reads `STATUS_FLAGS`, so this
    // register never trips contact detection during `CaptureSettle`; contact
    // is discovered only when `AnalyzeSettle` calls
    // `finish_velocity_current_zero_phase` after `poll_scored_rest_confirmation`
    // has already produced its one `RestSelection`.
    let mut fixture = RecoveryFixture::new(1_040, Some(14_000_000));
    fixture.command_motion();
    let handover_us = fixture.arrive_at_moving_target(1_040);
    let first = handover_us / 1_000 + 1;
    assert!(matches!(fixture.step(), RungOriginRecoveryStep::At(next) if next == first * 1_000));

    for sample in first..=first + 499 {
        let result = fixture.sample(sample * 1_000, moving_arrival_counts());
        if sample < first + 499 {
            assert!(matches!(result, RungOriginRecoveryStep::At(_)));
        } else {
            assert_eq!(result, RungOriginRecoveryStep::ReadyNow);
        }
    }
    let _ = fixture
        .backend
        .registers
        .insert(foci_tmc4671::registers::STATUS_FLAGS.addr(), 0x80);
    let (terminal, selections, analyses) = fixture.drive_to_terminal();

    let RungOriginRecoveryStep::Contained(report) = terminal else {
        panic!("contact discovered at the settle boundary must remain contained: {terminal:?}");
    };
    assert_eq!(
        report.outcome,
        RungOriginRecoveryOutcome::RecoveryCurrentHeadroom
    );
    assert_eq!(selections, 1);
    assert_eq!(analyses, 0, "ImmediateScored never yields RestAnalysis");
    assert!(selections + analyses <= 2);
}

#[test]
fn recovery_diagnostic_count_passive_settle_proven_after_contact() {
    // Same setup as `recovery_contact_passive_settle_proves_an_in_band_origin`:
    // `ProvePassiveSettle` marks contact detected in `Observe`, then still
    // runs the ordinary scored settle in `AnalyzeSettle`, so the run commits
    // one `RestSelection` but skips `RestAnalysis` (`ImmediateScored`).
    let mut fixture = RecoveryFixture::new_with_policy(
        1_040,
        Some(14_000_000),
        RecoveryBindingSource::SettledRate,
        RecoveryCurrentContactPolicy::ProvePassiveSettle,
    );
    fixture.command_motion();
    let _ = fixture.backend.registers.insert(
        foci_tmc4671::registers::INTERIM_PIDOUT_TARGET_TORQUE
            .data
            .addr(),
        2_816,
    );

    let handover_us = fixture.arrive_at_moving_target(1_040);
    assert_eq!(
        fixture.backend.registers.get(&PID_VELOCITY_TARGET.addr()),
        Some(&0)
    );
    let first = handover_us / 1_000 + 1;
    assert!(matches!(fixture.step(), RungOriginRecoveryStep::At(next) if next == first * 1_000));

    let _ = fixture.backend.registers.insert(
        foci_tmc4671::registers::INTERIM_PIDOUT_TARGET_TORQUE
            .data
            .addr(),
        100,
    );
    for sample in first..=first + 499 {
        let result = fixture.sample(sample * 1_000, 998);
        if sample < first + 499 {
            assert!(matches!(result, RungOriginRecoveryStep::At(_)));
        } else {
            assert_eq!(result, RungOriginRecoveryStep::ReadyNow);
        }
    }
    let (terminal, selections, analyses) = fixture.drive_to_terminal();

    let RungOriginRecoveryStep::Finished(report) = terminal else {
        panic!("passive in-band rest should recover: {terminal:?}");
    };
    assert_eq!(
        report.outcome,
        RungOriginRecoveryOutcome::RecoveredWithCurrentHeadroom
    );
    assert_eq!(selections, 1);
    assert_eq!(analyses, 0, "ImmediateScored never yields RestAnalysis");
    assert!(selections + analyses <= 2);
    assert!(
        selections + analyses >= 1,
        "outcome 5 must commit at least one diagnostic"
    );
}

#[test]
fn recovery_diagnostic_count_normal_completion_immediate() {
    // Same timeline as `recovery_controller_confirms_fixed_settle_and_reports_complete_geometry`.
    let mut fixture = RecoveryFixture::new(1_040, Some(14_000_000));
    fixture.command_motion();
    let handover_us = fixture.arrive_at_moving_target(1_040);
    let first = handover_us / 1_000 + 1;
    assert!(matches!(fixture.step(), RungOriginRecoveryStep::At(next) if next == first * 1_000));

    for sample in first..=first + 499 {
        let result = fixture.sample(sample * 1_000, moving_arrival_counts());
        if sample < first + 499 {
            assert!(matches!(result, RungOriginRecoveryStep::At(_)));
        } else {
            assert_eq!(result, RungOriginRecoveryStep::ReadyNow);
        }
    }
    let (terminal, selections, analyses) = fixture.drive_to_terminal();

    let RungOriginRecoveryStep::Finished(report) = terminal else {
        panic!("fixed rest should recover: {terminal:?}");
    };
    assert_eq!(report.outcome, RungOriginRecoveryOutcome::Recovered);
    assert_eq!(selections, 1);
    assert_eq!(analyses, 0, "ImmediateScored never yields RestAnalysis");
    assert!(selections + analyses <= 2);
    assert!(
        selections + analyses >= 1,
        "outcome 1 must commit at least one diagnostic"
    );
}

/// Pins the Step-3 structural argument: a `Stabilized` recovery that
/// completes normally commits exactly one `RestSelection` and one
/// `RestAnalysis`, never more.
///
/// Same timeline as `selected_recovery_discards_lead_in_before_scoring_rest`,
/// carried through to a confirmed, in-band completion.
#[test]
fn recovery_diagnostic_count_normal_completion_stabilized() {
    let mut fixture = RecoveryFixture::new_with_schedule(
        1_040,
        Some(14_000_000),
        RecoveryBindingSource::SettledRate,
        RecoveryCurrentContactPolicy::StopImmediately,
        RecoveryRestSchedule::Stabilized,
    );
    fixture.command_motion();
    let handover_us = fixture.arrive_at_moving_target(1_040);
    let first = handover_us / 1_000 + 1;
    assert!(matches!(fixture.step(), RungOriginRecoveryStep::At(next) if next == first * 1_000));

    for sample in first..=first + 499 {
        let offset = if sample % 2 == 0 {
            moving_arrival_counts() - 1
        } else {
            moving_arrival_counts()
        };
        let result = fixture.sample(sample * 1_000, offset);
        if sample < first + 499 {
            assert!(matches!(result, RungOriginRecoveryStep::At(_)));
        } else {
            assert_eq!(result, RungOriginRecoveryStep::At((first + 500) * 1_000));
        }
    }
    for sample in first + 500..=first + 999 {
        let offset = if sample % 2 == 0 {
            moving_arrival_counts() - 1
        } else {
            moving_arrival_counts()
        };
        let result = fixture.sample(sample * 1_000, offset);
        if sample < first + 999 {
            assert!(matches!(result, RungOriginRecoveryStep::At(_)));
        } else {
            assert_eq!(result, RungOriginRecoveryStep::ReadyNow);
        }
    }
    let (terminal, selections, analyses) = fixture.drive_to_terminal();

    let RungOriginRecoveryStep::Finished(report) = terminal else {
        panic!("confirmed stabilized rest should recover: {terminal:?}");
    };
    assert_eq!(report.outcome, RungOriginRecoveryOutcome::Recovered);
    assert_eq!(selections, 1);
    assert_eq!(analyses, 1);
    assert!(selections + analyses <= 2);
    assert!(
        selections + analyses >= 1,
        "outcome 1 must commit at least one diagnostic"
    );
}

#[test]
fn recovery_diagnostic_count_scored_rest_rejected_immediate() {
    // Same timeline as `recovery_safety_rejects_unconfirmed_rest`.
    let mut fixture = RecoveryFixture::new(1_020, Some(14_000_000));
    fixture.command_motion();
    let handover_us = fixture.arrive_at_moving_target(1_040);
    let first = handover_us / 1_000 + 1;
    assert!(matches!(fixture.step(), RungOriginRecoveryStep::At(next) if next == first * 1_000));
    for sample in first..=first + 499 {
        let offset = moving_arrival_counts() - (sample as i32 - first as i32);
        let result = fixture.sample(sample * 1_000, offset);
        if sample < first + 499 {
            assert!(matches!(result, RungOriginRecoveryStep::At(_)));
        } else {
            assert_eq!(result, RungOriginRecoveryStep::ReadyNow);
        }
    }
    let (terminal, selections, analyses) = fixture.drive_to_terminal();

    assert_eq!(
        terminal,
        RungOriginRecoveryStep::Fault {
            error: crate::autotune::CommissionError::VelocityRestNotConfirmed,
            safety: None,
        }
    );
    assert_eq!(selections, 1);
    assert_eq!(analyses, 0, "ImmediateScored never yields RestAnalysis");
    assert!(selections + analyses <= 2);
}

#[test]
fn recovery_diagnostic_count_scored_rest_rejected_stabilized() {
    // Same timeline as `selected_recovery_reports_completed_unconfirmed_rest_without_a_fault`.
    let mut fixture = RecoveryFixture::new_with_schedule(
        1_040,
        Some(14_000_000),
        RecoveryBindingSource::SettledRate,
        RecoveryCurrentContactPolicy::StopImmediately,
        RecoveryRestSchedule::Stabilized,
    );
    fixture.command_motion();
    let handover_us = fixture.arrive_at_moving_target(1_040);
    let first = handover_us / 1_000 + 1;
    assert!(matches!(fixture.step(), RungOriginRecoveryStep::At(next) if next == first * 1_000));
    for sample in first..=first + 499 {
        let result = fixture.sample(sample * 1_000, moving_arrival_counts());
        if sample < first + 499 {
            assert!(matches!(result, RungOriginRecoveryStep::At(_)));
        } else {
            assert_eq!(result, RungOriginRecoveryStep::At((first + 500) * 1_000));
        }
    }
    for sample in first + 500..=first + 999 {
        let offset = moving_arrival_counts() - (sample as i32 - (first as i32 + 499));
        let result = fixture.sample(sample * 1_000, offset);
        if sample < first + 999 {
            assert!(matches!(result, RungOriginRecoveryStep::At(_)));
        } else {
            assert_eq!(result, RungOriginRecoveryStep::ReadyNow);
        }
    }
    let (terminal, selections, analyses) = fixture.drive_to_terminal();

    assert_eq!(terminal, RungOriginRecoveryStep::RestUnconfirmed);
    assert_eq!(selections, 1);
    assert_eq!(analyses, 1);
    assert!(selections + analyses <= 2);
}
