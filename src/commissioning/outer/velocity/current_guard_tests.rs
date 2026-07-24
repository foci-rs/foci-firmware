use foci_tmc4671::current_limit::PidTorqueFluxLimit;

use super::current_guard::{
    VelocityCurrentClampSession, VelocityCurrentContact, VelocityCurrentObserver,
    VelocityCurrentZeroPhase, VelocityLimitEvidenceCapability, classify_phase_evidence,
    derive_probe_limit,
};
use super::integral_plan::{
    STAGE_C_EVIDENCE_SCHEMA_REVISION, StageCAuthority, plan_velocity_integral_response,
};
use super::reproduction::STAGE_B_EVIDENCE_SCHEMA_REVISION;
use super::sweep_plan::{RequestedVelocitySource, VelocitySweepRequest, plan_velocity_sweep};
use super::{
    VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT, VELOCITY_LIMIT_EVIDENCE_CAPABILITY_LATCHED_FLAG,
};
use crate::commission_dispatch::test_support::{MockBackend, SpiOp};
use crate::commissioning::outer::OuterMotionBudget;
use foci_tmc4671::registers::PID_TORQUE_FLUX_LIMITS;

const PID_V_OUTPUT_LIMIT: u32 = 1 << 7;

#[test]
fn velocity_current_guard_contract() {
    for (operational, expected_probe) in [(2_816, Some(2_534)), (10, Some(9)), (1, None)] {
        let result = derive_probe_limit(PidTorqueFluxLimit::new(operational).unwrap());
        assert_eq!(result.ok().map(PidTorqueFluxLimit::raw), expected_probe);
    }
    assert!(PidTorqueFluxLimit::new(0x8000).is_err());
    assert!(
        classify_phase_evidence(
            PidTorqueFluxLimit::ZERO,
            0,
            0,
            0,
            0,
            VelocityLimitEvidenceCapability::BestEffortPolling,
            VelocityCurrentZeroPhase::Braking,
        )
        .is_err()
    );

    let moving_flags = PID_V_OUTPUT_LIMIT | (1 << 3) | (1 << 26);
    let moving = classify_phase_evidence(
        PidTorqueFluxLimit::new(2_534).unwrap(),
        2_533,
        0,
        moving_flags,
        0,
        VelocityLimitEvidenceCapability::LatchedFlag,
        VelocityCurrentZeroPhase::Braking,
    )
    .unwrap();
    assert_eq!(moving.contact, VelocityCurrentContact::Moving);
    assert_eq!(moving.moving_status_flags, moving_flags);
    assert!(!moving.moving_clip_absence_proven());
    assert!(!moving.moving_clip_uncertain());

    let zero_flags = PID_V_OUTPUT_LIMIT | (1 << 15) | (1 << 26);
    let braking = classify_phase_evidence(
        PidTorqueFluxLimit::new(2_534).unwrap(),
        2_533,
        2_534,
        1 << 3,
        zero_flags,
        VelocityLimitEvidenceCapability::LatchedFlag,
        VelocityCurrentZeroPhase::Braking,
    )
    .unwrap();
    assert_eq!(braking.contact, VelocityCurrentContact::Braking);
    assert_eq!(braking.moving_status_flags, 1 << 3);
    assert_eq!(braking.zero_status_flags, zero_flags);
    assert!(braking.moving_clip_absence_proven());

    let recovery = classify_phase_evidence(
        PidTorqueFluxLimit::new(2_534).unwrap(),
        2_000,
        2_534,
        0,
        PID_V_OUTPUT_LIMIT,
        VelocityLimitEvidenceCapability::LatchedFlag,
        VelocityCurrentZeroPhase::Recovery,
    )
    .unwrap();
    assert_eq!(recovery.contact, VelocityCurrentContact::Recovery);

    let best_effort = classify_phase_evidence(
        PidTorqueFluxLimit::new(2_534).unwrap(),
        2_533,
        2_000,
        0,
        0,
        VelocityLimitEvidenceCapability::BestEffortPolling,
        VelocityCurrentZeroPhase::Braking,
    )
    .unwrap();
    assert_eq!(best_effort.contact, VelocityCurrentContact::None);
    assert!(!best_effort.moving_clip_absence_proven());
    assert!(best_effort.moving_clip_uncertain());

    // A 176-RPM target is inside one 375-RPM advanced-meter quantum. The
    // observed 0/375 samples imply signed errors 176/-199. At P=2048 the
    // sampled output peaks at 1592 counts, but a 1591-count 1 kHz sample cannot
    // exclude a between-sample clamp contact.
    let target_rpm = 176_i32;
    let advanced_meter_quantum_rpm = 375_i32;
    let p_raw = 2_048_i32;
    let spin_up_output = (p_raw * target_rpm / 256).unsigned_abs() as u16;
    let reversal_output =
        (p_raw * (target_rpm - advanced_meter_quantum_rpm) / 256).unsigned_abs() as u16;
    assert_eq!((spin_up_output, reversal_output), (1_408, 1_592));

    let latched_quantized = classify_phase_evidence(
        PidTorqueFluxLimit::new(1_592).unwrap(),
        1_591,
        0,
        PID_V_OUTPUT_LIMIT,
        0,
        VelocityLimitEvidenceCapability::LatchedFlag,
        VelocityCurrentZeroPhase::Braking,
    )
    .unwrap();
    assert_eq!(latched_quantized.contact, VelocityCurrentContact::Moving);

    let polled_quantized = classify_phase_evidence(
        PidTorqueFluxLimit::new(1_592).unwrap(),
        1_591,
        0,
        0,
        0,
        VelocityLimitEvidenceCapability::BestEffortPolling,
        VelocityCurrentZeroPhase::Braking,
    )
    .unwrap();
    assert_eq!(polled_quantized.contact, VelocityCurrentContact::None);
    assert!(polled_quantized.moving_clip_uncertain());

    let request = VelocitySweepRequest {
        requested_velocity_mrev_s: 6_666,
        machine_velocity_ceiling_mrev_s: 10_000,
        requested_velocity_source: RequestedVelocitySource::Defaulted,
        max_stroke_travel_mrev: 2_500,
        settle_travel_reserve_mrev: 250,
        negative_position_headroom_mrev: 7_025,
        positive_position_headroom_mrev: 7_025,
        max_duration_ms: 3_000,
    };
    let stage_b = plan_velocity_sweep(
        request,
        4_000,
        2_816,
        VELOCITY_LIMIT_EVIDENCE_CAPABILITY_LATCHED_FLAG,
    )
    .unwrap();
    assert_eq!(STAGE_B_EVIDENCE_SCHEMA_REVISION, 7);
    assert_eq!(
        stage_b.limit_evidence_capability,
        VelocityLimitEvidenceCapability::LatchedFlag
    );
    assert_eq!(
        stage_b
            .execution_plan(stage_b.digest())
            .limit_evidence_capability,
        VelocityLimitEvidenceCapability::LatchedFlag
    );
    let best_effort_stage_b = plan_velocity_sweep(
        request,
        4_000,
        2_816,
        VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT,
    )
    .unwrap();
    assert_ne!(stage_b.digest(), best_effort_stage_b.digest());

    let stage_c = plan_velocity_integral_response(
        StageCAuthority {
            channel: 0,
            stage_b_plan_digest: stage_b.digest(),
            final_p: 1_448,
            directional_memberships: [0x0038_0000, 0x0038_0000],
            joint_membership: 0x0038_0000,
            directional_pooled_intervals: [[17_000_000, 18_000_000], [-19_000_000, -18_000_000]],
        },
        stage_b.execution_plan(stage_b.digest()),
        OuterMotionBudget {
            requested_velocity_mrev_s: request.requested_velocity_mrev_s,
            machine_velocity_ceiling_mrev_s: request.machine_velocity_ceiling_mrev_s,
            requested_velocity_source: request.requested_velocity_source,
            max_stroke_travel_mrev: request.max_stroke_travel_mrev,
            settle_travel_reserve_mrev: request.settle_travel_reserve_mrev,
            negative_position_headroom_mrev: request.negative_position_headroom_mrev,
            positive_position_headroom_mrev: request.positive_position_headroom_mrev,
            max_duration_ms: request.max_duration_ms,
        },
        25_000,
        4_000,
    )
    .unwrap();
    assert_eq!(STAGE_C_EVIDENCE_SCHEMA_REVISION, 3);
    assert_eq!(
        stage_c.limit_evidence_capability,
        VelocityLimitEvidenceCapability::LatchedFlag
    );
    assert_eq!(
        stage_c.execution_plan().limit_evidence_capability,
        VelocityLimitEvidenceCapability::LatchedFlag
    );
    let mut best_effort_stage_c = stage_c;
    best_effort_stage_c.limit_evidence_capability =
        VelocityLimitEvidenceCapability::BestEffortPolling;
    assert_ne!(stage_c.digest(), best_effort_stage_c.digest());
}

#[test]
fn velocity_current_prevention_configuration_matrix() {
    const SUPPORTED_PWM_HZ: [u32; 1] = [25_000];
    const SUPPORTED_VELOCITY_METERS: [u8; 1] = [1];
    const SUPPORTED_VELOCITY_FILTERS_ENABLED: [bool; 2] = [false, true];

    for encoder_counts_per_rev in [1_000_u32, 65_536] {
        for pwm_hz in SUPPORTED_PWM_HZ {
            let meter_quantum_rpm =
                u32::try_from((u64::from(pwm_hz) * 60).div_ceil(u64::from(encoder_counts_per_rev)))
                    .unwrap();
            for target_rpm in [
                meter_quantum_rpm.saturating_sub(1).max(1),
                meter_quantum_rpm,
                meter_quantum_rpm + 1,
            ] {
                for velocity_meter in SUPPORTED_VELOCITY_METERS {
                    for filter_enabled in SUPPORTED_VELOCITY_FILTERS_ENABLED {
                        for operational_limit in [10_u32, 32_767] {
                            for (bed_mm, max_stroke_travel_mrev, headroom_mrev) in
                                [(120_u16, 2_500_u32, 3_000_u32), (300, 6_250, 7_500)]
                            {
                                let probe = derive_probe_limit(
                                    PidTorqueFluxLimit::new(operational_limit).unwrap(),
                                )
                                .unwrap();
                                assert_eq!(
                                    u32::from(probe.raw()),
                                    operational_limit * 9 / 10,
                                    "encoder={encoder_counts_per_rev} pwm={pwm_hz} \
                                     target={target_rpm} meter={velocity_meter} \
                                     filter={filter_enabled} bed={bed_mm}"
                                );

                                let request = VelocitySweepRequest {
                                    requested_velocity_mrev_s: 6_666,
                                    machine_velocity_ceiling_mrev_s: 10_000,
                                    requested_velocity_source: RequestedVelocitySource::Defaulted,
                                    max_stroke_travel_mrev,
                                    settle_travel_reserve_mrev: 250,
                                    negative_position_headroom_mrev: headroom_mrev,
                                    positive_position_headroom_mrev: headroom_mrev,
                                    max_duration_ms: 3_000,
                                };
                                let planned = plan_velocity_sweep(
                                    request,
                                    encoder_counts_per_rev,
                                    u16::try_from(operational_limit).unwrap(),
                                    VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT,
                                );
                                let plan = planned.unwrap_or_else(|error| {
                                    panic!(
                                        "matrix plan failed: encoder={encoder_counts_per_rev} \
                                         pwm={pwm_hz} target={target_rpm} \
                                         meter={velocity_meter} filter={filter_enabled} \
                                         limit={operational_limit} bed={bed_mm}: {error:?}"
                                    )
                                });
                                assert_eq!(plan.usable_torque_limit, probe.raw());

                                let below = classify_phase_evidence(
                                    probe,
                                    probe.raw() - 1,
                                    0,
                                    0,
                                    0,
                                    VelocityLimitEvidenceCapability::LatchedFlag,
                                    VelocityCurrentZeroPhase::Braking,
                                )
                                .unwrap();
                                assert_eq!(below.contact, VelocityCurrentContact::None);
                                assert!(below.moving_clip_absence_proven());

                                let at_boundary = classify_phase_evidence(
                                    probe,
                                    probe.raw(),
                                    0,
                                    0,
                                    0,
                                    VelocityLimitEvidenceCapability::LatchedFlag,
                                    VelocityCurrentZeroPhase::Braking,
                                )
                                .unwrap();
                                assert_eq!(at_boundary.contact, VelocityCurrentContact::Moving);

                                let p_raw = 2_048_i64;
                                let spin_up_output =
                                    (p_raw * i64::from(target_rpm) / 256).unsigned_abs();
                                let reversal_output = (p_raw
                                    * (i64::from(target_rpm) - i64::from(meter_quantum_rpm))
                                    / 256)
                                    .unsigned_abs();
                                let unconstrained_peak = spin_up_output.max(reversal_output);
                                let sampled_peak =
                                    unconstrained_peak.min(u64::from(probe.raw())) as u16;
                                let flags = if unconstrained_peak >= u64::from(probe.raw()) {
                                    PID_V_OUTPUT_LIMIT
                                } else {
                                    0
                                };
                                let quantized = classify_phase_evidence(
                                    probe,
                                    sampled_peak,
                                    0,
                                    flags,
                                    0,
                                    VelocityLimitEvidenceCapability::LatchedFlag,
                                    VelocityCurrentZeroPhase::Braking,
                                )
                                .unwrap();
                                assert_eq!(
                                    quantized.contact == VelocityCurrentContact::Moving,
                                    unconstrained_peak >= u64::from(probe.raw()),
                                    "encoder={encoder_counts_per_rev} pwm={pwm_hz} \
                                     target={target_rpm} meter={velocity_meter} \
                                     filter={filter_enabled} limit={operational_limit} \
                                     bed={bed_mm}"
                                );
                            }
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn velocity_current_clamp_session() {
    pollster::block_on(async {
        let mut session = VelocityCurrentClampSession::try_new(2_816).unwrap();
        assert_eq!(session.operational_limit().raw(), 2_816);
        assert_eq!(session.probe_limit().raw(), 2_534);
        assert!(!session.probe_is_installed());

        let mut backend = MockBackend::new();
        session.install_probe_limit(&mut backend).await.unwrap();
        assert!(session.probe_is_installed());
        assert!(matches!(
            backend.spi_log.as_slice(),
            [
                SpiOp::Write(addr, 2_534),
                SpiOp::Read,
            ] if *addr == PID_TORQUE_FLUX_LIMITS.addr()
        ));

        let retained_log_len = backend.spi_log.len();
        assert!(session.probe_is_installed());
        assert_eq!(backend.spi_log.len(), retained_log_len);

        session.install_probe_limit(&mut backend).await.unwrap();
        assert!(session.probe_is_installed());
        assert_eq!(backend.spi_log.len(), retained_log_len + 2);

        session
            .restore_operational_limit(&mut backend)
            .await
            .unwrap();
        assert!(!session.probe_is_installed());
        assert!(matches!(
            &backend.spi_log.as_slice()[retained_log_len + 2..],
            [
                SpiOp::Write(addr, 2_816),
                SpiOp::Read,
            ] if *addr == PID_TORQUE_FLUX_LIMITS.addr()
        ));

        #[derive(Clone, Copy)]
        enum FailureMode {
            Write,
            Read,
            CompleteWordMismatch,
        }
        for failure in [
            FailureMode::Write,
            FailureMode::Read,
            FailureMode::CompleteWordMismatch,
        ] {
            let mut session = VelocityCurrentClampSession::try_new(2_816).unwrap();
            let mut backend = MockBackend::new();
            match failure {
                FailureMode::Write => {
                    backend.fail_on_write = Some(PID_TORQUE_FLUX_LIMITS.addr());
                }
                FailureMode::Read => {
                    backend.fail_on_read = Some(PID_TORQUE_FLUX_LIMITS.addr());
                }
                FailureMode::CompleteWordMismatch => {
                    backend.read_override = Some((PID_TORQUE_FLUX_LIMITS.addr(), 0x0001_09e6));
                }
            }
            assert!(session.install_probe_limit(&mut backend).await.is_err());
            assert!(!session.probe_is_installed());
        }

        for failure in [
            FailureMode::Write,
            FailureMode::Read,
            FailureMode::CompleteWordMismatch,
        ] {
            let mut session = VelocityCurrentClampSession::try_new(2_816).unwrap();
            let mut backend = MockBackend::new();
            session.install_probe_limit(&mut backend).await.unwrap();
            match failure {
                FailureMode::Write => {
                    backend.fail_on_write = Some(PID_TORQUE_FLUX_LIMITS.addr());
                }
                FailureMode::Read => {
                    backend.fail_on_read = Some(PID_TORQUE_FLUX_LIMITS.addr());
                }
                FailureMode::CompleteWordMismatch => {
                    backend.read_override = Some((PID_TORQUE_FLUX_LIMITS.addr(), 0x0001_0b00));
                }
            }
            assert!(
                session
                    .restore_operational_limit(&mut backend)
                    .await
                    .is_err()
            );
            assert!(session.probe_is_installed());
        }

        for invalid in [0, 1, 0x8000, u32::MAX] {
            assert!(VelocityCurrentClampSession::try_new(invalid).is_err());
        }
    });
}

#[test]
fn velocity_current_observer_attributes_phases() {
    let limit = PidTorqueFluxLimit::new(2_816).unwrap();
    let mut observer =
        VelocityCurrentObserver::new(limit, VelocityLimitEvidenceCapability::LatchedFlag);

    for peak in [100, 2_815, 800] {
        observer.observe_moving_output(peak);
    }
    observer.observe_moving_flags((1 << 3) | (1 << 26));
    observer.observe_zero_output(2_000);
    observer.observe_zero_flags(1 << 15);
    let clear = observer
        .evidence(VelocityCurrentZeroPhase::Braking)
        .unwrap();
    assert_eq!(clear.contact, VelocityCurrentContact::None);
    assert_eq!(clear.moving_pid_output_peak, 2_815);
    assert_eq!(clear.zero_pid_output_peak, 2_000);
    assert_eq!(clear.moving_status_flags, (1 << 3) | (1 << 26));
    assert_eq!(clear.zero_status_flags, 1 << 15);
    assert!(clear.moving_clip_absence_proven());

    for contact_index in 0..3 {
        observer.reset(limit, VelocityLimitEvidenceCapability::LatchedFlag);
        for (index, peak) in [100, 200, 300].into_iter().enumerate() {
            observer.observe_moving_output(if index == contact_index {
                limit.raw()
            } else {
                peak
            });
        }
        assert_eq!(
            observer
                .evidence(VelocityCurrentZeroPhase::Braking)
                .unwrap()
                .contact,
            VelocityCurrentContact::Moving
        );
    }

    observer.reset(limit, VelocityLimitEvidenceCapability::LatchedFlag);
    observer.observe_moving_output(2_815);
    observer.observe_zero_flags(PID_V_OUTPUT_LIMIT | (1 << 15));
    let braking = observer
        .evidence(VelocityCurrentZeroPhase::Braking)
        .unwrap();
    assert_eq!(braking.contact, VelocityCurrentContact::Braking);
    assert_eq!(braking.zero_status_flags, PID_V_OUTPUT_LIMIT | (1 << 15));

    observer.reset(limit, VelocityLimitEvidenceCapability::LatchedFlag);
    observer.observe_zero_output(limit.raw());
    assert_eq!(
        observer
            .evidence(VelocityCurrentZeroPhase::Recovery)
            .unwrap()
            .contact,
        VelocityCurrentContact::Recovery
    );

    observer.reset(limit, VelocityLimitEvidenceCapability::BestEffortPolling);
    observer.observe_moving_output(2_815);
    let best_effort = observer
        .evidence(VelocityCurrentZeroPhase::Braking)
        .unwrap();
    assert_eq!(best_effort.contact, VelocityCurrentContact::None);
    assert!(best_effort.moving_clip_uncertain());
}
