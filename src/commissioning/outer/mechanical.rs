//! Phase 8: Mechanical identification.
//!
//! Bidirectional torque-step procedure that estimates effective inertia,
//! damping, mechanical time constant, and transport delay from encoder
//! response. Forward and reverse captures are compared for symmetry; if
//! the captures fail the acceptance gates the phase retries with a
//! larger torque step (up to `MECHANICAL_ID_MAX_ATTEMPTS`) before
//! falling back to the response-ramp velocity tuning.

#![allow(clippy::items_after_test_module)]

use super::{
    MechanicalIdData, MechanicalIdState, OUTER_WARNING_NONE, OuterDiagnostics, OuterPhase,
    TraceSummarySeqs, trace_diagnostic_warning, trace_pack_i16_pair, trace_pack_u16_pair,
    trace_summary_event,
};
use crate::autotune::{CommissionError, MechanicalModel, PhaseId, TransientBuffer};
use crate::commissioning::outer::{fail_outer, fail_outer_with_safety, response};
use crate::commissioning::types::{
    CommissioningBackend, OUTER_SAFETY_FAULT_POST_SWITCH_SETTLE,
    OUTER_SAFETY_FAULT_UNUSABLE_BUDGET, StepResult,
};
use crate::trace::events::{
    EVENT_COMMISSION_MECHANICAL_ID_ATTEMPT, EVENT_COMMISSION_MECHANICAL_ID_CLASSIFICATION,
    EVENT_COMMISSION_MECHANICAL_ID_RESPONSE, EVENT_COMMISSION_MECHANICAL_ID_RESULT,
    TraceEventLevel,
};

/// Settling time after removing torque (microseconds).
/// Allow the motor to decelerate and stop before reverse capture.
const MECHANICAL_ID_SETTLE_US: u64 = 500_000;

/// Phase-local settle tuning for the position->torque mode switch.
///
/// These values are still firmware constants today, but grouping them makes
/// it straightforward to promote them into a future board/profile tuning
/// surface without touching the state-machine logic.
struct MechanicalIdModeSwitchSettlePolicy {
    grace_us: u64,
    timeout_us: u64,
    stable_ticks: u8,
}

const MECHANICAL_ID_MODE_SWITCH_SETTLE_POLICY: MechanicalIdModeSwitchSettlePolicy =
    MechanicalIdModeSwitchSettlePolicy {
        // Ignore the immediate position->torque transient before sampling settle velocity.
        grace_us: 10_000,
        // Fail cleanly if the unloaded axis never comes to rest after the mode switch.
        timeout_us: 100_000,
        // Require a few consecutive quiet samples before arming MechanicalId safety.
        stable_ticks: 3,
    };

/// Forward/reverse agreement threshold (30% = 30/100).
/// If `|J_fwd - J_rev| / avg > 30%`, mechanical ID fails (or retries).
const MECHANICAL_ID_SYMMETRY_THRESHOLD_PCT: u32 = 30;

/// Operational PID output voltage limit for outer commissioning.
///
/// Matches closed-loop entry so torque, velocity, and position tests are
/// not constrained by the conservative safe-state ceiling.
const OUTER_PIDOUT_LIMIT: u32 = 16000;

/// Torque mode with `MODE_PID_TYPE` set for the TMC4671 advanced PI structure.
///
/// Current-loop commissioning writes the advanced flux/torque PI registers, so
/// validation and mechanical ID must exercise that same controller path.
const CURRENT_TORQUE_MODE: u32 = 0x8000_0001;

fn record_mechanical_id_summary<B: CommissioningBackend>(
    backend: &mut B,
    trace_summary_seqs: &mut TraceSummarySeqs,
    data: &MechanicalIdData,
    warning_code: u8,
    result_status: u8,
    level: TraceEventLevel,
) {
    let summary_seq = trace_summary_seqs.next_mechanical_id();
    trace_summary_event(
        backend,
        level,
        EVENT_COMMISSION_MECHANICAL_ID_RESULT,
        summary_seq,
        i32::from(result_status),
    );
    trace_summary_event(
        backend,
        level,
        EVENT_COMMISSION_MECHANICAL_ID_ATTEMPT,
        summary_seq,
        i32::from(data.attempt),
    );
    trace_summary_event(
        backend,
        level,
        EVENT_COMMISSION_MECHANICAL_ID_RESPONSE,
        summary_seq,
        trace_pack_i16_pair(data.forward_peak_velocity, data.reverse_peak_velocity),
    );
    trace_summary_event(
        backend,
        level,
        EVENT_COMMISSION_MECHANICAL_ID_CLASSIFICATION,
        summary_seq,
        trace_pack_u16_pair(u32::from(warning_code), u32::from(data.torque_step)),
    );
    trace_diagnostic_warning(backend, warning_code, data.torque_step);
}

fn mechanical_retry_warning_code(data: &MechanicalIdData) -> u8 {
    if data.attempt + 1 >= response::MECHANICAL_ID_MAX_ATTEMPTS {
        response::OUTER_WARNING_LOW_CONFIDENCE_MECHANICAL_ID
    } else {
        OUTER_WARNING_NONE
    }
}

fn derive_mechanical_tau_with_default_flag(j_eff: u32, b_eff_raw: u32) -> (u32, u32, bool) {
    let tau_ratio = j_eff.checked_div(b_eff_raw);
    let tau_defaulted = tau_ratio.is_none();
    let b_eff = b_eff_raw.max(1);
    let tau_mech_ms = tau_ratio.unwrap_or(1000);
    (b_eff, tau_mech_ms, tau_defaulted)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::autotune::{CommissionProfile, OperatingMode};
    use crate::commission_dispatch::test_support::{MockBackend, SpiOp};
    use crate::commissioning::outer::{OuterSafetyLimits, step_outer};
    use crate::trace::events::EVENT_COMMISSION_DIAGNOSTIC_WARNING;
    use foci_tmc4671::registers::{ABN_DECODER_PPR, MODE_RAMP_MODE_MOTION, PID_TORQUE_FLUX_TARGET};

    #[test]
    fn mechanical_id_marks_tau_defaulted_when_raw_b_eff_zero() {
        let (b_eff, tau_mech_ms, tau_defaulted) =
            derive_mechanical_tau_with_default_flag(100_000, 0);

        assert_eq!(b_eff, 1);
        assert_eq!(tau_mech_ms, 1000);
        assert!(tau_defaulted);
    }

    #[test]
    fn mechanical_id_no_motion_sets_low_confidence_warning() {
        assert!(response::mechanical_id_low_confidence(0, 0, 0, 0, 0, 0));
    }

    #[test]
    fn mechanical_id_summary_packs_exact_fragments() {
        let mut backend = MockBackend::new();
        let mut seqs = TraceSummarySeqs::new();
        let mut data = MechanicalIdData::new(300);
        data.attempt = 2;
        data.forward_peak_velocity = 123;
        data.reverse_peak_velocity = -45;

        record_mechanical_id_summary(
            &mut backend,
            &mut seqs,
            &data,
            response::OUTER_WARNING_LOW_CONFIDENCE_MECHANICAL_ID,
            0,
            TraceEventLevel::Warn,
        );

        assert_eq!(
            backend.trace_events[0].event_id,
            EVENT_COMMISSION_MECHANICAL_ID_RESULT
        );
        assert_eq!(backend.trace_events[0].payload, [0, 0]);
        assert_eq!(
            backend.trace_events[1].event_id,
            EVENT_COMMISSION_MECHANICAL_ID_ATTEMPT
        );
        assert_eq!(backend.trace_events[1].payload, [0, 2]);
        assert_eq!(
            backend.trace_events[2].event_id,
            EVENT_COMMISSION_MECHANICAL_ID_RESPONSE
        );
        assert_eq!(
            backend.trace_events[2].payload,
            [0, trace_pack_i16_pair(123, -45)]
        );
        assert_eq!(
            backend.trace_events[3].event_id,
            EVENT_COMMISSION_MECHANICAL_ID_CLASSIFICATION
        );
        assert_eq!(
            backend.trace_events[3].payload,
            [
                0,
                trace_pack_u16_pair(
                    u32::from(response::OUTER_WARNING_LOW_CONFIDENCE_MECHANICAL_ID),
                    300,
                )
            ]
        );
        assert_eq!(
            backend.trace_events[4].event_id,
            EVENT_COMMISSION_DIAGNOSTIC_WARNING
        );
        assert_eq!(
            backend.trace_events[4].payload,
            [
                i32::from(response::OUTER_WARNING_LOW_CONFIDENCE_MECHANICAL_ID),
                300,
            ]
        );
    }

    #[test]
    fn mechanical_id_setup_zeroes_torque_before_switching_modes() {
        pollster::block_on(async {
            let mut phase = OuterPhase::MechanicalId {
                state: MechanicalIdState::Setup,
                data: MechanicalIdData::new(0),
                safety: OuterSafetyLimits::new(
                    0,
                    crate::commissioning::outer::OuterMotionBudget::firmware_caps(),
                ),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();
            backend.current_limit_value = 2448;
            backend.mechanical_encoder_position_counts_value = 1234;
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
    fn mechanical_id_rejects_invalid_budget_before_actuation() {
        pollster::block_on(async {
            let mut phase = OuterPhase::MechanicalId {
                state: MechanicalIdState::Setup,
                data: MechanicalIdData::new(0),
                safety: OuterSafetyLimits::new(
                    0,
                    crate::commissioning::outer::OuterMotionBudget {
                        max_travel_mrev: 500,
                        max_velocity_mrev_s: 6_000,
                        max_duration_ms: 3_000,
                        direction_mask: 1,
                    },
                ),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();
            backend.current_limit_value = 2448;
            backend.mechanical_encoder_position_counts_value = 1234;
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
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::OuterFailed(CommissionError::SafetyEnvelopeViolation)
            ));
            assert!(!backend.enable_pin);
            assert!(
                !backend
                    .spi_log
                    .iter()
                    .any(|op| matches!(op, SpiOp::EnablePin(true)))
            );
            assert!(!backend.spi_log.iter().any(|op| matches!(
                op,
                SpiOp::Write(addr, val)
                    if *addr == PID_TORQUE_FLUX_TARGET.addr() && (*val >> 16) != 0
            )));
        });
    }

    #[test]
    fn mechanical_id_rejects_invalid_encoder_domain_before_actuation() {
        pollster::block_on(async {
            let mut phase = OuterPhase::MechanicalId {
                state: MechanicalIdState::Setup,
                data: MechanicalIdData::new(0),
                safety: OuterSafetyLimits::new(
                    0,
                    crate::commissioning::outer::OuterMotionBudget::firmware_caps(),
                ),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();
            backend.current_limit_value = 2448;
            backend.mechanical_encoder_position_counts_value = 1234;
            let _ = backend.registers.insert(ABN_DECODER_PPR.addr(), 0);

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
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::OuterFailed(CommissionError::EncoderFault)
            ));
            assert!(!backend.enable_pin);
            assert!(!backend.spi_log.iter().any(|op| matches!(
                op,
                SpiOp::Write(addr, _) if *addr == MODE_RAMP_MODE_MOTION.addr()
            )));
            assert!(!backend.spi_log.iter().any(|op| matches!(
                op,
                SpiOp::Write(addr, val)
                    if *addr == PID_TORQUE_FLUX_TARGET.addr() && (*val >> 16) != 0
            )));
        });
    }

    #[test]
    fn mechanical_id_rejects_tiny_budget_before_actuation() {
        pollster::block_on(async {
            let mut phase = OuterPhase::MechanicalId {
                state: MechanicalIdState::Setup,
                data: MechanicalIdData::new(0),
                safety: OuterSafetyLimits::new(
                    0,
                    crate::commissioning::outer::OuterMotionBudget {
                        max_travel_mrev: 1,
                        max_velocity_mrev_s: 6_000,
                        max_duration_ms: 3_000,
                        direction_mask: 3,
                    },
                ),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();
            backend.current_limit_value = 2448;
            backend.mechanical_encoder_position_counts_value = 1234;
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
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::OuterFailed(CommissionError::SafetyEnvelopeViolation)
            ));
            assert!(!backend.enable_pin);
            assert!(
                !backend
                    .spi_log
                    .iter()
                    .any(|op| matches!(op, SpiOp::EnablePin(true)))
            );
            assert!(!backend.spi_log.iter().any(|op| matches!(
                op,
                SpiOp::Write(addr, val)
                    if *addr == PID_TORQUE_FLUX_TARGET.addr() && (*val >> 16) != 0
            )));
        });
    }

    #[test]
    fn mechanical_id_ignores_immediate_post_switch_velocity_spike() {
        pollster::block_on(async {
            let mut phase = OuterPhase::MechanicalId {
                state: MechanicalIdState::Setup,
                data: MechanicalIdData::new(0),
                safety: OuterSafetyLimits::new(
                    0,
                    crate::commissioning::outer::OuterMotionBudget::firmware_caps(),
                ),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();
            backend.current_limit_value = 2448;
            backend.mechanical_encoder_position_counts_value = 1000;
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
                &mut backend,
            )
            .await;
            assert!(matches!(
                first,
                StepResult::PhaseChanged(PhaseId::MechanicalId)
            ));

            // Reproduce the field failure: the first sample after the position->torque
            // mode switch jumps by 25 counts/ms before the intended torque step starts.
            backend.mechanical_encoder_position_counts_value = 1025;

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
                data: MechanicalIdData::new(0),
                safety: OuterSafetyLimits::new(
                    0,
                    crate::commissioning::outer::OuterMotionBudget::firmware_caps(),
                ),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();
            backend.current_limit_value = 2448;
            backend.mechanical_encoder_position_counts_value = 1000;
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
                &mut backend,
            )
            .await;
            assert!(matches!(
                first,
                StepResult::PhaseChanged(PhaseId::MechanicalId)
            ));

            backend.elapsed_us_value = 10_000;
            backend.mechanical_encoder_position_counts_value = 1025;
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
                backend.mechanical_encoder_position_counts_value = pos;
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
                    assert_eq!(
                        safety.prev_encoder_count.map(|count| count.raw_counts()),
                        Some(1028)
                    );
                    assert_eq!(
                        data.start_position.map(|count| count.raw_counts()),
                        Some(1027)
                    );
                    assert_eq!(safety.primitive_start_us, Some(12_000));
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
    fn mechanical_id_post_switch_settle_accepts_origin_crossing() {
        pollster::block_on(async {
            let mut safety = OuterSafetyLimits::new(
                0,
                crate::commissioning::outer::OuterMotionBudget::firmware_caps(),
            );
            safety.configure_encoder_resolution(4_000).unwrap();
            safety.set_encoder_baseline(3_998).unwrap();
            safety.last_sample_us = Some(10_000);
            let mut data = MechanicalIdData::new(100);
            data.start_position = safety.prev_encoder_count;
            let mut phase = OuterPhase::MechanicalId {
                state: MechanicalIdState::PostSwitchSettle {
                    earliest_sample_us: 11_000,
                    timeout_deadline_us: 100_000,
                    stable_ticks: 0,
                },
                data,
                safety,
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 11_000;
            backend.mechanical_encoder_position_counts_value = 2;

            let result = step_mechanical_id(&mut phase, &mut buf, &mut backend).await;

            assert!(matches!(result, StepResult::Continue));
            match phase {
                OuterPhase::MechanicalId { state, safety, .. } => {
                    assert!(matches!(
                        state,
                        MechanicalIdState::PostSwitchSettle {
                            stable_ticks: 0,
                            ..
                        }
                    ));
                    assert_eq!(
                        safety.prev_encoder_count.map(|count| count.raw_counts()),
                        Some(2)
                    );
                }
                _ => panic!("expected MechanicalId phase"),
            }
        });
    }

    #[test]
    fn mechanical_id_post_switch_half_turn_stops_without_advancing_baseline() {
        pollster::block_on(async {
            let mut safety = OuterSafetyLimits::new(
                0,
                crate::commissioning::outer::OuterMotionBudget::firmware_caps(),
            );
            safety.configure_encoder_resolution(4_000).unwrap();
            safety.set_encoder_baseline(0).unwrap();
            safety.last_sample_us = Some(10_000);
            let mut data = MechanicalIdData::new(100);
            data.start_position = safety.prev_encoder_count;
            let mut phase = OuterPhase::MechanicalId {
                state: MechanicalIdState::PostSwitchSettle {
                    earliest_sample_us: 11_000,
                    timeout_deadline_us: 100_000,
                    stable_ticks: 0,
                },
                data,
                safety,
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 11_000;
            backend.mechanical_encoder_position_counts_value = 2_000;

            let result = step_mechanical_id(&mut phase, &mut buf, &mut backend).await;

            assert!(matches!(
                result,
                StepResult::OuterFailed(CommissionError::EncoderFault)
            ));
            assert_eq!(buf.len(), 0);
            assert!(backend.spi_log.iter().any(|op| matches!(
                op,
                SpiOp::Write(addr, 0) if *addr == PID_TORQUE_FLUX_TARGET.addr()
            )));
            assert!(backend.spi_log.iter().any(|op| matches!(
                op,
                SpiOp::Write(addr, 0) if *addr == MODE_RAMP_MODE_MOTION.addr()
            )));
        });
    }

    #[test]
    fn mechanical_id_capture_accepts_origin_crossing() {
        pollster::block_on(async {
            let mut safety = OuterSafetyLimits::new(
                0,
                crate::commissioning::outer::OuterMotionBudget::firmware_caps(),
            );
            safety.configure_encoder_resolution(4_000).unwrap();
            let domain = safety.encoder_domain.unwrap();
            let mut data = MechanicalIdData::new(100);
            data.start_position = Some(domain.count(3_998));
            let mut phase = OuterPhase::MechanicalId {
                state: MechanicalIdState::ForwardCapture {
                    next_sample_us: 1_000,
                    capture_start_us: 0,
                },
                data,
                safety,
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 1_000;
            backend.mechanical_encoder_position_counts_value = 2;

            let result = step_mechanical_id(&mut phase, &mut buf, &mut backend).await;

            assert!(matches!(result, StepResult::Continue));
            assert_eq!(buf.samples(), &[2]);
        });
    }

    #[test]
    fn mechanical_id_capture_half_turn_stops_without_appending_sample() {
        pollster::block_on(async {
            let mut safety = OuterSafetyLimits::new(
                0,
                crate::commissioning::outer::OuterMotionBudget::firmware_caps(),
            );
            safety.configure_encoder_resolution(4_000).unwrap();
            let domain = safety.encoder_domain.unwrap();
            let mut data = MechanicalIdData::new(100);
            data.start_position = Some(domain.count(0));
            let mut phase = OuterPhase::MechanicalId {
                state: MechanicalIdState::ForwardCapture {
                    next_sample_us: 1_000,
                    capture_start_us: 0,
                },
                data,
                safety,
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 1_000;
            backend.mechanical_encoder_position_counts_value = 2_000;

            let result = step_mechanical_id(&mut phase, &mut buf, &mut backend).await;

            assert!(matches!(
                result,
                StepResult::OuterFailed(CommissionError::EncoderFault)
            ));
            assert_eq!(buf.len(), 0);
            assert!(backend.spi_log.iter().any(|op| matches!(
                op,
                SpiOp::Write(addr, 0) if *addr == PID_TORQUE_FLUX_TARGET.addr()
            )));
            assert!(backend.spi_log.iter().any(|op| matches!(
                op,
                SpiOp::Write(addr, 0) if *addr == MODE_RAMP_MODE_MOTION.addr()
            )));
        });
    }

    #[test]
    fn mechanical_id_reverse_capture_accepts_origin_crossing_from_reverse_baseline() {
        pollster::block_on(async {
            let mut safety = OuterSafetyLimits::new(
                0,
                crate::commissioning::outer::OuterMotionBudget::firmware_caps(),
            );
            safety.configure_encoder_resolution(4_000).unwrap();
            let domain = safety.encoder_domain.unwrap();
            let mut data = MechanicalIdData::new(100);
            data.start_position = Some(domain.count(1_000));
            let mut phase = OuterPhase::MechanicalId {
                state: MechanicalIdState::ReverseStart,
                data,
                safety,
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();
            backend.mechanical_encoder_position_counts_value = 3_998;

            let start_result = step_mechanical_id(&mut phase, &mut buf, &mut backend).await;

            assert!(matches!(start_result, StepResult::Continue));
            match &phase {
                OuterPhase::MechanicalId { data, .. } => assert_eq!(
                    data.start_position.map(|count| count.raw_counts()),
                    Some(3_998)
                ),
                _ => panic!("expected MechanicalId phase"),
            }

            backend.elapsed_us_value = response::MECHANICAL_ID_SAMPLE_PERIOD_US;
            backend.mechanical_encoder_position_counts_value = 2;
            let capture_result = step_mechanical_id(&mut phase, &mut buf, &mut backend).await;

            assert!(matches!(capture_result, StepResult::Continue));
            assert_eq!(buf.samples(), &[2]);
        });
    }

    #[test]
    fn mechanical_id_reverse_half_turn_stops_without_appending_sample() {
        pollster::block_on(async {
            let mut safety = OuterSafetyLimits::new(
                0,
                crate::commissioning::outer::OuterMotionBudget::firmware_caps(),
            );
            safety.configure_encoder_resolution(4_000).unwrap();
            let domain = safety.encoder_domain.unwrap();
            let mut data = MechanicalIdData::new(100);
            data.start_position = Some(domain.count(1_000));
            let mut phase = OuterPhase::MechanicalId {
                state: MechanicalIdState::ReverseStart,
                data,
                safety,
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();
            backend.mechanical_encoder_position_counts_value = 0;
            let _ = step_mechanical_id(&mut phase, &mut buf, &mut backend).await;

            backend.elapsed_us_value = response::MECHANICAL_ID_SAMPLE_PERIOD_US;
            backend.mechanical_encoder_position_counts_value = 2_000;
            let result = step_mechanical_id(&mut phase, &mut buf, &mut backend).await;

            assert!(matches!(
                result,
                StepResult::OuterFailed(CommissionError::EncoderFault)
            ));
            assert!(matches!(
                phase,
                OuterPhase::Failed(CommissionError::EncoderFault)
            ));
            assert_eq!(buf.len(), 0);
            assert!(backend.spi_log.iter().any(|op| matches!(
                op,
                SpiOp::Write(addr, 0) if *addr == PID_TORQUE_FLUX_TARGET.addr()
            )));
            assert!(backend.spi_log.iter().any(|op| matches!(
                op,
                SpiOp::Write(addr, 0) if *addr == MODE_RAMP_MODE_MOTION.addr()
            )));
        });
    }

    #[test]
    fn mechanical_id_fails_if_post_switch_motion_never_settles() {
        pollster::block_on(async {
            let mut phase = OuterPhase::MechanicalId {
                state: MechanicalIdState::Setup,
                data: MechanicalIdData::new(0),
                safety: OuterSafetyLimits::new(
                    0,
                    crate::commissioning::outer::OuterMotionBudget::firmware_caps(),
                ),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();
            backend.current_limit_value = 2448;
            backend.mechanical_encoder_position_counts_value = 1000;
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
                backend.mechanical_encoder_position_counts_value = pos;
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
                    &mut backend,
                )
                .await;
            }

            backend.elapsed_us_value = 150_000;
            backend.mechanical_encoder_position_counts_value = 1250;
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
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::OuterFailed(CommissionError::SafetyEnvelopeViolation)
            ));
            assert!(matches!(
                phase,
                OuterPhase::FailedWithSafety {
                    error: CommissionError::SafetyEnvelopeViolation,
                    safety_fault
                } if safety_fault.reason == OUTER_SAFETY_FAULT_POST_SWITCH_SETTLE
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
                    start_position: None,
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
                safety: OuterSafetyLimits::new(
                    2448,
                    crate::commissioning::outer::OuterMotionBudget::firmware_caps(),
                ),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();

            let result = step_mechanical_id(&mut phase, &mut buf, &mut backend).await;

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
                    start_position: None,
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
                safety: OuterSafetyLimits::new(
                    2448,
                    crate::commissioning::outer::OuterMotionBudget::firmware_caps(),
                ),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();

            let result = step_mechanical_id(&mut phase, &mut buf, &mut backend).await;

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
                    start_position: None,
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
                safety: OuterSafetyLimits::new(
                    2448,
                    crate::commissioning::outer::OuterMotionBudget::firmware_caps(),
                ),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();

            let result = step_mechanical_id(&mut phase, &mut buf, &mut backend).await;

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
                    start_position: None,
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
                safety: OuterSafetyLimits::new(
                    2448,
                    crate::commissioning::outer::OuterMotionBudget::firmware_caps(),
                ),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();

            let _ = step_mechanical_id(&mut phase, &mut buf, &mut backend).await;

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
                    start_position: None,
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
                safety: OuterSafetyLimits::new(
                    2448,
                    crate::commissioning::outer::OuterMotionBudget::firmware_caps(),
                ),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();

            let _ = step_mechanical_id(&mut phase, &mut buf, &mut backend).await;

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
                    start_position: None,
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
                safety: OuterSafetyLimits::new(
                    2448,
                    crate::commissioning::outer::OuterMotionBudget::firmware_caps(),
                ),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();

            let _ = step_mechanical_id(&mut phase, &mut buf, &mut backend).await;

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
}

/// Advance mechanical identification sub-state.
///
/// Bidirectional torque step procedure:
/// 1. Switch to torque mode with tuned current-loop gains active.
/// 2. Record starting position.
/// 3. Apply torque step, sample encoder at 1 kHz into the transient buffer.
/// 4. Return to zero torque, wait for settling.
/// 5. Repeat in opposite direction.
/// 6. Estimate `J_eff`, `B_eff`, `tau_mech`, `theta_mech`.
/// 7. Validate gates; on failure either retry with stronger torque or
///    fall back to response-ramp velocity tuning.
#[allow(dead_code)]
pub(crate) async fn step_mechanical_id<B: CommissioningBackend>(
    phase: &mut OuterPhase,
    buf: &mut TransientBuffer,
    backend: &mut B,
) -> StepResult {
    let mut trace_summary_seqs = TraceSummarySeqs::new();
    step_mechanical_id_with_trace(phase, buf, &mut trace_summary_seqs, backend).await
}

pub(crate) async fn step_mechanical_id_with_trace<B: CommissioningBackend>(
    phase: &mut OuterPhase,
    buf: &mut TransientBuffer,
    trace_summary_seqs: &mut TraceSummarySeqs,
    backend: &mut B,
) -> StepResult {
    use foci_tmc4671::registers::*;

    let (state, data, safety) = match phase {
        OuterPhase::MechanicalId {
            state,
            data,
            safety,
        } => (state, data, safety),
        _ => return StepResult::Continue,
    };

    match state {
        MechanicalIdState::Setup => {
            // Read current encoder position as the reference point.
            let encoder_raw = match backend.try_read_mechanical_encoder_position_counts().await {
                Ok(value) => value,
                Err(_) => return fail_outer(phase, CommissionError::Spi),
            };

            // Compute torque step from configured run_current.
            let current_limit = backend.current_limit() & 0xFFFF;
            let torque_step = if current_limit > 0 {
                response::mechanical_id_torque_step(current_limit, data.attempt)
            } else {
                return fail_outer(phase, CommissionError::NoCurrent);
            };

            // Initialize data with real values.
            data.torque_step = torque_step;

            // Read encoder PPR and prove the host budget can produce useful
            // mechanical-ID evidence before any torque/mode writes or enable.
            let ppr_raw = match backend.read_register(ABN_DECODER_PPR).await {
                Ok(value) => value,
                Err(_) => return fail_outer(phase, CommissionError::Spi),
            };
            if safety.configure_encoder_resolution(ppr_raw).is_err()
                || safety.set_encoder_baseline(encoder_raw).is_err()
            {
                return fail_outer(phase, CommissionError::EncoderFault);
            }
            data.start_position = safety.prev_encoder_count;
            if !safety.budget_is_usable() {
                defmt::warn!(
                    "mechanical-id: unusable motion budget travel={} velocity={} duration={} dir={} window={} velocity_cap={}",
                    safety.budget.max_travel_mrev,
                    safety.budget.max_velocity_mrev_s,
                    safety.budget.max_duration_ms,
                    safety.budget.direction_mask,
                    safety.position_window,
                    safety.velocity_cap,
                );
                let fault = safety.fault(OUTER_SAFETY_FAULT_UNUSABLE_BUDGET, 0, 0, 0, 0);
                return fail_outer_with_safety(
                    phase,
                    CommissionError::SafetyEnvelopeViolation,
                    fault,
                );
            }

            // Zero torque before the position->torque mode switch so any stale
            // target from the prior closed-loop hold cannot bleed into setup.
            if backend
                .write_register(PID_TORQUE_FLUX_TARGET, 0)
                .await
                .is_err()
            {
                return fail_outer(phase, CommissionError::Spi);
            }

            // Ensure we are in torque mode with the same advanced PI
            // controller structure used by the tuned current loop.
            if backend
                .write_register(PIDOUT_UQ_UD_LIMITS, OUTER_PIDOUT_LIMIT)
                .await
                .is_err()
                || backend.write_register(PWM_SV_CHOP, 7).await.is_err()
                || backend.write_register(PHI_E_SELECTION, 3).await.is_err()
                || backend
                    .write_register(MODE_RAMP_MODE_MOTION, CURRENT_TORQUE_MODE)
                    .await
                    .is_err()
            {
                return fail_outer(phase, CommissionError::Spi);
            }

            let phi_ok = backend
                .read_register(PHI_E_SELECTION)
                .await
                .is_ok_and(|v| v == 3);
            if !phi_ok {
                return fail_outer(phase, CommissionError::EncoderNotAligned);
            }

            backend.set_enable_pin(true);

            // Prepare transient buffer for forward capture.
            buf.reset(response::MECHANICAL_ID_SAMPLE_PERIOD_US as u16);

            // Seed the safety baseline from the post-switch position, but keep
            // safety disarmed until a brief zero-torque settle completes.
            let post_switch_pos = match backend.try_read_mechanical_encoder_position_counts().await
            {
                Ok(value) => value,
                Err(_) => return fail_outer(phase, CommissionError::Spi),
            };
            safety
                .set_encoder_baseline(post_switch_pos)
                .expect("configured domain");
            safety.unwrapped_position = 0;
            data.start_position = safety.prev_encoder_count;
            safety.torque_cap = 0;

            let now = backend.elapsed_us();
            safety.last_sample_us = Some(now);
            *state = MechanicalIdState::PostSwitchSettle {
                earliest_sample_us: now + MECHANICAL_ID_MODE_SWITCH_SETTLE_POLICY.grace_us,
                timeout_deadline_us: now + MECHANICAL_ID_MODE_SWITCH_SETTLE_POLICY.timeout_us,
                stable_ticks: 0,
            };
            StepResult::PhaseChanged(PhaseId::MechanicalId)
        }

        MechanicalIdState::PostSwitchSettle {
            earliest_sample_us,
            timeout_deadline_us,
            stable_ticks,
        } => {
            let now = backend.elapsed_us();
            if now < *earliest_sample_us {
                return StepResult::Continue;
            }

            let settled_pos = match backend.try_read_mechanical_encoder_position_counts().await {
                Ok(value) => value,
                Err(_) => return fail_outer(phase, CommissionError::Spi),
            };
            let encoder_domain = safety
                .encoder_domain
                .expect("outer encoder domain configured before settle");
            let settled_count = encoder_domain.count(settled_pos);
            let delta_counts = match encoder_domain.delta(
                safety
                    .prev_encoder_count
                    .expect("post-switch baseline configured"),
                settled_count,
            ) {
                Ok(delta) => delta.signed_counts(),
                Err(_) => {
                    let _ = backend.write_register(PID_TORQUE_FLUX_TARGET, 0).await;
                    let _ = backend.write_register(MODE_RAMP_MODE_MOTION, 0).await;
                    return fail_outer(phase, CommissionError::EncoderFault);
                }
            };
            let dt_us = safety
                .last_sample_us
                .and_then(|last_sample_us| now.checked_sub(last_sample_us))
                .unwrap_or(0);
            let velocity_cpms = response::counts_per_ms(delta_counts, dt_us);
            safety
                .set_encoder_baseline(settled_pos)
                .expect("configured domain");
            safety.last_sample_us = Some(now);

            let settle_threshold = response::mechanical_id_settle_velocity_threshold_counts_per_ms(
                safety.counts_per_rev(),
            );
            if velocity_cpms.unsigned_abs() <= settle_threshold {
                *stable_ticks = stable_ticks.saturating_add(1);
            } else {
                *stable_ticks = 0;
            }

            if *stable_ticks >= MECHANICAL_ID_MODE_SWITCH_SETTLE_POLICY.stable_ticks {
                if safety.primitive_start_us.is_some() {
                    return fail_outer(phase, CommissionError::InvalidSchedule);
                }
                let current_limit = backend.current_limit() & 0xFFFF;
                safety
                    .set_encoder_baseline(settled_pos)
                    .expect("configured domain");
                safety.unwrapped_position = 0;
                safety.torque_cap = current_limit as u16;
                let safety_arm_us = backend.elapsed_us();
                if safety.arm_safety_observation(safety_arm_us).is_err() {
                    return fail_outer(phase, CommissionError::InvalidSchedule);
                }
                data.start_position = safety.prev_encoder_count;

                // Apply the first positive torque step immediately so the next
                // outer tick does not arm safety for an idle pre-excitation gap.
                let torque = data.torque_step as u32;
                let target = torque << 16;
                if backend
                    .write_register(PID_TORQUE_FLUX_TARGET, target)
                    .await
                    .is_err()
                {
                    return fail_outer(phase, CommissionError::Spi);
                }

                let capture_start_us = backend.elapsed_us();
                if safety.begin_primitive(capture_start_us).is_err() {
                    return fail_outer(phase, CommissionError::InvalidSchedule);
                }
                data.reset_sample_timestamps();
                let next_sample_us =
                    capture_start_us.wrapping_add(response::MECHANICAL_ID_SAMPLE_PERIOD_US);

                defmt::info!(
                    "mechanical-id: attempt={}, start pos={}, torque_step={}, current_limit={}",
                    data.attempt,
                    settled_pos,
                    data.torque_step,
                    current_limit
                );

                *state = MechanicalIdState::ForwardCapture {
                    next_sample_us,
                    capture_start_us,
                };
                return StepResult::Continue;
            }

            if now >= *timeout_deadline_us {
                defmt::warn!(
                    "mechanical-id: post-switch settle timed out at velocity={} (delta={} dt_us={})",
                    velocity_cpms.unsigned_abs(),
                    delta_counts,
                    dt_us,
                );
                let settle_start_us = timeout_deadline_us
                    .wrapping_sub(MECHANICAL_ID_MODE_SWITCH_SETTLE_POLICY.timeout_us);
                let elapsed_us = now.wrapping_sub(settle_start_us);
                let fault = safety.fault(
                    OUTER_SAFETY_FAULT_POST_SWITCH_SETTLE,
                    delta_counts,
                    dt_us,
                    velocity_cpms,
                    elapsed_us,
                );
                return fail_outer_with_safety(
                    phase,
                    CommissionError::SafetyEnvelopeViolation,
                    fault,
                );
            }

            StepResult::Continue
        }

        MechanicalIdState::ForwardCapture {
            next_sample_us,
            capture_start_us,
        } => {
            // Check if it's time for the next sample.
            let now = backend.elapsed_us();
            if now.wrapping_sub(*next_sample_us) >= 0x8000_0000 {
                // Not yet time for next sample.
                return StepResult::Continue;
            }

            // Sample encoder position.
            let encoder_raw = match backend.try_read_mechanical_encoder_position_counts().await {
                Ok(value) => value,
                Err(_) => return fail_outer(phase, CommissionError::Spi),
            };
            let encoder_domain = safety
                .encoder_domain
                .expect("outer encoder domain configured before capture");
            let encoder_count = encoder_domain.count(encoder_raw);
            let previous_count = if let Some(previous_raw) = buf.samples().last() {
                encoder_domain.count(*previous_raw)
            } else {
                data.start_position
                    .expect("mechanical capture baseline configured")
            };
            if encoder_domain.delta(previous_count, encoder_count).is_err() {
                let _ = backend.write_register(PID_TORQUE_FLUX_TARGET, 0).await;
                let _ = backend.write_register(MODE_RAMP_MODE_MOTION, 0).await;
                return fail_outer(phase, CommissionError::EncoderFault);
            }
            buf.push(encoder_count.raw_counts());
            data.push_sample_timestamp(now);

            if buf.is_full() {
                // Forward capture complete — zero torque.
                if backend
                    .write_register(PID_TORQUE_FLUX_TARGET, 0)
                    .await
                    .is_err()
                {
                    return fail_outer(phase, CommissionError::Spi);
                }
                safety.end_primitive();

                // Extract forward metrics from buffer before reset.
                let encoder_domain = safety
                    .encoder_domain
                    .expect("outer encoder domain configured before capture");
                let reference_position = data
                    .start_position
                    .expect("mechanical capture baseline configured");
                let metrics = response::extract_mechanical_id_metrics(
                    buf,
                    data.sample_timestamps(),
                    *capture_start_us,
                    reference_position,
                    encoder_domain,
                );
                data.fwd_slope = metrics.accel;
                data.forward_peak_velocity = metrics.peak_velocity;
                data.forward_delay_us = metrics.delay_us;
                data.forward_travel = metrics.travel;
                data.fwd_response = response::extract_mechanical_response_model(
                    buf,
                    data.sample_timestamps(),
                    *capture_start_us,
                    reference_position,
                    encoder_domain,
                    data.torque_step as u32,
                );

                defmt::info!(
                    "mechanical-id fwd: accel={} peak_vel={} delay_us={} travel={} \
                     resid_counts={} rise_mcpms={} samples={} steady={} tau={}ms response_resid={}",
                    data.fwd_slope.accel,
                    data.forward_peak_velocity,
                    data.forward_delay_us,
                    data.forward_travel,
                    data.fwd_slope.residual_counts,
                    data.fwd_slope.velocity_rise_mcpms,
                    data.fwd_slope.fit_sample_count,
                    data.fwd_response.steady_velocity_cpms,
                    data.fwd_response.tau_mech_ms,
                    data.fwd_response.residual_mcpms,
                );

                let deadline = backend.elapsed_us().wrapping_add(MECHANICAL_ID_SETTLE_US);
                *state = MechanicalIdState::ForwardSettle {
                    deadline_us: deadline,
                };
            } else {
                *next_sample_us = now.wrapping_add(response::MECHANICAL_ID_SAMPLE_PERIOD_US);
            }
            StepResult::Continue
        }

        MechanicalIdState::ForwardSettle { deadline_us } => {
            if backend.elapsed_us().wrapping_sub(*deadline_us) < 0x8000_0000 {
                // Settled — prepare for reverse capture.
                buf.reset(response::MECHANICAL_ID_SAMPLE_PERIOD_US as u16);
                *state = MechanicalIdState::ReverseStart;
            }
            StepResult::Continue
        }

        MechanicalIdState::ReverseStart => {
            if safety.primitive_start_us.is_some() {
                return fail_outer(phase, CommissionError::InvalidSchedule);
            }
            let encoder_raw = match backend.try_read_mechanical_encoder_position_counts().await {
                Ok(value) => value,
                Err(_) => return fail_outer(phase, CommissionError::Spi),
            };
            let encoder_domain = safety
                .encoder_domain
                .expect("outer encoder domain configured before capture");
            data.start_position = Some(encoder_domain.count(encoder_raw));

            // Apply negative torque step.
            // Torque is signed i16 in upper 16 bits.
            let neg_torque = (-(data.torque_step as i16)) as u16;
            let target = (neg_torque as u32) << 16;
            if backend
                .write_register(PID_TORQUE_FLUX_TARGET, target)
                .await
                .is_err()
            {
                return fail_outer(phase, CommissionError::Spi);
            }

            let capture_start_us = backend.elapsed_us();
            if safety.begin_primitive(capture_start_us).is_err() {
                return fail_outer(phase, CommissionError::InvalidSchedule);
            }
            data.reset_sample_timestamps();
            let next_sample_us =
                capture_start_us.wrapping_add(response::MECHANICAL_ID_SAMPLE_PERIOD_US);
            *state = MechanicalIdState::ReverseCapture {
                next_sample_us,
                capture_start_us,
            };
            StepResult::Continue
        }

        MechanicalIdState::ReverseCapture {
            next_sample_us,
            capture_start_us,
        } => {
            let now = backend.elapsed_us();
            if now.wrapping_sub(*next_sample_us) >= 0x8000_0000 {
                return StepResult::Continue;
            }

            let encoder_raw = match backend.try_read_mechanical_encoder_position_counts().await {
                Ok(value) => value,
                Err(_) => return fail_outer(phase, CommissionError::Spi),
            };
            let encoder_domain = safety
                .encoder_domain
                .expect("outer encoder domain configured before capture");
            let encoder_count = encoder_domain.count(encoder_raw);
            let previous_count = if let Some(previous_raw) = buf.samples().last() {
                encoder_domain.count(*previous_raw)
            } else {
                data.start_position
                    .expect("mechanical capture baseline configured")
            };
            if encoder_domain.delta(previous_count, encoder_count).is_err() {
                let _ = backend.write_register(PID_TORQUE_FLUX_TARGET, 0).await;
                let _ = backend.write_register(MODE_RAMP_MODE_MOTION, 0).await;
                return fail_outer(phase, CommissionError::EncoderFault);
            }
            buf.push(encoder_count.raw_counts());
            data.push_sample_timestamp(now);

            if buf.is_full() {
                // Reverse capture complete — zero torque.
                if backend
                    .write_register(PID_TORQUE_FLUX_TARGET, 0)
                    .await
                    .is_err()
                {
                    return fail_outer(phase, CommissionError::Spi);
                }
                safety.end_primitive();

                // For reverse capture, use the position at capture start
                // (first sample) as the reference, not the original start.
                let encoder_domain = safety
                    .encoder_domain
                    .expect("outer encoder domain configured before capture");
                let reverse_start = if !buf.is_empty() {
                    encoder_domain.count(buf.samples()[0])
                } else {
                    data.start_position
                        .expect("mechanical capture baseline configured")
                };
                let metrics = response::extract_mechanical_id_metrics(
                    buf,
                    data.sample_timestamps(),
                    *capture_start_us,
                    reverse_start,
                    encoder_domain,
                );
                data.rev_slope = metrics.accel;
                data.reverse_peak_velocity = metrics.peak_velocity.unsigned_abs() as i32;
                data.reverse_delay_us = metrics.delay_us;
                data.reverse_travel = metrics.travel;
                data.rev_response = response::extract_mechanical_response_model(
                    buf,
                    data.sample_timestamps(),
                    *capture_start_us,
                    reverse_start,
                    encoder_domain,
                    data.torque_step as u32,
                );

                defmt::info!(
                    "mechanical-id rev: accel={} peak_vel={} delay_us={} travel={} \
                     resid_counts={} rise_mcpms={} samples={} steady={} tau={}ms response_resid={}",
                    data.rev_slope.accel,
                    data.reverse_peak_velocity,
                    data.reverse_delay_us,
                    data.reverse_travel,
                    data.rev_slope.residual_counts,
                    data.rev_slope.velocity_rise_mcpms,
                    data.rev_slope.fit_sample_count,
                    data.rev_response.steady_velocity_cpms,
                    data.rev_response.tau_mech_ms,
                    data.rev_response.residual_mcpms,
                );

                let deadline = backend.elapsed_us().wrapping_add(MECHANICAL_ID_SETTLE_US);
                *state = MechanicalIdState::ReverseSettle {
                    deadline_us: deadline,
                };
            } else {
                *next_sample_us = now.wrapping_add(response::MECHANICAL_ID_SAMPLE_PERIOD_US);
            }
            StepResult::Continue
        }

        MechanicalIdState::ReverseSettle { deadline_us } => {
            if backend.elapsed_us().wrapping_sub(*deadline_us) < 0x8000_0000 {
                *state = MechanicalIdState::Estimate;
            }
            StepResult::Continue
        }

        MechanicalIdState::Estimate => {
            // Ensure motor is stopped.
            if backend
                .write_register(PID_TORQUE_FLUX_TARGET, 0)
                .await
                .is_err()
                || backend
                    .write_register(MODE_RAMP_MODE_MOTION, 0)
                    .await
                    .is_err()
            {
                return fail_outer(phase, CommissionError::Spi);
            }

            *state = MechanicalIdState::Finalize;
            StepResult::Continue
        }

        MechanicalIdState::Finalize => {
            // Assemble and validate mechanical model.
            let torque = data.torque_step as u32;

            let fwd_peak = data.forward_peak_velocity.unsigned_abs();
            let rev_peak = data.reverse_peak_velocity as u32;
            let peak_avg = (fwd_peak + rev_peak) / 2;
            let mut diagnostics = OuterDiagnostics::from_mechanical_id(data);

            if data.fwd_response.is_high_confidence() && data.rev_response.is_high_confidence() {
                let j_avg = (data.fwd_response.j_eff + data.rev_response.j_eff) / 2;
                let j_diff = data.fwd_response.j_eff.abs_diff(data.rev_response.j_eff);
                let asymmetry_pct = (j_diff * 100).checked_div(j_avg).unwrap_or(100);
                if asymmetry_pct > MECHANICAL_ID_SYMMETRY_THRESHOLD_PCT {
                    record_mechanical_id_summary(
                        backend,
                        trace_summary_seqs,
                        data,
                        mechanical_retry_warning_code(data),
                        0,
                        TraceEventLevel::Warn,
                    );
                    if response::retry_mechanical_id(state, data, "asymmetric full response") {
                        return StepResult::Continue;
                    }
                    defmt::warn!(
                        "mechanical-id: full-response asymmetry {}% exceeds {}%, using response fallback",
                        asymmetry_pct,
                        MECHANICAL_ID_SYMMETRY_THRESHOLD_PCT
                    );
                    let avg_delay_us =
                        (data.fwd_response.theta_mech_us + data.rev_response.theta_mech_us) / 2;
                    let (model, peak_velocity) =
                        response::response_mechanical_model_low_confidence(peak_avg, avg_delay_us);
                    *phase = response::response_velocity_phase(
                        model,
                        peak_velocity,
                        diagnostics,
                        *safety,
                    );
                    return StepResult::PhaseChanged(PhaseId::VelocityTune);
                } else {
                    let b_eff = (data.fwd_response.b_eff + data.rev_response.b_eff) / 2;
                    let tau_mech_ms =
                        (data.fwd_response.tau_mech_ms + data.rev_response.tau_mech_ms) / 2;
                    let theta_mech_us =
                        (data.fwd_response.theta_mech_us + data.rev_response.theta_mech_us) / 2;
                    let model = MechanicalModel {
                        j_eff: j_avg.max(1),
                        b_eff: b_eff.max(1),
                        tau_mech_ms: tau_mech_ms.max(1),
                        theta_mech_us,
                        t_f: 0,
                    };
                    defmt::info!(
                        "mechanical-id full-response: J={} B={} tau={}ms theta={}us steady={}/{}",
                        model.j_eff,
                        model.b_eff,
                        model.tau_mech_ms,
                        model.theta_mech_us,
                        data.fwd_response.steady_velocity_cpms,
                        data.rev_response.steady_velocity_cpms,
                    );
                    record_mechanical_id_summary(
                        backend,
                        trace_summary_seqs,
                        data,
                        OUTER_WARNING_NONE,
                        0,
                        TraceEventLevel::Info,
                    );
                    let carried_safety = match phase {
                        OuterPhase::MechanicalId { safety, .. } => *safety,
                        _ => unreachable!(),
                    };
                    *phase = OuterPhase::VelocityTune {
                        model,
                        peak_velocity: peak_avg.max(1),
                        diagnostics,
                        safety: carried_safety,
                    };
                    return StepResult::PhaseChanged(PhaseId::VelocityTune);
                }
            }

            // Confidence gate: if either slope is low-confidence, fall back via
            // the response-ramp fallback using a conservative J_eff. The
            // slope estimator's is_high_confidence() replaces the old
            // MECHANICAL_ID_MIN_RETRY_ACCEL_COUNTS_PER_MS2 magic-number gate.
            if !data.fwd_slope.is_high_confidence() || !data.rev_slope.is_high_confidence() {
                defmt::warn!(
                    "mechanical-id: low-confidence slope \
                     (fwd: rise={} samples={} resid={}, rev: rise={} samples={} resid={}) \
                     -> response fallback",
                    data.fwd_slope.velocity_rise_mcpms,
                    data.fwd_slope.fit_sample_count,
                    data.fwd_slope.residual_counts,
                    data.rev_slope.velocity_rise_mcpms,
                    data.rev_slope.fit_sample_count,
                    data.rev_slope.residual_counts,
                );
                let avg_delay_us = (data.forward_delay_us + data.reverse_delay_us) / 2;
                let (model, peak_velocity) =
                    response::response_mechanical_model_low_confidence(peak_avg, avg_delay_us);
                record_mechanical_id_summary(
                    backend,
                    trace_summary_seqs,
                    data,
                    response::OUTER_WARNING_LOW_CONFIDENCE_MECHANICAL_ID,
                    0,
                    TraceEventLevel::Warn,
                );
                *phase = response::response_velocity_phase(
                    model,
                    peak_velocity,
                    OuterDiagnostics::from_mechanical_id(data),
                    *safety,
                );
                return StepResult::PhaseChanged(PhaseId::VelocityTune);
            }

            let fwd_accel = data.fwd_slope.accel;
            let rev_accel = data.rev_slope.accel;

            // Gate: both accelerations must be non-zero.
            if fwd_accel == 0 || rev_accel == 0 {
                record_mechanical_id_summary(
                    backend,
                    trace_summary_seqs,
                    data,
                    mechanical_retry_warning_code(data),
                    0,
                    TraceEventLevel::Warn,
                );
                if response::retry_mechanical_id(state, data, "zero acceleration") {
                    return StepResult::Continue;
                }
                defmt::warn!(
                    "mechanical-id: zero acceleration after retries, using response velocity tuning"
                );
                let (model, peak_velocity) = response::response_mechanical_model(
                    data, torque, fwd_accel, rev_accel, fwd_peak, rev_peak,
                );
                *phase = response::response_velocity_phase(
                    model,
                    peak_velocity,
                    OuterDiagnostics::from_mechanical_id(data),
                    *safety,
                );
                return StepResult::PhaseChanged(PhaseId::VelocityTune);
            }

            // J_eff = torque / initial_acceleration.
            // Scale: torque is in DAC units, accel is in counts/ms^2.
            // We multiply torque by 1000 to get better fixed-point resolution.
            let j_eff_fwd = (torque * 1000) / fwd_accel.unsigned_abs().max(1);
            let j_eff_rev = (torque * 1000) / rev_accel.unsigned_abs().max(1);

            // Symmetry gate: |J_fwd - J_rev| / avg <= 30%.
            let j_avg = (j_eff_fwd + j_eff_rev) / 2;
            if j_avg == 0 {
                defmt::warn!("mechanical-id: zero average inertia");
                record_mechanical_id_summary(
                    backend,
                    trace_summary_seqs,
                    data,
                    OUTER_WARNING_NONE,
                    CommissionError::MechanicalIdFailed.status_code(),
                    TraceEventLevel::Error,
                );
                return fail_outer(phase, CommissionError::MechanicalIdFailed);
            }
            let j_diff = j_eff_fwd.abs_diff(j_eff_rev);
            let asymmetry_pct = (j_diff * 100) / j_avg;
            if asymmetry_pct > MECHANICAL_ID_SYMMETRY_THRESHOLD_PCT {
                record_mechanical_id_summary(
                    backend,
                    trace_summary_seqs,
                    data,
                    mechanical_retry_warning_code(data),
                    0,
                    TraceEventLevel::Warn,
                );
                if response::retry_mechanical_id(state, data, "asymmetric response") {
                    return StepResult::Continue;
                }
                defmt::warn!(
                    "mechanical-id: asymmetry {}% exceeds {}% after retries, using response velocity tuning",
                    asymmetry_pct,
                    MECHANICAL_ID_SYMMETRY_THRESHOLD_PCT
                );
                let (model, peak_velocity) = response::response_mechanical_model(
                    data, torque, fwd_accel, rev_accel, fwd_peak, rev_peak,
                );
                *phase = response::response_velocity_phase(
                    model,
                    peak_velocity,
                    OuterDiagnostics::from_mechanical_id(data),
                    *safety,
                );
                return StepResult::PhaseChanged(PhaseId::VelocityTune);
            }

            let j_eff = j_avg;

            // B_eff = torque / steady_state_velocity, scaled to match J_eff units.
            // No steady-state velocity reached -> pure inertial load; small minimum
            // (1) avoids division by zero in tau below.
            let b_eff_raw = (torque * 1000).checked_div(peak_avg).unwrap_or(1);
            let (b_eff, tau_mech_ms, tau_defaulted) =
                derive_mechanical_tau_with_default_flag(j_eff, b_eff_raw);

            // theta_mech from delay samples (time from torque step to
            // first measurable velocity change).
            let theta_mech_us = (data.forward_delay_us + data.reverse_delay_us) / 2;

            let model = MechanicalModel {
                j_eff,
                b_eff,
                tau_mech_ms,
                theta_mech_us,
                t_f: 0, // Static friction not measured in this phase.
            };
            diagnostics.mech_tau_defaulted = tau_defaulted;

            defmt::info!(
                "mechanical-id: J={}, B={}, tau={}ms, theta={}us",
                model.j_eff,
                model.b_eff,
                model.tau_mech_ms,
                model.theta_mech_us
            );

            if response::mechanical_id_low_confidence(
                fwd_accel.unsigned_abs(),
                rev_accel.unsigned_abs(),
                fwd_peak,
                rev_peak,
                data.forward_travel,
                data.reverse_travel,
            ) {
                record_mechanical_id_summary(
                    backend,
                    trace_summary_seqs,
                    data,
                    mechanical_retry_warning_code(data),
                    0,
                    TraceEventLevel::Warn,
                );
                if response::retry_mechanical_id(state, data, "low-confidence response") {
                    return StepResult::Continue;
                }

                defmt::warn!("mechanical-id: low-confidence data, using response velocity tuning");
                *phase = response::response_velocity_phase(model, peak_avg, diagnostics, *safety);
                return StepResult::PhaseChanged(PhaseId::VelocityTune);
            }

            record_mechanical_id_summary(
                backend,
                trace_summary_seqs,
                data,
                OUTER_WARNING_NONE,
                0,
                TraceEventLevel::Info,
            );

            // Carry safety limits forward to VelocityTune.
            let carried_safety = match phase {
                OuterPhase::MechanicalId { safety, .. } => *safety,
                _ => unreachable!(),
            };

            *phase = OuterPhase::VelocityTune {
                model,
                peak_velocity: peak_avg,
                diagnostics,
                safety: carried_safety,
            };
            StepResult::PhaseChanged(PhaseId::VelocityTune)
        }
    }
}
