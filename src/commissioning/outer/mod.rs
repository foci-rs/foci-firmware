//! Stage 2 (mechanical / velocity / position / filter) commissioning.
//!
//! `OuterPhase` is the cooperative state-machine type. `step_outer`
//! dispatches one tick to the per-phase submodule, prefixed by a
//! safety-envelope check that aborts on velocity- or position-window
//! violations. The phase variants and their sub-state types live here
//! so each per-phase submodule can pattern-match and construct them.

pub mod commit;
pub mod filter;
mod limits;
pub mod mechanical;
pub mod position;
pub mod response;
pub mod slope_estimator;
mod state;
mod trace;
pub mod velocity;

use crate::autotune::{CommissionError, CommissionProfile, OperatingMode, TransientBuffer};
use crate::commissioning::backend::CommissioningBackend;
use crate::commissioning::types::StepResult;

pub(crate) use limits::{
    MECHANICAL_ID_VELOCITY_CAP_COUNTS_PER_MS, OUTER_FALLBACK_POSITION_I, OUTER_FALLBACK_POSITION_P,
    OUTER_FALLBACK_VELOCITY_I, OUTER_FALLBACK_VELOCITY_P, OUTER_MAX_POSITION_I,
    OUTER_MAX_POSITION_P, OUTER_MAX_VELOCITY_I, OUTER_MAX_VELOCITY_P, OUTER_WARNING_NONE,
    OUTER_WARNING_UNSAFE_SYNTH_GAINS, POS_TUNE_INITIAL_P, REFERENCE_ENCODER_COUNTS_PER_REV,
    mechanical_id_velocity_cap_counts_per_ms, position_tune_step_counts, position_window_counts,
    response_fallback_velocity_limit_counts_per_ms,
};
use limits::{POSITION_WINDOW_COUNTS, normalized_counts_per_rev};
pub(crate) use state::{
    MechanicalIdData, MechanicalIdState, OuterDiagnostics, OuterPhase, PositionTuneData,
    PositionTuneState, VelocityTuneStrategy, VelocityValidateData, VelocityValidateState,
};
pub(crate) use trace::{
    TraceSummarySeqs, trace_diagnostic_warning, trace_pack_i16_pair, trace_pack_u16_pair,
    trace_summary_event,
};

/// Outer safety limits, computed once at the start of outer commissioning.
#[derive(Debug, Clone, Copy, defmt::Format)]
pub(crate) struct OuterSafetyLimits {
    /// Maximum absolute velocity in encoder counts per millisecond.
    pub(crate) velocity_cap: i32,
    /// Maximum distance from start position in encoder counts.
    pub(crate) position_window: i32,
    /// Unwrapped signed position accumulated from wrap-aware deltas.
    pub(crate) unwrapped_position: i32,
    /// Torque cap: the configured run_current limit.
    pub(crate) torque_cap: u16,
    /// Previous raw encoder count for delta computation.
    pub(crate) prev_encoder_count: i32,
    /// Encoder counts per revolution (PPR * 4). Used to normalize
    /// wrap-around deltas when the ABN decoder counter rolls over.
    pub(crate) counts_per_rev: i32,
    /// Timestamp (microseconds) of the previous safety sample.
    /// Zero means the next sample initializes the baseline.
    pub(crate) last_sample_us: u64,
}

impl OuterSafetyLimits {
    pub(crate) fn new(start_position: i32, torque_cap: u16) -> Self {
        Self {
            velocity_cap: MECHANICAL_ID_VELOCITY_CAP_COUNTS_PER_MS,
            position_window: POSITION_WINDOW_COUNTS,
            unwrapped_position: 0,
            torque_cap,
            prev_encoder_count: start_position,
            counts_per_rev: 0, // Set by MechanicalId::Setup after reading ABN_DECODER_PPR
            last_sample_us: 0,
        }
    }

    pub(crate) fn configure_encoder_resolution(&mut self, counts_per_rev: i32) {
        let counts_per_rev = normalized_counts_per_rev(counts_per_rev);
        self.counts_per_rev = counts_per_rev;
        self.velocity_cap = mechanical_id_velocity_cap_counts_per_ms(counts_per_rev);
        self.position_window = position_window_counts(counts_per_rev);
    }
}

// ---------------------------------------------------------------------------
// Safety helpers
// ---------------------------------------------------------------------------

/// Transition the outer phase to `Failed` and emit an `OuterFailed` result.
pub(crate) fn fail_outer(phase: &mut OuterPhase, err: CommissionError) -> StepResult {
    *phase = OuterPhase::Failed(err);
    StepResult::OuterFailed(err)
}

/// Map a safety-envelope violation to the dedicated error variant.
///
/// Always returns `SafetyEnvelopeViolation` so the OuterFailed handler
/// can classify it as a safety fault requiring hard disable, distinct
/// from normal phase failures that are eligible for soft restore.
pub(crate) fn safety_error_for_phase(_phase: &OuterPhase) -> CommissionError {
    CommissionError::SafetyEnvelopeViolation
}

/// Per-tick safety envelope check for all outer phases.
///
/// Reads the current encoder position, computes a wrap-aware delta,
/// normalizes velocity by actual elapsed time, and accumulates an
/// unwrapped signed position. Checks:
/// - Time-normalized velocity within cap.
/// - Unwrapped position within window from start.
///
/// Returns `true` if a safety limit is violated.
pub(crate) async fn check_outer_safety<B: CommissioningBackend>(
    safety: &mut OuterSafetyLimits,
    backend: &mut B,
) -> Result<bool, B::Error> {
    // Safety limits are initialized by MechanicalId::Setup after it reads
    // the real encoder baseline. Skip the pre-setup placeholder values.
    if safety.torque_cap == 0 {
        return Ok(false);
    }

    let encoder_raw = backend.try_read_encoder_count().await?;
    let encoder_count = encoder_raw as i32;
    let now_us = backend.elapsed_us();

    // Wrap-aware delta in encoder counts since the last safety sample.
    let delta_counts = response::encoder_delta(
        encoder_count,
        safety.prev_encoder_count,
        safety.counts_per_rev,
    );
    let dt_us = if safety.last_sample_us == 0 {
        0
    } else {
        now_us.wrapping_sub(safety.last_sample_us)
    };
    let velocity_cpms = response::counts_per_ms(delta_counts, dt_us);
    safety.prev_encoder_count = encoder_count;
    safety.last_sample_us = now_us;
    safety.unwrapped_position += delta_counts;

    // Check velocity cap (absolute value).
    let abs_velocity = velocity_cpms.unsigned_abs() as i32;
    if abs_velocity > safety.velocity_cap {
        defmt::warn!(
            "outer safety: velocity {} exceeds cap {} (delta={} dt_us={})",
            abs_velocity,
            safety.velocity_cap,
            delta_counts,
            dt_us,
        );
        return Ok(true);
    }

    // Check position window against unwrapped accumulator.
    let abs_position = safety.unwrapped_position.unsigned_abs() as i32;
    if abs_position > safety.position_window {
        defmt::warn!(
            "outer safety: position {} exceeds window {}",
            abs_position,
            safety.position_window
        );
        return Ok(true);
    }

    Ok(false)
}

/// Get a mutable reference to the safety limits for the current phase.
///
/// Returns `None` for terminal phases (Done/Failed/Commit) that don't
/// need per-tick safety checks.
fn outer_safety_mut(phase: &mut OuterPhase) -> Option<&mut OuterSafetyLimits> {
    match phase {
        OuterPhase::MechanicalId { safety, .. }
        | OuterPhase::VelocityTune { safety, .. }
        | OuterPhase::VelocityValidate { safety, .. }
        | OuterPhase::PositionTune { safety, .. }
        | OuterPhase::FilterTune { safety, .. } => Some(safety),
        OuterPhase::Commit { .. } | OuterPhase::Done(_) | OuterPhase::Failed(_) => None,
    }
}

// ---------------------------------------------------------------------------
// Dispatcher
// ---------------------------------------------------------------------------

/// Step outer phase state machine by one cooperative tick.
///
/// Dispatches to the per-phase handler in the matching submodule. Every
/// tick runs the safety envelope check before the phase-specific logic;
/// a violation aborts to `Failed(SafetyEnvelopeViolation)` after zeroing
/// excitation.
#[allow(clippy::too_many_arguments)]
#[allow(dead_code)]
pub(crate) async fn step_outer<B: CommissioningBackend>(
    phase: &mut OuterPhase,
    buf: &mut TransientBuffer,
    profile: CommissionProfile,
    mode: OperatingMode,
    inner_lambda_us: u32,
    theta_e_us: u32,
    current_ringing: bool,
    current_bandwidth_hz: u32,
    tau_e_us: u32,
    tau_e_crosscheck_us: u32,
    tau_residual_permille: u16,
    inner_warning_flags: u8,
    backend: &mut B,
) -> StepResult {
    let mut trace_summary_seqs = TraceSummarySeqs::new();
    step_outer_with_trace(
        phase,
        buf,
        profile,
        mode,
        inner_lambda_us,
        theta_e_us,
        current_ringing,
        current_bandwidth_hz,
        tau_e_us,
        tau_e_crosscheck_us,
        tau_residual_permille,
        inner_warning_flags,
        &mut trace_summary_seqs,
        backend,
    )
    .await
}

#[allow(clippy::too_many_arguments)]
pub(crate) async fn step_outer_with_trace<B: CommissioningBackend>(
    phase: &mut OuterPhase,
    buf: &mut TransientBuffer,
    profile: CommissionProfile,
    mode: OperatingMode,
    inner_lambda_us: u32,
    theta_e_us: u32,
    current_ringing: bool,
    current_bandwidth_hz: u32,
    tau_e_us: u32,
    tau_e_crosscheck_us: u32,
    tau_residual_permille: u16,
    inner_warning_flags: u8,
    trace_summary_seqs: &mut TraceSummarySeqs,
    backend: &mut B,
) -> StepResult {
    // `tau_e_us` and `tau_e_crosscheck_us` are persisted in the outer
    // engine for Phase 2 (adaptive measurement / fitted-tau crosscheck);
    // Phase 1 outer logic uses only `tau_residual_permille` and
    // `inner_warning_flags`. Keeping the wire/struct contract unchanged
    // avoids a Phase 2 schema bump.
    let _ = (tau_e_us, tau_e_crosscheck_us);
    if let Some(safety) = outer_safety_mut(phase) {
        match check_outer_safety(safety, backend).await {
            Ok(true) => {
                // Zero excitation immediately on any safety violation.
                let _ = backend
                    .write_register(foci_tmc4671::registers::PID_TORQUE_FLUX_TARGET, 0)
                    .await;
                let _ = backend
                    .write_register(foci_tmc4671::registers::MODE_RAMP_MODE_MOTION, 0)
                    .await;
                let err = safety_error_for_phase(phase);
                return fail_outer(phase, err);
            }
            Ok(false) => {}
            Err(_) => {
                defmt::warn!("outer safety: encoder read failed");
                return fail_outer(phase, CommissionError::Spi);
            }
        }
    }

    match phase {
        OuterPhase::MechanicalId { .. } => {
            mechanical::step_mechanical_id_with_trace(phase, buf, trace_summary_seqs, backend).await
        }
        OuterPhase::VelocityTune { .. } => {
            velocity::step_velocity_tune(
                phase,
                profile,
                mode,
                inner_lambda_us,
                theta_e_us,
                tau_residual_permille,
                inner_warning_flags,
                current_bandwidth_hz,
                backend,
            )
            .await
        }
        OuterPhase::VelocityValidate { .. } => {
            velocity::step_velocity_validate_with_trace(
                phase,
                buf,
                profile,
                mode,
                inner_lambda_us,
                theta_e_us,
                current_ringing,
                current_bandwidth_hz,
                trace_summary_seqs,
                backend,
            )
            .await
        }
        OuterPhase::PositionTune { .. } => {
            position::step_position_tune_with_trace(phase, buf, mode, trace_summary_seqs, backend)
                .await
        }
        OuterPhase::FilterTune { .. } => filter::step_filter_tune(phase, backend).await,
        OuterPhase::Commit { .. } => commit::step_commit(phase),
        OuterPhase::Done(result) => StepResult::OuterDone(result.clone()),
        OuterPhase::Failed(err) => StepResult::OuterFailed(*err),
    }
}

#[cfg(test)]
mod tests {
    use super::{
        MECHANICAL_ID_VELOCITY_CAP_COUNTS_PER_MS, MechanicalIdData, MechanicalIdState,
        OuterDiagnostics, OuterPhase, OuterSafetyLimits, VelocityTuneStrategy,
        VelocityValidateData, VelocityValidateState, check_outer_safety, mechanical, step_outer,
        velocity,
    };
    use crate::autotune::{
        CommissionError, CommissionProfile, MechanicalModel, OperatingMode, PhaseId,
        TransientBuffer, VelocityGains, VelocityValidation,
    };
    use crate::commission_dispatch::test_support::{MockBackend, SpiOp};
    use crate::commissioning::types::StepResult;
    use foci_tmc4671::registers::{
        ABN_DECODER_COUNT, ABN_DECODER_PPR, MODE_RAMP_MODE_MOTION, PID_TORQUE_FLUX_TARGET,
    };

    #[test]
    fn outer_safety_uses_mechanical_id_velocity_cap() {
        let safety = OuterSafetyLimits::new(0, 100);
        assert_eq!(
            safety.velocity_cap,
            MECHANICAL_ID_VELOCITY_CAP_COUNTS_PER_MS
        );
    }

    #[test]
    fn outer_safety_limits_scale_with_encoder_resolution() {
        let mut safety = OuterSafetyLimits::new(0, 100);

        safety.configure_encoder_resolution(8_000);

        assert_eq!(safety.counts_per_rev, 8_000);
        assert_eq!(safety.velocity_cap, 48);
        assert_eq!(safety.position_window, 16_000);
    }

    #[test]
    fn outer_safety_normalizes_velocity_by_elapsed_time() {
        pollster::block_on(async {
            let mut safety = OuterSafetyLimits::new(0, 100);
            safety.counts_per_rev = 4000;
            safety.prev_encoder_count = 2328;
            safety.last_sample_us = 100_000;
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 124_909;
            backend.encoder_count_value = 2525; // delta=197 over 24.909ms => ~7 counts/ms

            let violated = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("encoder read should succeed");

            assert!(
                !violated,
                "time-normalized velocity should stay below the cap"
            );
            assert_eq!(safety.prev_encoder_count, 2525);
            assert_eq!(safety.last_sample_us, 124_909);
            assert_eq!(safety.unwrapped_position, 197);
        });
    }

    #[test]
    fn outer_safety_read_error_fails_with_spi() {
        pollster::block_on(async {
            let mut safety = OuterSafetyLimits::new(0, 2448);
            safety.counts_per_rev = 4000;
            safety.last_sample_us = 1_000;
            let mut phase = OuterPhase::MechanicalId {
                state: MechanicalIdState::ForwardCapture {
                    next_sample_us: 2_000,
                    capture_start_us: 1_000,
                },
                data: MechanicalIdData::new(0, 367),
                safety,
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 2_000;
            backend.fail_on_read = Some(ABN_DECODER_COUNT.addr());

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
                StepResult::OuterFailed(CommissionError::Spi)
            ));
            assert!(matches!(phase, OuterPhase::Failed(CommissionError::Spi)));
        });
    }

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
