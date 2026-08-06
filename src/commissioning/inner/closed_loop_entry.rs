//! Phase 8: Closed-loop entry.
//!
//! Final phase before motion: writes the inner-loop (flux/torque/velocity/
//! position) PID gains and velocity limit, raises the post-calibration
//! voltage limit, locks `PHI_E_SELECTION` to the ABN encoder, zeros the
//! position registers and PID integrators, programs `STEP_WIDTH` and
//! biquad filters, enters position mode, and runs a 30 ms two-sample
//! stability check that aborts on runaway or large drift.

use super::InnerPhase;
use crate::autotune::{
    CLOSED_LOOP_ENTRY_STATUS_FAIL_DRIFT, CLOSED_LOOP_ENTRY_STATUS_FAIL_RUNAWAY,
    CLOSED_LOOP_ENTRY_STATUS_PASS, CLOSED_LOOP_ENTRY_STATUS_WARN_DRIFT, ClosedLoopEntryState,
    CommissionError, CommissionStatus, HOLD_GATE_DWELL_US, HOLD_GATE_MAX_SAMPLES,
    HOLD_GATE_SAMPLE_PERIOD_US, HOLD_GATE_STATUS_PASS, HoldGateAccumulator, HoldGateMetrics,
    HoldGateSample, InnerSequence, hold_status_failure_reason,
};
use crate::commissioning::types::{
    AdcCalResult, AdcResidualEvidence, CommissionResult, CommissioningBackend, InnerResult,
    StepResult,
};
use crate::selftest::{CommissionDiagnostic, CommissionDiagnostics};
use crate::tmc::{decode_signed_lower_half, decode_signed_upper_half};
use foci_tmc4671::pid::{PidIGain, PidPGain};

/// `phi_m_abn` feedback (low byte) with the standard velocity meter (high byte).
///
/// Datasheet 4.6.11 offers two velocity algorithms. Meter 1 synchronises
/// sampling to the PWM carrier, giving a 40 us update but a quantum of one
/// encoder count per carrier period -- 375 RPM at 25 kHz and 4000 counts/rev.
/// That put the commissioning target below the `389/2` crossover, where the
/// velocity loop's error is dominated by the quantisation excursion instead of
/// by the target, inflating measured peak controller output 3.47x.
///
/// Meter 0 differentiates at ~4369.067 Hz for a 229 us update and a 65.5 RPM
/// quantum, moving the crossover to ~33 RPM -- below the default commissioning
/// target on any travel and rotation distance. It also removes the reason the
/// 200 Hz velocity biquad existed, and that filter's ~796 us time constant
/// dominated loop lag: 229 us total here against roughly 836 us before.
///
/// The datasheet pairs meter 0 with the standard PI structure and meter 1 with
/// the advanced structure this firmware selects (`MODE_PID_TYPE = 1`), warning
/// that the loop "might tend to be unstable due to non-matched sampling". That
/// pairing is untested rather than prohibited, and the alternatives were worse:
/// shortening `MOVING_STROKE_US` cannot reach the crossover at default travel
/// (it needs a stroke below the 256-sample analysis tail), and adopting the
/// standard PI structure would invalidate the advanced-PI decay constant
/// `tau = 65536 / (I_raw * f_pwm)` that the I-rung seeding depends on.
/// Closed-loop entry's stability and runaway thresholds are the gate.
const CLOSED_LOOP_VELOCITY_SELECTION: u32 = 0x0000_0009;
// phi_m_abn selects the mechanical ABN angle directly; no pole-pair scaling applies.
const CLOSED_LOOP_POSITION_SELECTION: u32 = 9;
const CLOSED_LOOP_PWM_SV_CHOP: u32 = 7;
const CLOSED_LOOP_MOTOR_TYPE_N_POLE_PAIRS: u32 = (2 << 16) | 50;
const CLOSED_LOOP_PHI_E_SELECTION: u32 = 3;
const CLOSED_LOOP_POSITION_MODE: u32 = 0x8000_0003;
const CLOSED_LOOP_ENTRY_STABILITY_THRESHOLD: u32 = 2;
const CLOSED_LOOP_ENTRY_BOUNDED_DRIFT_WARN_THRESHOLD: u32 = 64;
const CLOSED_LOOP_ENTRY_RUNAWAY_THRESHOLD: u32 = 32760; // ~40 steps
#[allow(dead_code)]
pub(super) const CLOSED_LOOP_ENTRY_MAX_REGISTER_OPS_PER_ARM: usize = 48;

/// Number of attempts allowed for a bounded-drift `ClosedLoopEntryUnstable`
/// failure before surfacing a hard failure. Runaway failures (position error
/// large and still growing -- the signature of wrong-sign feedback or genuine
/// instability) never retry regardless of this budget.
const CLOSED_LOOP_ENTRY_MAX_ATTEMPTS: u8 = 3;

const DIAG_CLOSED_LOOP_ENTRY_RESULT: u8 = 1;

fn push_closed_loop_entry_result(
    diagnostics: &mut CommissionDiagnostics,
    entry_status: u8,
    position_1: i32,
    position_2: i32,
    drift: u32,
) {
    diagnostics.push(CommissionDiagnostic {
        phase: crate::autotune::PhaseId::ClosedLoopEntry.to_u8(),
        code: DIAG_CLOSED_LOOP_ENTRY_RESULT,
        status: entry_status,
        value0: position_1 as u32,
        value1: position_2 as u32,
        value2: drift,
    });
}

fn gain_registers(
    inner: crate::pid::InnerLoopGains,
    outer: crate::pid::OuterLoopGains,
) -> (u32, u32, u32, u32) {
    let (flux_reg, torque_reg) = inner.to_registers();
    let (velocity_reg, position_reg) = outer.to_registers();
    (flux_reg, torque_reg, velocity_reg, position_reg)
}

async fn verify_closed_loop_armed_state<B: CommissioningBackend>(
    backend: &mut B,
) -> Result<(), ()> {
    use foci_tmc4671::registers::*;

    let physical_step_width = match backend.configured_physical_step_width() {
        Some(value) if value != 0 => value,
        _ => {
            defmt::error!("closed-loop: configured physical STEP_WIDTH is missing or zero");
            return Err(());
        }
    };
    let encoder_ppr = backend.mechanical_encoder_quadrature_counts_per_rev();
    if encoder_ppr == 0 {
        defmt::error!("closed-loop: ABN_DECODER_PPR expected value is zero");
        return Err(());
    }

    let checks = [
        (PIDOUT_UQ_UD_LIMITS, backend.operational_voltage_limit()),
        (PWM_SV_CHOP, CLOSED_LOOP_PWM_SV_CHOP),
        (MODE_RAMP_MODE_MOTION, CLOSED_LOOP_POSITION_MODE),
        (MOTOR_TYPE_N_POLE_PAIRS, CLOSED_LOOP_MOTOR_TYPE_N_POLE_PAIRS),
        (PHI_E_SELECTION, CLOSED_LOOP_PHI_E_SELECTION),
        (POSITION_SELECTION, CLOSED_LOOP_POSITION_SELECTION),
        (VELOCITY_SELECTION, CLOSED_LOOP_VELOCITY_SELECTION),
        (ABN_DECODER_PPR, encoder_ppr),
        (ADC_I_SELECT, backend.tmc_config().adc_i_select),
        (
            PID_TORQUE_FLUX_LIMITS,
            backend.current_limit().register_value(),
        ),
        // STEP_WIDTH consumes position units per emitted physical TMC pulse.
        (STEP_WIDTH, physical_step_width),
    ];

    for (register, expected) in checks {
        match backend.read_register(register).await {
            Ok(value) if value == expected => {}
            Ok(value) => {
                defmt::error!(
                    "closed-loop: register {} readback {}, expected {}",
                    register.addr(),
                    value,
                    expected
                );
                return Err(());
            }
            Err(_) => {
                defmt::error!("closed-loop: register {} read failed", register.addr());
                return Err(());
            }
        }
    }

    Ok(())
}

fn saturating_u8(value: u16) -> u8 {
    u8::try_from(value).unwrap_or(u8::MAX)
}

fn saturating_u16(value: u32) -> u16 {
    u16::try_from(value).unwrap_or(u16::MAX)
}

fn copy_hold_metrics(inner: &mut InnerResult, metrics: HoldGateMetrics) {
    inner.current_hold_status = metrics.status;
    inner.current_hold_warning_flags = metrics.warning_flags;
    inner.current_hold_elapsed_us = metrics.elapsed_us;
    inner.current_hold_requested_sample_period_us = metrics.requested_sample_period_us;
    inner.current_hold_sample_count = saturating_u8(metrics.sample_count);
    inner.current_hold_position_span_count = saturating_u16(metrics.position_span_count);
    inner.current_hold_position_drift_count = saturating_u16(metrics.position_drift_count);
    inner.current_hold_torque_mean_count = metrics.torque_mean_count;
    inner.current_hold_torque_rms_count = metrics.torque_rms_count;
    inner.current_hold_torque_peak_to_peak_count = metrics.torque_peak_to_peak_count;
    inner.current_hold_torque_crossing_count = saturating_u8(metrics.torque_crossing_count);
    inner.current_hold_flux_mean_count = metrics.flux_mean_count;
    inner.current_hold_flux_rms_count = metrics.flux_rms_count;
    inner.current_hold_flux_peak_to_peak_count = metrics.flux_peak_to_peak_count;
    inner.current_hold_flux_crossing_count = saturating_u8(metrics.flux_crossing_count);
    inner.current_hold_status_flags_or = metrics.status_flags_or;
    inner.current_hold_actionable_status_count = saturating_u8(metrics.actionable_status_count);
}

#[expect(
    clippy::too_many_arguments,
    reason = "closed-loop terminal result mirrors the carried state fields"
)]
fn finish_commission(
    inner: InnerResult,
    velocity_p: PidPGain,
    velocity_i: PidIGain,
    position_p: PidPGain,
    position_i: PidIGain,
    velocity_limit: u32,
    velocity_filter_hz: u16,
    torque_filter_hz: u16,
    position_filter_hz: u16,
    flux_filter_hz: u16,
) -> StepResult {
    StepResult::CommissionDone(CommissionResult {
        inner,
        fallback_velocity_p: velocity_p.raw(),
        fallback_velocity_i: velocity_i.raw(),
        fallback_position_p: position_p.raw(),
        fallback_position_i: position_i.raw(),
        fallback_velocity_limit: velocity_limit,
        velocity_filter_hz,
        torque_filter_hz,
        position_filter_hz,
        flux_filter_hz,
    })
}

#[expect(
    clippy::too_many_arguments,
    reason = "failure helper preserves the same terminal state as success"
)]
async fn fail_sustained_hold<B: CommissioningBackend>(
    mut inner: InnerResult,
    backend: &mut B,
    failure_reason: u8,
    velocity_p: PidPGain,
    velocity_i: PidIGain,
    position_p: PidPGain,
    position_i: PidIGain,
    velocity_limit: u32,
    velocity_filter_hz: u16,
    torque_filter_hz: u16,
    position_filter_hz: u16,
    flux_filter_hz: u16,
) -> StepResult {
    crate::tmc_safe_state::restore_safe_state_via_backend(backend).await;
    inner.status = CommissionStatus::Failed(CommissionError::CurrentHoldFailed);
    inner.current_loop_failure_reason = failure_reason;
    finish_commission(
        inner,
        velocity_p,
        velocity_i,
        position_p,
        position_i,
        velocity_limit,
        velocity_filter_hz,
        torque_filter_hz,
        position_filter_hz,
        flux_filter_hz,
    )
}

fn copy_closed_loop_entry_metrics(
    inner: &mut InnerResult,
    entry_status: u8,
    position_1: i32,
    position_2: i32,
    drift: u32,
    runaway: bool,
) {
    inner.closed_loop_entry_status = entry_status;
    inner.closed_loop_entry_position_1 = position_1;
    inner.closed_loop_entry_position_2 = position_2;
    inner.closed_loop_entry_drift_count = saturating_u16(drift);
    inner.closed_loop_entry_threshold_count = CLOSED_LOOP_ENTRY_STABILITY_THRESHOLD as u16;
    inner.closed_loop_entry_runaway = u8::from(runaway);
}

#[expect(
    clippy::too_many_arguments,
    reason = "failure helper preserves the same terminal state as success"
)]
async fn fail_closed_loop_entry<B: CommissioningBackend>(
    inner_result: &mut Option<InnerResult>,
    backend: &mut B,
    entry_status: u8,
    position_1: i32,
    position_2: i32,
    drift: u32,
    runaway: bool,
    velocity_p: PidPGain,
    velocity_i: PidIGain,
    position_p: PidPGain,
    position_i: PidIGain,
    velocity_limit: u32,
    velocity_filter_hz: u16,
    torque_filter_hz: u16,
    position_filter_hz: u16,
    flux_filter_hz: u16,
) -> Option<StepResult> {
    let mut inner = inner_result.take()?;

    crate::tmc_safe_state::restore_safe_state_via_backend(backend).await;
    inner.status = CommissionStatus::Failed(CommissionError::ClosedLoopEntryUnstable);
    inner.current_loop_failure_reason = crate::autotune::CURRENT_LOOP_FAILURE_CLOSED_LOOP_ENTRY;
    copy_closed_loop_entry_metrics(
        &mut inner,
        entry_status,
        position_1,
        position_2,
        drift,
        runaway,
    );
    Some(finish_commission(
        inner,
        velocity_p,
        velocity_i,
        position_p,
        position_i,
        velocity_limit,
        velocity_filter_hz,
        torque_filter_hz,
        position_filter_hz,
        flux_filter_hz,
    ))
}

/// Advance closed-loop entry sub-state by one cooperative tick.
pub(crate) async fn step_closed_loop_entry<B: CommissioningBackend>(
    phase: &mut InnerPhase,
    sequence: InnerSequence,
    adc_cal: &mut Option<AdcCalResult>,
    diagnostics: &mut crate::selftest::CommissionDiagnostics,
    backend: &mut B,
) -> StepResult {
    use foci_tmc4671::registers::*;

    let (
        cle_state,
        flux_p,
        flux_i,
        torque_p,
        torque_i,
        velocity_p,
        velocity_i,
        position_p,
        position_i,
        velocity_limit,
        velocity_filter_hz,
        torque_filter_hz,
        position_filter_hz,
        flux_filter_hz,
        inner_result,
        attempt,
    ) = match phase {
        InnerPhase::ClosedLoopEntry {
            state,
            flux_p,
            flux_i,
            torque_p,
            torque_i,
            velocity_p,
            velocity_i,
            position_p,
            position_i,
            velocity_limit,
            velocity_filter_hz,
            torque_filter_hz,
            position_filter_hz,
            flux_filter_hz,
            inner_result,
            attempt,
        } => (
            state,
            *flux_p,
            *flux_i,
            *torque_p,
            *torque_i,
            *velocity_p,
            *velocity_i,
            *position_p,
            *position_i,
            *velocity_limit,
            *velocity_filter_hz,
            *torque_filter_hz,
            *position_filter_hz,
            *flux_filter_hz,
            inner_result,
            *attempt,
        ),
        _ => return StepResult::ReadyNow,
    };

    match cle_state {
        ClosedLoopEntryState::WriteGains => {
            let (flux_reg, torque_reg, velocity_reg, position_reg) = gain_registers(
                crate::pid::InnerLoopGains {
                    flux_p,
                    flux_i,
                    torque_p,
                    torque_i,
                },
                crate::pid::OuterLoopGains {
                    position_p,
                    position_i,
                    velocity_p,
                    velocity_i,
                },
            );
            // 1. Flux and torque PID gains.
            if backend
                .write_register(PID_FLUX_P_FLUX_I, flux_reg)
                .await
                .is_err()
                || backend
                    .write_register(PID_TORQUE_P_TORQUE_I, torque_reg)
                    .await
                    .is_err()
            {
                return super::fail_inner(phase, CommissionError::Spi);
            }

            // 2. Velocity PID gains.
            if backend
                .write_register(PID_VELOCITY_P_VELOCITY_I, velocity_reg)
                .await
                .is_err()
            {
                return super::fail_inner(phase, CommissionError::Spi);
            }

            // 3. Position PID gains.
            if backend
                .write_register(PID_POSITION_P_POSITION_I, position_reg)
                .await
                .is_err()
            {
                return super::fail_inner(phase, CommissionError::Spi);
            }

            *cle_state = ClosedLoopEntryState::WriteLimits;
            StepResult::ReadyNow
        }

        ClosedLoopEntryState::WriteLimits => {
            // 4. Velocity limit.
            if backend
                .write_register(PID_VELOCITY_LIMIT, velocity_limit)
                .await
                .is_err()
            {
                return super::fail_inner(phase, CommissionError::Spi);
            }

            // 5. Raise voltage limit for actual operation.
            // ADC cal sets it to 1024 as a safety measure.
            let voltage_limit = backend.operational_voltage_limit();
            if backend
                .write_register(PIDOUT_UQ_UD_LIMITS, voltage_limit)
                .await
                .is_err()
            {
                return super::fail_inner(phase, CommissionError::Spi);
            }
            let voltage_limit_readback = backend.read_register(PIDOUT_UQ_UD_LIMITS).await;
            match voltage_limit_readback {
                Ok(value) if value == voltage_limit => {}
                _ => {
                    defmt::error!("closed-loop: PIDOUT_UQ_UD_LIMITS readback mismatch");
                    return super::fail_inner(phase, CommissionError::Spi);
                }
            }

            *cle_state = ClosedLoopEntryState::VerifyPhiE;
            StepResult::ReadyNow
        }

        ClosedLoopEntryState::VerifyPhiE => {
            // Set PHI_E_SELECTION to ABN encoder (3). For Calibrate this
            // is already 3 from EncoderAlign::Commit; for Commission the
            // intervening ElectricalId/CurrentTune phases reset it to 1
            // (external phi_e for voltage-mode sweeps).
            if backend
                .write_register(PHI_E_SELECTION, CLOSED_LOOP_PHI_E_SELECTION)
                .await
                .is_err()
            {
                return super::fail_inner(phase, CommissionError::Spi);
            }
            match backend.read_register(PHI_E_SELECTION).await {
                Ok(CLOSED_LOOP_PHI_E_SELECTION) => {}
                Ok(val) => {
                    defmt::error!("closed-loop: PHI_E_SELECTION={}, expected 3", val);
                    return super::fail_inner(phase, CommissionError::Spi);
                }
                Err(_) => {
                    return super::fail_inner(phase, CommissionError::Spi);
                }
            }

            *cle_state = ClosedLoopEntryState::ZeroAndClear;
            StepResult::ReadyNow
        }

        ClosedLoopEntryState::ZeroAndClear => {
            // 6. Zero position (also zeros target per datasheet 0x6B).
            if backend
                .write_register(PID_POSITION_ACTUAL, 0)
                .await
                .is_err()
            {
                return super::fail_inner(phase, CommissionError::Spi);
            }

            // Clear PID integrators. These must succeed — entering
            // position mode with stale error sums causes transients.
            let integrators_ok = async {
                crate::commissioning::helpers::write_sub_register_checked(
                    backend,
                    PID_ERROR_PID_TORQUE_ERROR_SUM,
                    0,
                )
                .await?;
                crate::commissioning::helpers::write_sub_register_checked(
                    backend,
                    PID_ERROR_PID_FLUX_ERROR_SUM,
                    0,
                )
                .await?;
                crate::commissioning::helpers::write_sub_register_checked(
                    backend,
                    PID_ERROR_PID_VELOCITY_ERROR_SUM,
                    0,
                )
                .await?;
                crate::commissioning::helpers::write_sub_register_checked(
                    backend,
                    PID_ERROR_PID_POSITION_ERROR_SUM,
                    0,
                )
                .await?;
                backend
                    .write_register(STATUS_FLAGS, 0)
                    .await
                    .map_err(|_| ())
            };
            if integrators_ok.await.is_err() {
                return super::fail_inner(phase, CommissionError::Spi);
            }

            *cle_state = ClosedLoopEntryState::WritePositionConfig;
            StepResult::ReadyNow
        }

        ClosedLoopEntryState::WritePositionConfig => {
            // 7. Explicit selection setup:
            // - velocity source 9 uses ABN mechanical angle
            // - velocity meter 1 uses the advanced/PWM-synchronized meter
            // - position source 9 uses ABN mechanical angle
            if backend
                .write_register(VELOCITY_SELECTION, CLOSED_LOOP_VELOCITY_SELECTION)
                .await
                .is_err()
                || backend
                    .write_register(POSITION_SELECTION, CLOSED_LOOP_POSITION_SELECTION)
                    .await
                    .is_err()
            {
                return super::fail_inner(phase, CommissionError::Spi);
            }
            match backend.read_register(POSITION_SELECTION).await {
                Ok(CLOSED_LOOP_POSITION_SELECTION) => {}
                _ => {
                    defmt::error!("closed-loop: POSITION_SELECTION readback mismatch");
                    return super::fail_inner(phase, CommissionError::Spi);
                }
            }

            // 8. STEP_WIDTH consumes position units per emitted physical TMC pulse.
            let physical_step_width = match backend.configured_physical_step_width() {
                Some(value) if value != 0 => value,
                _ => {
                    defmt::error!("closed-loop: configured physical STEP_WIDTH is missing or zero");
                    return super::fail_inner(phase, CommissionError::Spi);
                }
            };
            if backend
                .write_register(STEP_WIDTH, physical_step_width)
                .await
                .is_err()
            {
                return super::fail_inner(phase, CommissionError::Spi);
            }
            // Readback verification.
            match backend.read_register(STEP_WIDTH).await {
                Ok(val) if val == physical_step_width => {}
                _ => {
                    defmt::error!("closed-loop: STEP_WIDTH readback mismatch");
                    return super::fail_inner(phase, CommissionError::Spi);
                }
            }
            defmt::info!("STEP_WIDTH(0x78) = {} written", physical_step_width);

            *cle_state = ClosedLoopEntryState::WriteFilters;
            StepResult::ReadyNow
        }

        ClosedLoopEntryState::WriteFilters => {
            let pwm_maxcnt = backend.pwm_maxcnt();

            // Best-effort filter writes (warn on failure, don't abort).
            if velocity_filter_hz > 0 {
                crate::commissioning::helpers::write_biquad_velocity(
                    backend,
                    velocity_filter_hz,
                    pwm_maxcnt,
                )
                .await;
            } else {
                crate::commissioning::helpers::write_sub_register(
                    backend,
                    CONFIG_BIQUAD_V_ENABLE,
                    0,
                )
                .await;
            }

            if torque_filter_hz > 0 {
                crate::commissioning::helpers::write_biquad_torque(
                    backend,
                    torque_filter_hz,
                    pwm_maxcnt,
                )
                .await;
            } else {
                crate::commissioning::helpers::write_sub_register(
                    backend,
                    CONFIG_BIQUAD_T_ENABLE,
                    0,
                )
                .await;
            }

            if position_filter_hz > 0 {
                crate::commissioning::helpers::write_biquad_position(
                    backend,
                    position_filter_hz,
                    pwm_maxcnt,
                )
                .await;
            } else {
                crate::commissioning::helpers::write_sub_register(
                    backend,
                    CONFIG_BIQUAD_X_ENABLE,
                    0,
                )
                .await;
            }

            if flux_filter_hz > 0 {
                crate::commissioning::helpers::write_biquad_flux(
                    backend,
                    flux_filter_hz,
                    pwm_maxcnt,
                )
                .await;
            } else {
                crate::commissioning::helpers::write_sub_register(
                    backend,
                    CONFIG_BIQUAD_F_ENABLE,
                    0,
                )
                .await;
            }

            *cle_state = ClosedLoopEntryState::EnterPositionMode;
            StepResult::ReadyNow
        }

        ClosedLoopEntryState::EnterPositionMode => {
            // MODE_PID_TYPE=1 (bit 31), MODE_MOTION=3 (position).
            if backend
                .write_register(MODE_RAMP_MODE_MOTION, CLOSED_LOOP_POSITION_MODE)
                .await
                .is_err()
            {
                return super::fail_inner(phase, CommissionError::Spi);
            }
            if verify_closed_loop_armed_state(backend).await.is_err() {
                return super::fail_inner(phase, CommissionError::Spi);
            }
            if crate::tmc_control::lifecycle::settle_closed_loop_hold_before_enable(backend)
                .await
                .is_err()
            {
                return super::fail_inner(phase, CommissionError::Spi);
            }
            backend.set_enable_pin(true);

            // Schedule stability check after 10ms.
            let now = backend.elapsed_us();
            let Some(deadline_us) = now.checked_add(10_000) else {
                return super::fail_inner(phase, CommissionError::InvalidSchedule);
            };
            *cle_state = ClosedLoopEntryState::StabilityCheck1 {
                position_1: 0,
                deadline_us,
            };
            StepResult::At(deadline_us)
        }

        ClosedLoopEntryState::StabilityCheck1 {
            position_1: _,
            deadline_us,
        } => {
            if backend.elapsed_us() < *deadline_us {
                return StepResult::At(*deadline_us);
            }

            // Read first position sample.
            let pos1 = match backend.read_register(PID_POSITION_ACTUAL).await {
                Ok(val) => val as i32,
                Err(_) => return super::fail_inner(phase, CommissionError::Spi),
            };
            let now = backend.elapsed_us();
            let Some(next_deadline_us) = now.checked_add(20_000) else {
                return super::fail_inner(phase, CommissionError::InvalidSchedule);
            };
            *cle_state = ClosedLoopEntryState::StabilityCheck2 {
                position_1: pos1,
                deadline_us: next_deadline_us,
            };
            StepResult::At(next_deadline_us)
        }

        ClosedLoopEntryState::StabilityCheck2 {
            position_1,
            deadline_us,
        } => {
            if backend.elapsed_us() < *deadline_us {
                return StepResult::At(*deadline_us);
            }

            // Read second position sample and check for runaway.
            let pos2 = match backend.read_register(PID_POSITION_ACTUAL).await {
                Ok(val) => val as i32,
                Err(_) => {
                    return super::fail_inner(phase, CommissionError::Spi);
                }
            };

            // Runaway detection: if position error is large and growing,
            // the feedback sign is wrong.
            if pos2.unsigned_abs() > CLOSED_LOOP_ENTRY_RUNAWAY_THRESHOLD
                && pos2.unsigned_abs() > position_1.unsigned_abs()
            {
                defmt::error!(
                    "closed-loop: runaway detected: pos1={}, pos2={}",
                    position_1,
                    pos2
                );
                let drift = (pos2.wrapping_sub(*position_1)).unsigned_abs();
                push_closed_loop_entry_result(
                    diagnostics,
                    CLOSED_LOOP_ENTRY_STATUS_FAIL_RUNAWAY,
                    *position_1,
                    pos2,
                    drift,
                );
                if matches!(sequence, InnerSequence::Commission)
                    && let Some(result) = fail_closed_loop_entry(
                        inner_result,
                        backend,
                        CLOSED_LOOP_ENTRY_STATUS_FAIL_RUNAWAY,
                        *position_1,
                        pos2,
                        drift,
                        true,
                        velocity_p,
                        velocity_i,
                        position_p,
                        position_i,
                        velocity_limit,
                        velocity_filter_hz,
                        torque_filter_hz,
                        position_filter_hz,
                        flux_filter_hz,
                    )
                    .await
                {
                    return result;
                }
                let _ = backend.write_register(MODE_RAMP_MODE_MOTION, 0).await;
                return super::fail_inner(phase, CommissionError::ClosedLoopEntryUnstable);
            }

            // Stability check: a small two-sample drift can be a bounded
            // hunt. In commissioning, keep it as evidence and let the
            // sustained-hold sampler decide with a longer window; larger
            // walk-off and all non-commissioning drift remain hard failures.
            let drift = (pos2.wrapping_sub(*position_1)).unsigned_abs();
            if drift > CLOSED_LOOP_ENTRY_STABILITY_THRESHOLD
                && matches!(sequence, InnerSequence::Commission)
                && drift <= CLOSED_LOOP_ENTRY_BOUNDED_DRIFT_WARN_THRESHOLD
            {
                defmt::warn!(
                    "closed-loop: bounded entry drift: pos1={}, pos2={}, drift={}",
                    position_1,
                    pos2,
                    drift
                );
                if let Some(inner) = inner_result.as_mut() {
                    copy_closed_loop_entry_metrics(
                        inner,
                        CLOSED_LOOP_ENTRY_STATUS_WARN_DRIFT,
                        *position_1,
                        pos2,
                        drift,
                        false,
                    );
                }
                *cle_state = ClosedLoopEntryState::SustainedHoldStart;
                return StepResult::ReadyNow;
            }

            if drift > CLOSED_LOOP_ENTRY_STABILITY_THRESHOLD {
                defmt::error!(
                    "closed-loop: unstable (attempt {}): pos1={}, pos2={}, drift={}",
                    attempt,
                    position_1,
                    pos2,
                    drift
                );
                push_closed_loop_entry_result(
                    diagnostics,
                    CLOSED_LOOP_ENTRY_STATUS_FAIL_DRIFT,
                    *position_1,
                    pos2,
                    drift,
                );
                let _ = backend.write_register(MODE_RAMP_MODE_MOTION, 0).await;

                let next_attempt = attempt + 1;
                if next_attempt < CLOSED_LOOP_ENTRY_MAX_ATTEMPTS {
                    let InnerPhase::ClosedLoopEntry { state, attempt, .. } = phase else {
                        unreachable!("phase was just matched as ClosedLoopEntry");
                    };
                    *state = ClosedLoopEntryState::WriteGains;
                    *attempt = next_attempt;
                    return StepResult::ReadyNow;
                }

                if matches!(sequence, InnerSequence::Commission)
                    && let Some(result) = fail_closed_loop_entry(
                        inner_result,
                        backend,
                        CLOSED_LOOP_ENTRY_STATUS_FAIL_DRIFT,
                        *position_1,
                        pos2,
                        drift,
                        false,
                        velocity_p,
                        velocity_i,
                        position_p,
                        position_i,
                        velocity_limit,
                        velocity_filter_hz,
                        torque_filter_hz,
                        position_filter_hz,
                        flux_filter_hz,
                    )
                    .await
                {
                    return result;
                }
                return super::fail_inner(phase, CommissionError::ClosedLoopEntryUnstable);
            }

            defmt::info!("closed-loop position mode active (engine)");

            match sequence {
                InnerSequence::Calibrate => {
                    let adc_result = adc_cal.clone().unwrap_or(AdcCalResult {
                        adc_i0: 0,
                        adc_i1: 0,
                        encoder_count: 0,
                        encoder_alignment_electrical_residual_counts: 0,
                        encoder_alignment_stability_counts: 0,
                        encoder_alignment_movement_counts: 0,
                        encoder_alignment_min_movement_counts: 0,
                        encoder_alignment_counts_per_electrical_rev: 0,
                        adc_residual_chop0: AdcResidualEvidence::EMPTY,
                        adc_residual_chop7: AdcResidualEvidence::EMPTY,
                    });
                    StepResult::CalibrateDone(adc_result)
                }
                InnerSequence::Commission => {
                    if let Some(inner) = inner_result.as_mut() {
                        copy_closed_loop_entry_metrics(
                            inner,
                            CLOSED_LOOP_ENTRY_STATUS_PASS,
                            *position_1,
                            pos2,
                            drift,
                            false,
                        );
                    }
                    *cle_state = ClosedLoopEntryState::SustainedHoldStart;
                    StepResult::ReadyNow
                }
                InnerSequence::Selftest => {
                    // Selftest never reaches ClosedLoopEntry.
                    unreachable!()
                }
            }
        }

        ClosedLoopEntryState::SustainedHoldStart => {
            let now = backend.elapsed_us();
            let Some(deadline_us) = now.checked_add(HOLD_GATE_DWELL_US as u64) else {
                return super::fail_inner(phase, CommissionError::InvalidSchedule);
            };
            *cle_state = ClosedLoopEntryState::SustainedHoldSample {
                accumulator: HoldGateAccumulator::commissioning(HOLD_GATE_SAMPLE_PERIOD_US as u16),
                started_us: now,
                next_sample_us: now,
                deadline_us,
                requested_sample_period_us: HOLD_GATE_SAMPLE_PERIOD_US as u16,
                sample_overrun: false,
            };
            StepResult::ReadyNow
        }

        ClosedLoopEntryState::SustainedHoldSample {
            accumulator,
            started_us,
            next_sample_us,
            deadline_us,
            requested_sample_period_us,
            sample_overrun,
        } => {
            let now = backend.elapsed_us();
            if now < *next_sample_us {
                return StepResult::At(*next_sample_us);
            }

            let pid_torque_flux_actual = match backend.read_register(PID_TORQUE_FLUX_ACTUAL).await {
                Ok(value) => value,
                Err(_) => {
                    let _ = backend.write_register(MODE_RAMP_MODE_MOTION, 0).await;
                    return super::fail_inner(phase, CommissionError::Spi);
                }
            };
            let position_count = match backend.read_register(PID_POSITION_ACTUAL).await {
                Ok(value) => value as i32,
                Err(_) => {
                    let _ = backend.write_register(MODE_RAMP_MODE_MOTION, 0).await;
                    return super::fail_inner(phase, CommissionError::Spi);
                }
            };
            let status_flags = match backend.read_register(STATUS_FLAGS).await {
                Ok(value) => value,
                Err(_) => {
                    let _ = backend.write_register(MODE_RAMP_MODE_MOTION, 0).await;
                    return super::fail_inner(phase, CommissionError::Spi);
                }
            };

            accumulator.push(HoldGateSample {
                elapsed_us: now.saturating_sub(*started_us).min(u64::from(u32::MAX)) as u32,
                position_count,
                torque_count: decode_signed_upper_half(pid_torque_flux_actual),
                flux_count: decode_signed_lower_half(pid_torque_flux_actual),
                status_flags,
            });

            if now > *next_sample_us {
                *sample_overrun = true;
            }

            if now < *deadline_us && accumulator.sample_count() < HOLD_GATE_MAX_SAMPLES {
                let next_deadline = match crate::commissioning::schedule::next_grid_deadline(
                    *next_sample_us,
                    u64::from(*requested_sample_period_us),
                    now,
                ) {
                    Ok(deadline) => deadline,
                    Err(error) => return super::fail_inner(phase, error),
                };
                *next_sample_us = next_deadline;
                return StepResult::At(next_deadline);
            }

            let elapsed_us = now.saturating_sub(*started_us).min(u64::from(u32::MAX)) as u32;
            let metrics = accumulator.finish(elapsed_us, false);
            let Some(mut inner) = inner_result.take() else {
                let _ = backend.write_register(MODE_RAMP_MODE_MOTION, 0).await;
                return super::fail_inner(phase, CommissionError::Spi);
            };
            copy_hold_metrics(&mut inner, metrics);

            if metrics.status != HOLD_GATE_STATUS_PASS {
                let failure_reason = hold_status_failure_reason(metrics.status)
                    .unwrap_or(crate::autotune::CURRENT_LOOP_FAILURE_HOLD_SAMPLE);
                return fail_sustained_hold(
                    inner,
                    backend,
                    failure_reason,
                    velocity_p,
                    velocity_i,
                    position_p,
                    position_i,
                    velocity_limit,
                    velocity_filter_hz,
                    torque_filter_hz,
                    position_filter_hz,
                    flux_filter_hz,
                )
                .await;
            }

            finish_commission(
                inner,
                velocity_p,
                velocity_i,
                position_p,
                position_i,
                velocity_limit,
                velocity_filter_hz,
                torque_filter_hz,
                position_filter_hz,
                flux_filter_hz,
            )
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::autotune::{
        CURRENT_VALIDATION_AXIS_TORQUE, HOLD_GATE_SAMPLE_PERIOD_US, HOLD_GATE_STATUS_PASS,
        HOLD_GATE_WARNING_SAMPLE_OVERRUN, HoldGateAccumulator, HoldGateSample,
        current_loop_actionable_status_mask,
    };
    use crate::commission_dispatch::test_support::{MockBackend, SpiOp, current_limit};
    use crate::commissioning::types::{CurrentValidationAxisEvidence, InductanceEvidence};
    use foci_tmc4671::registers::{
        ABN_DECODER_PPR, ADC_I_SELECT, MODE_RAMP_MODE_MOTION, MOTOR_TYPE_N_POLE_PAIRS,
        PHI_E_SELECTION, PID_POSITION_ACTUAL, PID_TORQUE_FLUX_ACTUAL, PID_TORQUE_FLUX_LIMITS,
        PID_VELOCITY_LIMIT, PIDOUT_UQ_UD_LIMITS, POSITION_SELECTION, PWM_SV_CHOP, STATUS_FLAGS,
        STEP_WIDTH, VELOCITY_SELECTION,
    };

    #[test]
    fn closed_loop_velocity_selection_uses_the_standard_meter_on_the_mechanical_angle() {
        use foci_tmc4671::registers::VelocitySelection;

        let selection = VelocitySelection::from(CLOSED_LOOP_VELOCITY_SELECTION);

        // Datasheet 4.6.11: meter 0 differentiates at ~4369.067 Hz, giving a
        // 65.5 RPM quantum at 4000 counts/rev against meter 1's PWM-synchronous
        // 375 RPM. The 375 RPM quantum put the commissioning target below the
        // 389/2 crossover, where the velocity loop's error is dominated by the
        // quantisation excursion rather than by the target.
        assert_eq!(
            selection.velocity_meter_selection(),
            0,
            "commissioning must run the standard velocity meter"
        );
        // The feedback source stays the mechanical ABN angle. `TmcMechanicalRpm`
        // documents that this field, not the meter, fixes the RPM domain shared
        // by PID_VELOCITY_TARGET and PID_VELOCITY_ACTUAL.
        assert_eq!(
            selection.velocity_selection(),
            9,
            "phi_m_abn must remain the velocity feedback source"
        );
    }

    fn closed_loop_phase_at(state: ClosedLoopEntryState) -> InnerPhase {
        InnerPhase::ClosedLoopEntry {
            state,
            flux_p: PidPGain::ZERO,
            flux_i: PidIGain::ZERO,
            torque_p: PidPGain::ZERO,
            torque_i: PidIGain::ZERO,
            velocity_p: PidPGain::ZERO,
            velocity_i: PidIGain::ZERO,
            position_p: PidPGain::ZERO,
            position_i: PidIGain::ZERO,
            velocity_limit: 500_000,
            velocity_filter_hz: 0,
            torque_filter_hz: 0,
            position_filter_hz: 0,
            flux_filter_hz: 0,
            inner_result: Some(sample_inner_result(CommissionStatus::Accepted)),
            attempt: 0,
        }
    }

    fn closed_loop_phase_at_without_inner(state: ClosedLoopEntryState) -> InnerPhase {
        let mut phase = closed_loop_phase_at(state);
        let InnerPhase::ClosedLoopEntry { inner_result, .. } = &mut phase else {
            unreachable!();
        };
        *inner_result = None;
        phase
    }

    fn closed_loop_phase_at_with_attempt(state: ClosedLoopEntryState, attempt: u8) -> InnerPhase {
        let mut phase = closed_loop_phase_at(state);
        let InnerPhase::ClosedLoopEntry {
            attempt: phase_attempt,
            ..
        } = &mut phase
        else {
            unreachable!();
        };
        *phase_attempt = attempt;
        phase
    }

    fn sample_inner_result(status: CommissionStatus) -> InnerResult {
        InnerResult {
            status,
            r_count_milli: 1706,
            l_count_micro: 1245,
            flux_p: 100,
            flux_i: 10,
            torque_p: 200,
            torque_i: 20,
            candidate_gains_source: crate::autotune::CURRENT_GAINS_SOURCE_MEASURED,
            candidate_axis_split_source: crate::autotune::AXIS_SPLIT_SOURCE_NONE,
            candidate_gains_tier: crate::autotune::CURRENT_GAINS_TIER_MEASURED_SYMMETRIC,
            candidate_measured_axis_split_permille: 1000,
            candidate_applied_axis_split_permille: 1000,
            candidate_axis_split_clamped: 0,
            candidate_flux_p: 100,
            candidate_flux_i: 10,
            candidate_torque_p: 200,
            candidate_torque_i: 20,
            candidate_attempt: 0,
            lambda_us: 100,
            theta_e_us: 50,
            ringing_count: 0,
            bandwidth_hz: 1000,
            tau_e_us: 730,
            inductance: InductanceEvidence::EMPTY,
            inner_warning_flags: 0,
            timing: [crate::commissioning::types::CommissionTimingSummary::NOT_RUN; 3],
            timing_summary: 0,
            resistance_selected_count_slope_milli: 1706,
            resistance_gain_path_count_slope_milli: 1706,
            resistance_axis0_count_slope_milli: 1700,
            resistance_axis1_count_slope_milli: 1712,
            resistance_axis0_intercept_count: 0,
            resistance_axis1_intercept_count: 0,
            resistance_axis0_rmse_permille: 0,
            resistance_axis1_rmse_permille: 0,
            resistance_selected_mask_axis0: 0b1111_1000,
            resistance_selected_mask_axis1: 0b1111_1000,
            resistance_axis0_signed_count_slope_milli: 1702,
            resistance_axis1_signed_count_slope_milli: 1710,
            resistance_axis0_signed_asymmetry_permille: 0,
            resistance_axis1_signed_asymmetry_permille: 0,
            resistance_axis0_drift_permille: 0,
            resistance_axis1_drift_permille: 0,
            resistance_status_flags_or: 0,
            resistance_warning_flags: 0,
            resistance_peak_abs_current_count: 1_440,
            resistance_max_abs_steady_mean_current_count: 1_280,
            resistance_current_ceiling_count: 1_600,
            resistance_power_stage_tripped: false,
            encoder_alignment_count: 0,
            encoder_alignment_electrical_residual_counts: 0,
            encoder_alignment_stability_counts: 0,
            encoder_alignment_movement_counts: 0,
            encoder_alignment_min_movement_counts: 0,
            encoder_alignment_counts_per_electrical_rev: 0,
            adc_residual_chop0: AdcResidualEvidence::EMPTY,
            adc_residual_chop7: AdcResidualEvidence::EMPTY,
            current_gains_source: crate::autotune::CURRENT_GAINS_SOURCE_MEASURED,
            axis_split_source: crate::autotune::AXIS_SPLIT_SOURCE_NONE,
            gains_tier: crate::autotune::CURRENT_GAINS_TIER_MEASURED_SYMMETRIC,
            measured_axis_split_permille: 1000,
            applied_axis_split_permille: 1000,
            axis_split_clamped: 0,
            current_validation_axes: CURRENT_VALIDATION_AXIS_TORQUE,
            retry_budget_exhausted: 0,
            current_loop_failure_reason: crate::autotune::CURRENT_LOOP_FAILURE_NONE,
            closed_loop_entry_status: crate::autotune::CLOSED_LOOP_ENTRY_STATUS_NOT_RUN,
            closed_loop_entry_position_1: 0,
            closed_loop_entry_position_2: 0,
            closed_loop_entry_drift_count: 0,
            closed_loop_entry_threshold_count: 0,
            closed_loop_entry_runaway: 0,
            current_hold_status: crate::autotune::HOLD_GATE_STATUS_NOT_RUN,
            current_hold_warning_flags: 0,
            current_hold_elapsed_us: 0,
            current_hold_requested_sample_period_us: crate::autotune::HOLD_GATE_SAMPLE_PERIOD_US
                as u16,
            current_hold_sample_count: 0,
            current_hold_position_span_count: 0,
            current_hold_position_drift_count: 0,
            current_hold_torque_mean_count: 0,
            current_hold_torque_rms_count: 0,
            current_hold_torque_peak_to_peak_count: 0,
            current_hold_torque_crossing_count: 0,
            current_hold_flux_mean_count: 0,
            current_hold_flux_rms_count: 0,
            current_hold_flux_peak_to_peak_count: 0,
            current_hold_flux_crossing_count: 0,
            current_hold_status_flags_or: 0,
            current_hold_actionable_status_count: 0,
            current_validation_step_amplitude: 128,
            current_validation_flux_step_amplitude: 128,
            current_validation_torque_step_amplitude: 128,
            current_validation_pidout_limit: 2400,
            current_validation_current_limited: 422,
            current_validation_voltage_limited: 128,
            current_validation_max_p: 3474,
            current_settled_validation_count: 0,
            current_settled_validation:
                [crate::commissioning::types::CurrentValidationSettledSampleEvidence::EMPTY;
                    crate::autotune::CURRENT_VALIDATION_SETTLED_MAX_SAMPLES],
            current_flux_validation_count: 0,
            current_flux_validation: [CurrentValidationAxisEvidence::EMPTY;
                crate::autotune::CURRENT_VALIDATION_MAX_SAMPLES_PER_AXIS],
            current_torque_validation_count: 0,
            current_torque_validation: [CurrentValidationAxisEvidence::EMPTY;
                crate::autotune::CURRENT_VALIDATION_MAX_SAMPLES_PER_AXIS],
        }
    }

    fn pack_torque_flux_actual(torque: i16, flux: i16) -> u32 {
        ((torque as u16 as u32) << 16) | flux as u16 as u32
    }

    fn seed_armed_state_registers(backend: &mut MockBackend) {
        backend.current_limit_value = current_limit(2816);
        backend.operational_voltage_limit_value = 29_000;
        backend.configured_physical_width = Some(16);

        let registers = [
            (PWM_SV_CHOP.addr(), 7),
            (MOTOR_TYPE_N_POLE_PAIRS.addr(), (2 << 16) | 50),
            (PHI_E_SELECTION.addr(), 3),
            (POSITION_SELECTION.addr(), CLOSED_LOOP_POSITION_SELECTION),
            (VELOCITY_SELECTION.addr(), CLOSED_LOOP_VELOCITY_SELECTION),
            (
                ABN_DECODER_PPR.addr(),
                backend.mechanical_encoder_quadrature_counts_per_rev_value,
            ),
            (ADC_I_SELECT.addr(), backend.tmc_config.adc_i_select),
            (
                PID_TORQUE_FLUX_LIMITS.addr(),
                backend.current_limit_value.register_value(),
            ),
            (
                STEP_WIDTH.addr(),
                backend.configured_physical_width.unwrap(),
            ),
            (
                PIDOUT_UQ_UD_LIMITS.addr(),
                backend.operational_voltage_limit_value,
            ),
            (MODE_RAMP_MODE_MOTION.addr(), 0x8000_0003),
        ];
        for (addr, value) in registers {
            backend.registers.insert(addr, value).ok();
        }
    }

    fn write_log(backend: &MockBackend) -> heapless::Vec<(u8, u32), 128> {
        let write_count = backend
            .spi_log
            .iter()
            .filter(|op| matches!(op, SpiOp::Write(_, _)))
            .count();
        let writes = backend
            .spi_log
            .iter()
            .filter_map(|op| {
                if let SpiOp::Write(addr, val) = op {
                    Some((*addr, *val))
                } else {
                    None
                }
            })
            .collect::<heapless::Vec<_, 128>>();
        assert_eq!(
            writes.len(),
            write_count,
            "write_log helper capacity truncated the SPI log"
        );
        writes
    }

    fn register_op_count(backend: &MockBackend) -> usize {
        backend
            .spi_log
            .iter()
            .filter(|op| !matches!(op, SpiOp::EnablePin(_)))
            .count()
    }

    #[test]
    fn write_position_config_programs_selection_registers() {
        pollster::block_on(async {
            let mut phase = closed_loop_phase_at(ClosedLoopEntryState::WritePositionConfig);
            let mut adc_cal = None;
            let mut backend = MockBackend::new();
            backend.configured_physical_width = Some(16);

            let result = step_closed_loop_entry(
                &mut phase,
                InnerSequence::Calibrate,
                &mut adc_cal,
                &mut CommissionDiagnostics::default(),
                &mut backend,
            )
            .await;

            assert!(matches!(result, StepResult::ReadyNow));
            assert!(matches!(
                phase,
                InnerPhase::ClosedLoopEntry {
                    state: ClosedLoopEntryState::WriteFilters,
                    ..
                }
            ));
            assert_eq!(
                write_log(&backend).as_slice(),
                &[
                    (VELOCITY_SELECTION.addr(), CLOSED_LOOP_VELOCITY_SELECTION),
                    (POSITION_SELECTION.addr(), 9),
                    (STEP_WIDTH.addr(), 16),
                ]
            );
        });
    }

    #[test]
    fn write_position_config_verifies_mechanical_selection_before_step_width() {
        pollster::block_on(async {
            let mut phase = closed_loop_phase_at(ClosedLoopEntryState::WritePositionConfig);
            let mut adc_cal = None;
            let mut backend = MockBackend::new();
            backend.configured_physical_width = Some(16);
            backend.read_override = Some((POSITION_SELECTION.addr(), 8));

            let result = step_closed_loop_entry(
                &mut phase,
                InnerSequence::Calibrate,
                &mut adc_cal,
                &mut CommissionDiagnostics::default(),
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::Spi)
            ));
            assert!(
                !write_log(&backend)
                    .as_slice()
                    .iter()
                    .any(|(addr, _)| *addr == STEP_WIDTH.addr()),
                "STEP_WIDTH must not be armed before mechanical selection verifies"
            );
        });
    }

    #[test]
    fn write_position_config_propagates_mechanical_selection_read_error() {
        pollster::block_on(async {
            let mut phase = closed_loop_phase_at(ClosedLoopEntryState::WritePositionConfig);
            let mut adc_cal = None;
            let mut backend = MockBackend::new();
            backend.configured_physical_width = Some(16);
            backend.fail_on_read = Some(POSITION_SELECTION.addr());

            let result = step_closed_loop_entry(
                &mut phase,
                InnerSequence::Calibrate,
                &mut adc_cal,
                &mut CommissionDiagnostics::default(),
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::Spi)
            ));
            assert!(
                !write_log(&backend)
                    .as_slice()
                    .iter()
                    .any(|(addr, _)| *addr == STEP_WIDTH.addr()),
                "POSITION_SELECTION read failure must block STEP_WIDTH"
            );
        });
    }

    #[test]
    fn write_position_config_uses_published_physical_width_not_legacy_mirror() {
        pollster::block_on(async {
            let mut phase = closed_loop_phase_at(ClosedLoopEntryState::WritePositionConfig);
            let mut adc_cal = None;
            let mut backend = MockBackend::new();
            backend.configured_physical_width = Some(8);

            let result = step_closed_loop_entry(
                &mut phase,
                InnerSequence::Calibrate,
                &mut adc_cal,
                &mut CommissionDiagnostics::default(),
                &mut backend,
            )
            .await;

            assert!(matches!(result, StepResult::ReadyNow));
            assert_eq!(
                write_log(&backend).as_slice(),
                &[
                    (VELOCITY_SELECTION.addr(), CLOSED_LOOP_VELOCITY_SELECTION),
                    (POSITION_SELECTION.addr(), CLOSED_LOOP_POSITION_SELECTION),
                    (STEP_WIDTH.addr(), 8),
                ]
            );
        });
    }

    #[test]
    fn write_position_config_rejects_unpublished_legacy_width() {
        pollster::block_on(async {
            let mut phase = closed_loop_phase_at(ClosedLoopEntryState::WritePositionConfig);
            let mut adc_cal = None;
            let mut backend = MockBackend::new();

            let result = step_closed_loop_entry(
                &mut phase,
                InnerSequence::Calibrate,
                &mut adc_cal,
                &mut CommissionDiagnostics::default(),
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::Spi)
            ));
            assert!(
                !write_log(&backend)
                    .as_slice()
                    .iter()
                    .any(|(addr, _)| *addr == STEP_WIDTH.addr()),
                "an unpublished legacy width must not arm STEP_WIDTH"
            );
        });
    }

    #[test]
    fn write_position_config_rejects_zero_published_physical_width() {
        pollster::block_on(async {
            let mut phase = closed_loop_phase_at(ClosedLoopEntryState::WritePositionConfig);
            let mut adc_cal = None;
            let mut backend = MockBackend::new();
            backend.configured_physical_width = Some(0);

            let result = step_closed_loop_entry(
                &mut phase,
                InnerSequence::Calibrate,
                &mut adc_cal,
                &mut CommissionDiagnostics::default(),
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::Spi)
            ));
            assert!(
                !write_log(&backend)
                    .as_slice()
                    .iter()
                    .any(|(addr, _)| *addr == STEP_WIDTH.addr()),
                "zero physical width must not arm STEP_WIDTH"
            );
        });
    }

    #[test]
    fn write_limits_uses_configured_operational_voltage_limit() {
        pollster::block_on(async {
            let mut phase = closed_loop_phase_at(ClosedLoopEntryState::WriteLimits);
            let mut adc_cal = None;
            let mut backend = MockBackend::new();
            backend.operational_voltage_limit_value = 20_000;

            let result = step_closed_loop_entry(
                &mut phase,
                InnerSequence::Calibrate,
                &mut adc_cal,
                &mut CommissionDiagnostics::default(),
                &mut backend,
            )
            .await;

            assert!(matches!(result, StepResult::ReadyNow));
            assert_eq!(
                write_log(&backend).as_slice(),
                &[
                    (PID_VELOCITY_LIMIT.addr(), 500_000),
                    (PIDOUT_UQ_UD_LIMITS.addr(), 20_000),
                ]
            );
            assert_eq!(
                backend
                    .spi_log
                    .iter()
                    .filter(|op| matches!(op, SpiOp::Read))
                    .count(),
                1
            );
            assert!(matches!(
                phase,
                InnerPhase::ClosedLoopEntry {
                    state: ClosedLoopEntryState::VerifyPhiE,
                    ..
                }
            ));
        });
    }

    #[test]
    fn enter_position_mode_accepts_expected_armed_state_readbacks() {
        pollster::block_on(async {
            let mut phase = closed_loop_phase_at(ClosedLoopEntryState::EnterPositionMode);
            let mut adc_cal = None;
            let mut backend = MockBackend::new();
            seed_armed_state_registers(&mut backend);

            let result = step_closed_loop_entry(
                &mut phase,
                InnerSequence::Calibrate,
                &mut adc_cal,
                &mut CommissionDiagnostics::default(),
                &mut backend,
            )
            .await;

            assert!(matches!(result, StepResult::At(10_000)));
            assert!(matches!(
                phase,
                InnerPhase::ClosedLoopEntry {
                    state: ClosedLoopEntryState::StabilityCheck1 { .. },
                    ..
                }
            ));
            assert!(
                backend.enable_pin,
                "position-mode checks must run energized"
            );
            assert!(
                matches!(backend.spi_log.last(), Some(SpiOp::EnablePin(true))),
                "enable pin must be asserted after closed-loop register verification"
            );
        });
    }

    #[test]
    fn stability_checks_return_exact_one_shot_deadlines() {
        pollster::block_on(async {
            let mut phase = closed_loop_phase_at(ClosedLoopEntryState::StabilityCheck1 {
                position_1: 0,
                deadline_us: 10_000,
            });
            let mut adc_cal = None;
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 9_999;

            let early = step_closed_loop_entry(
                &mut phase,
                InnerSequence::Calibrate,
                &mut adc_cal,
                &mut CommissionDiagnostics::default(),
                &mut backend,
            )
            .await;

            assert!(matches!(early, StepResult::At(10_000)));
            assert_eq!(register_op_count(&backend), 0);

            backend.elapsed_us_value = 10_000;
            backend.registers.insert(PID_POSITION_ACTUAL.addr(), 7).ok();
            let due = step_closed_loop_entry(
                &mut phase,
                InnerSequence::Calibrate,
                &mut adc_cal,
                &mut CommissionDiagnostics::default(),
                &mut backend,
            )
            .await;

            assert!(matches!(due, StepResult::At(30_000)));
            assert!(matches!(
                phase,
                InnerPhase::ClosedLoopEntry {
                    state: ClosedLoopEntryState::StabilityCheck2 {
                        position_1: 7,
                        deadline_us: 30_000,
                    },
                    ..
                }
            ));

            backend.spi_log.clear();
            backend.elapsed_us_value = 29_999;
            let second_early = step_closed_loop_entry(
                &mut phase,
                InnerSequence::Calibrate,
                &mut adc_cal,
                &mut CommissionDiagnostics::default(),
                &mut backend,
            )
            .await;

            assert!(matches!(second_early, StepResult::At(30_000)));
            assert_eq!(register_op_count(&backend), 0);
        });
    }

    #[test]
    fn filter_writes_respect_entry_operation_bound() {
        pollster::block_on(async {
            let mut phase = closed_loop_phase_at(ClosedLoopEntryState::WriteFilters);
            let InnerPhase::ClosedLoopEntry {
                velocity_filter_hz,
                torque_filter_hz,
                position_filter_hz,
                flux_filter_hz,
                ..
            } = &mut phase
            else {
                unreachable!();
            };
            *velocity_filter_hz = 100;
            *torque_filter_hz = 100;
            *position_filter_hz = 100;
            *flux_filter_hz = 100;
            let mut adc_cal = None;
            let mut backend = MockBackend::new();

            let result = step_closed_loop_entry(
                &mut phase,
                InnerSequence::Calibrate,
                &mut adc_cal,
                &mut CommissionDiagnostics::default(),
                &mut backend,
            )
            .await;

            assert!(matches!(result, StepResult::ReadyNow));
            assert_eq!(
                register_op_count(&backend),
                CLOSED_LOOP_ENTRY_MAX_REGISTER_OPS_PER_ARM
            );
        });
    }

    #[test]
    fn enter_position_mode_sets_hold_target_before_enable() {
        pollster::block_on(async {
            let mut phase = closed_loop_phase_at(ClosedLoopEntryState::EnterPositionMode);
            let mut adc_cal = None;
            let mut backend = MockBackend::new();
            seed_armed_state_registers(&mut backend);
            backend
                .registers
                .insert(PID_POSITION_ACTUAL.addr(), 11)
                .ok();
            backend
                .registers
                .insert(STATUS_FLAGS.addr(), 0xffff_ffff)
                .ok();

            let result = step_closed_loop_entry(
                &mut phase,
                InnerSequence::Calibrate,
                &mut adc_cal,
                &mut CommissionDiagnostics::default(),
                &mut backend,
            )
            .await;

            assert!(matches!(result, StepResult::At(10_000)));
            let enable_index = backend
                .spi_log
                .iter()
                .position(|op| matches!(op, SpiOp::EnablePin(true)))
                .expect("closed-loop entry should assert enable");
            let target_rewrite_index = backend
                .spi_log
                .iter()
                .position(|op| {
                    matches!(
                        op,
                        SpiOp::Write(addr, 11) if *addr == PID_POSITION_ACTUAL.addr()
                    )
                })
                .expect("closed-loop entry should rewrite PID_POSITION_ACTUAL to observed actual");
            let status_clear_index = backend
                .spi_log
                .iter()
                .position(|op| {
                    matches!(
                        op,
                        SpiOp::Write(addr, 0) if *addr == STATUS_FLAGS.addr()
                    )
                })
                .expect("closed-loop entry should clear STATUS_FLAGS before enable");
            assert!(target_rewrite_index < enable_index);
            assert!(status_clear_index < enable_index);
        });
    }

    #[test]
    fn enter_position_mode_rejects_unpublished_legacy_width() {
        pollster::block_on(async {
            let mut phase = closed_loop_phase_at(ClosedLoopEntryState::EnterPositionMode);
            let mut adc_cal = None;
            let mut backend = MockBackend::new();
            seed_armed_state_registers(&mut backend);
            backend.configured_physical_width = None;

            let result = step_closed_loop_entry(
                &mut phase,
                InnerSequence::Calibrate,
                &mut adc_cal,
                &mut CommissionDiagnostics::default(),
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::Spi)
            ));
            assert!(!backend.enable_pin, "missing scale must fail closed");
        });
    }

    #[test]
    fn enter_position_mode_rejects_divergent_legacy_width_without_enable() {
        pollster::block_on(async {
            let mut phase = closed_loop_phase_at(ClosedLoopEntryState::EnterPositionMode);
            let mut adc_cal = None;
            let mut backend = MockBackend::new();
            seed_armed_state_registers(&mut backend);
            backend.configured_physical_width = Some(8);

            let result = step_closed_loop_entry(
                &mut phase,
                InnerSequence::Calibrate,
                &mut adc_cal,
                &mut CommissionDiagnostics::default(),
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::Spi)
            ));
            assert!(!backend.enable_pin, "divergent STEP_WIDTH must fail closed");
        });
    }

    #[test]
    fn enter_position_mode_propagates_armed_state_read_error_without_enable() {
        pollster::block_on(async {
            let mut phase = closed_loop_phase_at(ClosedLoopEntryState::EnterPositionMode);
            let mut adc_cal = None;
            let mut backend = MockBackend::new();
            seed_armed_state_registers(&mut backend);
            backend.fail_on_read = Some(STEP_WIDTH.addr());

            let result = step_closed_loop_entry(
                &mut phase,
                InnerSequence::Calibrate,
                &mut adc_cal,
                &mut CommissionDiagnostics::default(),
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::Spi)
            ));
            assert!(
                !backend.enable_pin,
                "armed-state read failure must fail closed"
            );
        });
    }

    #[test]
    fn enter_position_mode_rejects_stale_armed_state_readbacks() {
        pollster::block_on(async {
            let stale_registers = [
                (PWM_SV_CHOP.addr(), 0),
                (MOTOR_TYPE_N_POLE_PAIRS.addr(), 3 << 16),
                (PHI_E_SELECTION.addr(), 1),
                (POSITION_SELECTION.addr(), 8),
                // The superseded PWM-synchronous meter: a chip that retained it
                // must not pass the armed-state readback.
                (VELOCITY_SELECTION.addr(), 0x0000_0109),
                (ABN_DECODER_PPR.addr(), 3999),
                (ADC_I_SELECT.addr(), 0),
                (PID_TORQUE_FLUX_LIMITS.addr(), 1024),
                (PIDOUT_UQ_UD_LIMITS.addr(), 16_000),
                (STEP_WIDTH.addr(), 8),
            ];

            for (addr, stale_value) in stale_registers {
                let mut phase = closed_loop_phase_at(ClosedLoopEntryState::EnterPositionMode);
                let mut adc_cal = None;
                let mut backend = MockBackend::new();
                seed_armed_state_registers(&mut backend);
                backend.registers.insert(addr, stale_value).ok();

                let result = step_closed_loop_entry(
                    &mut phase,
                    InnerSequence::Calibrate,
                    &mut adc_cal,
                    &mut CommissionDiagnostics::default(),
                    &mut backend,
                )
                .await;

                assert!(
                    matches!(result, StepResult::InnerFailed(CommissionError::Spi)),
                    "register 0x{addr:02x} stale value 0x{stale_value:08x} was accepted"
                );
            }
        });
    }

    #[test]
    fn commission_runs_sustained_hold_after_fast_stability_check() {
        pollster::block_on(async {
            let mut phase = closed_loop_phase_at(ClosedLoopEntryState::StabilityCheck2 {
                position_1: 0,
                deadline_us: 0,
            });
            let mut adc_cal = None;
            let mut backend = MockBackend::new();
            backend.registers.insert(PID_POSITION_ACTUAL.addr(), 0).ok();

            let result = step_closed_loop_entry(
                &mut phase,
                InnerSequence::Commission,
                &mut adc_cal,
                &mut CommissionDiagnostics::default(),
                &mut backend,
            )
            .await;

            assert!(matches!(
                phase,
                InnerPhase::ClosedLoopEntry {
                    state: ClosedLoopEntryState::SustainedHoldStart,
                    ..
                }
            ));
            assert!(matches!(result, StepResult::ReadyNow));
        });
    }

    #[test]
    fn commission_bounded_entry_drift_runs_sustained_hold_with_warning_evidence() {
        pollster::block_on(async {
            let mut phase = closed_loop_phase_at(ClosedLoopEntryState::StabilityCheck2 {
                position_1: 0,
                deadline_us: 0,
            });
            let mut adc_cal = None;
            let mut backend = MockBackend::new();
            backend
                .registers
                .insert(PID_POSITION_ACTUAL.addr(), 17)
                .ok();

            let result = step_closed_loop_entry(
                &mut phase,
                InnerSequence::Commission,
                &mut adc_cal,
                &mut CommissionDiagnostics::default(),
                &mut backend,
            )
            .await;

            assert!(matches!(result, StepResult::ReadyNow));
            let InnerPhase::ClosedLoopEntry {
                state: ClosedLoopEntryState::SustainedHoldStart,
                inner_result: Some(inner),
                ..
            } = phase
            else {
                panic!("expected bounded entry drift to continue into sustained hold");
            };
            assert_eq!(inner.status, CommissionStatus::Accepted);
            assert_eq!(
                inner.current_loop_failure_reason,
                crate::autotune::CURRENT_LOOP_FAILURE_NONE
            );
            assert_eq!(
                inner.closed_loop_entry_status,
                CLOSED_LOOP_ENTRY_STATUS_WARN_DRIFT
            );
            assert_eq!(inner.closed_loop_entry_position_1, 0);
            assert_eq!(inner.closed_loop_entry_position_2, 17);
            assert_eq!(inner.closed_loop_entry_drift_count, 17);
            assert_eq!(inner.closed_loop_entry_threshold_count, 2);
            assert_eq!(inner.closed_loop_entry_runaway, 0);
            assert!(
                !write_log(&backend)
                    .as_slice()
                    .contains(&(MODE_RAMP_MODE_MOTION.addr(), 0)),
                "bounded entry drift must leave position mode active for sustained hold"
            );
        });
    }

    #[test]
    fn calibrate_skips_sustained_hold_after_fast_stability_check() {
        pollster::block_on(async {
            let mut phase = closed_loop_phase_at(ClosedLoopEntryState::StabilityCheck2 {
                position_1: 0,
                deadline_us: 0,
            });
            let mut adc_cal = None;
            let mut backend = MockBackend::new();
            backend.registers.insert(PID_POSITION_ACTUAL.addr(), 0).ok();

            let result = step_closed_loop_entry(
                &mut phase,
                InnerSequence::Calibrate,
                &mut adc_cal,
                &mut CommissionDiagnostics::default(),
                &mut backend,
            )
            .await;

            assert!(matches!(result, StepResult::CalibrateDone(_)));
        });
    }

    #[test]
    fn stability_drift_retries_before_exhausting_attempts() {
        pollster::block_on(async {
            let mut phase = closed_loop_phase_at(ClosedLoopEntryState::StabilityCheck2 {
                position_1: 0,
                deadline_us: 0,
            });
            let mut adc_cal = None;
            let mut backend = MockBackend::new();
            backend
                .registers
                .insert(PID_POSITION_ACTUAL.addr(), 10)
                .ok();
            let mut diagnostics = CommissionDiagnostics::default();

            let result = step_closed_loop_entry(
                &mut phase,
                InnerSequence::Calibrate,
                &mut adc_cal,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(
                matches!(result, StepResult::ReadyNow),
                "bounded drift on the first attempt should retry, got {result:?}"
            );
            assert!(matches!(
                phase,
                InnerPhase::ClosedLoopEntry {
                    state: ClosedLoopEntryState::WriteGains,
                    attempt: 1,
                    ..
                }
            ));
            let events: heapless::Vec<_, 4> = diagnostics.iter().collect();
            assert_eq!(events.len(), 1);
            assert_eq!(
                events[0].phase,
                crate::autotune::PhaseId::ClosedLoopEntry.to_u8()
            );
            assert_eq!(events[0].code, DIAG_CLOSED_LOOP_ENTRY_RESULT);
            assert_eq!(events[0].status, CLOSED_LOOP_ENTRY_STATUS_FAIL_DRIFT);
            assert_eq!(events[0].value1, 10);
            assert_eq!(events[0].value2, 10);
        });
    }

    #[test]
    fn stability_drift_reports_closed_loop_unstable_after_exhausting_attempts() {
        pollster::block_on(async {
            let mut phase = closed_loop_phase_at_with_attempt(
                ClosedLoopEntryState::StabilityCheck2 {
                    position_1: 0,
                    deadline_us: 0,
                },
                CLOSED_LOOP_ENTRY_MAX_ATTEMPTS - 1,
            );
            let mut adc_cal = None;
            let mut backend = MockBackend::new();
            backend
                .registers
                .insert(PID_POSITION_ACTUAL.addr(), 10)
                .ok();

            let result = step_closed_loop_entry(
                &mut phase,
                InnerSequence::Calibrate,
                &mut adc_cal,
                &mut CommissionDiagnostics::default(),
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::ClosedLoopEntryUnstable)
            ));
        });
    }

    #[test]
    fn stability_runaway_never_retries_even_on_first_attempt() {
        pollster::block_on(async {
            let mut phase = closed_loop_phase_at(ClosedLoopEntryState::StabilityCheck2 {
                position_1: 0,
                deadline_us: 0,
            });
            let mut adc_cal = None;
            let mut backend = MockBackend::new();
            backend
                .registers
                .insert(
                    PID_POSITION_ACTUAL.addr(),
                    CLOSED_LOOP_ENTRY_RUNAWAY_THRESHOLD + 1,
                )
                .ok();
            let mut diagnostics = CommissionDiagnostics::default();

            let result = step_closed_loop_entry(
                &mut phase,
                InnerSequence::Calibrate,
                &mut adc_cal,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(
                matches!(
                    result,
                    StepResult::InnerFailed(CommissionError::ClosedLoopEntryUnstable)
                ),
                "runaway must fail immediately regardless of attempt budget, got {result:?}"
            );
            let events: heapless::Vec<_, 4> = diagnostics.iter().collect();
            assert_eq!(events.len(), 1);
            assert_eq!(events[0].status, CLOSED_LOOP_ENTRY_STATUS_FAIL_RUNAWAY);
        });
    }

    #[test]
    fn commission_large_entry_drift_returns_failed_result_with_entry_evidence() {
        pollster::block_on(async {
            let mut phase = closed_loop_phase_at_with_attempt(
                ClosedLoopEntryState::StabilityCheck2 {
                    position_1: 0,
                    deadline_us: 0,
                },
                CLOSED_LOOP_ENTRY_MAX_ATTEMPTS - 1,
            );
            let mut adc_cal = None;
            let mut backend = MockBackend::new();
            backend
                .registers
                .insert(PID_POSITION_ACTUAL.addr(), 65)
                .ok();

            let result = step_closed_loop_entry(
                &mut phase,
                InnerSequence::Commission,
                &mut adc_cal,
                &mut CommissionDiagnostics::default(),
                &mut backend,
            )
            .await;

            let StepResult::CommissionDone(result) = result else {
                panic!("expected failed CommissionDone with closed-loop entry evidence");
            };
            assert_eq!(
                result.inner.status,
                CommissionStatus::Failed(CommissionError::ClosedLoopEntryUnstable)
            );
            assert_eq!(result.inner.current_loop_failure_reason, 17);
            assert_eq!(
                result.inner.closed_loop_entry_status,
                CLOSED_LOOP_ENTRY_STATUS_FAIL_DRIFT
            );
            assert_eq!(result.inner.closed_loop_entry_position_1, 0);
            assert_eq!(result.inner.closed_loop_entry_position_2, 65);
            assert_eq!(result.inner.closed_loop_entry_drift_count, 65);
            assert_eq!(result.inner.closed_loop_entry_threshold_count, 2);
            assert_eq!(result.inner.closed_loop_entry_runaway, 0);
            assert!(
                write_log(&backend)
                    .as_slice()
                    .contains(&(MODE_RAMP_MODE_MOTION.addr(), 0)),
                "unstable entry must stop position mode before returning evidence"
            );
        });
    }

    #[test]
    fn sustained_hold_position_span_accepts_with_warning_evidence() {
        pollster::block_on(async {
            let mut accumulator =
                HoldGateAccumulator::new(0, current_loop_actionable_status_mask());
            accumulator.push(HoldGateSample {
                elapsed_us: 0,
                position_count: 0,
                torque_count: 0,
                flux_count: 0,
                status_flags: 0,
            });
            let mut phase = closed_loop_phase_at(ClosedLoopEntryState::SustainedHoldSample {
                accumulator,
                started_us: 0,
                next_sample_us: HOLD_GATE_SAMPLE_PERIOD_US as u64,
                deadline_us: HOLD_GATE_SAMPLE_PERIOD_US as u64,
                requested_sample_period_us: HOLD_GATE_SAMPLE_PERIOD_US as u16,
                sample_overrun: false,
            });
            let mut adc_cal = None;
            let mut backend = MockBackend::new();
            backend.enable_pin = true;
            backend.elapsed_us_value = HOLD_GATE_SAMPLE_PERIOD_US as u64;
            backend
                .registers
                .insert(PID_POSITION_ACTUAL.addr(), 17)
                .ok();
            backend
                .registers
                .insert(PID_TORQUE_FLUX_ACTUAL.addr(), 0)
                .ok();
            backend.registers.insert(STATUS_FLAGS.addr(), 0).ok();

            let result = step_closed_loop_entry(
                &mut phase,
                InnerSequence::Commission,
                &mut adc_cal,
                &mut CommissionDiagnostics::default(),
                &mut backend,
            )
            .await;

            let StepResult::CommissionDone(result) = result else {
                panic!("expected accepted CommissionDone with hold evidence");
            };
            assert_eq!(result.inner.status, CommissionStatus::Accepted);
            assert_eq!(
                result.inner.current_loop_failure_reason,
                crate::autotune::CURRENT_LOOP_FAILURE_NONE
            );
            assert_eq!(result.inner.current_hold_status, HOLD_GATE_STATUS_PASS);
            assert_eq!(
                result.inner.current_hold_warning_flags,
                crate::autotune::HOLD_GATE_WARNING_POSITION_SPAN
            );
            assert!(backend.enable_pin);
        });
    }

    #[test]
    fn sustained_hold_pass_returns_commission_done_with_metrics_and_upstream_evidence() {
        pollster::block_on(async {
            let mut accumulator =
                HoldGateAccumulator::new(0, current_loop_actionable_status_mask());
            accumulator.push(HoldGateSample {
                elapsed_us: 0,
                position_count: 4,
                torque_count: 20,
                flux_count: -4,
                status_flags: 0,
            });
            let mut phase = closed_loop_phase_at(ClosedLoopEntryState::SustainedHoldSample {
                accumulator,
                started_us: 0,
                next_sample_us: HOLD_GATE_SAMPLE_PERIOD_US as u64,
                deadline_us: HOLD_GATE_SAMPLE_PERIOD_US as u64,
                requested_sample_period_us: HOLD_GATE_SAMPLE_PERIOD_US as u16,
                sample_overrun: false,
            });
            let mut adc_cal = None;
            let mut backend = MockBackend::new();
            backend.enable_pin = true;
            backend.elapsed_us_value = HOLD_GATE_SAMPLE_PERIOD_US as u64;
            backend.registers.insert(PID_POSITION_ACTUAL.addr(), 6).ok();
            backend
                .registers
                .insert(
                    PID_TORQUE_FLUX_ACTUAL.addr(),
                    pack_torque_flux_actual(24, -2),
                )
                .ok();
            backend.registers.insert(STATUS_FLAGS.addr(), 0).ok();

            let result = step_closed_loop_entry(
                &mut phase,
                InnerSequence::Commission,
                &mut adc_cal,
                &mut CommissionDiagnostics::default(),
                &mut backend,
            )
            .await;

            let StepResult::CommissionDone(result) = result else {
                panic!("expected CommissionDone");
            };
            assert_eq!(result.inner.status, CommissionStatus::Accepted);
            assert_eq!(result.inner.r_count_milli, 1706);
            assert_eq!(
                result.inner.current_validation_axes,
                CURRENT_VALIDATION_AXIS_TORQUE
            );
            assert_eq!(result.inner.current_hold_status, HOLD_GATE_STATUS_PASS);
            assert_eq!(result.inner.current_hold_sample_count, 2);
            assert_eq!(
                result.inner.current_hold_elapsed_us,
                HOLD_GATE_SAMPLE_PERIOD_US
            );
            assert_eq!(result.inner.current_hold_position_span_count, 2);
            assert_eq!(result.inner.current_hold_torque_mean_count, 22);
            assert_eq!(result.inner.current_hold_flux_mean_count, -3);
        });
    }

    #[test]
    fn sustained_hold_position_span_returns_accepted_result_with_evidence() {
        pollster::block_on(async {
            let mut accumulator =
                HoldGateAccumulator::new(0, current_loop_actionable_status_mask());
            accumulator.push(HoldGateSample {
                elapsed_us: 0,
                position_count: 0,
                torque_count: 0,
                flux_count: 0,
                status_flags: 0,
            });
            let mut phase = closed_loop_phase_at(ClosedLoopEntryState::SustainedHoldSample {
                accumulator,
                started_us: 0,
                next_sample_us: HOLD_GATE_SAMPLE_PERIOD_US as u64,
                deadline_us: HOLD_GATE_SAMPLE_PERIOD_US as u64,
                requested_sample_period_us: HOLD_GATE_SAMPLE_PERIOD_US as u16,
                sample_overrun: false,
            });
            let mut adc_cal = None;
            let mut backend = MockBackend::new();
            backend.enable_pin = true;
            backend.elapsed_us_value = HOLD_GATE_SAMPLE_PERIOD_US as u64;
            backend
                .registers
                .insert(PID_POSITION_ACTUAL.addr(), 17)
                .ok();
            backend
                .registers
                .insert(PID_TORQUE_FLUX_ACTUAL.addr(), 0)
                .ok();
            backend.registers.insert(STATUS_FLAGS.addr(), 0).ok();

            let result = step_closed_loop_entry(
                &mut phase,
                InnerSequence::Commission,
                &mut adc_cal,
                &mut CommissionDiagnostics::default(),
                &mut backend,
            )
            .await;

            let StepResult::CommissionDone(result) = result else {
                panic!("expected accepted CommissionDone with hold evidence");
            };
            assert_eq!(result.inner.status, CommissionStatus::Accepted);
            assert_eq!(
                result.inner.current_loop_failure_reason,
                crate::autotune::CURRENT_LOOP_FAILURE_NONE
            );
            assert_eq!(result.inner.current_hold_status, HOLD_GATE_STATUS_PASS);
            assert_eq!(
                result.inner.current_hold_warning_flags,
                crate::autotune::HOLD_GATE_WARNING_POSITION_SPAN
            );
            assert_eq!(result.inner.current_hold_sample_count, 2);
            assert_eq!(result.inner.current_hold_position_span_count, 17);
            assert!(backend.enable_pin);
        });
    }

    #[test]
    fn sustained_hold_missing_inner_result_fails_without_default_evidence() {
        pollster::block_on(async {
            let mut accumulator =
                HoldGateAccumulator::new(0, current_loop_actionable_status_mask());
            accumulator.push(HoldGateSample {
                elapsed_us: 0,
                position_count: 4,
                torque_count: 20,
                flux_count: -4,
                status_flags: 0,
            });
            let mut phase =
                closed_loop_phase_at_without_inner(ClosedLoopEntryState::SustainedHoldSample {
                    accumulator,
                    started_us: 0,
                    next_sample_us: HOLD_GATE_SAMPLE_PERIOD_US as u64,
                    deadline_us: HOLD_GATE_SAMPLE_PERIOD_US as u64,
                    requested_sample_period_us: HOLD_GATE_SAMPLE_PERIOD_US as u16,
                    sample_overrun: false,
                });
            let mut adc_cal = None;
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = HOLD_GATE_SAMPLE_PERIOD_US as u64;
            backend.registers.insert(PID_POSITION_ACTUAL.addr(), 4).ok();
            backend
                .registers
                .insert(
                    PID_TORQUE_FLUX_ACTUAL.addr(),
                    pack_torque_flux_actual(20, -4),
                )
                .ok();
            backend.registers.insert(STATUS_FLAGS.addr(), 0).ok();

            let result = step_closed_loop_entry(
                &mut phase,
                InnerSequence::Commission,
                &mut adc_cal,
                &mut CommissionDiagnostics::default(),
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::Spi)
            ));
            assert!(
                write_log(&backend)
                    .as_slice()
                    .contains(&(MODE_RAMP_MODE_MOTION.addr(), 0)),
                "missing inner result must stop position mode before failing"
            );
        });
    }

    #[test]
    fn sustained_hold_spi_read_failure_stops_motion_mode() {
        pollster::block_on(async {
            let mut phase = closed_loop_phase_at(ClosedLoopEntryState::SustainedHoldSample {
                accumulator: HoldGateAccumulator::new(
                    HOLD_GATE_SAMPLE_PERIOD_US as u16,
                    current_loop_actionable_status_mask(),
                ),
                started_us: 0,
                next_sample_us: 0,
                deadline_us: HOLD_GATE_SAMPLE_PERIOD_US as u64,
                requested_sample_period_us: HOLD_GATE_SAMPLE_PERIOD_US as u16,
                sample_overrun: false,
            });
            let mut adc_cal = None;
            let mut backend = MockBackend::new();
            backend.fail_on_read = Some(PID_TORQUE_FLUX_ACTUAL.addr());

            let result = step_closed_loop_entry(
                &mut phase,
                InnerSequence::Commission,
                &mut adc_cal,
                &mut CommissionDiagnostics::default(),
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::Spi)
            ));
            assert!(
                write_log(&backend)
                    .as_slice()
                    .contains(&(MODE_RAMP_MODE_MOTION.addr(), 0)),
                "SPI read failure must stop position mode before returning the error"
            );
        });
    }

    #[test]
    fn sustained_hold_late_sample_marks_overrun_and_preserves_grid() {
        pollster::block_on(async {
            let mut phase = closed_loop_phase_at(ClosedLoopEntryState::SustainedHoldSample {
                accumulator: HoldGateAccumulator::new(
                    HOLD_GATE_SAMPLE_PERIOD_US as u16,
                    current_loop_actionable_status_mask(),
                ),
                started_us: 0,
                next_sample_us: HOLD_GATE_SAMPLE_PERIOD_US as u64,
                deadline_us: (HOLD_GATE_SAMPLE_PERIOD_US * 3) as u64,
                requested_sample_period_us: HOLD_GATE_SAMPLE_PERIOD_US as u16,
                sample_overrun: false,
            });
            let mut adc_cal = None;
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = HOLD_GATE_SAMPLE_PERIOD_US as u64 + 1;
            backend.registers.insert(PID_POSITION_ACTUAL.addr(), 0).ok();
            backend
                .registers
                .insert(PID_TORQUE_FLUX_ACTUAL.addr(), 0)
                .ok();
            backend.registers.insert(STATUS_FLAGS.addr(), 0).ok();

            let result = step_closed_loop_entry(
                &mut phase,
                InnerSequence::Commission,
                &mut adc_cal,
                &mut CommissionDiagnostics::default(),
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::At(next) if next == (HOLD_GATE_SAMPLE_PERIOD_US * 2) as u64
            ));
            assert!(matches!(
                phase,
                InnerPhase::ClosedLoopEntry {
                    state: ClosedLoopEntryState::SustainedHoldSample {
                        next_sample_us,
                        sample_overrun: true,
                        ..
                    },
                    ..
                } if next_sample_us == (HOLD_GATE_SAMPLE_PERIOD_US * 2) as u64
            ));
        });
    }

    #[test]
    fn sustained_hold_overrun_finishes_with_warning_not_failure() {
        pollster::block_on(async {
            let mut accumulator = HoldGateAccumulator::new(
                HOLD_GATE_SAMPLE_PERIOD_US as u16,
                current_loop_actionable_status_mask(),
            );
            accumulator.push(HoldGateSample {
                elapsed_us: 0,
                position_count: 0,
                torque_count: 0,
                flux_count: 0,
                status_flags: 0,
            });
            let mut phase = closed_loop_phase_at(ClosedLoopEntryState::SustainedHoldSample {
                accumulator,
                started_us: 0,
                next_sample_us: HOLD_GATE_SAMPLE_PERIOD_US as u64,
                deadline_us: HOLD_GATE_SAMPLE_PERIOD_US as u64 + 1,
                requested_sample_period_us: HOLD_GATE_SAMPLE_PERIOD_US as u16,
                sample_overrun: false,
            });
            let mut adc_cal = None;
            let mut backend = MockBackend::new();
            backend.enable_pin = true;
            backend.elapsed_us_value = HOLD_GATE_SAMPLE_PERIOD_US as u64 + 1;
            backend.registers.insert(PID_POSITION_ACTUAL.addr(), 0).ok();
            backend
                .registers
                .insert(PID_TORQUE_FLUX_ACTUAL.addr(), 0)
                .ok();
            backend.registers.insert(STATUS_FLAGS.addr(), 0).ok();

            let result = step_closed_loop_entry(
                &mut phase,
                InnerSequence::Commission,
                &mut adc_cal,
                &mut CommissionDiagnostics::default(),
                &mut backend,
            )
            .await;

            let StepResult::CommissionDone(result) = result else {
                panic!("expected accepted CommissionDone with hold evidence");
            };
            assert_eq!(result.inner.status, CommissionStatus::Accepted);
            assert_eq!(result.inner.current_hold_status, HOLD_GATE_STATUS_PASS);
            assert_eq!(
                result.inner.current_hold_warning_flags,
                HOLD_GATE_WARNING_SAMPLE_OVERRUN
            );
            assert!(
                backend.enable_pin,
                "warning-only overrun must leave the accepted hold enabled"
            );
        });
    }
}
