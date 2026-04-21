//! Stage 1 (current-loop) commissioning: phase enum, dispatcher, and
//! per-phase step submodules.
//!
//! [`InnerPhase`] encodes the current diagnostic / electrical-ID / current-
//! tune state. [`step_inner`] dispatches a single cooperative tick to the
//! handler in the matching submodule. [`fail_inner`] is the shared
//! short-circuit transition into [`InnerPhase::Failed`]. [`restore_entry_gains`]
//! is the soft-failure recovery routine called by the board after an outer
//! commissioning fault.

use crate::autotune::{
    ClosedLoopEntryState, CommissionError, CommissionProfile, ElectricalModel, InnerSequence,
    TransientBuffer,
};
use crate::commission::{
    AdcCalState, CoilCheckState, CoilObservation, CurrentValidateData, CurrentValidateState,
    ElectricalIdData, ElectricalIdState, EncoderAlignState, EncoderCheckState, EntrySnapshot,
};
use crate::commissioning::types::{AdcCalResult, CommissioningBackend, InnerResult, StepResult};

pub mod adc;
pub mod closed_loop_entry;
pub mod coil;
pub mod current_loop;
pub mod electrical_identification;
pub mod encoder;
pub mod phase;

/// Stage 1 inner-loop commissioning phase.
///
/// Orderable cooperative state machine. Each variant either holds a
/// nested sub-state for that phase or carries forward data needed by the
/// next phase. The dispatcher [`step_inner`] selects the matching
/// submodule's `step_*` function each tick; long-running register
/// operations can be chunked across 1ms `step()` calls.
#[derive(Debug, defmt::Format)]
pub(crate) enum InnerPhase {
    /// Phase 1: ADC offset calibration.
    AdcCalibrate(AdcCalState),
    /// Phase 2: Coil connectivity check.
    CoilCheck {
        state: CoilCheckState,
        /// Coil A observation (filled after coil A test).
        coil_a: Option<CoilObservation>,
    },
    /// Phase 3: Phase wiring validation.
    PhaseWiringCheck {
        coil_a: CoilObservation,
        coil_b: CoilObservation,
    },
    /// Phase 4: Encoder movement and direction detection.
    EncoderCheck { state: EncoderCheckState },
    /// Encoder D-axis alignment and zeroing.
    EncoderAlign { state: EncoderAlignState },
    /// Phase 5: Electrical identification (R, L, theta_e).
    ElectricalId {
        state: ElectricalIdState,
        data: ElectricalIdData,
    },
    /// Phase 6: Current-loop SIMC gain synthesis.
    CurrentTune { model: ElectricalModel },
    /// Phase 7: Current-loop step-response validation.
    CurrentValidate {
        state: CurrentValidateState,
        data: CurrentValidateData,
    },
    /// Closed-loop position mode entry with PID gains, filters, and stability check.
    ClosedLoopEntry {
        /// Current sub-state.
        state: ClosedLoopEntryState,
        /// Current-loop flux P gain (Q8.8) to write.
        flux_p: u16,
        /// Current-loop flux I gain (Q8.8) to write.
        flux_i: u16,
        /// Current-loop torque P gain (Q8.8) to write.
        torque_p: u16,
        /// Current-loop torque I gain (Q8.8) to write.
        torque_i: u16,
        /// Outer velocity P gain (Q8.8) to write.
        velocity_p: u16,
        /// Outer velocity I gain (Q8.8) to write.
        velocity_i: u16,
        /// Outer position P gain (Q8.8) to write.
        position_p: u16,
        /// Outer position I gain (Q8.8) to write.
        position_i: u16,
        /// PID velocity limit register value.
        velocity_limit: u32,
        /// Velocity biquad filter cutoff (Hz), 0 = disabled.
        velocity_filter_hz: u16,
        /// Torque biquad filter cutoff (Hz), 0 = disabled.
        torque_filter_hz: u16,
        /// Position biquad filter cutoff (Hz), 0 = disabled.
        position_filter_hz: u16,
        /// Flux biquad filter cutoff (Hz), 0 = disabled.
        flux_filter_hz: u16,
        /// Inner result carried forward for CommissionDone (Commission sequence only).
        inner_result: Option<InnerResult>,
    },
    /// A phase failed.
    Failed(CommissionError),
}

/// Transition the inner phase to `Failed` and emit an `InnerFailed` result.
pub(crate) fn fail_inner(phase: &mut InnerPhase, err: CommissionError) -> StepResult {
    *phase = InnerPhase::Failed(err);
    StepResult::InnerFailed(err)
}

/// Step inner phase state machine by one cooperative tick.
///
/// Dispatches to the per-phase handler in the matching submodule. Each
/// handler advances its sub-state by one chunk of work (bounded to ~1ms).
/// Phase transitions are signalled by replacing `*phase` and returning
/// `StepResult::PhaseChanged`.
pub(crate) async fn step_inner<B: CommissioningBackend>(
    phase: &mut InnerPhase,
    buf: &mut TransientBuffer,
    profile: CommissionProfile,
    sequence: InnerSequence,
    adc_cal: &mut Option<AdcCalResult>,
    selftest_results: &mut crate::selftest::SelftestResults,
    backend: &mut B,
) -> StepResult {
    match phase {
        InnerPhase::AdcCalibrate(_) => {
            adc::step_adc_calibrate(phase, sequence, adc_cal, selftest_results, backend).await
        }
        InnerPhase::CoilCheck { .. } => {
            coil::step_coil_check(phase, sequence, selftest_results, backend).await
        }
        InnerPhase::PhaseWiringCheck { coil_a, coil_b } => {
            let a = *coil_a;
            let b = *coil_b;
            phase::step_phase_wiring(phase, sequence, selftest_results, a, b)
        }
        InnerPhase::EncoderCheck { .. } => {
            encoder::step_encoder_check(phase, sequence, selftest_results, backend).await
        }
        InnerPhase::EncoderAlign { .. } => {
            encoder::step_encoder_align(phase, sequence, adc_cal, selftest_results, backend).await
        }
        InnerPhase::ElectricalId { .. } => {
            electrical_identification::step_electrical_identification(
                phase,
                buf,
                sequence,
                selftest_results,
                backend,
            )
            .await
        }
        InnerPhase::CurrentTune { .. } => {
            current_loop::step_current_tune(phase, profile, backend).await
        }
        InnerPhase::CurrentValidate { .. } => {
            current_loop::step_current_validate(phase, buf, profile, sequence, backend).await
        }
        InnerPhase::ClosedLoopEntry { .. } => {
            closed_loop_entry::step_closed_loop_entry(phase, sequence, adc_cal, backend).await
        }
        InnerPhase::Failed(err) => StepResult::InnerFailed(*err),
    }
}

/// Restore outer gains from an entry snapshot after Stage 2 soft failure.
///
/// Zeros torque/velocity targets, disables motion mode, clears all PID
/// integrators and status flags, reloads the snapshotted velocity/position
/// gains, velocity limit, and biquad filters, then re-enters position
/// mode. The caller is responsible for the post-restore stability check
/// because [`CommissioningBackend`] does not expose a delay primitive.
pub async fn restore_entry_gains<B: CommissioningBackend>(
    snapshot: &EntrySnapshot,
    backend: &mut B,
) -> Result<(), CommissionError> {
    use foci_tmc4671::registers::*;

    // 1. Zero torque, velocity targets, and velocity offset.
    backend
        .write_register(PID_TORQUE_FLUX_TARGET, 0)
        .await
        .map_err(|_| CommissionError::Spi)?;
    backend
        .write_register(PID_VELOCITY_TARGET, 0)
        .await
        .map_err(|_| CommissionError::Spi)?;
    backend
        .write_register(PID_VELOCITY_OFFSET, 0)
        .await
        .map_err(|_| CommissionError::Spi)?;

    // 2. Disable motion mode.
    backend
        .write_register(MODE_RAMP_MODE_MOTION, 0)
        .await
        .map_err(|_| CommissionError::Spi)?;

    // 3. Clear PID integrators.
    crate::commissioning::helpers::write_sub_register_checked(
        backend,
        PID_ERROR_PID_TORQUE_ERROR_SUM,
        0,
    )
    .await
    .map_err(|_| CommissionError::Spi)?;
    crate::commissioning::helpers::write_sub_register_checked(
        backend,
        PID_ERROR_PID_FLUX_ERROR_SUM,
        0,
    )
    .await
    .map_err(|_| CommissionError::Spi)?;
    crate::commissioning::helpers::write_sub_register_checked(
        backend,
        PID_ERROR_PID_VELOCITY_ERROR_SUM,
        0,
    )
    .await
    .map_err(|_| CommissionError::Spi)?;
    crate::commissioning::helpers::write_sub_register_checked(
        backend,
        PID_ERROR_PID_POSITION_ERROR_SUM,
        0,
    )
    .await
    .map_err(|_| CommissionError::Spi)?;

    // 4. Clear status flags.
    backend
        .write_register(STATUS_FLAGS, 0)
        .await
        .map_err(|_| CommissionError::Spi)?;

    // 5. Reload outer gains from snapshot.
    let velocity_reg = (snapshot.velocity_p as u32) << 16 | snapshot.velocity_i as u32;
    backend
        .write_register(PID_VELOCITY_P_VELOCITY_I, velocity_reg)
        .await
        .map_err(|_| CommissionError::Spi)?;
    let position_reg = (snapshot.position_p as u32) << 16 | snapshot.position_i as u32;
    backend
        .write_register(PID_POSITION_P_POSITION_I, position_reg)
        .await
        .map_err(|_| CommissionError::Spi)?;
    backend
        .write_register(PID_VELOCITY_LIMIT, snapshot.velocity_limit)
        .await
        .map_err(|_| CommissionError::Spi)?;

    // Reload filters (best-effort). Explicitly disable when Hz=0
    // to restore the full entry filter state.
    let pwm_maxcnt = backend.pwm_maxcnt();
    if snapshot.velocity_filter_hz > 0 {
        crate::commissioning::helpers::write_biquad_velocity(
            backend,
            snapshot.velocity_filter_hz,
            pwm_maxcnt,
        )
        .await;
    } else {
        crate::commissioning::helpers::write_sub_register(backend, CONFIG_BIQUAD_V_ENABLE, 0).await;
    }
    if snapshot.torque_filter_hz > 0 {
        crate::commissioning::helpers::write_biquad_torque(
            backend,
            snapshot.torque_filter_hz,
            pwm_maxcnt,
        )
        .await;
    } else {
        crate::commissioning::helpers::write_sub_register(backend, CONFIG_BIQUAD_T_ENABLE, 0).await;
    }
    if snapshot.position_filter_hz > 0 {
        crate::commissioning::helpers::write_biquad_position(
            backend,
            snapshot.position_filter_hz,
            pwm_maxcnt,
        )
        .await;
    } else {
        crate::commissioning::helpers::write_sub_register(backend, CONFIG_BIQUAD_X_ENABLE, 0).await;
    }
    if snapshot.flux_filter_hz > 0 {
        crate::commissioning::helpers::write_biquad_flux(
            backend,
            snapshot.flux_filter_hz,
            pwm_maxcnt,
        )
        .await;
    } else {
        crate::commissioning::helpers::write_sub_register(backend, CONFIG_BIQUAD_F_ENABLE, 0).await;
    }

    // 6. Reload voltage limit.
    backend
        .write_register(PIDOUT_UQ_UD_LIMITS, snapshot.voltage_limit)
        .await
        .map_err(|_| CommissionError::Spi)?;

    // 7. Zero position (also zeros target per datasheet 0x6B).
    backend
        .write_register(PID_POSITION_ACTUAL, 0)
        .await
        .map_err(|_| CommissionError::Spi)?;

    // 8. Re-enter position mode.
    backend
        .write_register(MODE_RAMP_MODE_MOTION, 0x8000_0003)
        .await
        .map_err(|_| CommissionError::Spi)?;

    Ok(())
}
