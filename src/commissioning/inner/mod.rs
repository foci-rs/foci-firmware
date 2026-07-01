//! Stage 1 (current-loop) commissioning: phase enum, dispatcher, and
//! per-phase step submodules.
//!
//! `InnerPhase` encodes the current diagnostic / electrical-ID / current-
//! tune state. `step_inner` dispatches a single cooperative tick to the
//! handler in the matching submodule. `fail_inner` is the shared
//! short-circuit transition into `InnerPhase::Failed`. [`restore_entry_gains`]
//! is the soft-failure recovery routine called by the board after an outer
//! commissioning fault.

use crate::autotune::{
    ClosedLoopEntryState, CommissionError, CommissionProfile, ElectricalModel, InnerSequence,
    TransientBuffer,
};
use crate::commissioning::EntrySnapshot;
use crate::commissioning::inner::current_loop::{CurrentValidateData, CurrentValidateState};
use crate::commissioning::types::{
    AdcCalResult, CommissioningBackend, INDUCTANCE_FIT_POINT_COUNT, InductanceFitEvidence,
    InductanceFitPoint, InnerResult, StepResult,
};

pub mod adc;
pub mod closed_loop_entry;
pub mod coil;
pub mod current_loop;
pub mod electrical_identification;
pub mod encoder;
mod impedance;
pub mod phase;

// ---------------------------------------------------------------------------
// Inner sub-state types
// ---------------------------------------------------------------------------

/// ADC calibration sub-state.
#[derive(Debug, defmt::Format)]
pub(crate) enum AdcCalState {
    /// Configure registers and enable outputs.
    Setup,
    /// Wait for ADC outputs to settle.
    Settle { deadline_us: u64 },
    /// Collect ADC samples one per step() call.
    Sample {
        sum_i0: i32,
        sum_i1: i32,
        count: u16,
    },
    /// Validate and program offsets.
    Finalize { sum_i0: i32, sum_i1: i32 },
}

/// Coil test sub-state (used for both coil A and coil B).
#[derive(Debug, defmt::Format)]
pub(crate) enum CoilTestState {
    /// Configure open-loop voltage mode for this phase.
    Setup { phase: u8 },
    /// Wait for current to settle.
    Settle { phase: u8, deadline_us: u64 },
    /// Read ADC and record result.
    Read { phase: u8 },
    /// Disable and cool down before next test.
    Cooldown { phase: u8, deadline_us: u64 },
}

/// Coil check phase state (wraps both coil tests).
#[derive(Debug, defmt::Format)]
pub(crate) enum CoilCheckState {
    /// Testing coil A (phase 0).
    CoilA(CoilTestState),
    /// Testing coil B (phase 1).
    CoilB(CoilTestState),
}

/// Accumulated coil test observations carried between sub-phases.
#[derive(Debug, Clone, Copy, defmt::Format)]
pub(crate) struct CoilObservation {
    /// Current on the expected ADC channel.
    pub(crate) expected: i16,
    /// Current on the other ADC channel.
    pub(crate) other: i16,
}

/// Encoder check sub-state.
#[derive(Debug, defmt::Format)]
pub(crate) enum EncoderCheckState {
    /// Set up current-regulated mode for direction sweep.
    Setup,
    /// Sweep PHI_E_EXT in steps, one per tick.
    Sweep {
        step: u16,
        deadline_us: u64,
        count_start: i32,
    },
    /// Read final count and determine direction.
    Finalize { count_start: i32 },
}

/// Encoder D-axis alignment sub-state.
#[derive(Debug, defmt::Format)]
pub(crate) enum EncoderAlignState {
    /// Configure external PHI_E=0 voltage drive.
    Setup {
        /// Encoder count captured before the direction sweep that ended at PHI_E=0.
        count_before: u32,
    },
    /// Wait for rotor to settle at the nearest D-axis pole.
    Settle {
        count_before: u32,
        deadline_us: u64,
        min_movement: u32,
    },
    /// First post-settle encoder read.
    StabilityCheck1 {
        count_before: u32,
        min_movement: u32,
    },
    /// Second encoder read after a short delay for stability validation.
    StabilityCheck2 {
        count_before: u32,
        count1: u32,
        deadline_us: u64,
        min_movement: u32,
    },
    /// Zero encoder count and offset while the rotor is still held.
    Commit {
        old_count: u32,
        electrical_residual_counts: u16,
        stability_counts: u16,
        movement_counts: u16,
        min_movement_counts: u16,
        counts_per_electrical_rev: u16,
    },
    /// Stop open-loop drive after PHI_E has been locked to ABN.
    StopDrive {
        old_count: u32,
        electrical_residual_counts: u16,
        stability_counts: u16,
        movement_counts: u16,
        min_movement_counts: u16,
        counts_per_electrical_rev: u16,
    },
}

/// Accumulated per-coil measurements for electrical identification.
#[derive(Debug, Clone, defmt::Format)]
pub(crate) struct ElectricalIdData {
    /// Excitation voltage used for all measurements (PIDOUT units).
    pub(crate) voltage: u32,
    /// Shared multi-point resistance-identification result. `Some` once the
    /// `ResistanceIdentify` sub-state has run the shared runner; the
    /// inductance/delay/gain path reads its count-slope-milli values from here.
    pub(crate) resistance_result: Option<crate::resistance_identification::ResistanceRunResult>,
    /// Per-coil, per-amplitude fit points for corrected L/tau.
    pub(crate) inductance_fit_points: [[InductanceFitPoint; INDUCTANCE_FIT_POINT_COUNT]; 2],
    /// Accepted per-coil fit evidence.
    pub(crate) inductance_fit: [InductanceFitEvidence; 2],
}

impl ElectricalIdData {
    pub(crate) fn new(voltage: u32) -> Self {
        Self {
            voltage,
            resistance_result: None,
            inductance_fit_points: [[InductanceFitPoint::EMPTY; INDUCTANCE_FIT_POINT_COUNT]; 2],
            inductance_fit: [InductanceFitEvidence::EMPTY; 2],
        }
    }
}

/// Electrical identification sub-state machine (Phase 5).
#[derive(Debug, defmt::Format)]
pub(crate) enum ElectricalIdState {
    // --- Resistance identification ---
    /// Run shared multi-point resistance identification before the L work.
    ResistanceIdentify,

    // --- Inductance measurement (per coil, square-wave di/dt) ---
    /// Configure for di/dt toggling.
    InductanceSetup { coil: u8, point: u8 },
    /// Brief settle before toggling.
    InductanceSettle {
        coil: u8,
        point: u8,
        deadline_us: u64,
    },
    /// Toggle voltage and accumulate |delta_I| (chunked per step).
    ///
    /// `warmup_cycles_remaining` excludes startup transients from the
    /// accumulation. `accumulation_start_us` records the timestamp of the
    /// first measured (post-warm-up) cycle so the elapsed window only
    /// covers accumulated samples.
    InductanceToggle {
        coil: u8,
        point: u8,
        total_cycles: u32,
        remaining_cycles: u32,
        prev_current: i16,
        point_sum_delta: u64,
        point_sample_count: u32,
        current_sum: i64,
        start_us: u64,
        warmup_cycles_remaining: u32,
        accumulation_start_us: Option<u64>,
    },
    /// Disable motor and cool down.
    InductanceDisable {
        coil: u8,
        point: u8,
        deadline_us: u64,
        skip_remaining_points: bool,
    },

    // --- Delay identification ---
    /// Configure for delay step response capture.
    DelaySetup,
    /// Brief settle at zero voltage before step.
    DelaySettle { deadline_us: u64 },
    /// Apply voltage step and start capturing.
    DelayStep,
    /// Capture 256 samples into TransientBuffer at 160us intervals.
    DelaySample { baseline: i16, next_sample_us: u64 },
    /// Disable and restore safe state.
    DelayDisable { deadline_us: u64 },

    // --- Final ---
    /// Assemble ElectricalModel and validate.
    Validate,
}

/// Stage 1 inner-loop commissioning phase.
///
/// Orderable cooperative state machine. Each variant either holds a
/// nested sub-state for that phase or carries forward data needed by the
/// next phase. The dispatcher `step_inner` selects the matching
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
    CurrentTune {
        model: ElectricalModel,
        /// Phase 5 model-assembly warning bits.
        ///
        /// Bits 0, 1, and 3 are warnings; bit 2 is reserved for tau residual
        /// telemetry and is not emitted as a warning. Carried forward so
        /// current validation can OR in bits 4–5 and the final InnerResult
        /// reports the full set.
        inner_warning_flags: u8,
        /// L/R-vs-transient tau residual in permille from Phase 5.
        tau_residual_permille: u16,
        /// Shared multi-point resistance-identification result from Phase 5.
        /// Carried forward so the final InnerResult can persist the count-space
        /// fit evidence and the 64x count-slope-milli audit comparison.
        resistance_result: crate::resistance_identification::ResistanceRunResult,
        /// Accepted per-coil corrected inductance fit evidence.
        inductance_fit: [InductanceFitEvidence; 2],
        /// Per-amplitude points used by the accepted corrected inductance fits.
        inductance_fit_points: [[InductanceFitPoint; INDUCTANCE_FIT_POINT_COUNT]; 2],
    },
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
        /// Current-loop flux I gain (raw Q8.8) to write.
        flux_i: u16,
        /// Current-loop torque P gain (Q8.8) to write.
        torque_p: u16,
        /// Current-loop torque I gain (raw Q8.8) to write.
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

impl InnerPhase {
    /// Return the stable phase ID for the active non-terminal inner phase.
    pub(crate) fn current_phase_id(&self) -> Option<crate::autotune::PhaseId> {
        use crate::autotune::PhaseId;

        match self {
            Self::AdcCalibrate(_) => Some(PhaseId::AdcCalibrate),
            Self::CoilCheck { .. } => Some(PhaseId::CoilCheck),
            Self::PhaseWiringCheck { .. } => Some(PhaseId::PhaseWiringCheck),
            Self::EncoderCheck { .. } => Some(PhaseId::EncoderCheck),
            Self::EncoderAlign { .. } => Some(PhaseId::EncoderAlign),
            Self::ElectricalId { .. } => Some(PhaseId::ElectricalId),
            Self::CurrentTune { .. } => Some(PhaseId::CurrentTune),
            Self::CurrentValidate { .. } => Some(PhaseId::CurrentValidate),
            Self::ClosedLoopEntry { .. } => Some(PhaseId::ClosedLoopEntry),
            Self::Failed(_) => None,
        }
    }
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
///
/// On any `StepResult::InnerFailed(_)` returned by a phase handler, this
/// wrapper runs the canonical safe-state restore so failures cannot leave
/// the TMC4671 energized. Phase-local handlers must not call the
/// safe-state helper directly — let this wrapper own the cleanup boundary.
#[allow(clippy::too_many_arguments)]
pub(crate) async fn step_inner<B: CommissioningBackend>(
    phase: &mut InnerPhase,
    buf: &mut TransientBuffer,
    profile: CommissionProfile,
    sequence: InnerSequence,
    adc_cal: &mut Option<AdcCalResult>,
    calibrate_entry_gains: Option<crate::commissioning::CalibrateEntryGains>,
    selftest_results: &mut crate::selftest::SelftestResults,
    diagnostics: &mut crate::selftest::CommissionDiagnostics,
    backend: &mut B,
) -> StepResult {
    let result = step_inner_dispatch(
        phase,
        buf,
        profile,
        sequence,
        adc_cal,
        calibrate_entry_gains,
        selftest_results,
        diagnostics,
        backend,
    )
    .await;
    if matches!(result, StepResult::InnerFailed(_)) {
        // Defense in depth: `commission_dispatch::drive_commission_result`
        // already runs the canonical safe-state helper on
        // `StepResult::InnerFailed`. We run it again here so the chip is
        // guaranteed de-energized before the failure even leaves the
        // engine — the dispatcher boundary is one layer up. The duplicate
        // 6 SPI writes are harmless on the failure path; if a deferred
        // drive of the dispatcher is added later, this layer keeps the
        // safety contract intact.
        crate::tmc_safe_state::restore_safe_state_via_backend(backend).await;
    }
    result
}

#[allow(clippy::too_many_arguments)]
async fn step_inner_dispatch<B: CommissioningBackend>(
    phase: &mut InnerPhase,
    buf: &mut TransientBuffer,
    profile: CommissionProfile,
    sequence: InnerSequence,
    adc_cal: &mut Option<AdcCalResult>,
    calibrate_entry_gains: Option<crate::commissioning::CalibrateEntryGains>,
    selftest_results: &mut crate::selftest::SelftestResults,
    diagnostics: &mut crate::selftest::CommissionDiagnostics,
    backend: &mut B,
) -> StepResult {
    match phase {
        InnerPhase::AdcCalibrate(_) => {
            adc::step_adc_calibrate(phase, sequence, adc_cal, selftest_results, backend).await
        }
        InnerPhase::CoilCheck { .. } => {
            coil::step_coil_check(phase, sequence, selftest_results, diagnostics, backend).await
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
            encoder::step_encoder_align(
                phase,
                sequence,
                adc_cal,
                calibrate_entry_gains,
                selftest_results,
                backend,
            )
            .await
        }
        InnerPhase::ElectricalId { .. } => {
            electrical_identification::step_electrical_identification(
                phase,
                buf,
                sequence,
                selftest_results,
                diagnostics,
                backend,
            )
            .await
        }
        InnerPhase::CurrentTune { .. } => {
            current_loop::step_current_tune(phase, profile, backend).await
        }
        InnerPhase::CurrentValidate { .. } => {
            current_loop::step_current_validate(phase, buf, profile, sequence, adc_cal, backend)
                .await
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
/// mode. The caller owns the post-restore stability policy so this helper
/// remains limited to the register restore sequence.
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

#[cfg(test)]
mod tests {
    use super::{AdcCalState, InnerPhase};
    use crate::autotune::{CommissionError, PhaseId};

    #[test]
    fn inner_phase_id_maps_active_and_failed_phases() {
        assert_eq!(
            InnerPhase::AdcCalibrate(AdcCalState::Setup).current_phase_id(),
            Some(PhaseId::AdcCalibrate)
        );
        assert_eq!(
            InnerPhase::Failed(CommissionError::Spi).current_phase_id(),
            None
        );
    }
}
