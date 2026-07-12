//! Phase 5: Electrical identification (R, L, transport delay).
//!
//! Shared resistance identification, rotating-field inductance measurement,
//! and electrical transport delay from a 256-sample current transient.
//! Assembles an `ElectricalModel` and validates it before transitioning to
//! `CurrentTune` (commission flow) or emitting `SelftestDone` (selftest).

use super::{ElectricalIdData, ElectricalIdState, InductanceCaptureState, InnerPhase};
use crate::autotune::{
    CommissionError, ELECTRICAL_MODEL_RESISTANCE_SCALE, ElectricalModel, InnerSequence, PhaseId,
    TransientBuffer,
};
use crate::commissioning::backend::timed_read_register;
use crate::commissioning::capture_timing::{CaptureTiming, CaptureTimingEvidence, TimingEnvelope};
use crate::commissioning::encoder_domain::{EncoderDelta, EncoderDomain, EncoderMathError};
use crate::commissioning::types::{
    CommissioningBackend, INDUCTANCE_SOURCE_ROTATING_FIELD, INDUCTANCE_WARN_DRIFT,
    INDUCTANCE_WARN_HIGH_RMS, INDUCTANCE_WARN_MAG_CROSSCHECK, INDUCTANCE_WARN_STATUS_FLAGS,
    InductanceEstimateEvidence, InductanceEvidence, InductanceFrameEvidence, InductanceRunEvidence,
    StepResult,
};
use foci_tmc4671::registers::Register;

/// Sampling interval for resistance and transient captures (microseconds).
/// Matches PWM rate (~160us at 25 kHz) for Nyquist-rate current sampling.
///
/// Re-exported for the validation phase, which captures step responses at
/// the same cadence.
pub(crate) const ELECTRICAL_SAMPLE_PERIOD_US: u64 = 160;

/// Cooldown time after disabling a per-coil measurement (microseconds).
const ELECTRICAL_COOLDOWN_US: u64 = 50_000;

/// Brief zero-voltage settle before the delay step response capture.
const DELAY_PRE_SETTLE_US: u64 = 10_000;

const INDUCTANCE_AC_UD_EXT: i16 = 768;
const INDUCTANCE_OPENLOOP_VELOCITY_TARGET_RPM: u32 = 1_200;
const INDUCTANCE_OPENLOOP_ACCELERATION: u32 = 0x000f_ffff;
const INDUCTANCE_OPENLOOP_RAMP_TIMEOUT_US: u64 = 2_000;
const INDUCTANCE_AC_SETTLE_US: u64 = 8_500;
const INDUCTANCE_AC_SAMPLE_PERIOD_US: u16 = 100;
const INDUCTANCE_AC_TARGET_PHI_DELTA_COUNTS: u32 = 10 * 65_536;
const INDUCTANCE_MIN_ABS_IQ_MILLI_COUNT: i32 = 30_000;
const INDUCTANCE_MAX_ENCODER_DELTA_COUNTS: u32 = 8;
const INDUCTANCE_MAX_DRIFT_WARN_PERMILLE: u16 = 70;
const INDUCTANCE_MAX_IQ_RMS_WARN_PERMILLE: u16 = 250;
const INDUCTANCE_MAX_MAG_CROSSCHECK_WARN_PERMILLE: u16 = 100;
const INDUCTANCE_SALIENCY_BRACKET_TOLERANCE_COUNT_RATIO_MILLI: u32 = 256;
const INDUCTANCE_REALIZED_FREQUENCY_TARGET_MILLIHZ: u32 = 1_000_000;
const INDUCTANCE_REALIZED_FREQUENCY_TOLERANCE_MILLIHZ: u32 = 50_000;
const INDUCTANCE_MIN_AC_SAMPLE_COUNT: u16 = 90;
const INDUCTANCE_MAX_AC_SAMPLE_COUNT: u16 = 256;
const PHI_E_SELECTION_OPENLOOP: u32 = 2;
const L_INT_OMEGA_SCALE_MILLI: u64 = 6_283;

#[cfg(test)]
const fn scheduled_samples_for_fixed_window(frequency_millihz: u32) -> u32 {
    if frequency_millihz == 0 {
        return 0;
    }
    let window_us = 10_000_000_000u64 / frequency_millihz as u64;
    (window_us / INDUCTANCE_AC_SAMPLE_PERIOD_US as u64) as u32
}

/// Number of transient samples for delay identification.
/// Matches TransientBuffer capacity. Re-exported because the current-loop
/// validation phase uses the same buffer length cap.
pub(crate) const DELAY_SAMPLE_COUNT: u16 = 256;

const DIAG_ELECTRICAL_ID_EXCITATION: u8 = 1;
const DIAG_ELECTRICAL_ID_RESISTANCE_A: u8 = 2;
const DIAG_ELECTRICAL_ID_RESISTANCE_B: u8 = 3;
const DIAG_ELECTRICAL_ID_REJECT_NO_SAMPLES: u8 = 20;
const DIAG_ELECTRICAL_ID_REJECT_MODEL_SCALE_ZERO: u8 = 22;
// Codes 23–25 (resistance short/open/mismatch) are retired: the shared
// resistance runner now owns those quality gates.
const DIAG_ELECTRICAL_ID_REJECT_THETA_TOO_LARGE: u8 = 29;
const DIAG_ELECTRICAL_ID_REJECT_INDUCTANCE_FREQUENCY: u8 = 32;
const DIAG_ELECTRICAL_ID_REJECT_INDUCTANCE_CAPTURE: u8 = 33;

// ---------------------------------------------------------------------------
// Phase 1 model-assembly gates and warning bits.
// ---------------------------------------------------------------------------

/// Per-coil resistance disagreement >= 5 % (warning).
pub(crate) const INNER_WARN_BIT_RESISTANCE_MISMATCH: u8 = 1 << 0;
/// theta_e_us >= tau_e_us / 4 (warning).
pub(crate) const INNER_WARN_BIT_THETA_TAU: u8 = 1 << 3;

/// Per-coil resistance disagreement warn threshold (permille).
/// The hard resistance gates now live in the shared runner; this remains as a
/// non-critical confidence warning computed from the runner's per-axis slopes.
const RESISTANCE_MISMATCH_WARN_PERMILLE: u32 = 50;

/// Successful model assembly: the assembled `ElectricalModel` and the
/// non-critical warning bitfield.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) struct ModelAssembly {
    pub(crate) model: ElectricalModel,
    pub(crate) warning_flags: u8,
}

/// Reason a model assembly was rejected. Each variant logs distinct
/// `defmt::warn!` text and maps to `CommissionError::ElectricalIdFailed`
/// at the phase boundary.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) enum ModelAssemblyError {
    /// At least one coil had zero samples or zero current.
    SampleCount,
    /// L_int or L_uh evaluated to zero after AC reactance conversion.
    /// Distinct from `SampleCount` so RTT diagnostics name the actual
    /// failure (likely a fixed-point scale or rounding edge case).
    ModelScaleZero { l_int: u32, l_uh: u32 },
    /// theta_e_us >= tau_e_us / 2.
    ThetaTooLarge { theta_us: u32, tau_us: u32 },
}

fn log_model_assembly_failure(err: ModelAssemblyError) {
    match err {
        ModelAssemblyError::SampleCount => {
            defmt::warn!("electrical id: no usable per-coil samples");
        }
        ModelAssemblyError::ModelScaleZero { l_int, l_uh } => {
            defmt::warn!(
                "electrical id: model scale zero (l_int={}, l_uh={})",
                l_int,
                l_uh
            );
        }
        ModelAssemblyError::ThetaTooLarge { theta_us, tau_us } => {
            defmt::warn!(
                "electrical id: theta={}us >= tau={}us / 2",
                theta_us,
                tau_us
            );
        }
    }
}

fn diagnostic_for_model_assembly_failure(
    err: ModelAssemblyError,
) -> crate::selftest::CommissionDiagnostic {
    let (code, value0, value1, value2) = match err {
        ModelAssemblyError::SampleCount => (DIAG_ELECTRICAL_ID_REJECT_NO_SAMPLES, 0, 0, 0),
        ModelAssemblyError::ModelScaleZero { l_int, l_uh } => {
            (DIAG_ELECTRICAL_ID_REJECT_MODEL_SCALE_ZERO, l_int, l_uh, 0)
        }
        ModelAssemblyError::ThetaTooLarge { theta_us, tau_us } => (
            DIAG_ELECTRICAL_ID_REJECT_THETA_TOO_LARGE,
            theta_us,
            tau_us,
            0,
        ),
    };
    crate::selftest::CommissionDiagnostic {
        phase: PhaseId::ElectricalId.to_u8(),
        code,
        status: 1,
        value0,
        value1,
        value2,
    }
}

impl From<ModelAssemblyError> for CommissionError {
    fn from(err: ModelAssemblyError) -> Self {
        match err {
            ModelAssemblyError::SampleCount => Self::ElectricalModelNoSamples,
            ModelAssemblyError::ModelScaleZero { .. } => Self::ElectricalModelScaleZero,
            ModelAssemblyError::ThetaTooLarge { .. } => Self::ElectricalThetaTooLarge,
        }
    }
}

fn permille_diff(a: u32, b: u32) -> u32 {
    let average = (a as u64 + b as u64) / 2;
    if average == 0 {
        return 0;
    }
    let diff = a.abs_diff(b) as u64;
    (diff * 1000 / average) as u32
}

fn electrical_id_measurement_diagnostic(
    code: u8,
    value0: u32,
    value1: u32,
    value2: u32,
) -> crate::selftest::CommissionDiagnostic {
    crate::selftest::CommissionDiagnostic {
        phase: PhaseId::ElectricalId.to_u8(),
        code,
        status: 0,
        value0,
        value1,
        value2,
    }
}

/// Per-coil resistance count-slope from the shared runner result. `None` until
/// the resistance sweep has populated `data.resistance_result`.
///
/// The count-slope is in `ELECTRICAL_MODEL_RESISTANCE_SCALE` (milli) units
/// (`dV_count / dI_count * 1000`), so it feeds the per-coil tau and gain math
/// directly without conversion. Returns `(count_slope, count_slope, samples)`;
/// the slope is reused as the display `r_mohm`.
fn per_coil_resistance_slope(data: &ElectricalIdData, coil_idx: usize) -> Option<(u32, u32, u32)> {
    let result = data.resistance_result?;
    let axis = if coil_idx == 0 {
        result.axis0
    } else {
        result.axis1
    };
    let r_int = axis.r_count_slope_milli.max(0) as u32;
    if r_int == 0 {
        return None;
    }
    // `r_mohm` diagnostic value is the count-slope-milli reused as derived
    // display evidence; the runner owns the physical-scale gating.
    Some((r_int, r_int, u32::from(axis.selected_count)))
}

fn l_int_from_reactance(
    x_average_count_ratio_milli: u32,
    realized_frequency_millihz: u32,
) -> Option<u32> {
    if x_average_count_ratio_milli == 0 || realized_frequency_millihz == 0 {
        return None;
    }
    let numerator = u128::from(x_average_count_ratio_milli).checked_mul(1_000_000_000)?;
    let denominator =
        u128::from(L_INT_OMEGA_SCALE_MILLI).checked_mul(u128::from(realized_frequency_millihz))?;
    u32::try_from(numerator.checked_div(denominator)?).ok()
}

fn record_electrical_id_measurement_diagnostics(
    data: &ElectricalIdData,
    diagnostics: &mut crate::selftest::CommissionDiagnostics,
) {
    diagnostics.push(electrical_id_measurement_diagnostic(
        DIAG_ELECTRICAL_ID_EXCITATION,
        data.voltage,
        INDUCTANCE_AC_UD_EXT as u32,
        INDUCTANCE_OPENLOOP_VELOCITY_TARGET_RPM,
    ));
    for coil_idx in 0..2 {
        if let Some((avg_current, r_mohm, count)) = per_coil_resistance_slope(data, coil_idx) {
            diagnostics.push(electrical_id_measurement_diagnostic(
                if coil_idx == 0 {
                    DIAG_ELECTRICAL_ID_RESISTANCE_A
                } else {
                    DIAG_ELECTRICAL_ID_RESISTANCE_B
                },
                avg_current,
                r_mohm,
                count,
            ));
        }
    }
}

fn assemble_electrical_model(
    data: &ElectricalIdData,
    configured_theta_e_us: u32,
) -> Result<ModelAssembly, ModelAssemblyError> {
    // --- Per-coil resistance (from the shared multi-point runner) ---
    //
    // `r_int` and the per-coil entries come from the runner's count-space fit,
    // in `ELECTRICAL_MODEL_RESISTANCE_SCALE` (milli) units. The runner already
    // enforces the resistance quality gates (short/open/per-axis mismatch,
    // residual, drift); this path only translates its slopes into the model.
    let result = data
        .resistance_result
        .ok_or(ModelAssemblyError::SampleCount)?;
    let mut r_int_per_coil = [0u32; 2];
    let axis_results = [result.axis0, result.axis1];
    for (coil_idx, axis) in axis_results.iter().enumerate() {
        let r_int_coil = axis.r_count_slope_milli.max(0) as u32;
        if r_int_coil == 0 {
            return Err(ModelAssemblyError::SampleCount);
        }
        r_int_per_coil[coil_idx] = r_int_coil;
    }
    // Averaged model resistance is the run-selected count-slope.
    let r_int = result.selected_r_count_slope_milli.max(0) as u32;
    if r_int == 0 {
        return Err(ModelAssemblyError::SampleCount);
    }
    // `r_mohm` is derived display evidence only: the count-slope-milli reused
    // as a physical-display stand-in. The host applies real scale metadata.
    let r_mohm = r_int;

    // --- Hard gates (in spec order) ---
    //
    // Resistance short/open/per-axis-mismatch gates are owned by the shared
    // runner's count-space quality gates. The per-coil resistance
    // disagreement is still surfaced as a confidence warning below, computed
    // from the runner's per-axis slopes.
    let r_diff_permille = permille_diff(r_int_per_coil[0], r_int_per_coil[1]);
    let inductance = data.inductance.ok_or(ModelAssemblyError::SampleCount)?;
    let l_int = l_int_from_reactance(
        inductance.estimate.x_average_count_ratio_milli,
        inductance.run.realized_frequency_millihz,
    )
    .ok_or(ModelAssemblyError::ModelScaleZero { l_int: 0, l_uh: 0 })?;

    let tau_e_us =
        (u64::from(l_int) * u64::from(ELECTRICAL_MODEL_RESISTANCE_SCALE) / u64::from(r_int)) as u32;
    let l_uh = u64::from(l_int);
    if l_int == 0 || l_uh == 0 {
        return Err(ModelAssemblyError::ModelScaleZero {
            l_int,
            l_uh: l_uh as u32,
        });
    }

    // --- Configured transport-delay gate ---
    // tau_e_us is guaranteed non-zero here (model scale gate passed).
    if configured_theta_e_us >= tau_e_us / 2 {
        return Err(ModelAssemblyError::ThetaTooLarge {
            theta_us: configured_theta_e_us,
            tau_us: tau_e_us,
        });
    }

    // --- Warning flags (same evaluation order as hard gates) ---
    let mut warning_flags: u8 = 0;
    if r_diff_permille >= RESISTANCE_MISMATCH_WARN_PERMILLE {
        warning_flags |= INNER_WARN_BIT_RESISTANCE_MISMATCH;
    }
    if configured_theta_e_us >= tau_e_us / 4 {
        warning_flags |= INNER_WARN_BIT_THETA_TAU;
    }

    Ok(ModelAssembly {
        model: ElectricalModel {
            r_int,
            l_int,
            tau_e_us,
            theta_e_us: configured_theta_e_us,
            r_mohm,
            l_uh: l_uh as u32,
        },
        warning_flags,
    })
}

/// Adapter that exposes a [`CommissioningBackend`] as a
/// [`crate::resistance_identification::ResistanceBackend`] for the shared
/// resistance runner during commissioning.
///
/// This is the sanctioned commissioning managed-enable exception to the
/// direct-enable rule: the runner owns enable-pin assertion, so
/// [`is_armed_for_open_loop_voltage`](Self::is_armed_for_open_loop_voltage)
/// reports `true` and [`set_enable_pin`](Self::set_enable_pin) forwards to the
/// real backend.
struct CommissionResistanceBackend<'a, B: CommissioningBackend> {
    backend: &'a mut B,
}

impl<B: CommissioningBackend> crate::resistance_identification::ResistanceBackend
    for CommissionResistanceBackend<'_, B>
{
    async fn read_register(
        &mut self,
        addr: u8,
    ) -> Result<u32, crate::resistance_identification::ResistanceBackendError> {
        self.backend
            .read_register(Register::new(addr))
            .await
            .map_err(|_| crate::resistance_identification::ResistanceBackendError::Spi)
    }

    async fn write_register(
        &mut self,
        addr: u8,
        value: u32,
    ) -> Result<(), crate::resistance_identification::ResistanceBackendError> {
        self.backend
            .write_register(Register::new(addr), value)
            .await
            .map_err(|_| crate::resistance_identification::ResistanceBackendError::Spi)
    }

    async fn delay_ms(&mut self, ms: u64) {
        // Cooperative wait through the backend monotonic. `delay_us` is the
        // sanctioned commissioning yield; loop in <=1s chunks to keep the u32
        // microsecond argument well within range for long settle windows.
        let mut remaining_ms = ms;
        while remaining_ms > 0 {
            let chunk_ms = remaining_ms.min(1_000);
            self.backend.delay_us((chunk_ms * 1_000) as u32).await;
            remaining_ms -= chunk_ms;
        }
    }

    fn elapsed_us(&self) -> u64 {
        self.backend.elapsed_us()
    }

    fn set_enable_pin(&mut self, enabled: bool) {
        self.backend.set_enable_pin(enabled);
    }

    fn is_armed_for_open_loop_voltage(&self) -> bool {
        true
    }

    fn profile(&self) -> crate::resistance_identification::ResistanceProfile {
        self.backend.tmc_config().resistance_profile(
            self.backend
                .tmc_config()
                .resistance_scale_metadata_validated,
        )
    }

    fn encoder_counts_per_rev(&self) -> u32 {
        self.backend.mechanical_encoder_quadrature_counts_per_rev()
    }

    fn adc_offsets_ready(&self) -> bool {
        self.backend.adc_offsets_ready()
    }
}

fn begin_inductance_restore<B: CommissioningBackend>(
    state: &mut ElectricalIdState,
    backend: &mut B,
    primary_error: Option<CommissionError>,
    evidence: Option<InductanceEvidence>,
    timing: Option<CaptureTimingEvidence>,
) -> StepResult {
    backend.set_enable_pin(false);
    *state = ElectricalIdState::InductanceRestore {
        operation: 0,
        primary_error,
        evidence,
        timing,
    };
    StepResult::ReadyNow
}

fn inductance_status_is_critical(flags: u32) -> bool {
    flags & crate::autotune::current_loop_actionable_status_mask() != 0
}

fn pack_ud_drive(ud_ext: i16) -> u32 {
    foci_tmc4671::registers::UqUdExt::new()
        .with_ud_ext(ud_ext)
        .with_uq_ext(0)
        .into_bits()
}

fn decode_rotating_frame_current(actual: u32) -> (i16, i16) {
    let id_count = (actual & 0xffff) as u16 as i16;
    let iq_count = (actual >> 16) as u16 as i16;
    (id_count, iq_count)
}

fn inductance_encoder_delta(
    domain: EncoderDomain,
    start: u32,
    end: u32,
) -> Result<EncoderDelta, InductanceReject> {
    domain
        .delta(domain.count(start), domain.count(end))
        .map_err(|error| match error {
            EncoderMathError::InvalidCountsPerRev | EncoderMathError::AmbiguousHalfTurn => {
                InductanceReject::EncoderMotion
            }
        })
}

fn permille_delta(a: u32, b: u32) -> Option<u16> {
    let base = a.max(b);
    if base == 0 {
        return Some(0);
    }
    let delta = a.abs_diff(b) as u64;
    u16::try_from((delta * 1000 / u64::from(base)).min(u64::from(u16::MAX))).ok()
}

fn ratio_permille(numerator: u32, denominator: u32) -> Option<u16> {
    if denominator == 0 {
        return None;
    }
    u16::try_from((u64::from(numerator) * 1000 / u64::from(denominator)).min(u64::from(u16::MAX)))
        .ok()
}

fn selected_resistance_count_slope(data: &ElectricalIdData) -> Option<i32> {
    Some(data.resistance_result?.selected_r_count_slope_milli)
}

fn realized_frequency_in_range(realized_millihz: u32) -> bool {
    realized_millihz.abs_diff(INDUCTANCE_REALIZED_FREQUENCY_TARGET_MILLIHZ)
        <= INDUCTANCE_REALIZED_FREQUENCY_TOLERANCE_MILLIHZ
}

fn inductance_frequency_rejection_diagnostic(
    realized_frequency_millihz: u32,
    elapsed_us: u32,
    sample_count: u16,
) -> crate::selftest::CommissionDiagnostic {
    crate::selftest::CommissionDiagnostic {
        phase: PhaseId::ElectricalId.to_u8(),
        code: DIAG_ELECTRICAL_ID_REJECT_INDUCTANCE_FREQUENCY,
        status: 1,
        value0: realized_frequency_millihz,
        value1: elapsed_us,
        value2: u32::from(sample_count),
    }
}

#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
enum InductanceCaptureRejectReason {
    PhiInterval = 1,
    ZeroPhiDelta = 2,
    FrameAccumulator = 3,
    SaliencyAccumulator = 4,
    FirstHalfAccumulator = 5,
    SecondHalfAccumulator = 6,
    ElapsedInterpolation = 7,
    SaliencyAverageBelowBracket = 9,
    SaliencyAverageAboveBracket = 10,
}

impl InductanceCaptureRejectReason {
    fn code(self) -> u32 {
        self as u32
    }
}

fn pack_u16_pair(high: u16, low: u16) -> u32 {
    (u32::from(high) << 16) | u32::from(low)
}

fn pack_i16_pair(high: i16, low: i16) -> u32 {
    pack_u16_pair(high as u16, low as u16)
}

fn low_u16(value: u32) -> u16 {
    value.min(u32::from(u16::MAX)) as u16
}

fn inductance_capture_rejection_diagnostic(
    reason: InductanceCaptureRejectReason,
    sample_count: u16,
    value2: u32,
) -> crate::selftest::CommissionDiagnostic {
    crate::selftest::CommissionDiagnostic {
        phase: PhaseId::ElectricalId.to_u8(),
        code: DIAG_ELECTRICAL_ID_REJECT_INDUCTANCE_CAPTURE,
        status: 1,
        value0: reason.code(),
        value1: u32::from(sample_count),
        value2,
    }
}

fn push_inductance_capture_rejection(
    diagnostics: &mut crate::selftest::CommissionDiagnostics,
    reason: InductanceCaptureRejectReason,
    sample_count: u16,
    value2: u32,
) {
    diagnostics.push(inductance_capture_rejection_diagnostic(
        reason,
        sample_count,
        value2,
    ));
}

fn push_inductance_saliency_bracket_rejection(
    diagnostics: &mut crate::selftest::CommissionDiagnostics,
    estimate: InductanceEstimateEvidence,
) {
    let low = estimate
        .x_d_count_ratio_milli
        .min(estimate.x_q_count_ratio_milli);
    let high = estimate
        .x_d_count_ratio_milli
        .max(estimate.x_q_count_ratio_milli);
    let (reason, bound) = if estimate.x_average_count_ratio_milli < low {
        (
            InductanceCaptureRejectReason::SaliencyAverageBelowBracket,
            low,
        )
    } else {
        (
            InductanceCaptureRejectReason::SaliencyAverageAboveBracket,
            high,
        )
    };
    diagnostics.push(inductance_capture_rejection_diagnostic(
        reason,
        estimate.saliency_permille,
        pack_u16_pair(
            low_u16(estimate.x_average_count_ratio_milli),
            low_u16(bound),
        ),
    ));
}

enum InductanceSamplePoll {
    At,
    Complete {
        elapsed_us: u32,
        timing: CaptureTimingEvidence,
    },
}

async fn poll_inductance_sample<B: CommissioningBackend>(
    backend: &mut B,
    diagnostics: &mut crate::selftest::CommissionDiagnostics,
    capture: &mut InductanceCaptureState,
    now_us: u64,
) -> Result<InductanceSamplePoll, CommissionError> {
    use foci_tmc4671::registers::{OPENLOOP_PHI, PID_TORQUE_FLUX_ACTUAL};

    if now_us < capture.next_sample_us {
        return Ok(InductanceSamplePoll::At);
    }
    if backend.is_shutdown_requested() {
        return Err(CommissionError::Shutdown);
    }
    if capture.sample_count >= INDUCTANCE_MAX_AC_SAMPLE_COUNT {
        return Err(CommissionError::InductanceCaptureWindow);
    }

    let phi_io = timed_read_register(backend, OPENLOOP_PHI).await;
    let current_phi = phi_io.result.map_err(|_| CommissionError::Spi)? as u16;
    let current_io = timed_read_register(backend, PID_TORQUE_FLUX_ACTUAL).await;
    let actual = current_io.result.map_err(|_| CommissionError::Spi)?;
    let observation_us = backend.elapsed_us();
    if phi_io.overflowed || current_io.overflowed {
        return Err(CommissionError::InductanceTimingRejected);
    }
    let spi_wall_us = phi_io
        .wall_us
        .checked_add(current_io.wall_us)
        .ok_or(CommissionError::InductanceTimingRejected)?;
    let slot = capture
        .timing
        .record_sample(
            capture.next_sample_us,
            now_us,
            observation_us,
            observation_us,
            spi_wall_us,
        )
        .map_err(|_| CommissionError::InductanceTimingRejected)?;
    capture
        .timing
        .finish()
        .map_err(|_| CommissionError::InductanceTimingRejected)?;
    capture.next_sample_us = slot.next_deadline_us;

    let interval = crate::electrical_capture::phi_locked_ac_interval(
        capture.accumulated_phi_delta_counts,
        INDUCTANCE_AC_TARGET_PHI_DELTA_COUNTS,
        capture.previous_phi,
        current_phi,
    )
    .ok_or_else(|| {
        push_inductance_capture_rejection(
            diagnostics,
            InductanceCaptureRejectReason::PhiInterval,
            capture.sample_count,
            pack_u16_pair(capture.previous_phi, current_phi),
        );
        CommissionError::InductanceCaptureRejected
    })?;
    if interval.sample_weight_counts == 0 {
        push_inductance_capture_rejection(
            diagnostics,
            InductanceCaptureRejectReason::ZeroPhiDelta,
            capture.sample_count,
            pack_u16_pair(capture.previous_phi, current_phi),
        );
        return Err(CommissionError::InductanceCaptureRejected);
    }

    let (id_count, iq_count) = decode_rotating_frame_current(actual);
    let current_sample = pack_i16_pair(id_count, iq_count);
    if capture
        .accumulator
        .push_weighted(id_count, iq_count, interval.sample_weight_counts)
        .is_none()
    {
        push_inductance_capture_rejection(
            diagnostics,
            InductanceCaptureRejectReason::FrameAccumulator,
            capture.sample_count,
            current_sample,
        );
        return Err(CommissionError::InductanceCaptureRejected);
    }
    if capture
        .saliency
        .push_weighted_sample(
            current_phi,
            interval.sample_weight_counts,
            INDUCTANCE_AC_UD_EXT,
            id_count,
            iq_count,
        )
        .is_none()
    {
        push_inductance_capture_rejection(
            diagnostics,
            InductanceCaptureRejectReason::SaliencyAccumulator,
            capture.sample_count,
            current_sample,
        );
        return Err(CommissionError::InductanceCaptureRejected);
    }

    let half_target = INDUCTANCE_AC_TARGET_PHI_DELTA_COUNTS / 2;
    let before = capture.accumulated_phi_delta_counts;
    if before < half_target {
        let first_weight = interval.sample_weight_counts.min(half_target - before);
        if capture
            .first_half
            .push_weighted(id_count, iq_count, first_weight)
            .is_none()
        {
            push_inductance_capture_rejection(
                diagnostics,
                InductanceCaptureRejectReason::FirstHalfAccumulator,
                capture.sample_count,
                current_sample,
            );
            return Err(CommissionError::InductanceCaptureRejected);
        }
        let second_weight = interval.sample_weight_counts - first_weight;
        if second_weight != 0
            && capture
                .second_half
                .push_weighted(id_count, iq_count, second_weight)
                .is_none()
        {
            push_inductance_capture_rejection(
                diagnostics,
                InductanceCaptureRejectReason::SecondHalfAccumulator,
                capture.sample_count,
                current_sample,
            );
            return Err(CommissionError::InductanceCaptureRejected);
        }
    } else if capture
        .second_half
        .push_weighted(id_count, iq_count, interval.sample_weight_counts)
        .is_none()
    {
        push_inductance_capture_rejection(
            diagnostics,
            InductanceCaptureRejectReason::SecondHalfAccumulator,
            capture.sample_count,
            current_sample,
        );
        return Err(CommissionError::InductanceCaptureRejected);
    }

    capture.sample_count = capture.sample_count.saturating_add(1);
    let current_elapsed_us = observation_us
        .checked_sub(capture.start_us)
        .and_then(|elapsed| u32::try_from(elapsed).ok())
        .ok_or(CommissionError::InductanceTimingRejected)?;
    if interval.complete {
        let completed_elapsed_us = crate::electrical_capture::interpolate_phi_window_elapsed_us(
            capture.previous_elapsed_us,
            current_elapsed_us,
            interval,
        )
        .ok_or_else(|| {
            push_inductance_capture_rejection(
                diagnostics,
                InductanceCaptureRejectReason::ElapsedInterpolation,
                capture.sample_count,
                pack_u16_pair(
                    low_u16(capture.previous_elapsed_us),
                    low_u16(current_elapsed_us),
                ),
            );
            CommissionError::InductanceCaptureRejected
        })?;
        capture.accumulated_phi_delta_counts = interval.accumulated_phi_delta_counts;
        let timing = capture
            .timing
            .finish()
            .map_err(|_| CommissionError::InductanceTimingRejected)?;
        return Ok(InductanceSamplePoll::Complete {
            elapsed_us: completed_elapsed_us,
            timing,
        });
    }
    capture.previous_elapsed_us = current_elapsed_us;
    capture.previous_phi = current_phi;
    capture.accumulated_phi_delta_counts = interval.accumulated_phi_delta_counts;
    Ok(InductanceSamplePoll::At)
}

struct InductanceCaptureFinish<'a> {
    data: &'a ElectricalIdData,
    encoder_domain: EncoderDomain,
    encoder_start: u32,
    encoder_after: u32,
    elapsed_us: u32,
    phi_delta_counts: u32,
    sample_count: u16,
    status_flags_or: u32,
    accumulator: crate::electrical_capture::AcFrameAccumulator,
    first_half: crate::electrical_capture::AcFrameAccumulator,
    second_half: crate::electrical_capture::AcFrameAccumulator,
    saliency_accumulator: crate::electrical_capture::AcSaliencyAccumulator,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum InductanceReject {
    CaptureWindow,
    Frequency,
    LowQuadratureSignal,
    MissingResistance,
    Reactance,
    Drift,
    Saliency,
    EncoderMotion,
}

impl InductanceReject {
    fn commission_error(self) -> CommissionError {
        match self {
            Self::EncoderMotion => CommissionError::InductanceEncoderMotion,
            Self::CaptureWindow => CommissionError::InductanceCaptureWindow,
            Self::Frequency => CommissionError::InductanceFrequencyOutOfRange,
            Self::LowQuadratureSignal => CommissionError::InductanceLowQuadratureSignal,
            Self::MissingResistance => CommissionError::InductanceMissingResistanceEvidence,
            Self::Reactance => CommissionError::InductanceReactanceInvalid,
            Self::Drift => CommissionError::InductanceDriftInvalid,
            Self::Saliency => CommissionError::InductanceSaliencyInvalid,
        }
    }
}

fn finish_inductance_evidence(
    input: InductanceCaptureFinish<'_>,
) -> Result<InductanceEvidence, InductanceReject> {
    let InductanceCaptureFinish {
        data,
        encoder_domain,
        encoder_start,
        encoder_after,
        elapsed_us,
        phi_delta_counts,
        sample_count,
        status_flags_or,
        accumulator,
        first_half,
        second_half,
        saliency_accumulator,
    } = input;

    if phi_delta_counts != INDUCTANCE_AC_TARGET_PHI_DELTA_COUNTS
        || sample_count < INDUCTANCE_MIN_AC_SAMPLE_COUNT
    {
        return Err(InductanceReject::CaptureWindow);
    }

    let realized_frequency =
        crate::electrical_capture::realized_frequency_millihz(phi_delta_counts, elapsed_us)
            .ok_or(InductanceReject::Frequency)?;
    if !realized_frequency_in_range(realized_frequency) {
        return Err(InductanceReject::Frequency);
    }

    let frame = accumulator
        .finish(elapsed_us, realized_frequency, phi_delta_counts)
        .ok_or(InductanceReject::CaptureWindow)?;
    if frame.sample_count < INDUCTANCE_MIN_AC_SAMPLE_COUNT
        || frame.iq_mean_milli_count.abs() < INDUCTANCE_MIN_ABS_IQ_MILLI_COUNT
    {
        return Err(InductanceReject::LowQuadratureSignal);
    }

    let r_count_slope_milli =
        selected_resistance_count_slope(data).ok_or(InductanceReject::MissingResistance)?;
    let reactance = crate::electrical_capture::rotating_frame_reactance(
        frame,
        INDUCTANCE_AC_UD_EXT,
        r_count_slope_milli,
    )
    .ok_or(InductanceReject::Reactance)?;

    let half_phi_delta = phi_delta_counts / 2;
    let first_frame = first_half
        .finish(elapsed_us / 2, realized_frequency, half_phi_delta)
        .ok_or(InductanceReject::Drift)?;
    let second_frame = second_half
        .finish(elapsed_us / 2, realized_frequency, half_phi_delta)
        .ok_or(InductanceReject::Drift)?;
    let first_reactance = crate::electrical_capture::rotating_frame_reactance(
        first_frame,
        INDUCTANCE_AC_UD_EXT,
        r_count_slope_milli,
    )
    .ok_or(InductanceReject::Drift)?;
    let second_reactance = crate::electrical_capture::rotating_frame_reactance(
        second_frame,
        INDUCTANCE_AC_UD_EXT,
        r_count_slope_milli,
    )
    .ok_or(InductanceReject::Drift)?;
    let drift_permille = permille_delta(
        first_reactance.x_quad_count_ratio_milli,
        second_reactance.x_quad_count_ratio_milli,
    )
    .ok_or(InductanceReject::Drift)?;

    let saliency = match saliency_accumulator
        .finish(phi_delta_counts)
        .and_then(crate::electrical_capture::extract_saliency_from_second_harmonic)
    {
        Some(evidence) => evidence,
        None => crate::electrical_capture::AcSaliencyEvidence::not_determinable(
            reactance.x_quad_count_ratio_milli,
        ),
    };
    if saliency.status == crate::electrical_capture::AC_SALIENCY_DETERMINABLE
        && !saliency.brackets_average()
    {
        return Err(InductanceReject::Saliency);
    }

    let mut warning_flags = 0u16;
    let iq_rms_permille = ratio_permille(
        frame.iq_rms_milli_count,
        frame.iq_mean_milli_count.unsigned_abs(),
    )
    .ok_or(InductanceReject::LowQuadratureSignal)?;
    if iq_rms_permille > INDUCTANCE_MAX_IQ_RMS_WARN_PERMILLE {
        warning_flags |= INDUCTANCE_WARN_HIGH_RMS;
    }
    if drift_permille > INDUCTANCE_MAX_DRIFT_WARN_PERMILLE {
        warning_flags |= INDUCTANCE_WARN_DRIFT;
    }
    if reactance.x_mag_vs_quad_permille > INDUCTANCE_MAX_MAG_CROSSCHECK_WARN_PERMILLE {
        warning_flags |= INDUCTANCE_WARN_MAG_CROSSCHECK;
    }
    if status_flags_or != 0 {
        warning_flags |= INDUCTANCE_WARN_STATUS_FLAGS;
    }

    let encoder_delta = inductance_encoder_delta(encoder_domain, encoder_start, encoder_after)?;
    if encoder_delta.absolute_counts() > INDUCTANCE_MAX_ENCODER_DELTA_COUNTS {
        return Err(InductanceReject::EncoderMotion);
    }
    let encoder_delta_counts = encoder_delta.signed_counts();

    Ok(InductanceEvidence {
        run: InductanceRunEvidence {
            source: INDUCTANCE_SOURCE_ROTATING_FIELD,
            status: 0,
            warning_flags,
            ud_count: INDUCTANCE_AC_UD_EXT,
            realized_frequency_millihz: realized_frequency,
            elapsed_us,
            openloop_phi_delta_counts: phi_delta_counts,
            sample_count: frame.sample_count,
            encoder_delta_counts,
            status_flags_or,
        },
        frame: InductanceFrameEvidence {
            id_mean_milli_count: frame.id_mean_milli_count,
            iq_mean_milli_count: frame.iq_mean_milli_count,
            id_rms_milli_count: frame.id_rms_milli_count,
            iq_rms_milli_count: frame.iq_rms_milli_count,
            drift_permille,
            zero_id_mean_milli_count: 0,
            zero_iq_mean_milli_count: 0,
        },
        estimate: InductanceEstimateEvidence {
            x_average_count_ratio_milli: reactance.x_quad_count_ratio_milli,
            x_d_count_ratio_milli: saliency.x_d_count_ratio_milli,
            x_q_count_ratio_milli: saliency.x_q_count_ratio_milli,
            saliency_status: saliency.status,
            saliency_permille: saliency.saliency_permille,
            x_mag_nominal_count_ratio_milli: reactance.x_mag_nominal_count_ratio_milli,
            x_mag_shift_minus_permille: reactance.x_mag_shift_minus_permille,
            x_mag_shift_plus_permille: reactance.x_mag_shift_plus_permille,
            x_mag_vs_quad_permille: reactance.x_mag_vs_quad_permille,
        },
    })
}

fn saliency_for_frame_average(
    frame_average_count_ratio_milli: u32,
    saliency: crate::electrical_capture::AcSaliencyEvidence,
) -> Result<crate::electrical_capture::AcSaliencyEvidence, InductanceReject> {
    if saliency.status != crate::electrical_capture::AC_SALIENCY_DETERMINABLE {
        return Ok(saliency);
    }

    let low = saliency
        .x_d_count_ratio_milli
        .min(saliency.x_q_count_ratio_milli);
    let high = saliency
        .x_d_count_ratio_milli
        .max(saliency.x_q_count_ratio_milli);
    if low <= frame_average_count_ratio_milli && frame_average_count_ratio_milli <= high {
        return Ok(saliency);
    }

    let miss = if frame_average_count_ratio_milli < low {
        low - frame_average_count_ratio_milli
    } else {
        frame_average_count_ratio_milli - high
    };
    if miss > INDUCTANCE_SALIENCY_BRACKET_TOLERANCE_COUNT_RATIO_MILLI {
        return Err(InductanceReject::Saliency);
    }

    // Low-saliency D/Q split evidence is below the measurement consistency
    // floor. Keep the angle-averaged production L and decline to publish a
    // physically weak split rather than hard-failing a valid capture.
    Ok(
        crate::electrical_capture::AcSaliencyEvidence::not_determinable(
            frame_average_count_ratio_milli,
        ),
    )
}

/// Advance electrical identification sub-state.
///
/// Runs shared multi-point resistance identification, then per-coil
/// rotating-field inductance and delay (transient capture with a two-point
/// fit). Assembles an `ElectricalModel` and validates before
/// transitioning.
pub(crate) async fn step_electrical_identification<B: CommissioningBackend>(
    phase: &mut InnerPhase,
    buf: &mut TransientBuffer,
    sequence: InnerSequence,
    selftest_results: &mut crate::selftest::SelftestResults,
    diagnostics: &mut crate::selftest::CommissionDiagnostics,
    backend: &mut B,
) -> StepResult {
    use foci_tmc4671::registers::*;

    let (electrical_id_state, data) = match phase {
        InnerPhase::ElectricalId { state, data } => (state, data),
        _ => return StepResult::Continue,
    };

    match electrical_id_state {
        // ---------------------------------------------------------------
        // Resistance identification (shared multi-point runner)
        // ---------------------------------------------------------------
        ElectricalIdState::ResistanceStart => {
            data.encoder_domain =
                match EncoderDomain::new(backend.mechanical_encoder_quadrature_counts_per_rev()) {
                    Ok(domain) => Some(domain),
                    Err(EncoderMathError::InvalidCountsPerRev) => {
                        return super::fail_inner(phase, CommissionError::EncoderFault);
                    }
                    Err(EncoderMathError::AmbiguousHalfTurn) => unreachable!(),
                };
            let config = backend.tmc_config();
            let profile = config.resistance_profile(config.resistance_scale_metadata_validated);
            let runner = match crate::resistance_identification::ResistanceRunner::new(
                crate::resistance_identification::ResistanceRunMode::CommissioningManagedEnable,
                profile,
                backend.elapsed_us(),
            ) {
                Ok(runner) => runner,
                Err(error) => {
                    defmt::warn!("resistance id failed: {}", error);
                    return super::fail_inner(phase, CommissionError::from(error));
                }
            };
            *electrical_id_state = ElectricalIdState::ResistanceIdentify { runner };
            StepResult::ReadyNow
        }

        ElectricalIdState::ResistanceIdentify { runner } => {
            let now_us = backend.elapsed_us();
            let poll = runner
                .poll(&mut CommissionResistanceBackend { backend }, now_us)
                .await;

            match poll {
                crate::resistance_identification::ResistancePoll::ReadyNow => StepResult::ReadyNow,
                crate::resistance_identification::ResistancePoll::At(deadline_us) => {
                    StepResult::At(deadline_us)
                }
                crate::resistance_identification::ResistancePoll::Complete(Ok(resistance)) => {
                    data.resistance_result = Some(resistance);
                    *electrical_id_state = ElectricalIdState::InductanceSetup;
                    StepResult::ReadyNow
                }
                crate::resistance_identification::ResistancePoll::Complete(Err(error)) => {
                    defmt::warn!("resistance id failed: {}", error);
                    super::fail_inner(phase, CommissionError::from(error))
                }
            }
        }

        // ---------------------------------------------------------------
        // Inductance measurement (phi-locked rotating field)
        // ---------------------------------------------------------------
        ElectricalIdState::InductanceSetup => {
            if data.encoder_domain.is_none() {
                return super::fail_inner(phase, CommissionError::EncoderFault);
            }
            *electrical_id_state = ElectricalIdState::InductanceSetupOperation { operation: 0 };
            StepResult::ReadyNow
        }

        ElectricalIdState::InductanceSetupOperation { operation } => {
            let pidout_limit = data.voltage.min(INDUCTANCE_AC_UD_EXT as u32);
            let result = match *operation {
                0 => {
                    backend
                        .write_register(PIDOUT_UQ_UD_LIMITS, pidout_limit)
                        .await
                }
                1 => backend.write_register(UQ_UD_EXT, 0).await,
                2 => backend.write_register(OPENLOOP_VELOCITY_TARGET, 0).await,
                3 => backend.write_register(OPENLOOP_MODE, 0).await,
                4 => {
                    backend
                        .write_register(OPENLOOP_ACCELERATION, INDUCTANCE_OPENLOOP_ACCELERATION)
                        .await
                }
                5 => {
                    backend
                        .write_register(
                            OPENLOOP_VELOCITY_TARGET,
                            INDUCTANCE_OPENLOOP_VELOCITY_TARGET_RPM,
                        )
                        .await
                }
                6 => {
                    backend
                        .write_register(PHI_E_SELECTION, PHI_E_SELECTION_OPENLOOP)
                        .await
                }
                7 => {
                    backend
                        .write_register(MODE_RAMP_MODE_MOTION, MotionMode::UqUdExt as u32)
                        .await
                }
                8 => backend
                    .write_register_verified(PWM_SV_CHOP, 7)
                    .await
                    .map(|_| ()),
                _ => backend.write_register(STATUS_FLAGS, 0).await,
            };
            if result.is_err() {
                return begin_inductance_restore(
                    electrical_id_state,
                    backend,
                    Some(CommissionError::Spi),
                    None,
                    None,
                );
            }
            if *operation < 9 {
                *operation += 1;
                return StepResult::ReadyNow;
            }
            let Some(deadline_us) = backend
                .elapsed_us()
                .checked_add(INDUCTANCE_OPENLOOP_RAMP_TIMEOUT_US)
            else {
                return begin_inductance_restore(
                    electrical_id_state,
                    backend,
                    Some(CommissionError::InvalidSchedule),
                    None,
                    None,
                );
            };
            backend.set_enable_pin(true);
            *electrical_id_state = ElectricalIdState::InductanceWaitVelocity { deadline_us };
            StepResult::ReadyNow
        }

        ElectricalIdState::InductanceWaitVelocity { deadline_us } => {
            let velocity = match backend.read_register(OPENLOOP_VELOCITY_ACTUAL).await {
                Ok(velocity) => velocity,
                Err(_) => {
                    return begin_inductance_restore(
                        electrical_id_state,
                        backend,
                        Some(CommissionError::Spi),
                        None,
                        None,
                    );
                }
            };
            if velocity != INDUCTANCE_OPENLOOP_VELOCITY_TARGET_RPM {
                if backend.elapsed_us() < *deadline_us {
                    return StepResult::At(*deadline_us);
                }
                return begin_inductance_restore(
                    electrical_id_state,
                    backend,
                    Some(CommissionError::InductanceVelocityTimeout),
                    None,
                    None,
                );
            }
            *electrical_id_state = ElectricalIdState::InductanceReadEncoderStart;
            StepResult::ReadyNow
        }

        ElectricalIdState::InductanceReadEncoderStart => {
            let encoder_start = match backend.read_register(ABN_DECODER_COUNT).await {
                Ok(count) => count,
                Err(_) => {
                    return begin_inductance_restore(
                        electrical_id_state,
                        backend,
                        Some(CommissionError::Spi),
                        None,
                        None,
                    );
                }
            };
            *electrical_id_state = ElectricalIdState::InductanceApplyDrive { encoder_start };
            StepResult::ReadyNow
        }

        ElectricalIdState::InductanceApplyDrive { encoder_start } => {
            if backend
                .write_register(UQ_UD_EXT, pack_ud_drive(INDUCTANCE_AC_UD_EXT))
                .await
                .is_err()
            {
                return begin_inductance_restore(
                    electrical_id_state,
                    backend,
                    Some(CommissionError::Spi),
                    None,
                    None,
                );
            }
            let Some(deadline_us) = backend.elapsed_us().checked_add(INDUCTANCE_AC_SETTLE_US)
            else {
                return begin_inductance_restore(
                    electrical_id_state,
                    backend,
                    Some(CommissionError::InvalidSchedule),
                    None,
                    None,
                );
            };
            let encoder_domain = data
                .encoder_domain
                .expect("validated before inductance setup");
            *electrical_id_state = ElectricalIdState::InductanceDriveSettle {
                deadline_us,
                encoder_start: *encoder_start,
                encoder_domain,
            };
            StepResult::At(deadline_us)
        }

        ElectricalIdState::InductanceDriveSettle {
            deadline_us,
            encoder_start,
            encoder_domain,
        } => {
            if backend.elapsed_us() < *deadline_us {
                return StepResult::At(*deadline_us);
            }
            *electrical_id_state = ElectricalIdState::InductanceReadPreStatus {
                encoder_start: *encoder_start,
                encoder_domain: *encoder_domain,
            };
            StepResult::ReadyNow
        }

        ElectricalIdState::InductanceReadPreStatus {
            encoder_start,
            encoder_domain,
        } => {
            let status_flags_or = match backend.read_register(STATUS_FLAGS).await {
                Ok(status) => status,
                Err(_) => {
                    return begin_inductance_restore(
                        electrical_id_state,
                        backend,
                        Some(CommissionError::Spi),
                        None,
                        None,
                    );
                }
            };
            if inductance_status_is_critical(status_flags_or) {
                return begin_inductance_restore(
                    electrical_id_state,
                    backend,
                    Some(CommissionError::InductanceCaptureRejected),
                    None,
                    None,
                );
            }
            *electrical_id_state = ElectricalIdState::InductanceReadStartPhi {
                encoder_start: *encoder_start,
                encoder_domain: *encoder_domain,
                status_flags_or,
            };
            StepResult::ReadyNow
        }

        ElectricalIdState::InductanceReadStartPhi {
            encoder_start,
            encoder_domain,
            status_flags_or,
        } => {
            let previous_phi = match backend.read_register(OPENLOOP_PHI).await {
                Ok(phi) => phi as u16,
                Err(_) => {
                    return begin_inductance_restore(
                        electrical_id_state,
                        backend,
                        Some(CommissionError::Spi),
                        None,
                        None,
                    );
                }
            };
            let start_us = backend.elapsed_us();
            let Some(next_sample_us) =
                start_us.checked_add(u64::from(INDUCTANCE_AC_SAMPLE_PERIOD_US))
            else {
                return begin_inductance_restore(
                    electrical_id_state,
                    backend,
                    Some(CommissionError::InvalidSchedule),
                    None,
                    None,
                );
            };
            *electrical_id_state = ElectricalIdState::InductanceSample {
                capture: InductanceCaptureState {
                    encoder_domain: *encoder_domain,
                    encoder_start: *encoder_start,
                    start_us,
                    previous_elapsed_us: 0,
                    previous_phi,
                    accumulated_phi_delta_counts: 0,
                    accumulator: crate::electrical_capture::AcFrameAccumulator::new(),
                    first_half: crate::electrical_capture::AcFrameAccumulator::new(),
                    second_half: crate::electrical_capture::AcFrameAccumulator::new(),
                    saliency: crate::electrical_capture::AcSaliencyAccumulator::new(),
                    sample_count: 0,
                    status_flags_or: *status_flags_or,
                    next_sample_us,
                    timing: CaptureTiming::new(
                        INDUCTANCE_AC_SAMPLE_PERIOD_US,
                        TimingEnvelope::INDUCTANCE,
                    ),
                },
            };
            StepResult::At(next_sample_us)
        }

        ElectricalIdState::InductanceSample { capture } => {
            let now_us = backend.elapsed_us();
            match poll_inductance_sample(backend, diagnostics, capture, now_us).await {
                Ok(InductanceSamplePoll::At) => StepResult::At(capture.next_sample_us),
                Ok(InductanceSamplePoll::Complete { elapsed_us, timing }) => {
                    *electrical_id_state = ElectricalIdState::InductanceReadStatus {
                        capture: *capture,
                        completed_elapsed_us: elapsed_us,
                        timing,
                    };
                    StepResult::ReadyNow
                }
                Err(error) => {
                    begin_inductance_restore(electrical_id_state, backend, Some(error), None, None)
                }
            }
        }

        ElectricalIdState::InductanceReadStatus {
            capture,
            completed_elapsed_us,
            timing,
        } => {
            let mut next_capture = *capture;
            let completed_elapsed_us = *completed_elapsed_us;
            let timing = *timing;
            let status = match backend.read_register(STATUS_FLAGS).await {
                Ok(status) => status,
                Err(_) => {
                    return begin_inductance_restore(
                        electrical_id_state,
                        backend,
                        Some(CommissionError::Spi),
                        None,
                        Some(timing),
                    );
                }
            };
            next_capture.status_flags_or |= status;
            if inductance_status_is_critical(status) {
                return begin_inductance_restore(
                    electrical_id_state,
                    backend,
                    Some(CommissionError::InductanceCaptureRejected),
                    None,
                    Some(timing),
                );
            }
            *electrical_id_state = ElectricalIdState::InductanceReadEncoder {
                capture: next_capture,
                completed_elapsed_us,
                timing,
            };
            StepResult::ReadyNow
        }

        ElectricalIdState::InductanceReadEncoder {
            capture,
            completed_elapsed_us,
            timing,
        } => {
            let capture = *capture;
            let completed_elapsed_us = *completed_elapsed_us;
            let timing = *timing;
            let encoder_after = match backend.read_register(ABN_DECODER_COUNT).await {
                Ok(count) => count,
                Err(_) => {
                    return begin_inductance_restore(
                        electrical_id_state,
                        backend,
                        Some(CommissionError::Spi),
                        None,
                        Some(timing),
                    );
                }
            };
            *electrical_id_state = ElectricalIdState::InductanceEvaluate {
                capture,
                encoder_after,
                completed_elapsed_us,
                timing,
            };
            StepResult::ReadyNow
        }

        ElectricalIdState::InductanceEvaluate {
            capture,
            encoder_after,
            completed_elapsed_us,
            timing,
        } => {
            let capture = *capture;
            let encoder_after = *encoder_after;
            let completed_elapsed_us = *completed_elapsed_us;
            let timing = *timing;
            let evidence = finish_inductance_evidence(InductanceCaptureFinish {
                data,
                encoder_domain: capture.encoder_domain,
                encoder_start: capture.encoder_start,
                encoder_after,
                elapsed_us: completed_elapsed_us,
                phi_delta_counts: capture.accumulated_phi_delta_counts,
                sample_count: capture.sample_count,
                status_flags_or: capture.status_flags_or,
                accumulator: capture.accumulator,
                first_half: capture.first_half,
                second_half: capture.second_half,
                saliency_accumulator: capture.saliency,
            });
            match evidence {
                Ok(mut evidence) => {
                    let saliency = crate::electrical_capture::AcSaliencyEvidence {
                        status: evidence.estimate.saliency_status,
                        x_average_count_ratio_milli: evidence.estimate.x_average_count_ratio_milli,
                        x_d_count_ratio_milli: evidence.estimate.x_d_count_ratio_milli,
                        x_q_count_ratio_milli: evidence.estimate.x_q_count_ratio_milli,
                        saliency_permille: evidence.estimate.saliency_permille,
                    };
                    match saliency_for_frame_average(
                        evidence.estimate.x_average_count_ratio_milli,
                        saliency,
                    ) {
                        Ok(saliency) => {
                            evidence.estimate.x_d_count_ratio_milli =
                                saliency.x_d_count_ratio_milli;
                            evidence.estimate.x_q_count_ratio_milli =
                                saliency.x_q_count_ratio_milli;
                            evidence.estimate.saliency_status = saliency.status;
                            evidence.estimate.saliency_permille = saliency.saliency_permille;
                            begin_inductance_restore(
                                electrical_id_state,
                                backend,
                                None,
                                Some(evidence),
                                Some(timing),
                            )
                        }
                        Err(InductanceReject::Saliency) => {
                            push_inductance_saliency_bracket_rejection(
                                diagnostics,
                                evidence.estimate,
                            );
                            begin_inductance_restore(
                                electrical_id_state,
                                backend,
                                Some(CommissionError::InductanceCaptureRejected),
                                None,
                                Some(timing),
                            )
                        }
                        Err(error) => begin_inductance_restore(
                            electrical_id_state,
                            backend,
                            Some(error.commission_error()),
                            None,
                            Some(timing),
                        ),
                    }
                }
                Err(error) => {
                    if error == InductanceReject::Frequency {
                        let frequency = crate::electrical_capture::realized_frequency_millihz(
                            capture.accumulated_phi_delta_counts,
                            completed_elapsed_us,
                        )
                        .unwrap_or(0);
                        diagnostics.push(inductance_frequency_rejection_diagnostic(
                            frequency,
                            completed_elapsed_us,
                            capture.sample_count,
                        ));
                    }
                    begin_inductance_restore(
                        electrical_id_state,
                        backend,
                        Some(error.commission_error()),
                        None,
                        Some(timing),
                    )
                }
            }
        }

        ElectricalIdState::InductanceRestore {
            operation,
            primary_error,
            evidence,
            timing,
        } => {
            let (register, value) = match *operation {
                0 => (UQ_UD_EXT, 0),
                1 => (OPENLOOP_VELOCITY_TARGET, 0),
                2 => (MODE_RAMP_MODE_MOTION, MotionMode::Stopped as u32),
                _ => (PHI_E_SELECTION, 0),
            };
            if backend.write_register(register, value).await.is_err() {
                *primary_error = Some(CommissionError::Spi);
            }
            if *operation < 3 {
                *operation += 1;
                return StepResult::ReadyNow;
            }
            if let Some(error) = *primary_error {
                return super::fail_inner(phase, error);
            }
            let Some(evidence) = *evidence else {
                return super::fail_inner(phase, CommissionError::InductanceCaptureRejected);
            };
            data.inductance = Some(evidence);
            data.inductance_timing = *timing;
            let Some(deadline_us) = backend.elapsed_us().checked_add(ELECTRICAL_COOLDOWN_US) else {
                return super::fail_inner(phase, CommissionError::InvalidSchedule);
            };
            *electrical_id_state = ElectricalIdState::InductanceDisable { deadline_us };
            StepResult::At(deadline_us)
        }

        ElectricalIdState::InductanceDisable { deadline_us } => {
            if backend.elapsed_us() < *deadline_us {
                return StepResult::At(*deadline_us);
            }
            *electrical_id_state = ElectricalIdState::DelaySetup;
            StepResult::ReadyNow
        }

        // ---------------------------------------------------------------
        // Delay identification (transient capture)
        // ---------------------------------------------------------------
        ElectricalIdState::DelaySetup => {
            // Configure for step response on coil A (D-axis).
            if backend.write_register(PHI_E_SELECTION, 1).await.is_err()
                || backend.write_register(PHI_E_EXT, 0).await.is_err()
                || backend.write_register(UQ_UD_EXT, 0).await.is_err()
                || backend
                    .write_register(MODE_RAMP_MODE_MOTION, 8)
                    .await
                    .is_err()
            {
                return super::fail_inner(phase, CommissionError::Spi);
            }
            backend.set_enable_pin(true);

            // Reset transient buffer for 160us sampling.
            buf.reset(ELECTRICAL_SAMPLE_PERIOD_US as u16);

            let deadline = backend.elapsed_us().wrapping_add(DELAY_PRE_SETTLE_US);
            *electrical_id_state = ElectricalIdState::DelaySettle {
                deadline_us: deadline,
            };
            StepResult::Continue
        }

        ElectricalIdState::DelaySettle { deadline_us } => {
            if backend.elapsed_us().wrapping_sub(*deadline_us) >= 0x8000_0000 {
                return StepResult::Continue; // Not yet.
            }
            *electrical_id_state = ElectricalIdState::DelayStep;
            StepResult::Continue
        }

        ElectricalIdState::DelayStep => {
            // Read baseline current (should be ~0 with UQ_UD_EXT=0).
            let baseline = match backend.read_register(PID_TORQUE_FLUX_ACTUAL).await {
                Ok(actual) => (actual & 0xFFFF) as i16,
                Err(_) => return super::fail_inner(phase, CommissionError::Spi),
            };

            // Apply voltage step.
            if backend
                .write_register(UQ_UD_EXT, data.voltage & 0xFFFF)
                .await
                .is_err()
            {
                return super::fail_inner(phase, CommissionError::Spi);
            }

            let next = backend
                .elapsed_us()
                .wrapping_add(ELECTRICAL_SAMPLE_PERIOD_US);
            *electrical_id_state = ElectricalIdState::DelaySample {
                baseline,
                next_sample_us: next,
            };
            StepResult::Continue
        }

        ElectricalIdState::DelaySample {
            baseline,
            next_sample_us,
        } => {
            // Capture samples into TransientBuffer at 160us intervals.
            loop {
                if buf.len() >= DELAY_SAMPLE_COUNT || buf.is_full() {
                    // Capture complete. Disable motor.
                    let _ = backend.write_register(MODE_RAMP_MODE_MOTION, 0).await;
                    let _ = backend.write_register(UQ_UD_EXT, 0).await;
                    backend.set_enable_pin(false);
                    let deadline = backend.elapsed_us().wrapping_add(ELECTRICAL_COOLDOWN_US);
                    *electrical_id_state = ElectricalIdState::DelayDisable {
                        deadline_us: deadline,
                    };
                    return StepResult::Continue;
                }
                if backend.elapsed_us().wrapping_sub(*next_sample_us) >= 0x8000_0000 {
                    break; // Not yet time, yield.
                }
                match backend.read_register(PID_TORQUE_FLUX_ACTUAL).await {
                    Ok(actual) => {
                        // Store absolute current relative to baseline.
                        let current = (actual & 0xFFFF) as i16;
                        let relative = (current as i32 - *baseline as i32).unsigned_abs();
                        buf.push(relative);
                    }
                    Err(_) => {
                        return super::fail_inner(phase, CommissionError::Spi);
                    }
                }
                *next_sample_us = next_sample_us.wrapping_add(ELECTRICAL_SAMPLE_PERIOD_US);
            }
            StepResult::Continue
        }

        ElectricalIdState::DelayDisable { deadline_us } => {
            if backend.elapsed_us().wrapping_sub(*deadline_us) >= 0x8000_0000 {
                return StepResult::Continue; // Not yet.
            }
            *electrical_id_state = ElectricalIdState::Validate;
            StepResult::Continue
        }

        // ---------------------------------------------------------------
        // Model assembly and validation
        // ---------------------------------------------------------------
        ElectricalIdState::Validate => {
            let assembly = match assemble_electrical_model(
                data,
                backend.tmc_config().electrical_model_theta_e_us,
            ) {
                Ok(assembly) => assembly,
                Err(err) => {
                    log_model_assembly_failure(err);
                    record_electrical_id_measurement_diagnostics(data, diagnostics);
                    diagnostics.push(diagnostic_for_model_assembly_failure(err));
                    return super::fail_inner(phase, CommissionError::from(err));
                }
            };

            match sequence {
                InnerSequence::Selftest => {
                    selftest_results.set(
                        7,
                        crate::selftest::SelftestStageValue {
                            status: 0,
                            value: assembly.model.r_mohm,
                        },
                    );
                    selftest_results.set(
                        8,
                        crate::selftest::SelftestStageValue {
                            status: 0,
                            value: assembly.model.l_uh,
                        },
                    );
                    crate::tmc_safe_state::restore_safe_state_via_backend(backend).await;
                    StepResult::SelftestDone
                }
                InnerSequence::Commission => {
                    // Carry the model + Phase 5 confidence flags into the
                    // current-tune phase. Bits 4–6 (retry / fallback / host
                    // default) are added later by current validation and
                    // host parameter resolution. The resistance result is
                    // carried so the final InnerResult can persist the
                    // count-space fit and the 64x audit comparison. It is
                    // `Some` here because `ResistanceIdentify` always runs
                    // before `Validate` in the commission sequence.
                    let resistance_result = data
                        .resistance_result
                        .expect("resistance result populated before Validate");
                    *phase = InnerPhase::CurrentTune {
                        model: assembly.model,
                        inner_warning_flags: assembly.warning_flags,
                        resistance_result,
                        inductance: data
                            .inductance
                            .unwrap_or(crate::commissioning::types::InductanceEvidence::EMPTY),
                    };
                    StepResult::PhaseChanged(PhaseId::CurrentTune)
                }
                InnerSequence::Calibrate => {
                    // Calibrate never reaches electrical ID.
                    unreachable!()
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::autotune::TransientBuffer;
    use crate::commission_dispatch::test_support::{MockBackend, SpiOp};
    use crate::commissioning::backend::CommissionBoardConfig;
    use crate::commissioning::inner::ElectricalIdData;
    use crate::commissioning::types::{INDUCTANCE_WARN_STATUS_FLAGS, StepResult};
    use crate::resistance_identification::{
        ElectricalAxis, ResistanceAxisResult, ResistanceRunResult, ResistanceWarningFlags,
        SignedAnchorEvidence,
    };
    use foci_tmc4671::registers::{
        MODE_RAMP_MODE_MOTION, OPENLOOP_PHI, OPENLOOP_VELOCITY_TARGET, PHI_E_SELECTION,
        PID_TORQUE_FLUX_ACTUAL, PIDOUT_UQ_UD_LIMITS, PWM_SV_CHOP, STATUS_FLAGS, UQ_UD_EXT,
    };

    const TEST_THETA_E_US: u32 = 160;

    #[test]
    fn inductance_fixed_window_keeps_minimum_samples_at_upper_frequency() {
        assert_eq!(INDUCTANCE_AC_SAMPLE_PERIOD_US, 100);
        assert_eq!(INDUCTANCE_AC_TARGET_PHI_DELTA_COUNTS, 10 * 65_536);
        assert_eq!(scheduled_samples_for_fixed_window(1_050_000), 95);
        assert!(
            scheduled_samples_for_fixed_window(1_050_000)
                >= u32::from(INDUCTANCE_MIN_AC_SAMPLE_COUNT)
        );
    }

    /// Build a synthetic per-axis fit at the given count-slope (milli-units).
    fn axis_result(axis: ElectricalAxis, slope_milli: i32) -> ResistanceAxisResult {
        ResistanceAxisResult {
            axis,
            r_count_slope_milli: slope_milli,
            slope_num: i64::from(slope_milli),
            slope_den: 1000,
            intercept_count: 0,
            rmse_permille: 0,
            selected_mask: 0b1111_1000,
            excluded_point_mask: 0b0000_0111,
            selected_count: 5,
            signed_anchor: Some(SignedAnchorEvidence {
                positive_current_count: 900,
                negative_current_count: -900,
                signed_count_slope_milli: slope_milli,
                asymmetry_permille: 0,
                repeated_positive_current_count: 900,
                drift_permille: 0,
            }),
            warning_flags: ResistanceWarningFlags::NONE,
        }
    }

    /// Build a synthetic two-axis resistance result with the given per-axis
    /// count-slopes (milli-units). The selected slope is their average, matching
    /// the runner's `evaluate_resistance_result` behavior.
    fn resistance_result(slope0_milli: i32, slope1_milli: i32) -> ResistanceRunResult {
        ResistanceRunResult {
            selected_r_count_slope_milli: (slope0_milli + slope1_milli) / 2,
            status_flags_or: 0,
            axis0: axis_result(ElectricalAxis::PHASE0, slope0_milli),
            axis1: axis_result(ElectricalAxis::PHASE90, slope1_milli),
            warning_flags: ResistanceWarningFlags::NONE,
        }
    }

    fn inductance_evidence_fixture(
        x_average_count_ratio_milli: u32,
        realized_frequency_millihz: u32,
    ) -> InductanceEvidence {
        InductanceEvidence {
            run: InductanceRunEvidence {
                source: INDUCTANCE_SOURCE_ROTATING_FIELD,
                status: 0,
                warning_flags: 0,
                ud_count: INDUCTANCE_AC_UD_EXT,
                realized_frequency_millihz,
                elapsed_us: 8_000,
                openloop_phi_delta_counts: INDUCTANCE_AC_TARGET_PHI_DELTA_COUNTS,
                sample_count: 104,
                encoder_delta_counts: 0,
                status_flags_or: 0,
            },
            frame: InductanceFrameEvidence {
                id_mean_milli_count: 20_000,
                iq_mean_milli_count: -84_000,
                id_rms_milli_count: 5_000,
                iq_rms_milli_count: 21_000,
                drift_permille: 40,
                zero_id_mean_milli_count: 0,
                zero_iq_mean_milli_count: 0,
            },
            estimate: InductanceEstimateEvidence {
                x_average_count_ratio_milli,
                x_d_count_ratio_milli: x_average_count_ratio_milli + 600,
                x_q_count_ratio_milli: x_average_count_ratio_milli.saturating_sub(600),
                saliency_status: crate::electrical_capture::AC_SALIENCY_DETERMINABLE,
                saliency_permille: 140,
                x_mag_nominal_count_ratio_milli: x_average_count_ratio_milli + 180,
                x_mag_shift_minus_permille: 4,
                x_mag_shift_plus_permille: 4,
                x_mag_vs_quad_permille: 20,
            },
        }
    }

    /// Per-coil resistance count-slopes of 1706 milli and accepted rotating-
    /// field reactance evidence from the AC estimator.
    fn representative_data() -> ElectricalIdData {
        let mut data = ElectricalIdData::new(512);
        data.resistance_result = Some(resistance_result(1706, 1706));
        data.inductance = Some(inductance_evidence_fixture(8_600, 1_000_000));
        data.encoder_domain = Some(EncoderDomain::new(4000).unwrap());
        data
    }

    fn resistance_result_fixture() -> ResistanceRunResult {
        resistance_result(1706, 1706)
    }

    fn inductance_capture_state(
        previous_elapsed_us: u32,
        previous_phi: u16,
        accumulated_phi_delta_counts: u32,
        sample_count: u16,
    ) -> InductanceCaptureState {
        InductanceCaptureState {
            encoder_domain: EncoderDomain::new(4000).unwrap(),
            encoder_start: 0,
            start_us: 0,
            previous_elapsed_us,
            previous_phi,
            accumulated_phi_delta_counts,
            accumulator: crate::electrical_capture::AcFrameAccumulator::new(),
            first_half: crate::electrical_capture::AcFrameAccumulator::new(),
            second_half: crate::electrical_capture::AcFrameAccumulator::new(),
            saliency: crate::electrical_capture::AcSaliencyAccumulator::new(),
            sample_count,
            status_flags_or: 0,
            next_sample_us: 0,
            timing: CaptureTiming::new(INDUCTANCE_AC_SAMPLE_PERIOD_US, TimingEnvelope::INDUCTANCE),
        }
    }

    #[test]
    fn electrical_id_enters_rotating_inductance_after_resistance() {
        let mut data = ElectricalIdData::new(768);
        data.resistance_result = Some(resistance_result_fixture());

        let state = ElectricalIdState::InductanceSetup;

        assert!(data.resistance_result.is_some());
        assert!(matches!(state, ElectricalIdState::InductanceSetup));
    }

    #[test]
    fn model_assembly_uses_full_width_resistance_math() {
        let data = representative_data();

        let assembly = assemble_electrical_model(&data, TEST_THETA_E_US).unwrap();

        assert_eq!(assembly.model.r_int, 1706);
        assert_eq!(assembly.model.r_mohm, 1706);
        assert!(
            (795..=805).contains(&assembly.model.tau_e_us),
            "tau should use R=1.706, not truncated: {}",
            assembly.model.tau_e_us
        );
        // l_int = X / omega = 8600e9 / (6283 * 1_000_000) ≈ 1368.
        assert!(
            (1360..=1375).contains(&assembly.model.l_int),
            "l_int should preserve the internal L scale: {}",
            assembly.model.l_int
        );
    }

    #[test]
    fn model_assembly_uses_realized_frequency_rotating_inductance() {
        let mut data = representative_data();
        data.inductance = Some(inductance_evidence_fixture(8_600, 900_000));

        let assembly =
            assemble_electrical_model(&data, TEST_THETA_E_US).expect("model should assemble");

        let expected_l_int = l_int_from_reactance(8_600, 900_000).unwrap();
        assert_eq!(assembly.model.l_int, expected_l_int);
        assert_eq!(
            assembly.model.tau_e_us,
            (u64::from(expected_l_int) * u64::from(ELECTRICAL_MODEL_RESISTANCE_SCALE) / 1706)
                as u32
        );
    }

    #[test]
    fn model_assembly_uses_configured_theta() {
        let mut data = ElectricalIdData::new(512);
        // r_count_slope_milli = 1706 (V=512 / I=300 count-slope).
        data.resistance_result = Some(resistance_result(1706, 1706));
        data.inductance = Some(inductance_evidence_fixture(9_500, 1_000_000));

        let assembly = assemble_electrical_model(&data, 320).unwrap();

        assert_eq!(assembly.model.theta_e_us, 320);
    }

    #[test]
    fn validate_transitions_to_current_tune_with_model() {
        pollster::block_on(async {
            let mut phase = InnerPhase::ElectricalId {
                state: ElectricalIdState::Validate,
                data: representative_data(),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let result = step_electrical_identification(
                &mut phase,
                &mut buf,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::PhaseChanged(PhaseId::CurrentTune)
            ));
            let InnerPhase::CurrentTune {
                model,
                inner_warning_flags,
                resistance_result,
                inductance: _,
            } = phase
            else {
                panic!("expected CurrentTune, got {phase:?}");
            };
            assert_eq!(
                resistance_result.selected_r_count_slope_milli, 1706,
                "resistance result from Phase 5 should carry forward unchanged"
            );
            assert!(
                (795..=805).contains(&model.tau_e_us),
                "tau should be reported from the rotating-field L/R model: {}",
                model.tau_e_us
            );
            assert_eq!(inner_warning_flags & (1 << 2), 0);
            assert_eq!(diagnostics.iter().count(), 0);
        });
    }

    /// Pack a `MOTOR_TYPE_N_POLE_PAIRS` value reporting a two-phase stepper,
    /// matching the runner's `InvalidMotorType` entry check.
    fn two_phase_motor_type_n_pole_pairs() -> u32 {
        foci_tmc4671::registers::MotorTypeNPolePairs::new()
            .with_motor_type(foci_tmc4671::registers::MotorType::TwoPhase as u8)
            .with_n_pole_pairs(50)
            .into()
    }

    /// Seed the power-stage readback registers the shared resistance runner
    /// verifies against `backend.tmc_config()`'s profile (`PWM_MAXCNT`,
    /// `PWM_BBM_H_BBM_L`, `DSADC_MDEC_B_MDEC_A`), plus `MOTOR_TYPE_N_POLE_PAIRS`.
    /// `PWM_SV_CHOP` is written and read back by the runner itself, so it does
    /// not need pre-seeding.
    fn seed_resistance_power_stage_registers(backend: &mut MockBackend) {
        use foci_tmc4671::registers::{
            DSADC_MDEC_B_MDEC_A, MOTOR_TYPE_N_POLE_PAIRS, PWM_BBM_H_BBM_L, PWM_MAXCNT,
        };
        let config = backend.tmc_config().clone();
        backend
            .registers
            .insert(
                MOTOR_TYPE_N_POLE_PAIRS.addr(),
                two_phase_motor_type_n_pole_pairs(),
            )
            .ok();
        backend
            .registers
            .insert(PWM_MAXCNT.addr(), u32::from(config.pwm_maxcnt))
            .ok();
        backend
            .registers
            .insert(
                PWM_BBM_H_BBM_L.addr(),
                (u32::from(config.bbm_h) << 8) | u32::from(config.bbm_l),
            )
            .ok();
        backend
            .registers
            .insert(
                DSADC_MDEC_B_MDEC_A.addr(),
                (u32::from(config.dsadc_mdec_b) << 16) | u32::from(config.dsadc_mdec_a),
            )
            .ok();
    }

    fn register_operation_count(log: &[SpiOp]) -> usize {
        log.iter()
            .filter(|operation| matches!(operation, SpiOp::Read | SpiOp::Write(_, _)))
            .count()
    }

    fn write_position(log: &[SpiOp], register: Register, value: u32) -> usize {
        log.iter()
            .position(|op| {
                matches!(op, SpiOp::Write(addr, val) if *addr == register.addr() && *val == value)
            })
            .unwrap_or_else(|| {
                panic!(
                    "expected write of register {:#x} to {:#x}",
                    register.addr(),
                    value
                )
            })
    }

    fn enable_position(log: &[SpiOp], enabled: bool) -> usize {
        log.iter()
            .position(|op| matches!(op, SpiOp::EnablePin(state) if *state == enabled))
            .unwrap_or_else(|| panic!("expected set_enable_pin({enabled})"))
    }

    fn assert_ac_teardown_after_failure(backend: &MockBackend) {
        assert!(!backend.enable_pin, "AC failure must lower the enable pin");
        let enable_low = enable_position(&backend.spi_log, false);
        for (register, value) in [
            (UQ_UD_EXT, 0),
            (OPENLOOP_VELOCITY_TARGET, 0),
            (MODE_RAMP_MODE_MOTION, 0),
            (PHI_E_SELECTION, 0),
        ] {
            let position = write_position(&backend.spi_log, register, value);
            assert!(
                enable_low < position,
                "enable-low must precede register {:#x} teardown write",
                register.addr()
            );
        }
    }

    async fn finish_inductance_restore(
        phase: &mut InnerPhase,
        buf: &mut TransientBuffer,
        selftest_results: &mut crate::selftest::SelftestResults,
        diagnostics: &mut crate::selftest::CommissionDiagnostics,
        backend: &mut MockBackend,
    ) -> StepResult {
        for _ in 0..5 {
            let result = step_electrical_identification(
                phase,
                buf,
                InnerSequence::Commission,
                selftest_results,
                diagnostics,
                backend,
            )
            .await;
            if !matches!(result, StepResult::ReadyNow) {
                return result;
            }
        }
        panic!("inductance restoration did not terminate");
    }

    async fn drive_inductance_schedule(
        phase: &mut InnerPhase,
        buf: &mut TransientBuffer,
        selftest_results: &mut crate::selftest::SelftestResults,
        diagnostics: &mut crate::selftest::CommissionDiagnostics,
        backend: &mut MockBackend,
    ) -> StepResult {
        for _ in 0..512 {
            let result = step_electrical_identification(
                phase,
                buf,
                InnerSequence::Commission,
                selftest_results,
                diagnostics,
                backend,
            )
            .await;
            if matches!(
                phase,
                InnerPhase::ElectricalId {
                    state: ElectricalIdState::InductanceDisable { .. },
                    ..
                }
            ) {
                return StepResult::Continue;
            }
            match result {
                StepResult::ReadyNow => {}
                StepResult::At(deadline_us) => backend.elapsed_us_value = deadline_us,
                other => return other,
            }
        }
        panic!("inductance schedule did not terminate");
    }

    fn find_inductance_capture_rejection(
        diagnostics: &crate::selftest::CommissionDiagnostics,
    ) -> crate::selftest::CommissionDiagnostic {
        diagnostics
            .iter()
            .find(|event| event.phase == PhaseId::ElectricalId.to_u8() && event.code == 33)
            .expect("inductance capture rejection should retain branch detail")
    }

    #[test]
    fn saliency_bracket_rejection_records_violated_bound() {
        let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

        push_inductance_saliency_bracket_rejection(
            &mut diagnostics,
            InductanceEstimateEvidence {
                x_average_count_ratio_milli: 8_500,
                x_d_count_ratio_milli: 8_616,
                x_q_count_ratio_milli: 8_552,
                saliency_status: crate::electrical_capture::AC_SALIENCY_DETERMINABLE,
                saliency_permille: 7,
                x_mag_nominal_count_ratio_milli: 0,
                x_mag_shift_minus_permille: 0,
                x_mag_shift_plus_permille: 0,
                x_mag_vs_quad_permille: 0,
            },
        );
        push_inductance_saliency_bracket_rejection(
            &mut diagnostics,
            InductanceEstimateEvidence {
                x_average_count_ratio_milli: 8_700,
                x_d_count_ratio_milli: 8_616,
                x_q_count_ratio_milli: 8_552,
                saliency_status: crate::electrical_capture::AC_SALIENCY_DETERMINABLE,
                saliency_permille: 7,
                x_mag_nominal_count_ratio_milli: 0,
                x_mag_shift_minus_permille: 0,
                x_mag_shift_plus_permille: 0,
                x_mag_vs_quad_permille: 0,
            },
        );

        let mut details = diagnostics.iter();
        let below = details
            .next()
            .expect("below-bound saliency diagnostic missing");
        assert_eq!(below.value0, 9);
        assert_eq!(below.value1, 7);
        assert_eq!(below.value2, (8_500u32 << 16) | 8_552);

        let above = details
            .next()
            .expect("above-bound saliency diagnostic missing");
        assert_eq!(above.value0, 10);
        assert_eq!(above.value1, 7);
        assert_eq!(above.value2, (8_700u32 << 16) | 8_616);
    }

    #[test]
    fn saliency_bracket_small_miss_marks_split_not_determinable() {
        let below = saliency_for_frame_average(
            8_659,
            crate::electrical_capture::AcSaliencyEvidence {
                status: crate::electrical_capture::AC_SALIENCY_DETERMINABLE,
                x_average_count_ratio_milli: 8_834,
                x_d_count_ratio_milli: 8_936,
                x_q_count_ratio_milli: 8_732,
                saliency_permille: 23,
            },
        )
        .expect("small below-bracket miss should keep average L");
        let above = saliency_for_frame_average(
            8_735,
            crate::electrical_capture::AcSaliencyEvidence {
                status: crate::electrical_capture::AC_SALIENCY_DETERMINABLE,
                x_average_count_ratio_milli: 8_586,
                x_d_count_ratio_milli: 8_629,
                x_q_count_ratio_milli: 8_543,
                saliency_permille: 10,
            },
        )
        .expect("small above-bracket miss should keep average L");
        let p95_tail = saliency_for_frame_average(
            8_832,
            crate::electrical_capture::AcSaliencyEvidence {
                status: crate::electrical_capture::AC_SALIENCY_DETERMINABLE,
                x_average_count_ratio_milli: 8_586,
                x_d_count_ratio_milli: 8_629,
                x_q_count_ratio_milli: 8_543,
                saliency_permille: 10,
            },
        )
        .expect("p95-scale benign mismatch should keep average L");

        assert_eq!(
            below,
            crate::electrical_capture::AcSaliencyEvidence::not_determinable(8_659)
        );
        assert_eq!(
            above,
            crate::electrical_capture::AcSaliencyEvidence::not_determinable(8_735)
        );
        assert_eq!(
            p95_tail,
            crate::electrical_capture::AcSaliencyEvidence::not_determinable(8_832)
        );
    }

    #[test]
    fn saliency_bracket_large_miss_still_rejects() {
        let result = saliency_for_frame_average(
            9_200,
            crate::electrical_capture::AcSaliencyEvidence {
                status: crate::electrical_capture::AC_SALIENCY_DETERMINABLE,
                x_average_count_ratio_milli: 8_586,
                x_d_count_ratio_milli: 8_629,
                x_q_count_ratio_milli: 8_543,
                saliency_permille: 10,
            },
        );

        assert_eq!(result, Err(InductanceReject::Saliency));
    }

    #[test]
    fn saliency_bracketed_average_preserves_split_evidence() {
        let saliency = crate::electrical_capture::AcSaliencyEvidence {
            status: crate::electrical_capture::AC_SALIENCY_DETERMINABLE,
            x_average_count_ratio_milli: 8_600,
            x_d_count_ratio_milli: 8_900,
            x_q_count_ratio_milli: 8_300,
            saliency_permille: 70,
        };

        let result =
            saliency_for_frame_average(8_620, saliency).expect("bracketed average should pass");

        assert_eq!(result, saliency);
    }

    #[test]
    fn inductance_setup_bounds_voltage_before_enable_and_starts_hardware_ramp() {
        pollster::block_on(async {
            let mut data = ElectricalIdData::new(1024);
            data.encoder_domain = Some(EncoderDomain::new(4000).unwrap());
            let mut phase = InnerPhase::ElectricalId {
                state: ElectricalIdState::InductanceSetup,
                data,
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let result = step_electrical_identification(
                &mut phase,
                &mut buf,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(
                matches!(result, StepResult::ReadyNow),
                "scheduler gap must not make the AC window fail: {result:?}"
            );
            for _ in 0..10 {
                let result = step_electrical_identification(
                    &mut phase,
                    &mut buf,
                    InnerSequence::Commission,
                    &mut selftest_results,
                    &mut diagnostics,
                    &mut backend,
                )
                .await;
                assert!(matches!(result, StepResult::ReadyNow));
            }
            assert!(backend.enable_pin);
            assert!(matches!(
                phase,
                InnerPhase::ElectricalId {
                    state: ElectricalIdState::InductanceWaitVelocity { .. },
                    ..
                }
            ));

            let enable_high = enable_position(&backend.spi_log, true);
            for (register, value) in [
                (PIDOUT_UQ_UD_LIMITS, INDUCTANCE_AC_UD_EXT as u32),
                (UQ_UD_EXT, 0),
                (OPENLOOP_VELOCITY_TARGET, 0),
                (
                    OPENLOOP_VELOCITY_TARGET,
                    INDUCTANCE_OPENLOOP_VELOCITY_TARGET_RPM,
                ),
                (PHI_E_SELECTION, PHI_E_SELECTION_OPENLOOP),
                (PWM_SV_CHOP, 7),
                (STATUS_FLAGS, 0),
            ] {
                let position = write_position(&backend.spi_log, register, value);
                assert!(
                    position < enable_high,
                    "register {:#x} setup write must precede enable-high",
                    register.addr()
                );
            }
        });
    }

    #[test]
    fn inductance_velocity_timeout_tears_down_openloop_drive() {
        pollster::block_on(async {
            let mut phase = InnerPhase::ElectricalId {
                state: ElectricalIdState::InductanceWaitVelocity { deadline_us: 10 },
                data: representative_data(),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();
            backend.enable_pin = true;
            backend.elapsed_us_value = 20;
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let result = step_electrical_identification(
                &mut phase,
                &mut buf,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::InductanceVelocityTimeout)
            ));
            assert_ac_teardown_after_failure(&backend);
        });
    }

    #[test]
    fn inductance_sample_spi_failure_tears_down_openloop_drive() {
        pollster::block_on(async {
            let mut phase = InnerPhase::ElectricalId {
                state: ElectricalIdState::InductanceSampleLegacy {
                    encoder_start: 0,
                    start_us: 0,
                    previous_elapsed_us: 0,
                    start_phi: 0,
                    previous_phi: 0,
                    accumulated_phi_delta_counts: 0,
                    accumulator: crate::electrical_capture::AcFrameAccumulator::new(),
                    first_half: crate::electrical_capture::AcFrameAccumulator::new(),
                    second_half: crate::electrical_capture::AcFrameAccumulator::new(),
                    saliency: crate::electrical_capture::AcSaliencyAccumulator::new(),
                    sample_count: 0,
                    status_flags_or: 0,
                },
                data: representative_data(),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();
            backend.enable_pin = true;
            backend.fail_on_read = Some(PID_TORQUE_FLUX_ACTUAL.addr());
            backend.registers.insert(OPENLOOP_PHI.addr(), 1).ok();
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let result = step_electrical_identification(
                &mut phase,
                &mut buf,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::Spi)
            ));
            assert_ac_teardown_after_failure(&backend);
        });
    }

    #[test]
    fn inductance_capture_rejection_records_zero_phi_delta_detail() {
        pollster::block_on(async {
            let mut phase = InnerPhase::ElectricalId {
                state: ElectricalIdState::InductanceSampleLegacy {
                    encoder_start: 0,
                    start_us: 0,
                    previous_elapsed_us: 0,
                    start_phi: 0,
                    previous_phi: 0,
                    accumulated_phi_delta_counts: 0,
                    accumulator: crate::electrical_capture::AcFrameAccumulator::new(),
                    first_half: crate::electrical_capture::AcFrameAccumulator::new(),
                    second_half: crate::electrical_capture::AcFrameAccumulator::new(),
                    saliency: crate::electrical_capture::AcSaliencyAccumulator::new(),
                    sample_count: 0,
                    status_flags_or: 0,
                },
                data: representative_data(),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();
            backend.enable_pin = true;
            backend.set_openloop_phi_sequence(&[1_000, 1_000], 80);
            backend.set_pid_torque_flux_actual_sequence(&[((-84i16 as u16 as u32) << 16) | 20]);
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let result = step_electrical_identification(
                &mut phase,
                &mut buf,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::InductanceCaptureRejected)
            ));
            let detail = find_inductance_capture_rejection(&diagnostics);
            assert_eq!(detail.status, 1);
            assert_eq!(detail.value0, 2);
            assert_eq!(detail.value1, 0);
            assert_eq!(detail.value2, (1_000u32 << 16) | 1_000u32);
            assert_ac_teardown_after_failure(&backend);
        });
    }

    #[test]
    fn inductance_capture_rejection_records_saliency_sample_detail() {
        pollster::block_on(async {
            let mut phase = InnerPhase::ElectricalId {
                state: ElectricalIdState::InductanceSampleLegacy {
                    encoder_start: 0,
                    start_us: 0,
                    previous_elapsed_us: 0,
                    start_phi: 0,
                    previous_phi: 0,
                    accumulated_phi_delta_counts: 0,
                    accumulator: crate::electrical_capture::AcFrameAccumulator::new(),
                    first_half: crate::electrical_capture::AcFrameAccumulator::new(),
                    second_half: crate::electrical_capture::AcFrameAccumulator::new(),
                    saliency: crate::electrical_capture::AcSaliencyAccumulator::new(),
                    sample_count: 0,
                    status_flags_or: 0,
                },
                data: representative_data(),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();
            backend.enable_pin = true;
            backend.set_openloop_phi_sequence(&[0, 5_243], 80);
            backend.set_pid_torque_flux_actual_sequence(&[0]);
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let result = step_electrical_identification(
                &mut phase,
                &mut buf,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::InductanceCaptureRejected)
            ));
            let detail = find_inductance_capture_rejection(&diagnostics);
            assert_eq!(detail.status, 1);
            assert_eq!(detail.value0, 4);
            assert_eq!(detail.value1, 0);
            assert_eq!(detail.value2, 0);
            assert_ac_teardown_after_failure(&backend);
        });
    }

    #[test]
    fn inductance_capture_rejection_records_elapsed_interpolation_detail() {
        pollster::block_on(async {
            let mut phase = InnerPhase::ElectricalId {
                state: ElectricalIdState::InductanceSampleLegacy {
                    encoder_start: 0,
                    start_us: 0,
                    previous_elapsed_us: 9_000,
                    start_phi: 0,
                    previous_phi: 0,
                    accumulated_phi_delta_counts: INDUCTANCE_AC_TARGET_PHI_DELTA_COUNTS - 1_000,
                    accumulator: crate::electrical_capture::AcFrameAccumulator::new(),
                    first_half: crate::electrical_capture::AcFrameAccumulator::new(),
                    second_half: crate::electrical_capture::AcFrameAccumulator::new(),
                    saliency: crate::electrical_capture::AcSaliencyAccumulator::new(),
                    sample_count: INDUCTANCE_MIN_AC_SAMPLE_COUNT,
                    status_flags_or: 0,
                },
                data: representative_data(),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();
            backend.enable_pin = true;
            backend.set_openloop_phi_sequence(&[1_000], 80);
            backend.set_pid_torque_flux_actual_sequence(&[((-84i16 as u16 as u32) << 16) | 20]);
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let result = step_electrical_identification(
                &mut phase,
                &mut buf,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::InductanceCaptureRejected)
            ));
            let detail = find_inductance_capture_rejection(&diagnostics);
            assert_eq!(detail.status, 1);
            assert_eq!(detail.value0, 7);
            assert_eq!(detail.value1, u32::from(INDUCTANCE_MIN_AC_SAMPLE_COUNT + 1));
            assert_eq!(detail.value2, (9_000u32 << 16) | 80u32);
            assert_ac_teardown_after_failure(&backend);
        });
    }

    #[test]
    fn inductance_sample_completes_phi_window_in_one_bounded_step() {
        pollster::block_on(async {
            let mut phase = InnerPhase::ElectricalId {
                state: ElectricalIdState::InductanceSampleLegacy {
                    encoder_start: 0,
                    start_us: 0,
                    previous_elapsed_us: 0,
                    start_phi: 0,
                    previous_phi: 0,
                    accumulated_phi_delta_counts: 0,
                    accumulator: crate::electrical_capture::AcFrameAccumulator::new(),
                    first_half: crate::electrical_capture::AcFrameAccumulator::new(),
                    second_half: crate::electrical_capture::AcFrameAccumulator::new(),
                    saliency: crate::electrical_capture::AcSaliencyAccumulator::new(),
                    sample_count: 0,
                    status_flags_or: 0,
                },
                data: representative_data(),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();
            backend.enable_pin = true;
            backend
                .registers
                .insert(foci_tmc4671::registers::STATUS_FLAGS.addr(), 0)
                .ok();

            let mut phi_samples = heapless::Vec::<u32, 128>::new();
            let mut current_samples = heapless::Vec::<u32, 128>::new();
            let actual = ((-84i16 as u16 as u32) << 16) | 20u32;
            for index in 1..=100u32 {
                phi_samples.push(index.wrapping_mul(5_243) & 0xffff).ok();
                current_samples.push(actual).ok();
            }
            backend.set_openloop_phi_sequence(phi_samples.as_slice(), 80);
            backend.set_pid_torque_flux_actual_sequence(current_samples.as_slice());

            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let result = step_electrical_identification(
                &mut phase,
                &mut buf,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(matches!(result, StepResult::Continue));
            let InnerPhase::ElectricalId {
                state: ElectricalIdState::InductanceDisable { .. },
                data,
            } = phase
            else {
                panic!("expected completed inductance capture, got {phase:?}");
            };
            let evidence = data
                .inductance
                .expect("inductance evidence should be retained");
            assert!(evidence.run.sample_count >= INDUCTANCE_MIN_AC_SAMPLE_COUNT);
            assert_eq!(
                evidence.run.openloop_phi_delta_counts,
                INDUCTANCE_AC_TARGET_PHI_DELTA_COUNTS
            );
        });
    }

    #[test]
    fn inductance_sample_excludes_scheduler_gap_before_first_phi_sample() {
        pollster::block_on(async {
            let mut phase = InnerPhase::ElectricalId {
                state: ElectricalIdState::InductanceSampleLegacy {
                    encoder_start: 0,
                    start_us: 0,
                    previous_elapsed_us: 0,
                    start_phi: 0,
                    previous_phi: 0,
                    accumulated_phi_delta_counts: 0,
                    accumulator: crate::electrical_capture::AcFrameAccumulator::new(),
                    first_half: crate::electrical_capture::AcFrameAccumulator::new(),
                    second_half: crate::electrical_capture::AcFrameAccumulator::new(),
                    saliency: crate::electrical_capture::AcSaliencyAccumulator::new(),
                    sample_count: 0,
                    status_flags_or: 0,
                },
                data: representative_data(),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();
            backend.enable_pin = true;
            backend.elapsed_us_value = 1_046;
            backend
                .registers
                .insert(foci_tmc4671::registers::STATUS_FLAGS.addr(), 0)
                .ok();

            let mut phi_samples = heapless::Vec::<u32, 128>::new();
            let mut current_samples = heapless::Vec::<u32, 128>::new();
            let actual = ((-84i16 as u16 as u32) << 16) | 20u32;
            for index in 0..=100u32 {
                phi_samples.push(index.wrapping_mul(5_243) & 0xffff).ok();
                current_samples.push(actual).ok();
            }
            backend.set_openloop_phi_sequence(phi_samples.as_slice(), 80);
            backend.set_pid_torque_flux_actual_sequence(current_samples.as_slice());

            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let result = step_electrical_identification(
                &mut phase,
                &mut buf,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(matches!(result, StepResult::Continue));
            let InnerPhase::ElectricalId {
                state: ElectricalIdState::InductanceDisable { .. },
                data,
            } = phase
            else {
                panic!("expected completed inductance capture, got {phase:?}");
            };
            let evidence = data
                .inductance
                .expect("inductance evidence should be retained");
            assert!(
                (7_900..=8_100).contains(&evidence.run.elapsed_us),
                "scheduler gap must not be charged to AC elapsed_us: {}",
                evidence.run.elapsed_us
            );
            assert!(realized_frequency_in_range(
                evidence.run.realized_frequency_millihz
            ));
        });
    }

    #[test]
    fn inductance_frequency_rejection_records_measured_window_detail() {
        pollster::block_on(async {
            let mut phase = InnerPhase::ElectricalId {
                state: ElectricalIdState::InductanceSampleLegacy {
                    encoder_start: 0,
                    start_us: 0,
                    previous_elapsed_us: 0,
                    start_phi: 0,
                    previous_phi: 0,
                    accumulated_phi_delta_counts: 0,
                    accumulator: crate::electrical_capture::AcFrameAccumulator::new(),
                    first_half: crate::electrical_capture::AcFrameAccumulator::new(),
                    second_half: crate::electrical_capture::AcFrameAccumulator::new(),
                    saliency: crate::electrical_capture::AcSaliencyAccumulator::new(),
                    sample_count: 0,
                    status_flags_or: 0,
                },
                data: representative_data(),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();
            backend.enable_pin = true;
            backend
                .registers
                .insert(foci_tmc4671::registers::STATUS_FLAGS.addr(), 0)
                .ok();

            let mut phi_samples = heapless::Vec::<u32, 128>::new();
            let mut current_samples = heapless::Vec::<u32, 128>::new();
            let actual = ((-84i16 as u16 as u32) << 16) | 20u32;
            for index in 1..=100u32 {
                phi_samples.push(index.wrapping_mul(5_243) & 0xffff).ok();
                current_samples.push(actual).ok();
            }
            backend.set_openloop_phi_sequence(phi_samples.as_slice(), 40);
            backend.set_pid_torque_flux_actual_sequence(current_samples.as_slice());

            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let result = step_electrical_identification(
                &mut phase,
                &mut buf,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::InductanceFrequencyOutOfRange)
            ));
            let detail = diagnostics
                .iter()
                .find(|event| event.code == DIAG_ELECTRICAL_ID_REJECT_INDUCTANCE_FREQUENCY)
                .expect("frequency rejection should retain realized-frequency evidence");
            assert_eq!(detail.phase, PhaseId::ElectricalId.to_u8());
            assert_eq!(detail.status, 1);
            assert!(detail.value0 > INDUCTANCE_REALIZED_FREQUENCY_TARGET_MILLIHZ);
            assert!(detail.value1 < 5_000);
            assert!(detail.value2 >= u32::from(INDUCTANCE_MIN_AC_SAMPLE_COUNT));
            assert_ac_teardown_after_failure(&backend);
        });
    }

    #[test]
    fn inductance_finish_accepts_benign_status_flags_as_warning() {
        let mut accumulator = crate::electrical_capture::AcFrameAccumulator::new();
        let mut first_half = crate::electrical_capture::AcFrameAccumulator::new();
        let mut second_half = crate::electrical_capture::AcFrameAccumulator::new();
        let mut saliency = crate::electrical_capture::AcSaliencyAccumulator::new();

        for index in 0..100u32 {
            let weight = if index == 99 { 5_276 } else { 5_243 };
            let phi = ((index * INDUCTANCE_AC_TARGET_PHI_DELTA_COUNTS / 100) & 0xffff) as u16;
            accumulator.push_weighted(20, -84, weight).unwrap();
            saliency
                .push_weighted_sample(phi, weight, INDUCTANCE_AC_UD_EXT, 20, -84)
                .unwrap();
            if index < 50 {
                first_half.push_weighted(20, -84, weight).unwrap();
            } else {
                second_half.push_weighted(20, -84, weight).unwrap();
            }
        }

        let evidence = finish_inductance_evidence(InductanceCaptureFinish {
            data: &representative_data(),
            encoder_domain: EncoderDomain::new(4000).unwrap(),
            encoder_start: 10,
            encoder_after: 10,
            elapsed_us: 8_000,
            phi_delta_counts: INDUCTANCE_AC_TARGET_PHI_DELTA_COUNTS,
            sample_count: 100,
            status_flags_or: 0x7000_4400,
            accumulator,
            first_half,
            second_half,
            saliency_accumulator: saliency,
        })
        .expect("non-actionable hardware status flags should not reject inductance evidence");

        assert_eq!(evidence.run.status_flags_or, 0x7000_4400);
        assert_ne!(evidence.run.warning_flags & INDUCTANCE_WARN_STATUS_FLAGS, 0);
    }

    #[test]
    fn inductance_finish_reports_encoder_motion_rejection() {
        let mut accumulator = crate::electrical_capture::AcFrameAccumulator::new();
        let mut first_half = crate::electrical_capture::AcFrameAccumulator::new();
        let mut second_half = crate::electrical_capture::AcFrameAccumulator::new();
        let mut saliency = crate::electrical_capture::AcSaliencyAccumulator::new();

        for index in 0..100u32 {
            let weight = if index == 99 { 5_276 } else { 5_243 };
            let phi = ((index * INDUCTANCE_AC_TARGET_PHI_DELTA_COUNTS / 100) & 0xffff) as u16;
            accumulator.push_weighted(20, -84, weight).unwrap();
            saliency
                .push_weighted_sample(phi, weight, INDUCTANCE_AC_UD_EXT, 20, -84)
                .unwrap();
            if index < 50 {
                first_half.push_weighted(20, -84, weight).unwrap();
            } else {
                second_half.push_weighted(20, -84, weight).unwrap();
            }
        }

        let rejection = match finish_inductance_evidence(InductanceCaptureFinish {
            data: &representative_data(),
            encoder_domain: EncoderDomain::new(4000).unwrap(),
            encoder_start: 10,
            encoder_after: 10 + INDUCTANCE_MAX_ENCODER_DELTA_COUNTS + 1,
            elapsed_us: 8_000,
            phi_delta_counts: INDUCTANCE_AC_TARGET_PHI_DELTA_COUNTS,
            sample_count: 100,
            status_flags_or: 0,
            accumulator,
            first_half,
            second_half,
            saliency_accumulator: saliency,
        }) {
            Ok(_) => panic!("encoder motion must reject the inductance capture"),
            Err(rejection) => rejection,
        };

        assert_eq!(rejection, InductanceReject::EncoderMotion);
    }

    #[test]
    fn inductance_encoder_delta_crosses_origin_forward() {
        let domain = EncoderDomain::new(4000).unwrap();

        assert_eq!(
            inductance_encoder_delta(domain, 3998, 2)
                .unwrap()
                .signed_counts(),
            4
        );
    }

    #[test]
    fn inductance_encoder_delta_maps_half_turn_to_motion_rejection() {
        let domain = EncoderDomain::new(4000).unwrap();

        assert_eq!(
            inductance_encoder_delta(domain, 0, 2000),
            Err(InductanceReject::EncoderMotion)
        );
        assert_eq!(
            InductanceReject::EncoderMotion.commission_error(),
            CommissionError::InductanceEncoderMotion
        );
    }

    #[test]
    fn inductance_setup_rejects_missing_validated_domain_before_excitation() {
        pollster::block_on(async {
            let mut phase = InnerPhase::ElectricalId {
                state: ElectricalIdState::InductanceSetup,
                data: ElectricalIdData::new(1024),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();
            backend.mechanical_encoder_quadrature_counts_per_rev_value = 0;
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let result = step_electrical_identification(
                &mut phase,
                &mut buf,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::EncoderFault)
            ));
            assert!(!backend.enable_pin);
            assert!(backend.spi_log.is_empty());
        });
    }

    #[test]
    fn resistance_rejects_invalid_domain_before_identification() {
        pollster::block_on(async {
            let mut phase = InnerPhase::ElectricalId {
                state: ElectricalIdState::ResistanceStart,
                data: ElectricalIdData::new(1024),
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();
            backend.mechanical_encoder_quadrature_counts_per_rev_value = 0;
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let result = step_electrical_identification(
                &mut phase,
                &mut buf,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::EncoderFault)
            ));
            assert!(matches!(
                phase,
                InnerPhase::Failed(CommissionError::EncoderFault)
            ));
            assert!(!backend.enable_pin);
            assert!(
                backend.spi_log.is_empty(),
                "invalid encoder configuration must not start resistance excitation"
            );
        });
    }

    #[test]
    fn inductance_half_turn_path_tears_down_without_committing_evidence() {
        pollster::block_on(async {
            let mut data = ElectricalIdData::new(512);
            data.resistance_result = Some(resistance_result_fixture());
            data.encoder_domain = Some(EncoderDomain::new(4000).unwrap());
            let mut phase = InnerPhase::ElectricalId {
                state: ElectricalIdState::InductanceSampleLegacy {
                    encoder_start: 0,
                    start_us: 0,
                    previous_elapsed_us: 0,
                    start_phi: 0,
                    previous_phi: 0,
                    accumulated_phi_delta_counts: 0,
                    accumulator: crate::electrical_capture::AcFrameAccumulator::new(),
                    first_half: crate::electrical_capture::AcFrameAccumulator::new(),
                    second_half: crate::electrical_capture::AcFrameAccumulator::new(),
                    saliency: crate::electrical_capture::AcSaliencyAccumulator::new(),
                    sample_count: 0,
                    status_flags_or: 0,
                },
                data,
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();
            backend.enable_pin = true;
            let _ = backend.registers.insert(STATUS_FLAGS.addr(), 0);
            let _ = backend
                .registers
                .insert(foci_tmc4671::registers::ABN_DECODER_COUNT.addr(), 2000);
            let mut phi_samples = heapless::Vec::<u32, 128>::new();
            let mut current_samples = heapless::Vec::<u32, 128>::new();
            let actual = ((-84i16 as u16 as u32) << 16) | 20u32;
            for index in 1..=100u32 {
                phi_samples.push(index.wrapping_mul(5_243) & 0xffff).ok();
                current_samples.push(actual).ok();
            }
            backend.set_openloop_phi_sequence(phi_samples.as_slice(), 80);
            backend.set_pid_torque_flux_actual_sequence(current_samples.as_slice());
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let result = step_electrical_identification(
                &mut phase,
                &mut buf,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::InductanceEncoderMotion)
            ));
            assert!(matches!(
                phase,
                InnerPhase::Failed(CommissionError::InductanceEncoderMotion)
            ));
            assert_ac_teardown_after_failure(&backend);
        });
    }

    #[test]
    fn inductance_low_quadrature_rejection_maps_to_specific_error() {
        assert_eq!(
            InductanceReject::LowQuadratureSignal.commission_error(),
            CommissionError::InductanceLowQuadratureSignal
        );
    }

    #[test]
    fn resistance_runner_is_polled_with_bounded_spi_work() {
        pollster::block_on(async {
            let mut backend = MockBackend::new();
            seed_resistance_power_stage_registers(&mut backend);
            backend.script_resistance_current_proportional_to_ud = true;

            let mut phase = InnerPhase::ElectricalId {
                state: ElectricalIdState::ResistanceStart,
                data: ElectricalIdData::new(u32::from(backend.tmc_config().excitation_voltage)),
            };
            let mut buf = TransientBuffer::default();
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let mut reached_inductance = false;
            for _ in 0..2_000 {
                let before = register_operation_count(&backend.spi_log);
                let step = step_electrical_identification(
                    &mut phase,
                    &mut buf,
                    InnerSequence::Commission,
                    &mut selftest_results,
                    &mut diagnostics,
                    &mut backend,
                )
                .await;
                let operations = register_operation_count(&backend.spi_log) - before;
                assert!(
                    operations <= 2,
                    "resistance poll performed {operations} SPI ops"
                );

                if matches!(
                    phase,
                    InnerPhase::ElectricalId {
                        state: ElectricalIdState::InductanceSetup,
                        ..
                    }
                ) {
                    assert!(matches!(step, StepResult::ReadyNow));
                    reached_inductance = true;
                    break;
                }
                match step {
                    StepResult::ReadyNow => {}
                    StepResult::At(deadline_us) => backend.elapsed_us_value = deadline_us,
                    other => panic!("unexpected resistance poll result: {other:?}"),
                }
            }
            assert!(reached_inductance, "resistance runner did not complete");
            let InnerPhase::ElectricalId { data, .. } = &phase else {
                panic!("expected ElectricalId, got {phase:?}");
            };
            assert!(
                data.resistance_result.is_some(),
                "resistance result should be stored on success"
            );
        });
    }

    #[test]
    fn resistance_critical_status_restores_before_terminal_failure() {
        pollster::block_on(async {
            let mut backend = MockBackend::new();
            seed_resistance_power_stage_registers(&mut backend);
            backend.script_resistance_current_proportional_to_ud = true;

            let mut phase = InnerPhase::ElectricalId {
                state: ElectricalIdState::ResistanceStart,
                data: ElectricalIdData::new(u32::from(backend.tmc_config().excitation_voltage)),
            };
            let mut buf = TransientBuffer::default();
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();
            let critical_status = 1 << 1;
            let mut injected = false;
            let mut containment_seen = false;
            let mut restore_polls = 0;

            for _ in 0..200 {
                let before = register_operation_count(&backend.spi_log);
                let step = step_electrical_identification(
                    &mut phase,
                    &mut buf,
                    InnerSequence::Commission,
                    &mut selftest_results,
                    &mut diagnostics,
                    &mut backend,
                )
                .await;
                let operations = register_operation_count(&backend.spi_log) - before;
                assert!(
                    operations <= 2,
                    "resistance poll performed {operations} SPI ops"
                );

                if injected && !backend.enable_pin && !containment_seen {
                    assert!(matches!(step, StepResult::ReadyNow));
                    let InnerPhase::ElectricalId {
                        state: ElectricalIdState::ResistanceIdentify { runner },
                        ..
                    } = &phase
                    else {
                        panic!("critical status failed before runner restoration: {phase:?}");
                    };
                    assert_eq!(runner.status_flags_or() & critical_status, critical_status);
                    assert_eq!(
                        runner.primary_error(),
                        Some(crate::resistance_identification::ResistanceError::CriticalStatus)
                    );
                    containment_seen = true;
                    continue;
                }

                if containment_seen && matches!(step, StepResult::ReadyNow) {
                    restore_polls += 1;
                }
                if let StepResult::InnerFailed(error) = step {
                    assert_eq!(error, CommissionError::ResistanceIdFailed);
                    assert!(containment_seen);
                    assert!(restore_polls > 0, "failure bypassed resumable restoration");
                    assert!(!backend.enable_pin);
                    return;
                }
                if let StepResult::At(deadline_us) = step {
                    backend.elapsed_us_value = deadline_us;
                }
                if backend.enable_pin && !injected {
                    backend.read_override = Some((STATUS_FLAGS.addr(), critical_status));
                    injected = true;
                }
            }
            panic!("critical resistance status did not reach terminal failure");
        });
    }

    #[test]
    fn resistance_warning_status_is_retained_without_terminal_failure() {
        pollster::block_on(async {
            let mut backend = MockBackend::new();
            seed_resistance_power_stage_registers(&mut backend);
            backend.script_resistance_current_proportional_to_ud = true;

            let mut phase = InnerPhase::ElectricalId {
                state: ElectricalIdState::ResistanceStart,
                data: ElectricalIdData::new(u32::from(backend.tmc_config().excitation_voltage)),
            };
            let mut buf = TransientBuffer::default();
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();
            let warning_status = 1 << 31;
            let mut injected = false;

            for _ in 0..2_000 {
                let before = register_operation_count(&backend.spi_log);
                let step = step_electrical_identification(
                    &mut phase,
                    &mut buf,
                    InnerSequence::Commission,
                    &mut selftest_results,
                    &mut diagnostics,
                    &mut backend,
                )
                .await;
                let operations = register_operation_count(&backend.spi_log) - before;
                assert!(
                    operations <= 2,
                    "resistance poll performed {operations} SPI ops"
                );

                if let InnerPhase::ElectricalId {
                    state: ElectricalIdState::InductanceSetup,
                    data,
                } = &phase
                {
                    assert!(matches!(step, StepResult::ReadyNow));
                    let result = data.resistance_result.expect("resistance result");
                    assert_eq!(result.status_flags_or & warning_status, warning_status);
                    assert!(
                        result
                            .warning_flags
                            .contains(ResistanceWarningFlags::STATUS_FLAGS)
                    );
                    return;
                }
                match step {
                    StepResult::ReadyNow => {}
                    StepResult::At(deadline_us) => backend.elapsed_us_value = deadline_us,
                    other => panic!("warning-only status became terminal: {other:?}"),
                }
                if backend.enable_pin && !injected {
                    backend.read_override = Some((STATUS_FLAGS.addr(), warning_status));
                    injected = true;
                }
            }
            panic!("warning-only resistance run did not complete");
        });
    }

    #[test]
    fn resistance_reports_specific_quality_gate_failure_after_restoration() {
        pollster::block_on(async {
            let mut backend = MockBackend::new();
            seed_resistance_power_stage_registers(&mut backend);
            // No current scripting: ADC_IWY_IUX always reads back 0, so no
            // measured point can support the fit or the signed anchor. The
            // adaptive runner reports this as an unsupported measurement
            // envelope rather than applying unsafe higher excitation.

            let mut phase = InnerPhase::ElectricalId {
                state: ElectricalIdState::ResistanceStart,
                data: ElectricalIdData::new(u32::from(backend.tmc_config().excitation_voltage)),
            };
            let mut buf = TransientBuffer::default();
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let expected_status =
                crate::resistance_identification::ResistanceError::MeasurementEnvelopeUnsupported
                    .status_code();
            for _ in 0..2_000 {
                let step = step_electrical_identification(
                    &mut phase,
                    &mut buf,
                    InnerSequence::Commission,
                    &mut selftest_results,
                    &mut diagnostics,
                    &mut backend,
                )
                .await;
                match step {
                    StepResult::ReadyNow => {}
                    StepResult::At(deadline_us) => backend.elapsed_us_value = deadline_us,
                    StepResult::InnerFailed(error) => {
                        assert_eq!(error.status_code(), expected_status);
                        assert_ne!(
                            error.status_code(),
                            CommissionError::ResistanceIdFailed.status_code()
                        );
                        return;
                    }
                    other => panic!("unexpected resistance poll result: {other:?}"),
                }
            }
            panic!("resistance quality gate did not fail");
        });
    }

    // -----------------------------------------------------------------
    // Phase 1 model-assembly gates (inductance / delay)
    // -----------------------------------------------------------------

    /// Build model data for per-coil resistance currents `c0`/`c1` (counts).
    ///
    /// The per-axis count-slopes are `512000/c` (V=512 in count-slope-milli).
    fn data_with_per_coil_currents(c0: i32, c1: i32) -> ElectricalIdData {
        let slope0 = (512_000 / c0.max(1)).max(1);
        let slope1 = (512_000 / c1.max(1)).max(1);
        let mut data = ElectricalIdData::new(512);
        data.resistance_result = Some(resistance_result(slope0, slope1));
        data.inductance = Some(inductance_evidence_fixture(8_600, 1_000_000));
        data
    }

    #[test]
    fn model_failure_diagnostic_codes_are_stable() {
        let no_samples = diagnostic_for_model_assembly_failure(ModelAssemblyError::SampleCount);
        assert_eq!(no_samples.phase, PhaseId::ElectricalId.to_u8());
        assert_eq!(no_samples.code, 20);
        assert_eq!(no_samples.status, 1);

        let scale_zero =
            diagnostic_for_model_assembly_failure(ModelAssemblyError::ModelScaleZero {
                l_int: 0,
                l_uh: 0,
            });
        assert_eq!(scale_zero.code, 22);

        let theta = diagnostic_for_model_assembly_failure(ModelAssemblyError::ThetaTooLarge {
            theta_us: 320,
            tau_us: 600,
        });
        assert_eq!(theta.code, 29);
        assert_eq!(theta.value0, 320);
        assert_eq!(theta.value1, 600);
    }

    #[test]
    fn permille_diff_uses_average_denominator_for_coil_disagreement() {
        // |130 - 100| / ((130 + 100) / 2) = 30 / 115 = 260 permille.
        assert_eq!(permille_diff(100, 130), 260);
    }

    #[test]
    fn assemble_accepts_representative_data() {
        let data = representative_data();
        let assembly = assemble_electrical_model(&data, TEST_THETA_E_US).unwrap();
        assert_eq!(
            assembly.warning_flags & INNER_WARN_BIT_RESISTANCE_MISMATCH,
            0
        );
        assert_eq!(assembly.warning_flags & INNER_WARN_BIT_THETA_TAU, 0);
    }

    #[test]
    fn assemble_rejects_missing_inductance_evidence() {
        let mut data = representative_data();
        data.inductance = None;
        let err = assemble_electrical_model(&data, TEST_THETA_E_US).unwrap_err();
        assert!(matches!(err, ModelAssemblyError::SampleCount));
    }

    #[test]
    fn model_assembly_errors_map_to_specific_commission_errors() {
        assert_eq!(
            CommissionError::from(ModelAssemblyError::SampleCount),
            CommissionError::ElectricalModelNoSamples
        );
        assert_eq!(
            CommissionError::from(ModelAssemblyError::ModelScaleZero { l_int: 0, l_uh: 0 }),
            CommissionError::ElectricalModelScaleZero
        );
        assert_eq!(
            CommissionError::from(ModelAssemblyError::ThetaTooLarge {
                theta_us: 120,
                tau_us: 200,
            }),
            CommissionError::ElectricalThetaTooLarge
        );
    }

    #[test]
    fn assemble_rejects_zero_l_int_with_model_scale_zero() {
        let mut data = ElectricalIdData::new(512);
        data.resistance_result = Some(resistance_result(100, 100));
        data.inductance = Some(inductance_evidence_fixture(1, 1_000_000));
        let err = assemble_electrical_model(&data, TEST_THETA_E_US).unwrap_err();
        assert!(matches!(
            err,
            ModelAssemblyError::ModelScaleZero { l_int: 0, l_uh: 0 }
        ));
    }

    #[test]
    fn assemble_warns_on_borderline_resistance_mismatch() {
        // 300 vs 320 → ~6% (above warn 5%, below fail 15%).
        let data = data_with_per_coil_currents(300, 320);
        let assembly = assemble_electrical_model(&data, TEST_THETA_E_US).unwrap();
        assert_ne!(
            assembly.warning_flags & INNER_WARN_BIT_RESISTANCE_MISMATCH,
            0
        );
    }
}
