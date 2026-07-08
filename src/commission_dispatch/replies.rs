//! Shared commissioning reply emission.
//!
//! Board crates keep transport-specific ankyra sender calls behind
//! [`CommissionReplySink`]. This module owns the semantic mapping from terminal
//! commissioning payloads to reply events so boards cannot drift by dropping
//! fields locally.

use crate::autotune::{CURRENT_AXIS_FLUX, CURRENT_AXIS_TORQUE, PhaseId};
use crate::commission_dispatch::ReplyPayload;
use crate::commissioning::types::{
    AdcCalResult, CommissionResult, CurrentValidationAxisEvidence,
    CurrentValidationSettledSampleEvidence, INDUCTANCE_SOURCE_NONE, InnerResult, OuterResult,
};
use crate::selftest::{CommissionDiagnostic, CommissionDiagnostics, SelftestStageValue};
use crate::tmc_diagnostics::{
    ClosedLoopEntryReport, CurrentLoopFiltersReport, CurrentLoopHoldReport, CurrentLoopRunReport,
    CurrentValidationAxisReport, CurrentValidationEnvelopeReport,
    CurrentValidationSettledSampleReport, EncoderAlignmentReport, InductanceEstimateReport,
    InductanceFrameReport, InductanceRunReport, ResistanceAxisReport, ResistanceRunReport,
};

/// TMC4671 external electrical angle for electrical axis 0 (PHASE0).
const RESISTANCE_AXIS0_PHI_E_EXT: u16 = 0;

/// TMC4671 external electrical angle for electrical axis 1 (PHASE90).
const RESISTANCE_AXIS1_PHI_E_EXT: u16 = 16_384;

/// Board-local sink for commissioning-family Klipper replies.
///
/// Implementations call board-generated ankyra reply macros. Methods return
/// `()` to preserve the existing fire-and-forget reply behavior.
pub trait CommissionReplySink {
    /// Emit one retained commissioning diagnostic detail event.
    fn foci_commission_detail(&mut self, oid: u8, event: CommissionDiagnostic);

    /// Emit a commissioning phase reply.
    fn foci_commission_phase(&mut self, oid: u8, phase: u8, status: u8);

    /// Emit the Stage 1 commissioning result.
    fn foci_commission_result(&mut self, oid: u8, status: u8, result: &CommissionResult);

    /// Emit the resistance-identification per-run evidence captured during
    /// Stage 1 commissioning. `FociCommissionResult` is size-capped, so this
    /// reuses the `foci_resistance_run` reply (Task 6) to carry the evidence
    /// that lives on `InnerResult` to the host.
    fn foci_resistance_run(&mut self, report: ResistanceRunReport);

    /// Emit one resistance-identification per-axis fit result captured
    /// during Stage 1 commissioning. Reuses the `foci_resistance_axis` reply
    /// (Task 6) for the same reason as [`Self::foci_resistance_run`].
    fn foci_resistance_axis(&mut self, report: ResistanceAxisReport);

    /// Emit encoder D-axis alignment evidence captured during Stage 1
    /// commissioning. This makes settled current-validation cross-axis tails
    /// correlatable with the alignment residual without expanding the
    /// size-capped terminal commission result.
    fn foci_encoder_alignment(&mut self, report: EncoderAlignmentReport);

    /// Emit run-level production inductance evidence captured during Stage 1
    /// commissioning.
    fn foci_inductance_run(&mut self, report: InductanceRunReport);

    /// Emit rotating-frame production inductance current evidence captured
    /// during Stage 1 commissioning.
    fn foci_inductance_frame(&mut self, report: InductanceFrameReport);

    /// Emit accepted production inductance estimate evidence captured during
    /// Stage 1 commissioning.
    fn foci_inductance_estimate(&mut self, report: InductanceEstimateReport);

    /// Emit sustained-hold current-loop evidence captured during Stage 1
    /// commissioning before the run-level current-loop summary.
    fn foci_current_loop_hold(&mut self, report: CurrentLoopHoldReport);

    /// Emit fast closed-loop entry stability evidence captured before the
    /// sustained-hold sampler.
    fn foci_closed_loop_entry(&mut self, report: ClosedLoopEntryReport);

    /// Emit applied filter cutoffs for the commissioned closed-loop frame.
    fn foci_current_loop_filters(&mut self, report: CurrentLoopFiltersReport);

    /// Emit run-level current-loop commissioning evidence captured during
    /// Stage 1 commissioning. `FociCommissionResult` is size-capped, so this
    /// supplemental reply carries source, tier, split, validation coverage,
    /// and failure-reason evidence separately.
    fn foci_current_loop_run(&mut self, report: CurrentLoopRunReport);

    /// Emit one per-axis, per-delay current validation evidence sample
    /// captured during Stage 1 commissioning.
    fn foci_current_validation_axis(&mut self, report: CurrentValidationAxisReport);

    /// Emit one raw settled-flux current validation read captured during Stage 1
    /// commissioning.
    fn foci_current_validation_settled_sample(
        &mut self,
        report: CurrentValidationSettledSampleReport,
    );

    /// Emit the current-validation target envelope captured during Stage 1
    /// commissioning.
    fn foci_current_validation_envelope(&mut self, report: CurrentValidationEnvelopeReport);

    /// Emit the Stage 2 tune result.
    fn foci_tune_result(&mut self, oid: u8, status: u8, result: &OuterResult);

    /// Emit the ADC calibration result.
    fn foci_calibrate_result(&mut self, oid: u8, status: u8, result: &AdcCalResult);

    /// Emit one self-test stage result.
    fn foci_selftest_result(&mut self, oid: u8, stage: u8, value: SelftestStageValue);

    /// Emit the terminal self-test completion reply.
    fn foci_selftest_done(&mut self, oid: u8, status: u8);
}

/// Emit all Klipper replies represented by one terminal commissioning payload.
pub fn emit_terminal_reply<S: CommissionReplySink>(sink: &mut S, payload: ReplyPayload) {
    match payload {
        ReplyPayload::CommissionPhase {
            oid,
            phase,
            status,
            diagnostics,
        } => {
            emit_commission_diagnostics(sink, oid, diagnostics);
            sink.foci_commission_phase(oid, phase, status);
        }
        ReplyPayload::CalibrateResult {
            oid,
            status,
            result,
        } => sink.foci_calibrate_result(oid, status, &result),
        ReplyPayload::CommissionResult {
            oid,
            status,
            result,
        } => {
            // FociCommissionResult is size-capped (23 fields, near the
            // 64-byte Klipper message limit) and cannot carry the evidence
            // that lives alongside `InnerResult`. Emit supplemental evidence
            // before the terminal commission result so the result stays the
            // true terminal/"done" signal for the whole commission flow.
            //
            // Resistance axes stay before the resistance run, mirroring the
            // diagnostic path's axes-then-run ordering (`emit_diagnostic_report`).
            let inner = &result.inner;
            sink.foci_resistance_axis(commission_resistance_axis_report(
                oid,
                0,
                RESISTANCE_AXIS0_PHI_E_EXT,
                inner,
            ));
            sink.foci_resistance_axis(commission_resistance_axis_report(
                oid,
                1,
                RESISTANCE_AXIS1_PHI_E_EXT,
                inner,
            ));
            sink.foci_resistance_run(commission_resistance_run_report(oid, inner));
            sink.foci_encoder_alignment(commission_encoder_alignment_report(oid, inner));
            emit_inductance_evidence(sink, oid, inner);
            emit_current_loop_evidence(sink, oid, status, &result);
            sink.foci_commission_result(oid, status, &result);
        }
        ReplyPayload::TuneResult {
            oid,
            status,
            result,
        } => sink.foci_tune_result(oid, status, &result),
        ReplyPayload::Selftest {
            oid,
            results,
            diagnostics,
            done_status,
        } => {
            emit_commission_diagnostics(sink, oid, diagnostics);
            for (stage, value) in results.iter() {
                sink.foci_selftest_result(oid, stage, value);
            }
            sink.foci_selftest_done(oid, done_status);
        }
    }
}

/// Emit a phase-transition reply with success status.
pub fn emit_phase_reply<S: CommissionReplySink>(sink: &mut S, oid: u8, phase_id: PhaseId) {
    sink.foci_commission_phase(oid, phase_id.to_u8(), 0);
}

fn emit_commission_diagnostics<S: CommissionReplySink>(
    sink: &mut S,
    oid: u8,
    diagnostics: CommissionDiagnostics,
) {
    for event in diagnostics.iter() {
        sink.foci_commission_detail(oid, event);
    }
}

/// Build the resistance-identification run report carried by the
/// commission flow's `foci_resistance_run` emission.
///
/// `status` is always 0: this report is only built once `InnerResult` has
/// already reached `CommissionDone`, which implies the resistance run
/// that produced it succeeded. Power-stage readback fields
/// (`pwm_maxcnt_readback`, `bbm_readback`, `dsadc_mdec_readback`,
/// `pwm_sv_chop_readback`) have no counterpart on `InnerResult` and are set
/// to 0; only the standalone diagnostic path (`foci_resistance_test`)
/// retains those readbacks.
fn commission_resistance_run_report(oid: u8, inner: &InnerResult) -> ResistanceRunReport {
    ResistanceRunReport {
        oid,
        status: 0,
        selected_r_count_slope_milli: inner.resistance_selected_count_slope_milli,
        gain_path_count_slope_milli: inner.resistance_gain_path_count_slope_milli,
        warning_flags: inner.resistance_warning_flags,
        status_flags_or: inner.resistance_status_flags_or,
        pwm_maxcnt_readback: 0,
        bbm_readback: 0,
        dsadc_mdec_readback: 0,
        pwm_sv_chop_readback: 0,
    }
}

/// Build one electrical-axis resistance fit report carried by the
/// commission flow's `foci_resistance_axis` emission.
///
/// `excluded_point_mask`, `selected_count`, and per-axis `warning_flags`
/// have no counterpart on `InnerResult` (only a run-level
/// `resistance_warning_flags` is retained) and are set to 0.
fn commission_resistance_axis_report(
    oid: u8,
    electrical_axis: u8,
    phi_e_ext: u16,
    inner: &InnerResult,
) -> ResistanceAxisReport {
    let (r_count_slope_milli, intercept_count, rmse_permille, selected_mask) = match electrical_axis
    {
        0 => (
            inner.resistance_axis0_count_slope_milli,
            inner.resistance_axis0_intercept_count,
            inner.resistance_axis0_rmse_permille,
            inner.resistance_selected_mask_axis0,
        ),
        _ => (
            inner.resistance_axis1_count_slope_milli,
            inner.resistance_axis1_intercept_count,
            inner.resistance_axis1_rmse_permille,
            inner.resistance_selected_mask_axis1,
        ),
    };
    let (signed_count_slope_milli, signed_asymmetry_permille, drift_permille) =
        match electrical_axis {
            0 => (
                inner.resistance_axis0_signed_count_slope_milli,
                inner.resistance_axis0_signed_asymmetry_permille,
                inner.resistance_axis0_drift_permille,
            ),
            _ => (
                inner.resistance_axis1_signed_count_slope_milli,
                inner.resistance_axis1_signed_asymmetry_permille,
                inner.resistance_axis1_drift_permille,
            ),
        };

    ResistanceAxisReport {
        oid,
        electrical_axis,
        phi_e_ext,
        r_count_slope_milli,
        intercept_count,
        rmse_permille,
        selected_mask,
        excluded_point_mask: 0,
        selected_count: 0,
        signed_count_slope_milli,
        signed_asymmetry_permille,
        drift_permille,
        warning_flags: 0,
    }
}

fn commission_encoder_alignment_report(oid: u8, inner: &InnerResult) -> EncoderAlignmentReport {
    EncoderAlignmentReport {
        oid,
        encoder_count: inner.encoder_alignment_count,
        electrical_residual_counts: inner.encoder_alignment_electrical_residual_counts,
        stability_counts: inner.encoder_alignment_stability_counts,
        movement_counts: inner.encoder_alignment_movement_counts,
        min_movement_counts: inner.encoder_alignment_min_movement_counts,
        counts_per_electrical_rev: inner.encoder_alignment_counts_per_electrical_rev,
    }
}

fn emit_inductance_evidence<S: CommissionReplySink>(sink: &mut S, oid: u8, inner: &InnerResult) {
    let evidence = inner.inductance;
    if evidence.run.source == INDUCTANCE_SOURCE_NONE || evidence.run.sample_count == 0 {
        return;
    }

    sink.foci_inductance_run(InductanceRunReport {
        oid,
        source: evidence.run.source,
        status: evidence.run.status,
        warning_flags: evidence.run.warning_flags,
        ud_count: evidence.run.ud_count,
        realized_frequency_millihz: evidence.run.realized_frequency_millihz,
        elapsed_us: evidence.run.elapsed_us,
        openloop_phi_delta_counts: evidence.run.openloop_phi_delta_counts,
        sample_count: evidence.run.sample_count,
        encoder_delta_counts: evidence.run.encoder_delta_counts,
        status_flags_or: evidence.run.status_flags_or,
    });
    sink.foci_inductance_frame(InductanceFrameReport {
        oid,
        id_mean_milli_count: evidence.frame.id_mean_milli_count,
        iq_mean_milli_count: evidence.frame.iq_mean_milli_count,
        id_rms_milli_count: evidence.frame.id_rms_milli_count,
        iq_rms_milli_count: evidence.frame.iq_rms_milli_count,
        drift_permille: evidence.frame.drift_permille,
        zero_id_mean_milli_count: evidence.frame.zero_id_mean_milli_count,
        zero_iq_mean_milli_count: evidence.frame.zero_iq_mean_milli_count,
    });
    sink.foci_inductance_estimate(InductanceEstimateReport {
        oid,
        x_average_count_ratio_milli: evidence.estimate.x_average_count_ratio_milli,
        x_d_count_ratio_milli: evidence.estimate.x_d_count_ratio_milli,
        x_q_count_ratio_milli: evidence.estimate.x_q_count_ratio_milli,
        saliency_status: evidence.estimate.saliency_status,
        saliency_permille: evidence.estimate.saliency_permille,
        x_mag_nominal_count_ratio_milli: evidence.estimate.x_mag_nominal_count_ratio_milli,
        x_mag_shift_minus_permille: evidence.estimate.x_mag_shift_minus_permille,
        x_mag_shift_plus_permille: evidence.estimate.x_mag_shift_plus_permille,
        x_mag_vs_quad_permille: evidence.estimate.x_mag_vs_quad_permille,
    });
}

fn emit_current_loop_evidence<S: CommissionReplySink>(
    sink: &mut S,
    oid: u8,
    status: u8,
    result: &CommissionResult,
) {
    let inner = &result.inner;
    sink.foci_current_validation_envelope(current_validation_envelope_report(oid, inner));
    for evidence in inner
        .current_settled_validation
        .iter()
        .take(inner.current_settled_validation_count as usize)
    {
        sink.foci_current_validation_settled_sample(current_validation_settled_sample_report(
            oid, evidence,
        ));
    }
    for (sample_index, evidence) in inner
        .current_flux_validation
        .iter()
        .take(inner.current_flux_validation_count as usize)
        .enumerate()
    {
        sink.foci_current_validation_axis(current_validation_axis_report(
            oid,
            CURRENT_AXIS_FLUX,
            sample_index as u8,
            evidence,
        ));
    }
    for (sample_index, evidence) in inner
        .current_torque_validation
        .iter()
        .take(inner.current_torque_validation_count as usize)
        .enumerate()
    {
        sink.foci_current_validation_axis(current_validation_axis_report(
            oid,
            CURRENT_AXIS_TORQUE,
            sample_index as u8,
            evidence,
        ));
    }
    sink.foci_current_loop_hold(commission_current_loop_hold_report(oid, inner));
    sink.foci_closed_loop_entry(commission_closed_loop_entry_report(oid, inner));
    sink.foci_current_loop_filters(commission_current_loop_filters_report(oid, result));
    sink.foci_current_loop_run(commission_current_loop_run_report(oid, status, inner));
}

fn current_validation_envelope_report(
    oid: u8,
    inner: &InnerResult,
) -> CurrentValidationEnvelopeReport {
    CurrentValidationEnvelopeReport {
        oid,
        step_amplitude: inner.current_validation_step_amplitude,
        flux_step_amplitude: inner.current_validation_flux_step_amplitude,
        torque_step_amplitude: inner.current_validation_torque_step_amplitude,
        pidout_limit: inner.current_validation_pidout_limit,
        current_limited: inner.current_validation_current_limited,
        voltage_limited: inner.current_validation_voltage_limited,
        max_p: inner.current_validation_max_p,
    }
}

fn current_validation_axis_report(
    oid: u8,
    axis: u8,
    sample_index: u8,
    evidence: &CurrentValidationAxisEvidence,
) -> CurrentValidationAxisReport {
    CurrentValidationAxisReport {
        oid,
        axis,
        sample_index,
        status: evidence.status,
        attempt: evidence.attempt,
        target: evidence.target,
        sample_delay_ms: evidence.sample_delay_ms,
        positive_response_permille: evidence.positive_response_permille,
        negative_response_permille: evidence.negative_response_permille,
        cross_axis_permille: evidence.cross_axis_permille,
        cross_axis_peak_permille: evidence.cross_axis_peak_permille,
        voltage_output_permille: evidence.voltage_output_permille,
        encoder_delta_counts: evidence.encoder_delta_counts,
        positive_encoder_delta_counts: evidence.positive_encoder_delta_counts,
        negative_encoder_delta_counts: evidence.negative_encoder_delta_counts,
        status_flags_or: evidence.status_flags_or,
    }
}

fn current_validation_settled_sample_report(
    oid: u8,
    evidence: &CurrentValidationSettledSampleEvidence,
) -> CurrentValidationSettledSampleReport {
    CurrentValidationSettledSampleReport {
        oid,
        axis: evidence.axis,
        sample_index: evidence.sample_index,
        direction: evidence.direction,
        raw_index: evidence.raw_index,
        attempt: evidence.attempt,
        target: evidence.target,
        sample_delay_ms: evidence.sample_delay_ms,
        same_axis_count: evidence.same_axis_count,
        cross_axis_count: evidence.cross_axis_count,
        cross_axis_permille: evidence.cross_axis_permille,
        voltage_output_permille: evidence.voltage_output_permille,
        encoder_delta_counts: evidence.encoder_delta_counts,
        status_flags: evidence.status_flags,
    }
}

fn commission_current_loop_run_report(
    oid: u8,
    status: u8,
    inner: &InnerResult,
) -> CurrentLoopRunReport {
    CurrentLoopRunReport {
        oid,
        status,
        gains_source: inner.current_gains_source,
        candidate_gains_source: inner.candidate_gains_source,
        axis_split_source: inner.axis_split_source,
        candidate_axis_split_source: inner.candidate_axis_split_source,
        gains_tier: inner.gains_tier,
        candidate_gains_tier: inner.candidate_gains_tier,
        measured_axis_split_permille: inner.measured_axis_split_permille,
        candidate_measured_axis_split_permille: inner.candidate_measured_axis_split_permille,
        applied_axis_split_permille: inner.applied_axis_split_permille,
        candidate_applied_axis_split_permille: inner.candidate_applied_axis_split_permille,
        axis_split_clamped: inner.axis_split_clamped,
        candidate_axis_split_clamped: inner.candidate_axis_split_clamped,
        candidate_flux_p: inner.candidate_flux_p,
        candidate_flux_i: inner.candidate_flux_i,
        candidate_torque_p: inner.candidate_torque_p,
        candidate_torque_i: inner.candidate_torque_i,
        candidate_attempt: inner.candidate_attempt,
        current_validation_axes: inner.current_validation_axes,
        flux_validation_sample_count: inner.current_flux_validation_count,
        torque_validation_sample_count: inner.current_torque_validation_count,
        retry_budget_exhausted: inner.retry_budget_exhausted,
        failure_reason: inner.current_loop_failure_reason,
    }
}

fn commission_current_loop_filters_report(
    oid: u8,
    result: &CommissionResult,
) -> CurrentLoopFiltersReport {
    CurrentLoopFiltersReport {
        oid,
        velocity_filter_hz: result.velocity_filter_hz,
        torque_filter_hz: result.torque_filter_hz,
        position_filter_hz: result.position_filter_hz,
        flux_filter_hz: result.flux_filter_hz,
    }
}

fn commission_current_loop_hold_report(oid: u8, inner: &InnerResult) -> CurrentLoopHoldReport {
    CurrentLoopHoldReport {
        oid,
        hold_status: inner.current_hold_status,
        warning_flags: inner.current_hold_warning_flags,
        elapsed_us: inner.current_hold_elapsed_us,
        requested_sample_period_us: inner.current_hold_requested_sample_period_us,
        sample_count: inner.current_hold_sample_count,
        position_span_count: inner.current_hold_position_span_count,
        position_drift_count: inner.current_hold_position_drift_count,
        torque_mean_count: inner.current_hold_torque_mean_count,
        torque_rms_count: inner.current_hold_torque_rms_count,
        torque_peak_to_peak_count: inner.current_hold_torque_peak_to_peak_count,
        torque_crossing_count: inner.current_hold_torque_crossing_count,
        flux_mean_count: inner.current_hold_flux_mean_count,
        flux_rms_count: inner.current_hold_flux_rms_count,
        flux_peak_to_peak_count: inner.current_hold_flux_peak_to_peak_count,
        flux_crossing_count: inner.current_hold_flux_crossing_count,
        status_flags_or: inner.current_hold_status_flags_or,
        actionable_status_count: inner.current_hold_actionable_status_count,
    }
}

fn commission_closed_loop_entry_report(oid: u8, inner: &InnerResult) -> ClosedLoopEntryReport {
    ClosedLoopEntryReport {
        oid,
        entry_status: inner.closed_loop_entry_status,
        position_1: inner.closed_loop_entry_position_1,
        position_2: inner.closed_loop_entry_position_2,
        drift_count: inner.closed_loop_entry_drift_count,
        threshold_count: inner.closed_loop_entry_threshold_count,
        runaway: inner.closed_loop_entry_runaway,
    }
}

#[cfg(test)]
mod tests {
    use std::vec::Vec;

    use super::*;
    use crate::autotune::{
        AXIS_SPLIT_SOURCE_NONE, CURRENT_AXIS_FLUX, CURRENT_AXIS_TORQUE,
        CURRENT_GAINS_SOURCE_FAILED, CURRENT_GAINS_SOURCE_MEASURED,
        CURRENT_GAINS_TIER_MEASURED_SYMMETRIC, CURRENT_GAINS_TIER_NONE, CURRENT_LOOP_FAILURE_NONE,
        CURRENT_LOOP_FAILURE_SATURATION, CURRENT_VALIDATION_AXIS_FLUX,
        CURRENT_VALIDATION_AXIS_TORQUE, CommissionError, CommissionStatus, PhaseId,
    };
    use crate::commission_dispatch::ReplyPayload;
    use crate::commissioning::types::{
        AdcCalResult, CommissionResult, CurrentValidationAxisEvidence,
        CurrentValidationSettledSampleEvidence, InductanceEvidence, InnerResult, OuterResult,
    };
    use crate::selftest::{
        COMMISSION_DIAGNOSTIC_CAPACITY, CommissionDiagnostic, CommissionDiagnostics,
        SelftestResults, SelftestStageValue,
    };

    #[derive(Debug, Clone, PartialEq, Eq)]
    enum Event {
        Detail {
            oid: u8,
            phase: u8,
            code: u8,
            status: u8,
            value0: u32,
            value1: u32,
            value2: u32,
        },
        Phase {
            oid: u8,
            phase: u8,
            status: u8,
        },
        Calibrate {
            oid: u8,
            status: u8,
            adc_i0: u16,
            adc_i1: u16,
            encoder_count: u32,
        },
        Commission {
            oid: u8,
            status: u8,
            r_int: u32,
            fallback_velocity_limit: u32,
        },
        ResistanceRun {
            oid: u8,
            status: u8,
            selected_r_count_slope_milli: i32,
            gain_path_count_slope_milli: i32,
            warning_flags: u32,
            status_flags_or: u32,
        },
        ResistanceAxis {
            oid: u8,
            electrical_axis: u8,
            phi_e_ext: u16,
            r_count_slope_milli: i32,
            intercept_count: i32,
            rmse_permille: u16,
            selected_mask: u16,
            signed_count_slope_milli: i32,
            signed_asymmetry_permille: u16,
            drift_permille: u16,
        },
        EncoderAlignment {
            oid: u8,
            encoder_count: u32,
            electrical_residual_counts: u16,
            stability_counts: u16,
            movement_counts: u16,
            min_movement_counts: u16,
            counts_per_electrical_rev: u16,
        },
        InductanceRun {
            oid: u8,
            source: u8,
            status: u8,
            warning_flags: u16,
            ud_count: i16,
            realized_frequency_millihz: u32,
            elapsed_us: u32,
            openloop_phi_delta_counts: u32,
            sample_count: u16,
            encoder_delta_counts: i32,
            status_flags_or: u32,
        },
        InductanceFrame {
            oid: u8,
            id_mean_milli_count: i32,
            iq_mean_milli_count: i32,
            id_rms_milli_count: u32,
            iq_rms_milli_count: u32,
            drift_permille: u16,
            zero_id_mean_milli_count: i32,
            zero_iq_mean_milli_count: i32,
        },
        InductanceEstimate {
            oid: u8,
            x_average_count_ratio_milli: u32,
            x_d_count_ratio_milli: u32,
            x_q_count_ratio_milli: u32,
            saliency_status: u8,
            saliency_permille: u16,
            x_mag_nominal_count_ratio_milli: u32,
            x_mag_shift_minus_permille: u16,
            x_mag_shift_plus_permille: u16,
            x_mag_vs_quad_permille: u16,
        },
        CurrentValidationAxis {
            oid: u8,
            axis: u8,
            sample_index: u8,
            status: u8,
            attempt: u8,
            target: i16,
            sample_delay_ms: u16,
            positive_response_permille: u16,
            negative_response_permille: u16,
            cross_axis_permille: u16,
            cross_axis_peak_permille: u16,
            voltage_output_permille: u16,
            encoder_delta_counts: u16,
            positive_encoder_delta_counts: i32,
            negative_encoder_delta_counts: i32,
            status_flags_or: u32,
        },
        CurrentValidationSettledSample {
            oid: u8,
            axis: u8,
            sample_index: u8,
            direction: u8,
            raw_index: u8,
            attempt: u8,
            target: i16,
            sample_delay_ms: u16,
            same_axis_count: i16,
            cross_axis_count: i16,
            cross_axis_permille: u16,
            voltage_output_permille: u16,
            encoder_delta_counts: i32,
            status_flags: u32,
        },
        CurrentValidationEnvelope {
            oid: u8,
            step_amplitude: u16,
            flux_step_amplitude: u16,
            torque_step_amplitude: u16,
            pidout_limit: u16,
            current_limited: u16,
            voltage_limited: u16,
            max_p: u16,
        },
        CurrentLoopHold {
            oid: u8,
            hold_status: u8,
            sample_count: u8,
            torque_rms_count: u16,
            flux_rms_count: u16,
        },
        ClosedLoopEntry {
            oid: u8,
            entry_status: u8,
            position_1: i32,
            position_2: i32,
            drift_count: u16,
            threshold_count: u16,
            runaway: u8,
        },
        CurrentLoopFilters {
            oid: u8,
            velocity_filter_hz: u16,
            torque_filter_hz: u16,
            position_filter_hz: u16,
            flux_filter_hz: u16,
        },
        CurrentLoopRun {
            oid: u8,
            status: u8,
            gains_source: u8,
            candidate_gains_source: u8,
            axis_split_source: u8,
            candidate_axis_split_source: u8,
            gains_tier: u8,
            candidate_gains_tier: u8,
            measured_axis_split_permille: u16,
            candidate_measured_axis_split_permille: u16,
            applied_axis_split_permille: u16,
            candidate_applied_axis_split_permille: u16,
            axis_split_clamped: u8,
            candidate_axis_split_clamped: u8,
            candidate_flux_p: u16,
            candidate_flux_i: u16,
            candidate_torque_p: u16,
            candidate_torque_i: u16,
            candidate_attempt: u8,
            current_validation_axes: u8,
            flux_validation_sample_count: u8,
            torque_validation_sample_count: u8,
            retry_budget_exhausted: u8,
            failure_reason: u8,
        },
        Tune {
            oid: u8,
            status: u8,
            velocity_p: u16,
            position_p: u16,
            warning_code: u8,
        },
        SelftestResult {
            oid: u8,
            stage: u8,
            status: u8,
            value: u32,
        },
        SelftestDone {
            oid: u8,
            status: u8,
        },
    }

    #[derive(Default)]
    struct RecordingSink {
        events: Vec<Event>,
    }

    impl CommissionReplySink for RecordingSink {
        fn foci_commission_detail(&mut self, oid: u8, event: CommissionDiagnostic) {
            self.events.push(Event::Detail {
                oid,
                phase: event.phase,
                code: event.code,
                status: event.status,
                value0: event.value0,
                value1: event.value1,
                value2: event.value2,
            });
        }

        fn foci_commission_phase(&mut self, oid: u8, phase: u8, status: u8) {
            self.events.push(Event::Phase { oid, phase, status });
        }

        fn foci_commission_result(&mut self, oid: u8, status: u8, result: &CommissionResult) {
            self.events.push(Event::Commission {
                oid,
                status,
                r_int: result.inner.r_int,
                fallback_velocity_limit: result.fallback_velocity_limit,
            });
        }

        fn foci_resistance_run(&mut self, report: crate::tmc_diagnostics::ResistanceRunReport) {
            self.events.push(Event::ResistanceRun {
                oid: report.oid,
                status: report.status,
                selected_r_count_slope_milli: report.selected_r_count_slope_milli,
                gain_path_count_slope_milli: report.gain_path_count_slope_milli,
                warning_flags: report.warning_flags,
                status_flags_or: report.status_flags_or,
            });
        }

        fn foci_resistance_axis(&mut self, report: crate::tmc_diagnostics::ResistanceAxisReport) {
            self.events.push(Event::ResistanceAxis {
                oid: report.oid,
                electrical_axis: report.electrical_axis,
                phi_e_ext: report.phi_e_ext,
                r_count_slope_milli: report.r_count_slope_milli,
                intercept_count: report.intercept_count,
                rmse_permille: report.rmse_permille,
                selected_mask: report.selected_mask,
                signed_count_slope_milli: report.signed_count_slope_milli,
                signed_asymmetry_permille: report.signed_asymmetry_permille,
                drift_permille: report.drift_permille,
            });
        }

        fn foci_encoder_alignment(
            &mut self,
            report: crate::tmc_diagnostics::EncoderAlignmentReport,
        ) {
            self.events.push(Event::EncoderAlignment {
                oid: report.oid,
                encoder_count: report.encoder_count,
                electrical_residual_counts: report.electrical_residual_counts,
                stability_counts: report.stability_counts,
                movement_counts: report.movement_counts,
                min_movement_counts: report.min_movement_counts,
                counts_per_electrical_rev: report.counts_per_electrical_rev,
            });
        }

        fn foci_inductance_run(&mut self, report: crate::tmc_diagnostics::InductanceRunReport) {
            self.events.push(Event::InductanceRun {
                oid: report.oid,
                source: report.source,
                status: report.status,
                warning_flags: report.warning_flags,
                ud_count: report.ud_count,
                realized_frequency_millihz: report.realized_frequency_millihz,
                elapsed_us: report.elapsed_us,
                openloop_phi_delta_counts: report.openloop_phi_delta_counts,
                sample_count: report.sample_count,
                encoder_delta_counts: report.encoder_delta_counts,
                status_flags_or: report.status_flags_or,
            });
        }

        fn foci_inductance_frame(&mut self, report: crate::tmc_diagnostics::InductanceFrameReport) {
            self.events.push(Event::InductanceFrame {
                oid: report.oid,
                id_mean_milli_count: report.id_mean_milli_count,
                iq_mean_milli_count: report.iq_mean_milli_count,
                id_rms_milli_count: report.id_rms_milli_count,
                iq_rms_milli_count: report.iq_rms_milli_count,
                drift_permille: report.drift_permille,
                zero_id_mean_milli_count: report.zero_id_mean_milli_count,
                zero_iq_mean_milli_count: report.zero_iq_mean_milli_count,
            });
        }

        fn foci_inductance_estimate(
            &mut self,
            report: crate::tmc_diagnostics::InductanceEstimateReport,
        ) {
            self.events.push(Event::InductanceEstimate {
                oid: report.oid,
                x_average_count_ratio_milli: report.x_average_count_ratio_milli,
                x_d_count_ratio_milli: report.x_d_count_ratio_milli,
                x_q_count_ratio_milli: report.x_q_count_ratio_milli,
                saliency_status: report.saliency_status,
                saliency_permille: report.saliency_permille,
                x_mag_nominal_count_ratio_milli: report.x_mag_nominal_count_ratio_milli,
                x_mag_shift_minus_permille: report.x_mag_shift_minus_permille,
                x_mag_shift_plus_permille: report.x_mag_shift_plus_permille,
                x_mag_vs_quad_permille: report.x_mag_vs_quad_permille,
            });
        }

        fn foci_current_loop_hold(
            &mut self,
            report: crate::tmc_diagnostics::CurrentLoopHoldReport,
        ) {
            self.events.push(Event::CurrentLoopHold {
                oid: report.oid,
                hold_status: report.hold_status,
                sample_count: report.sample_count,
                torque_rms_count: report.torque_rms_count,
                flux_rms_count: report.flux_rms_count,
            });
        }

        fn foci_closed_loop_entry(
            &mut self,
            report: crate::tmc_diagnostics::ClosedLoopEntryReport,
        ) {
            self.events.push(Event::ClosedLoopEntry {
                oid: report.oid,
                entry_status: report.entry_status,
                position_1: report.position_1,
                position_2: report.position_2,
                drift_count: report.drift_count,
                threshold_count: report.threshold_count,
                runaway: report.runaway,
            });
        }

        fn foci_current_loop_filters(
            &mut self,
            report: crate::tmc_diagnostics::CurrentLoopFiltersReport,
        ) {
            self.events.push(Event::CurrentLoopFilters {
                oid: report.oid,
                velocity_filter_hz: report.velocity_filter_hz,
                torque_filter_hz: report.torque_filter_hz,
                position_filter_hz: report.position_filter_hz,
                flux_filter_hz: report.flux_filter_hz,
            });
        }

        fn foci_current_loop_run(&mut self, report: crate::tmc_diagnostics::CurrentLoopRunReport) {
            self.events.push(Event::CurrentLoopRun {
                oid: report.oid,
                status: report.status,
                gains_source: report.gains_source,
                candidate_gains_source: report.candidate_gains_source,
                axis_split_source: report.axis_split_source,
                candidate_axis_split_source: report.candidate_axis_split_source,
                gains_tier: report.gains_tier,
                candidate_gains_tier: report.candidate_gains_tier,
                measured_axis_split_permille: report.measured_axis_split_permille,
                candidate_measured_axis_split_permille: report
                    .candidate_measured_axis_split_permille,
                applied_axis_split_permille: report.applied_axis_split_permille,
                candidate_applied_axis_split_permille: report.candidate_applied_axis_split_permille,
                axis_split_clamped: report.axis_split_clamped,
                candidate_axis_split_clamped: report.candidate_axis_split_clamped,
                candidate_flux_p: report.candidate_flux_p,
                candidate_flux_i: report.candidate_flux_i,
                candidate_torque_p: report.candidate_torque_p,
                candidate_torque_i: report.candidate_torque_i,
                candidate_attempt: report.candidate_attempt,
                current_validation_axes: report.current_validation_axes,
                flux_validation_sample_count: report.flux_validation_sample_count,
                torque_validation_sample_count: report.torque_validation_sample_count,
                retry_budget_exhausted: report.retry_budget_exhausted,
                failure_reason: report.failure_reason,
            });
        }

        fn foci_current_validation_axis(
            &mut self,
            report: crate::tmc_diagnostics::CurrentValidationAxisReport,
        ) {
            self.events.push(Event::CurrentValidationAxis {
                oid: report.oid,
                axis: report.axis,
                sample_index: report.sample_index,
                status: report.status,
                attempt: report.attempt,
                target: report.target,
                sample_delay_ms: report.sample_delay_ms,
                positive_response_permille: report.positive_response_permille,
                negative_response_permille: report.negative_response_permille,
                cross_axis_permille: report.cross_axis_permille,
                cross_axis_peak_permille: report.cross_axis_peak_permille,
                voltage_output_permille: report.voltage_output_permille,
                encoder_delta_counts: report.encoder_delta_counts,
                positive_encoder_delta_counts: report.positive_encoder_delta_counts,
                negative_encoder_delta_counts: report.negative_encoder_delta_counts,
                status_flags_or: report.status_flags_or,
            });
        }

        fn foci_current_validation_settled_sample(
            &mut self,
            report: crate::tmc_diagnostics::CurrentValidationSettledSampleReport,
        ) {
            self.events.push(Event::CurrentValidationSettledSample {
                oid: report.oid,
                axis: report.axis,
                sample_index: report.sample_index,
                direction: report.direction,
                raw_index: report.raw_index,
                attempt: report.attempt,
                target: report.target,
                sample_delay_ms: report.sample_delay_ms,
                same_axis_count: report.same_axis_count,
                cross_axis_count: report.cross_axis_count,
                cross_axis_permille: report.cross_axis_permille,
                voltage_output_permille: report.voltage_output_permille,
                encoder_delta_counts: report.encoder_delta_counts,
                status_flags: report.status_flags,
            });
        }

        fn foci_current_validation_envelope(
            &mut self,
            report: crate::tmc_diagnostics::CurrentValidationEnvelopeReport,
        ) {
            self.events.push(Event::CurrentValidationEnvelope {
                oid: report.oid,
                step_amplitude: report.step_amplitude,
                flux_step_amplitude: report.flux_step_amplitude,
                torque_step_amplitude: report.torque_step_amplitude,
                pidout_limit: report.pidout_limit,
                current_limited: report.current_limited,
                voltage_limited: report.voltage_limited,
                max_p: report.max_p,
            });
        }

        fn foci_tune_result(&mut self, oid: u8, status: u8, result: &OuterResult) {
            self.events.push(Event::Tune {
                oid,
                status,
                velocity_p: result.velocity_p,
                position_p: result.position_p,
                warning_code: result.warning_code,
            });
        }

        fn foci_calibrate_result(&mut self, oid: u8, status: u8, result: &AdcCalResult) {
            self.events.push(Event::Calibrate {
                oid,
                status,
                adc_i0: result.adc_i0,
                adc_i1: result.adc_i1,
                encoder_count: result.encoder_count,
            });
        }

        fn foci_selftest_result(&mut self, oid: u8, stage: u8, value: SelftestStageValue) {
            self.events.push(Event::SelftestResult {
                oid,
                stage,
                status: value.status,
                value: value.value,
            });
        }

        fn foci_selftest_done(&mut self, oid: u8, status: u8) {
            self.events.push(Event::SelftestDone { oid, status });
        }
    }

    fn diagnostic(phase: u8, code: u8) -> CommissionDiagnostic {
        CommissionDiagnostic {
            phase,
            code,
            status: code % 3,
            value0: 100 + u32::from(code),
            value1: 200 + u32::from(code),
            value2: 300 + u32::from(code),
        }
    }

    fn diagnostics<const N: usize>(items: [CommissionDiagnostic; N]) -> CommissionDiagnostics {
        let mut diagnostics = CommissionDiagnostics::default();
        for item in items {
            diagnostics.push(item);
        }
        diagnostics
    }

    fn commission_result() -> CommissionResult {
        let mut flux_validation = [CurrentValidationAxisEvidence::EMPTY;
            crate::autotune::CURRENT_VALIDATION_MAX_SAMPLES_PER_AXIS];
        flux_validation[0] = CurrentValidationAxisEvidence {
            status: 0,
            attempt: 0,
            target: 250,
            sample_delay_ms: 2,
            positive_response_permille: 960,
            negative_response_permille: 940,
            cross_axis_permille: 32,
            cross_axis_peak_permille: 32,
            voltage_output_permille: 250,
            encoder_delta_counts: 1,
            positive_encoder_delta_counts: 1,
            negative_encoder_delta_counts: -1,
            status_flags_or: 0,
        };
        let mut torque_validation = [CurrentValidationAxisEvidence::EMPTY;
            crate::autotune::CURRENT_VALIDATION_MAX_SAMPLES_PER_AXIS];
        torque_validation[0] = CurrentValidationAxisEvidence {
            status: 0,
            attempt: 0,
            target: 250,
            sample_delay_ms: 1,
            positive_response_permille: 900,
            negative_response_permille: 880,
            cross_axis_permille: 24,
            cross_axis_peak_permille: 24,
            voltage_output_permille: 280,
            encoder_delta_counts: 2,
            positive_encoder_delta_counts: 2,
            negative_encoder_delta_counts: -2,
            status_flags_or: 4,
        };
        let mut settled_validation = [CurrentValidationSettledSampleEvidence::EMPTY;
            crate::autotune::CURRENT_VALIDATION_SETTLED_MAX_SAMPLES];
        settled_validation[0] = CurrentValidationSettledSampleEvidence {
            axis: CURRENT_AXIS_FLUX,
            sample_index: 3,
            direction: 0,
            raw_index: 0,
            attempt: 0,
            target: 128,
            sample_delay_ms: 100,
            same_axis_count: 120,
            cross_axis_count: 45,
            cross_axis_permille: 351,
            voltage_output_permille: 410,
            encoder_delta_counts: 1,
            status_flags: 0x40,
        };

        CommissionResult {
            inner: InnerResult {
                status: CommissionStatus::Accepted,
                r_int: 11,
                l_int: 12,
                r_mohm: 13,
                l_uh: 14,
                flux_p: 15,
                flux_i: 16,
                torque_p: 17,
                torque_i: 18,
                candidate_gains_source: CURRENT_GAINS_SOURCE_MEASURED,
                candidate_axis_split_source: AXIS_SPLIT_SOURCE_NONE,
                candidate_gains_tier: CURRENT_GAINS_TIER_MEASURED_SYMMETRIC,
                candidate_measured_axis_split_permille: 1000,
                candidate_applied_axis_split_permille: 1000,
                candidate_axis_split_clamped: 0,
                candidate_flux_p: 15,
                candidate_flux_i: 16,
                candidate_torque_p: 17,
                candidate_torque_i: 18,
                candidate_attempt: 0,
                lambda_us: 19,
                theta_e_us: 20,
                ringing_count: 21,
                bandwidth_hz: 22,
                tau_e_us: 23,
                inductance: InductanceEvidence::test_fixture(),
                inner_warning_flags: 26,
                resistance_selected_count_slope_milli: 1050,
                resistance_gain_path_count_slope_milli: 16,
                resistance_axis0_count_slope_milli: 1040,
                resistance_axis1_count_slope_milli: 1060,
                resistance_axis0_intercept_count: 5,
                resistance_axis1_intercept_count: 6,
                resistance_axis0_rmse_permille: 7,
                resistance_axis1_rmse_permille: 8,
                resistance_selected_mask_axis0: 0b1111_1000,
                resistance_selected_mask_axis1: 0b0111_1100,
                resistance_axis0_signed_count_slope_milli: 1045,
                resistance_axis1_signed_count_slope_milli: 1055,
                resistance_axis0_signed_asymmetry_permille: 9,
                resistance_axis1_signed_asymmetry_permille: 10,
                resistance_axis0_drift_permille: 11,
                resistance_axis1_drift_permille: 12,
                resistance_status_flags_or: 0,
                resistance_warning_flags: 0,
                encoder_alignment_count: 163,
                encoder_alignment_electrical_residual_counts: 3,
                encoder_alignment_stability_counts: 1,
                encoder_alignment_movement_counts: 37,
                encoder_alignment_min_movement_counts: 2,
                encoder_alignment_counts_per_electrical_rev: 80,
                current_gains_source: CURRENT_GAINS_SOURCE_MEASURED,
                axis_split_source: AXIS_SPLIT_SOURCE_NONE,
                gains_tier: CURRENT_GAINS_TIER_MEASURED_SYMMETRIC,
                measured_axis_split_permille: 1000,
                applied_axis_split_permille: 1000,
                axis_split_clamped: 0,
                current_validation_axes: CURRENT_VALIDATION_AXIS_FLUX
                    | CURRENT_VALIDATION_AXIS_TORQUE,
                retry_budget_exhausted: 0,
                current_loop_failure_reason: CURRENT_LOOP_FAILURE_NONE,
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
                current_settled_validation_count: 1,
                current_settled_validation: settled_validation,
                current_flux_validation_count: 1,
                current_flux_validation: flux_validation,
                current_torque_validation_count: 1,
                current_torque_validation: torque_validation,
            },
            fallback_velocity_p: 31,
            fallback_velocity_i: 32,
            fallback_position_p: 33,
            fallback_position_i: 34,
            fallback_velocity_limit: 35,
            velocity_filter_hz: 0,
            torque_filter_hz: 3000,
            position_filter_hz: 0,
            flux_filter_hz: 3000,
        }
    }

    fn outer_result() -> OuterResult {
        OuterResult {
            status: CommissionStatus::AcceptedWithWarnings,
            velocity_p: 41,
            velocity_i: 42,
            velocity_limit: 43,
            position_p: 44,
            position_i: 45,
            velocity_filter_hz: 46,
            position_filter_hz: 47,
            flux_filter_hz: 48,
            torque_filter_hz: 49,
            j_eff: 50,
            b_eff: 51,
            warning_code: 52,
            mech_torque_step: 53,
            mech_fwd_accel: 54,
            mech_rev_accel: 55,
            mech_fwd_peak_velocity: 56,
            mech_rev_peak_velocity: 57,
            mech_fwd_travel: 58,
            mech_rev_travel: 59,
            synth_velocity_p: 60,
            synth_velocity_i: 61,
            synth_position_p: 62,
            synth_position_i: 63,
        }
    }

    fn current_validation_event(
        oid: u8,
        axis: u8,
        sample_index: u8,
        evidence: CurrentValidationAxisEvidence,
    ) -> Event {
        Event::CurrentValidationAxis {
            oid,
            axis,
            sample_index,
            status: evidence.status,
            attempt: evidence.attempt,
            target: evidence.target,
            sample_delay_ms: evidence.sample_delay_ms,
            positive_response_permille: evidence.positive_response_permille,
            negative_response_permille: evidence.negative_response_permille,
            cross_axis_permille: evidence.cross_axis_permille,
            cross_axis_peak_permille: evidence.cross_axis_peak_permille,
            voltage_output_permille: evidence.voltage_output_permille,
            encoder_delta_counts: evidence.encoder_delta_counts,
            positive_encoder_delta_counts: evidence.positive_encoder_delta_counts,
            negative_encoder_delta_counts: evidence.negative_encoder_delta_counts,
            status_flags_or: evidence.status_flags_or,
        }
    }

    fn current_loop_events(oid: u8, status: u8) -> Vec<Event> {
        let mut events = Vec::new();
        events.push(Event::CurrentValidationEnvelope {
            oid,
            step_amplitude: 128,
            flux_step_amplitude: 128,
            torque_step_amplitude: 128,
            pidout_limit: 2400,
            current_limited: 422,
            voltage_limited: 128,
            max_p: 3474,
        });
        events.push(Event::CurrentValidationSettledSample {
            oid,
            axis: CURRENT_AXIS_FLUX,
            sample_index: 3,
            direction: 0,
            raw_index: 0,
            attempt: 0,
            target: 128,
            sample_delay_ms: 100,
            same_axis_count: 120,
            cross_axis_count: 45,
            cross_axis_permille: 351,
            voltage_output_permille: 410,
            encoder_delta_counts: 1,
            status_flags: 0x40,
        });
        events.push(current_validation_event(
            oid,
            CURRENT_AXIS_FLUX,
            0,
            CurrentValidationAxisEvidence {
                status: 0,
                attempt: 0,
                target: 250,
                sample_delay_ms: 2,
                positive_response_permille: 960,
                negative_response_permille: 940,
                cross_axis_permille: 32,
                cross_axis_peak_permille: 32,
                voltage_output_permille: 250,
                encoder_delta_counts: 1,
                positive_encoder_delta_counts: 1,
                negative_encoder_delta_counts: -1,
                status_flags_or: 0,
            },
        ));
        events.push(current_validation_event(
            oid,
            CURRENT_AXIS_TORQUE,
            0,
            CurrentValidationAxisEvidence {
                status: 0,
                attempt: 0,
                target: 250,
                sample_delay_ms: 1,
                positive_response_permille: 900,
                negative_response_permille: 880,
                cross_axis_permille: 24,
                cross_axis_peak_permille: 24,
                voltage_output_permille: 280,
                encoder_delta_counts: 2,
                positive_encoder_delta_counts: 2,
                negative_encoder_delta_counts: -2,
                status_flags_or: 4,
            },
        ));
        events.push(Event::CurrentLoopHold {
            oid,
            hold_status: crate::autotune::HOLD_GATE_STATUS_NOT_RUN,
            sample_count: 0,
            torque_rms_count: 0,
            flux_rms_count: 0,
        });
        events.push(Event::ClosedLoopEntry {
            oid,
            entry_status: crate::autotune::CLOSED_LOOP_ENTRY_STATUS_NOT_RUN,
            position_1: 0,
            position_2: 0,
            drift_count: 0,
            threshold_count: 0,
            runaway: 0,
        });
        events.push(Event::CurrentLoopFilters {
            oid,
            velocity_filter_hz: 0,
            torque_filter_hz: 3000,
            position_filter_hz: 0,
            flux_filter_hz: 3000,
        });
        events.push(Event::CurrentLoopRun {
            oid,
            status,
            gains_source: CURRENT_GAINS_SOURCE_MEASURED,
            candidate_gains_source: CURRENT_GAINS_SOURCE_MEASURED,
            axis_split_source: AXIS_SPLIT_SOURCE_NONE,
            candidate_axis_split_source: AXIS_SPLIT_SOURCE_NONE,
            gains_tier: CURRENT_GAINS_TIER_MEASURED_SYMMETRIC,
            candidate_gains_tier: CURRENT_GAINS_TIER_MEASURED_SYMMETRIC,
            measured_axis_split_permille: 1000,
            candidate_measured_axis_split_permille: 1000,
            applied_axis_split_permille: 1000,
            candidate_applied_axis_split_permille: 1000,
            axis_split_clamped: 0,
            candidate_axis_split_clamped: 0,
            candidate_flux_p: 15,
            candidate_flux_i: 16,
            candidate_torque_p: 17,
            candidate_torque_i: 18,
            candidate_attempt: 0,
            current_validation_axes: CURRENT_VALIDATION_AXIS_FLUX | CURRENT_VALIDATION_AXIS_TORQUE,
            flux_validation_sample_count: 1,
            torque_validation_sample_count: 1,
            retry_budget_exhausted: 0,
            failure_reason: CURRENT_LOOP_FAILURE_NONE,
        });
        events
    }

    fn inductance_events(oid: u8) -> Vec<Event> {
        let evidence = InductanceEvidence::test_fixture();
        std::vec![
            Event::InductanceRun {
                oid,
                source: evidence.run.source,
                status: evidence.run.status,
                warning_flags: evidence.run.warning_flags,
                ud_count: evidence.run.ud_count,
                realized_frequency_millihz: evidence.run.realized_frequency_millihz,
                elapsed_us: evidence.run.elapsed_us,
                openloop_phi_delta_counts: evidence.run.openloop_phi_delta_counts,
                sample_count: evidence.run.sample_count,
                encoder_delta_counts: evidence.run.encoder_delta_counts,
                status_flags_or: evidence.run.status_flags_or,
            },
            Event::InductanceFrame {
                oid,
                id_mean_milli_count: evidence.frame.id_mean_milli_count,
                iq_mean_milli_count: evidence.frame.iq_mean_milli_count,
                id_rms_milli_count: evidence.frame.id_rms_milli_count,
                iq_rms_milli_count: evidence.frame.iq_rms_milli_count,
                drift_permille: evidence.frame.drift_permille,
                zero_id_mean_milli_count: evidence.frame.zero_id_mean_milli_count,
                zero_iq_mean_milli_count: evidence.frame.zero_iq_mean_milli_count,
            },
            Event::InductanceEstimate {
                oid,
                x_average_count_ratio_milli: evidence.estimate.x_average_count_ratio_milli,
                x_d_count_ratio_milli: evidence.estimate.x_d_count_ratio_milli,
                x_q_count_ratio_milli: evidence.estimate.x_q_count_ratio_milli,
                saliency_status: evidence.estimate.saliency_status,
                saliency_permille: evidence.estimate.saliency_permille,
                x_mag_nominal_count_ratio_milli: evidence.estimate.x_mag_nominal_count_ratio_milli,
                x_mag_shift_minus_permille: evidence.estimate.x_mag_shift_minus_permille,
                x_mag_shift_plus_permille: evidence.estimate.x_mag_shift_plus_permille,
                x_mag_vs_quad_permille: evidence.estimate.x_mag_vs_quad_permille,
            },
        ]
    }

    #[test]
    fn commission_result_emits_current_loop_evidence_before_terminal_result() {
        let mut sink = RecordingSink::default();
        emit_terminal_reply(
            &mut sink,
            ReplyPayload::CommissionResult {
                oid: 6,
                status: 7,
                result: commission_result(),
            },
        );

        let mut expected = std::vec![
            Event::ResistanceAxis {
                oid: 6,
                electrical_axis: 0,
                phi_e_ext: 0,
                r_count_slope_milli: 1040,
                intercept_count: 5,
                rmse_permille: 7,
                selected_mask: 0b1111_1000,
                signed_count_slope_milli: 1045,
                signed_asymmetry_permille: 9,
                drift_permille: 11,
            },
            Event::ResistanceAxis {
                oid: 6,
                electrical_axis: 1,
                phi_e_ext: 16_384,
                r_count_slope_milli: 1060,
                intercept_count: 6,
                rmse_permille: 8,
                selected_mask: 0b0111_1100,
                signed_count_slope_milli: 1055,
                signed_asymmetry_permille: 10,
                drift_permille: 12,
            },
            Event::ResistanceRun {
                oid: 6,
                status: 0,
                selected_r_count_slope_milli: 1050,
                gain_path_count_slope_milli: 16,
                warning_flags: 0,
                status_flags_or: 0,
            },
            Event::EncoderAlignment {
                oid: 6,
                encoder_count: 163,
                electrical_residual_counts: 3,
                stability_counts: 1,
                movement_counts: 37,
                min_movement_counts: 2,
                counts_per_electrical_rev: 80,
            },
        ];
        expected.extend(inductance_events(6));
        expected.extend(current_loop_events(6, 7));
        expected.push(Event::Commission {
            oid: 6,
            status: 7,
            r_int: 11,
            fallback_velocity_limit: 35,
        });

        assert_eq!(sink.events, expected);
    }

    #[test]
    fn current_loop_hold_emits_before_current_loop_run() {
        let mut sink = RecordingSink::default();
        let mut result = commission_result();
        result.inner.current_hold_status = crate::autotune::HOLD_GATE_STATUS_PASS;
        result.inner.current_hold_sample_count = 250;
        result.inner.current_hold_torque_rms_count = 12;
        result.inner.current_hold_flux_rms_count = 9;

        emit_terminal_reply(
            &mut sink,
            ReplyPayload::CommissionResult {
                oid: 6,
                status: 0,
                result,
            },
        );

        let hold_index = sink
            .events
            .iter()
            .position(|e| matches!(e, Event::CurrentLoopHold { .. }))
            .expect("current loop hold event");
        let run_index = sink
            .events
            .iter()
            .position(|e| matches!(e, Event::CurrentLoopRun { .. }))
            .expect("current loop run event");
        assert!(hold_index < run_index);
    }

    #[test]
    fn current_loop_filters_emit_before_current_loop_run() {
        let mut sink = RecordingSink::default();
        let mut result = commission_result();
        result.velocity_filter_hz = 0;
        result.torque_filter_hz = 3000;
        result.position_filter_hz = 0;
        result.flux_filter_hz = 3000;

        emit_terminal_reply(
            &mut sink,
            ReplyPayload::CommissionResult {
                oid: 6,
                status: 0,
                result,
            },
        );

        let filters_index = sink
            .events
            .iter()
            .position(|e| {
                matches!(
                    e,
                    Event::CurrentLoopFilters {
                        oid: 6,
                        velocity_filter_hz: 0,
                        torque_filter_hz: 3000,
                        position_filter_hz: 0,
                        flux_filter_hz: 3000,
                    }
                )
            })
            .expect("current loop filters event");
        let run_index = sink
            .events
            .iter()
            .position(|e| matches!(e, Event::CurrentLoopRun { .. }))
            .expect("current loop run event");
        assert!(filters_index < run_index);
    }

    #[test]
    fn commission_result_emits_inductance_evidence_before_terminal_result() {
        let mut sink = RecordingSink::default();
        let mut result = commission_result();
        result.inner.inductance = crate::commissioning::types::InductanceEvidence::test_fixture();

        emit_terminal_reply(
            &mut sink,
            ReplyPayload::CommissionResult {
                oid: 7,
                status: 0,
                result,
            },
        );

        let alignment_index = sink
            .events
            .iter()
            .position(|e| matches!(e, Event::EncoderAlignment { .. }))
            .expect("encoder alignment event");
        let run_index = sink
            .events
            .iter()
            .position(|e| matches!(e, Event::InductanceRun { .. }))
            .expect("inductance run event");
        let frame_index = sink
            .events
            .iter()
            .position(|e| matches!(e, Event::InductanceFrame { .. }))
            .expect("inductance frame event");
        let estimate_index = sink
            .events
            .iter()
            .position(|e| matches!(e, Event::InductanceEstimate { .. }))
            .expect("inductance estimate event");
        let current_loop_index = sink
            .events
            .iter()
            .position(|e| matches!(e, Event::CurrentLoopRun { .. }))
            .expect("current loop run event");
        let terminal_index = sink
            .events
            .iter()
            .position(|e| matches!(e, Event::Commission { .. }))
            .expect("terminal commission event");
        assert!(alignment_index < run_index);
        assert!(run_index < frame_index);
        assert!(frame_index < estimate_index);
        assert!(estimate_index < current_loop_index);
        assert!(current_loop_index < terminal_index);
    }

    #[test]
    fn commission_result_skips_empty_inductance_evidence() {
        let mut sink = RecordingSink::default();
        let mut result = commission_result();
        result.inner.inductance = InductanceEvidence::EMPTY;

        emit_terminal_reply(
            &mut sink,
            ReplyPayload::CommissionResult {
                oid: 7,
                status: 0,
                result,
            },
        );

        assert!(
            !sink
                .events
                .iter()
                .any(|e| matches!(e, Event::InductanceRun { .. }))
        );
        assert!(
            !sink
                .events
                .iter()
                .any(|e| matches!(e, Event::InductanceFrame { .. }))
        );
        assert!(
            !sink
                .events
                .iter()
                .any(|e| matches!(e, Event::InductanceEstimate { .. }))
        );
    }

    #[test]
    fn failed_commission_result_emits_current_loop_failure_before_terminal_result() {
        let mut result = commission_result();
        result.inner.status = CommissionStatus::Failed(CommissionError::CurrentValidationFailed);
        result.inner.current_gains_source = CURRENT_GAINS_SOURCE_FAILED;
        result.inner.gains_tier = CURRENT_GAINS_TIER_NONE;
        result.inner.candidate_gains_source = CURRENT_GAINS_SOURCE_MEASURED;
        result.inner.candidate_axis_split_source = AXIS_SPLIT_SOURCE_NONE;
        result.inner.candidate_gains_tier = CURRENT_GAINS_TIER_MEASURED_SYMMETRIC;
        result.inner.candidate_flux_p = 711;
        result.inner.candidate_flux_i = 26;
        result.inner.candidate_torque_p = 711;
        result.inner.candidate_torque_i = 26;
        result.inner.candidate_attempt = 1;
        result.inner.current_validation_axes = CURRENT_VALIDATION_AXIS_FLUX;
        result.inner.retry_budget_exhausted = 1;
        result.inner.current_loop_failure_reason = CURRENT_LOOP_FAILURE_SATURATION;
        let mut sink = RecordingSink::default();

        emit_terminal_reply(
            &mut sink,
            ReplyPayload::CommissionResult {
                oid: 6,
                status: CommissionError::CurrentValidationFailed.status_code(),
                result,
            },
        );

        let current_loop_index = sink.events.iter().position(|event| {
            matches!(
                event,
                Event::CurrentLoopRun {
                    status: 9,
                    gains_source: CURRENT_GAINS_SOURCE_FAILED,
                    candidate_gains_source: CURRENT_GAINS_SOURCE_MEASURED,
                    candidate_axis_split_source: AXIS_SPLIT_SOURCE_NONE,
                    gains_tier: CURRENT_GAINS_TIER_NONE,
                    candidate_gains_tier: CURRENT_GAINS_TIER_MEASURED_SYMMETRIC,
                    candidate_flux_p: 711,
                    candidate_flux_i: 26,
                    candidate_torque_p: 711,
                    candidate_torque_i: 26,
                    candidate_attempt: 1,
                    current_validation_axes: CURRENT_VALIDATION_AXIS_FLUX,
                    retry_budget_exhausted: 1,
                    failure_reason: CURRENT_LOOP_FAILURE_SATURATION,
                    ..
                }
            )
        });
        let terminal_index = sink
            .events
            .iter()
            .position(|event| matches!(event, Event::Commission { status: 9, .. }));

        assert!(current_loop_index.is_some());
        assert!(terminal_index.is_some());
        assert!(current_loop_index < terminal_index);
    }

    #[test]
    fn commission_phase_emits_details_before_phase_reply() {
        let mut sink = RecordingSink::default();
        emit_terminal_reply(
            &mut sink,
            ReplyPayload::CommissionPhase {
                oid: 7,
                phase: 9,
                status: 3,
                diagnostics: diagnostics([diagnostic(4, 1), diagnostic(5, 2)]),
            },
        );

        assert_eq!(
            sink.events,
            std::vec![
                Event::Detail {
                    oid: 7,
                    phase: 4,
                    code: 1,
                    status: 1,
                    value0: 101,
                    value1: 201,
                    value2: 301,
                },
                Event::Detail {
                    oid: 7,
                    phase: 5,
                    code: 2,
                    status: 2,
                    value0: 102,
                    value1: 202,
                    value2: 302,
                },
                Event::Phase {
                    oid: 7,
                    phase: 9,
                    status: 3,
                },
            ]
        );
    }

    #[test]
    fn selftest_emits_details_results_then_done() {
        let mut results = SelftestResults::default();
        results.set(
            1,
            SelftestStageValue {
                status: 0,
                value: 111,
            },
        );
        results.set(
            3,
            SelftestStageValue {
                status: 2,
                value: 333,
            },
        );

        let mut sink = RecordingSink::default();
        emit_terminal_reply(
            &mut sink,
            ReplyPayload::Selftest {
                oid: 8,
                results,
                diagnostics: diagnostics([diagnostic(6, 3)]),
                done_status: 5,
            },
        );

        assert_eq!(
            sink.events,
            std::vec![
                Event::Detail {
                    oid: 8,
                    phase: 6,
                    code: 3,
                    status: 0,
                    value0: 103,
                    value1: 203,
                    value2: 303,
                },
                Event::SelftestResult {
                    oid: 8,
                    stage: 1,
                    status: 0,
                    value: 111,
                },
                Event::SelftestResult {
                    oid: 8,
                    stage: 3,
                    status: 2,
                    value: 333,
                },
                Event::SelftestDone { oid: 8, status: 5 },
            ]
        );
    }

    #[test]
    fn diagnostics_emit_only_retained_capacity() {
        let mut diagnostics = CommissionDiagnostics::default();
        for code in 0..(COMMISSION_DIAGNOSTIC_CAPACITY as u8 + 2) {
            diagnostics.push(diagnostic(2, code));
        }

        let mut sink = RecordingSink::default();
        emit_terminal_reply(
            &mut sink,
            ReplyPayload::CommissionPhase {
                oid: 9,
                phase: 2,
                status: 0,
                diagnostics,
            },
        );

        let detail_codes: Vec<u8> = sink
            .events
            .iter()
            .filter_map(|event| match event {
                Event::Detail { code, .. } => Some(*code),
                _ => None,
            })
            .collect();
        assert_eq!(detail_codes.len(), COMMISSION_DIAGNOSTIC_CAPACITY);
        assert_eq!(detail_codes[COMMISSION_DIAGNOSTIC_CAPACITY - 1], 15);
    }

    #[test]
    fn empty_diagnostics_still_emit_terminal_replies() {
        let mut sink = RecordingSink::default();
        emit_terminal_reply(
            &mut sink,
            ReplyPayload::Selftest {
                oid: 10,
                results: SelftestResults::default(),
                diagnostics: CommissionDiagnostics::default(),
                done_status: 0,
            },
        );

        assert_eq!(
            sink.events,
            std::vec![Event::SelftestDone { oid: 10, status: 0 }]
        );
    }

    #[test]
    fn result_variants_emit_single_result_events() {
        let mut sink = RecordingSink::default();
        emit_terminal_reply(
            &mut sink,
            ReplyPayload::CalibrateResult {
                oid: 1,
                status: 2,
                result: AdcCalResult {
                    adc_i0: 3,
                    adc_i1: 4,
                    encoder_count: 5,
                    encoder_alignment_electrical_residual_counts: 0,
                    encoder_alignment_stability_counts: 0,
                    encoder_alignment_movement_counts: 0,
                    encoder_alignment_min_movement_counts: 0,
                    encoder_alignment_counts_per_electrical_rev: 0,
                },
            },
        );
        emit_terminal_reply(
            &mut sink,
            ReplyPayload::CommissionResult {
                oid: 6,
                status: 7,
                result: commission_result(),
            },
        );
        emit_terminal_reply(
            &mut sink,
            ReplyPayload::TuneResult {
                oid: 8,
                status: 9,
                result: outer_result(),
            },
        );

        let mut expected = std::vec![
            Event::Calibrate {
                oid: 1,
                status: 2,
                adc_i0: 3,
                adc_i1: 4,
                encoder_count: 5,
            },
            Event::ResistanceAxis {
                oid: 6,
                electrical_axis: 0,
                phi_e_ext: 0,
                r_count_slope_milli: 1040,
                intercept_count: 5,
                rmse_permille: 7,
                selected_mask: 0b1111_1000,
                signed_count_slope_milli: 1045,
                signed_asymmetry_permille: 9,
                drift_permille: 11,
            },
            Event::ResistanceAxis {
                oid: 6,
                electrical_axis: 1,
                phi_e_ext: 16_384,
                r_count_slope_milli: 1060,
                intercept_count: 6,
                rmse_permille: 8,
                selected_mask: 0b0111_1100,
                signed_count_slope_milli: 1055,
                signed_asymmetry_permille: 10,
                drift_permille: 12,
            },
            Event::ResistanceRun {
                oid: 6,
                status: 0,
                selected_r_count_slope_milli: 1050,
                gain_path_count_slope_milli: 16,
                warning_flags: 0,
                status_flags_or: 0,
            },
            Event::EncoderAlignment {
                oid: 6,
                encoder_count: 163,
                electrical_residual_counts: 3,
                stability_counts: 1,
                movement_counts: 37,
                min_movement_counts: 2,
                counts_per_electrical_rev: 80,
            },
        ];
        expected.extend(inductance_events(6));
        expected.extend(current_loop_events(6, 7));
        expected.extend([
            Event::Commission {
                oid: 6,
                status: 7,
                r_int: 11,
                fallback_velocity_limit: 35,
            },
            Event::Tune {
                oid: 8,
                status: 9,
                velocity_p: 41,
                position_p: 44,
                warning_code: 52,
            },
        ]);

        assert_eq!(sink.events, expected);
    }

    #[test]
    fn phase_helper_emits_zero_status_phase_reply() {
        let mut sink = RecordingSink::default();
        emit_phase_reply(&mut sink, 12, PhaseId::ClosedLoopEntry);

        assert_eq!(
            sink.events,
            std::vec![Event::Phase {
                oid: 12,
                phase: 17,
                status: 0,
            }]
        );
    }
}
