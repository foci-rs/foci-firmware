//! Shared commissioning reply emission.
//!
//! Board crates keep transport-specific ankyra sender calls behind
//! [`CommissionReplySink`]. This module owns the semantic mapping from terminal
//! commissioning payloads to reply events so boards cannot drift by dropping
//! fields locally.

use crate::autotune::PhaseId;
use crate::commission_dispatch::ReplyPayload;
use crate::commissioning::types::{AdcCalResult, CommissionResult, InnerResult, OuterResult};
use crate::selftest::{CommissionDiagnostic, CommissionDiagnostics, SelftestStageValue};
use crate::tmc_diagnostics::{ResistanceAxisReport, ResistanceRunReport};

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
            // 64-byte Klipper message limit) and cannot carry the
            // resistance-identification evidence that lives on
            // `InnerResult`. Emit it separately through the existing
            // resistance replies (Task 6) so the host can persist it.
            // Axes before run, mirroring the diagnostic path's
            // axes-then-run ordering (`emit_diagnostic_report`); the
            // commission result stays the true terminal/"done" signal for
            // the whole commission flow, so it is emitted last.
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
        profile_version: inner.resistance_profile_version,
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

#[cfg(test)]
mod tests {
    use std::vec::Vec;

    use super::*;
    use crate::autotune::{CommissionStatus, PhaseId};
    use crate::commission_dispatch::ReplyPayload;
    use crate::commissioning::types::{AdcCalResult, CommissionResult, InnerResult, OuterResult};
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
            signed_count_slope_milli: i32,
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
                signed_count_slope_milli: report.signed_count_slope_milli,
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
                lambda_us: 19,
                theta_e_us: 20,
                ringing_count: 21,
                bandwidth_hz: 22,
                tau_e_us: 23,
                tau_e_crosscheck_us: 24,
                tau_residual_permille: 25,
                inner_warning_flags: 26,
                resistance_profile_version: 1,
                resistance_selected_count_slope_milli: 1050,
                resistance_gain_path_count_slope_milli: 16,
                resistance_axis0_count_slope_milli: 1040,
                resistance_axis1_count_slope_milli: 1060,
                resistance_axis0_intercept_count: 5,
                resistance_axis1_intercept_count: 6,
                resistance_axis0_rmse_permille: 7,
                resistance_axis1_rmse_permille: 8,
                resistance_selected_mask_axis0: 0b1111_1000,
                resistance_selected_mask_axis1: 0b1111_1000,
                resistance_axis0_signed_count_slope_milli: 1045,
                resistance_axis1_signed_count_slope_milli: 1055,
                resistance_axis0_signed_asymmetry_permille: 9,
                resistance_axis1_signed_asymmetry_permille: 10,
                resistance_axis0_drift_permille: 11,
                resistance_axis1_drift_permille: 12,
                resistance_status_flags_or: 0,
                resistance_warning_flags: 0,
            },
            fallback_velocity_p: 31,
            fallback_velocity_i: 32,
            fallback_position_p: 33,
            fallback_position_i: 34,
            fallback_velocity_limit: 35,
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

        assert_eq!(
            sink.events,
            std::vec![
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
                    signed_count_slope_milli: 1045,
                },
                Event::ResistanceAxis {
                    oid: 6,
                    electrical_axis: 1,
                    phi_e_ext: 16_384,
                    r_count_slope_milli: 1060,
                    intercept_count: 6,
                    signed_count_slope_milli: 1055,
                },
                Event::ResistanceRun {
                    oid: 6,
                    status: 0,
                    selected_r_count_slope_milli: 1050,
                    gain_path_count_slope_milli: 16,
                    warning_flags: 0,
                    status_flags_or: 0,
                },
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
            ]
        );
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
