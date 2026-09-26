//! Shared active diagnostic reply emission.

use super::{
    CurrentStepReply, CurrentStepReport, CurrentTorqueSampleReport, DiagnosticReport,
    ResistanceAxisReport, ResistanceProfileReport, ResistanceRunReport, VoltageStepReport,
};
use crate::tmc::CurrentStepAxis;

/// Board-local active diagnostic reply sink.
pub trait DiagnosticReplySink {
    /// Emit a scalar current-step diagnostic result.
    fn foci_current_step_result(
        &mut self,
        report_seq: u8,
        axis: CurrentStepAxis,
        report: CurrentStepReport,
    );

    /// Emit a vector current-step diagnostic result.
    fn foci_current_vector_step_result(&mut self, report_seq: u8, report: CurrentStepReport);

    /// Emit a current torque sample PID result.
    fn foci_current_torque_sample_pid_result(
        &mut self,
        report_seq: u8,
        report: &CurrentTorqueSampleReport,
    );

    /// Emit a current torque sample summary result.
    fn foci_current_torque_sample_result(
        &mut self,
        report_seq: u8,
        report: CurrentTorqueSampleReport,
    );

    /// Emit a current torque sample detail result.
    fn foci_current_torque_sample_detail_result(
        &mut self,
        report_seq: u8,
        report: &CurrentTorqueSampleReport,
    );

    /// Emit an open-loop voltage-step diagnostic result.
    fn foci_voltage_step_result(&mut self, report_seq: u8, report: VoltageStepReport);

    /// Emit an open-loop voltage-step encoder detail fragment.
    ///
    /// Emitted before the terminal, only when the step succeeded.
    fn foci_voltage_step_detail_result(&mut self, report_seq: u8, report: &VoltageStepReport);

    /// Emit resistance-test board/profile metadata.
    fn foci_resistance_profile(&mut self, report_seq: u8, report: ResistanceProfileReport);

    /// Emit a resistance-test per-axis fit result.
    fn foci_resistance_axis(&mut self, report_seq: u8, report: ResistanceAxisReport);

    /// Emit the resistance-test per-run status/warnings/readbacks result.
    /// This is the diagnostic's terminal reply.
    fn foci_resistance_run(&mut self, report_seq: u8, report: ResistanceRunReport);
}

/// Emit active diagnostic replies for a completed diagnostic report.
///
/// `channel` is the channel of the `TmcRequest` the report answers. One
/// report sequence value is drawn for the whole report and shared by every
/// fragment and its terminal reply.
pub fn emit_diagnostic_report<S: DiagnosticReplySink>(
    sink: &mut S,
    channel: u8,
    report: DiagnosticReport,
) {
    let report_seq = crate::state::report_seq::next_report_seq(channel);
    match report {
        DiagnosticReport::CurrentStep { reply, report } => match reply {
            CurrentStepReply::Vector => sink.foci_current_vector_step_result(report_seq, report),
            CurrentStepReply::Scalar(axis) => {
                sink.foci_current_step_result(report_seq, axis, report)
            }
        },
        DiagnosticReport::CurrentTorqueSample(report) => {
            if report.status == 0 {
                sink.foci_current_torque_sample_pid_result(report_seq, &report);
                sink.foci_current_torque_sample_detail_result(report_seq, &report);
            }
            sink.foci_current_torque_sample_result(report_seq, report);
        }
        DiagnosticReport::VoltageStep(report) => {
            if report.status == 0 {
                sink.foci_voltage_step_detail_result(report_seq, &report);
            }
            sink.foci_voltage_step_result(report_seq, report);
        }
        DiagnosticReport::ResistanceTest { profile, run, axes } => {
            sink.foci_resistance_profile(report_seq, profile);
            if let Some([axis0, axis1]) = axes {
                sink.foci_resistance_axis(report_seq, axis0);
                sink.foci_resistance_axis(report_seq, axis1);
            }
            // Run reply last: it is the diagnostic's terminal/"done" signal.
            sink.foci_resistance_run(report_seq, run);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tmc::TorqueSampleKind;
    use crate::tmc_diagnostics::{
        CURRENT_STEP_STATUS_HOST_LIVENESS, CURRENT_STEP_STATUS_NOT_READY, CURRENT_STEP_STATUS_SPI,
    };
    use foci_firmware_macros::fixture_test;
    use std::vec::Vec;

    #[derive(Debug, Clone, PartialEq, Eq)]
    enum Event {
        CurrentStep {
            oid: u8,
            report_seq: u8,
            status: u8,
            target: i16,
            axis: CurrentStepAxis,
        },
        CurrentVectorStep {
            oid: u8,
            report_seq: u8,
            status: u8,
            torque_target: i16,
            flux_target: i16,
        },
        TorquePid {
            oid: u8,
            report_seq: u8,
            torque_error_sum: i32,
        },
        TorqueDetail {
            oid: u8,
            report_seq: u8,
            encoder_sample: u32,
        },
        TorqueSummary {
            oid: u8,
            report_seq: u8,
            kind: TorqueSampleKind,
            status: u8,
            target: i16,
            status_flags: u32,
        },
        VoltageStep {
            oid: u8,
            report_seq: u8,
            status: u8,
            uq_ext: i16,
            status_flags: u32,
        },
        VoltageStepDetail {
            oid: u8,
            report_seq: u8,
            encoder_sample: u32,
        },
        ResistanceProfile {
            oid: u8,
            report_seq: u8,
        },
        ResistanceAxis {
            oid: u8,
            report_seq: u8,
            electrical_axis: u8,
        },
        ResistanceDone {
            oid: u8,
            report_seq: u8,
            status: u8,
            peak_abs_current_count: u16,
            max_abs_steady_mean_current_count: u16,
            current_ceiling_count: u16,
            power_stage_tripped: bool,
        },
    }

    #[derive(Default)]
    struct RecordingSink {
        events: Vec<Event>,
    }

    impl DiagnosticReplySink for RecordingSink {
        fn foci_current_step_result(
            &mut self,
            report_seq: u8,
            axis: CurrentStepAxis,
            report: CurrentStepReport,
        ) {
            self.events.push(Event::CurrentStep {
                oid: report.oid,
                report_seq,
                status: report.status,
                target: report.torque_target,
                axis,
            });
        }

        fn foci_current_vector_step_result(&mut self, report_seq: u8, report: CurrentStepReport) {
            self.events.push(Event::CurrentVectorStep {
                oid: report.oid,
                report_seq,
                status: report.status,
                torque_target: report.torque_target,
                flux_target: report.flux_target,
            });
        }

        fn foci_current_torque_sample_pid_result(
            &mut self,
            report_seq: u8,
            report: &CurrentTorqueSampleReport,
        ) {
            self.events.push(Event::TorquePid {
                oid: report.oid,
                report_seq,
                torque_error_sum: report.torque_error_sum,
            });
        }

        fn foci_current_torque_sample_result(
            &mut self,
            report_seq: u8,
            report: CurrentTorqueSampleReport,
        ) {
            self.events.push(Event::TorqueSummary {
                oid: report.oid,
                report_seq,
                kind: report.kind,
                status: report.status,
                target: report.target,
                status_flags: report.status_flags,
            });
        }

        fn foci_current_torque_sample_detail_result(
            &mut self,
            report_seq: u8,
            report: &CurrentTorqueSampleReport,
        ) {
            self.events.push(Event::TorqueDetail {
                oid: report.oid,
                report_seq,
                encoder_sample: report.encoder_sample,
            });
        }

        fn foci_voltage_step_result(&mut self, report_seq: u8, report: VoltageStepReport) {
            self.events.push(Event::VoltageStep {
                oid: report.oid,
                report_seq,
                status: report.status,
                uq_ext: report.uq_ext,
                status_flags: report.status_flags,
            });
        }

        fn foci_voltage_step_detail_result(&mut self, report_seq: u8, report: &VoltageStepReport) {
            self.events.push(Event::VoltageStepDetail {
                oid: report.oid,
                report_seq,
                encoder_sample: report.encoder_sample,
            });
        }

        fn foci_resistance_profile(&mut self, report_seq: u8, report: ResistanceProfileReport) {
            self.events.push(Event::ResistanceProfile {
                oid: report.oid,
                report_seq,
            });
        }

        fn foci_resistance_axis(&mut self, report_seq: u8, report: ResistanceAxisReport) {
            self.events.push(Event::ResistanceAxis {
                oid: report.oid,
                report_seq,
                electrical_axis: report.electrical_axis,
            });
        }

        fn foci_resistance_run(&mut self, report_seq: u8, report: ResistanceRunReport) {
            self.events.push(Event::ResistanceDone {
                oid: report.oid,
                report_seq,
                status: report.status,
                peak_abs_current_count: report.peak_abs_current_count,
                max_abs_steady_mean_current_count: report.max_abs_steady_mean_current_count,
                current_ceiling_count: report.current_ceiling_count,
                power_stage_tripped: report.power_stage_tripped,
            });
        }
    }

    fn current_step_report() -> CurrentStepReport {
        CurrentStepReport {
            oid: 7,
            status: 0,
            torque_target: -123,
            flux_target: 45,
            duration_ms: 6,
            voltage_limit: 700,
            torque_before: 1,
            torque_during: 2,
            torque_after: 3,
            flux_during: 4,
            iq_during: 5,
            id_during: 6,
            uq_limited: 7,
            ud_limited: 8,
            encoder_before: 9,
            encoder_after: 10,
            encoder_delta: 11,
            adc_vm_raw: 12,
        }
    }

    fn torque_report() -> CurrentTorqueSampleReport {
        CurrentTorqueSampleReport {
            oid: 8,
            kind: TorqueSampleKind::CurrentTorqueSample,
            status: 0,
            target: 250,
            flux_target: -125,
            sample_delay_ms: 3,
            voltage_limit: 900,
            torque_before: 1,
            torque_sample: 2,
            torque_after: 3,
            flux_sample: 4,
            iq_sample: 5,
            id_sample: 6,
            uq_limited: 7,
            ud_limited: 8,
            encoder_before: 9,
            encoder_sample: 10,
            encoder_after: 11,
            encoder_delta_sample: 12,
            encoder_delta_after: 13,
            adc_vm_raw: 14,
            pidin_target_torque: 15,
            pidin_target_flux: 16,
            pidout_target_torque: 17,
            pidout_target_flux: 18,
            pid_torque_target_monitor: 19,
            torque_error: 20,
            flux_error: 21,
            torque_error_sum: 22,
            flux_error_sum: 23,
            uq_prelimit: 24,
            ud_prelimit: 25,
            ff_velocity: 26,
            ff_torque: 27,
            status_flags: 0x1234,
        }
    }

    fn voltage_report() -> VoltageStepReport {
        VoltageStepReport {
            oid: 9,
            status: 2,
            uq_ext: -300,
            ud_ext: 150,
            sample_delay_ms: 4,
            torque_before: 1,
            torque_sample: 2,
            torque_after: 3,
            flux_sample: 4,
            iq_sample: 5,
            id_sample: 6,
            uq_limited: 7,
            ud_limited: 8,
            uux_sample: 9,
            uwy_sample: 10,
            pwm_ux_sample: 11,
            pwm_wy_sample: 12,
            pwm_sv_chop: 13,
            pwm_bbm: 14,
            pwm_maxcnt: 15,
            phi_e_sample: 16,
            phi_m_sample: 17,
            encoder_before: 18,
            encoder_sample: 19,
            encoder_after: 20,
            encoder_delta_sample: 21,
            encoder_delta_after: 22,
            adc_vm_raw: 23,
            status_flags: 0x5678,
        }
    }

    #[fixture_test]
    fn scalar_current_step_report_emits_current_step_reply() {
        crate::state::report_seq::reset_report_seq();
        let mut sink = RecordingSink::default();

        emit_diagnostic_report(
            &mut sink,
            0,
            DiagnosticReport::CurrentStep {
                reply: CurrentStepReply::Scalar(CurrentStepAxis::Torque),
                report: current_step_report(),
            },
        );

        assert_eq!(
            sink.events,
            std::vec![Event::CurrentStep {
                oid: 7,
                report_seq: 0,
                status: 0,
                target: -123,
                axis: CurrentStepAxis::Torque,
            }]
        );
    }

    #[fixture_test]
    fn scalar_current_step_report_emits_its_axis() {
        for axis in [CurrentStepAxis::Torque, CurrentStepAxis::Flux] {
            crate::state::report_seq::reset_report_seq();
            let mut sink = RecordingSink::default();

            emit_diagnostic_report(
                &mut sink,
                0,
                DiagnosticReport::CurrentStep {
                    reply: CurrentStepReply::Scalar(axis),
                    report: current_step_report(),
                },
            );

            assert_eq!(
                sink.events,
                std::vec![Event::CurrentStep {
                    oid: 7,
                    report_seq: 0,
                    status: 0,
                    target: -123,
                    axis
                }]
            );
        }
    }

    #[fixture_test]
    fn vector_current_step_report_emits_vector_reply() {
        crate::state::report_seq::reset_report_seq();
        let mut sink = RecordingSink::default();

        emit_diagnostic_report(
            &mut sink,
            0,
            DiagnosticReport::CurrentStep {
                reply: CurrentStepReply::Vector,
                report: current_step_report(),
            },
        );

        assert_eq!(
            sink.events,
            std::vec![Event::CurrentVectorStep {
                oid: 7,
                report_seq: 0,
                status: 0,
                torque_target: -123,
                flux_target: 45,
            }]
        );
    }

    #[fixture_test]
    fn successful_torque_sample_emits_pid_then_detail_then_terminal() {
        crate::state::report_seq::reset_report_seq();
        let mut sink = RecordingSink::default();

        emit_diagnostic_report(
            &mut sink,
            0,
            DiagnosticReport::CurrentTorqueSample(torque_report()),
        );

        assert_eq!(
            sink.events,
            std::vec![
                Event::TorquePid {
                    oid: 8,
                    report_seq: 0,
                    torque_error_sum: 22
                },
                Event::TorqueDetail {
                    oid: 8,
                    report_seq: 0,
                    encoder_sample: 10
                },
                Event::TorqueSummary {
                    oid: 8,
                    report_seq: 0,
                    kind: TorqueSampleKind::CurrentTorqueSample,
                    status: 0,
                    target: 250,
                    status_flags: 0x1234,
                },
            ]
        );
    }

    #[fixture_test]
    fn torque_sample_fragments_share_the_terminal_seq() {
        crate::state::report_seq::reset_report_seq();
        let mut sink = RecordingSink::default();

        // Draw an unused seq first so the shared value isn't the default 0,
        // which would also pass if the seq were accidentally never threaded.
        let _ = crate::state::report_seq::next_report_seq(0);

        emit_diagnostic_report(
            &mut sink,
            0,
            DiagnosticReport::CurrentTorqueSample(torque_report()),
        );

        let seqs: std::vec::Vec<u8> = sink
            .events
            .iter()
            .map(|event| match event {
                Event::TorquePid { report_seq, .. }
                | Event::TorqueDetail { report_seq, .. }
                | Event::TorqueSummary { report_seq, .. } => *report_seq,
                other => panic!("unexpected event {other:?}"),
            })
            .collect();

        assert_eq!(seqs, std::vec![1, 1, 1]);
    }

    #[fixture_test]
    fn failed_torque_sample_emits_terminal_only() {
        for status in [
            CURRENT_STEP_STATUS_NOT_READY,
            CURRENT_STEP_STATUS_SPI,
            CURRENT_STEP_STATUS_HOST_LIVENESS,
        ] {
            crate::state::report_seq::reset_report_seq();
            let mut sink = RecordingSink::default();
            let report = CurrentTorqueSampleReport {
                status,
                ..torque_report()
            };

            emit_diagnostic_report(&mut sink, 0, DiagnosticReport::CurrentTorqueSample(report));

            assert!(matches!(
                sink.events.as_slice(),
                [Event::TorqueSummary { .. }]
            ));
        }
    }

    #[fixture_test]
    fn voltage_step_report_emits_voltage_reply() {
        crate::state::report_seq::reset_report_seq();
        let mut sink = RecordingSink::default();

        emit_diagnostic_report(
            &mut sink,
            0,
            DiagnosticReport::VoltageStep(voltage_report()),
        );

        assert_eq!(
            sink.events,
            std::vec![Event::VoltageStep {
                oid: 9,
                report_seq: 0,
                status: 2,
                uq_ext: -300,
                status_flags: 0x5678,
            }]
        );
    }

    #[fixture_test]
    fn voltage_step_success_emits_detail_then_terminal_with_one_seq() {
        crate::state::report_seq::reset_report_seq();
        let mut sink = RecordingSink::default();
        let report = VoltageStepReport {
            status: 0,
            ..voltage_report()
        };

        emit_diagnostic_report(&mut sink, 0, DiagnosticReport::VoltageStep(report));

        assert_eq!(
            sink.events,
            std::vec![
                Event::VoltageStepDetail {
                    oid: 9,
                    report_seq: 0,
                    encoder_sample: 19,
                },
                Event::VoltageStep {
                    oid: 9,
                    report_seq: 0,
                    status: 0,
                    uq_ext: -300,
                    status_flags: 0x5678,
                },
            ]
        );
    }

    #[fixture_test]
    fn voltage_step_failure_emits_terminal_only() {
        for status in [
            CURRENT_STEP_STATUS_NOT_READY,
            CURRENT_STEP_STATUS_SPI,
            CURRENT_STEP_STATUS_HOST_LIVENESS,
        ] {
            crate::state::report_seq::reset_report_seq();
            let mut sink = RecordingSink::default();
            let report = VoltageStepReport {
                status,
                ..voltage_report()
            };

            emit_diagnostic_report(&mut sink, 0, DiagnosticReport::VoltageStep(report));

            assert!(matches!(
                sink.events.as_slice(),
                [Event::VoltageStep { .. }]
            ));
        }
    }

    #[fixture_test]
    fn consecutive_reports_on_a_channel_advance_the_seq() {
        crate::state::report_seq::reset_report_seq();
        let mut sink = RecordingSink::default();

        emit_diagnostic_report(
            &mut sink,
            0,
            DiagnosticReport::CurrentStep {
                reply: CurrentStepReply::Vector,
                report: current_step_report(),
            },
        );
        emit_diagnostic_report(
            &mut sink,
            0,
            DiagnosticReport::CurrentStep {
                reply: CurrentStepReply::Vector,
                report: current_step_report(),
            },
        );
        emit_diagnostic_report(
            &mut sink,
            1,
            DiagnosticReport::CurrentStep {
                reply: CurrentStepReply::Vector,
                report: current_step_report(),
            },
        );

        let seqs: std::vec::Vec<u8> = sink
            .events
            .iter()
            .map(|event| match event {
                Event::CurrentVectorStep { report_seq, .. } => *report_seq,
                other => panic!("unexpected event {other:?}"),
            })
            .collect();

        assert_eq!(seqs, std::vec![0, 1, 0]);
    }

    fn resistance_profile_report() -> ResistanceProfileReport {
        ResistanceProfileReport {
            oid: 7,
            pwm_maxcnt: 3999,
            bbm_h: 9,
            bbm_l: 9,
            dsadc_mdec_a: 331,
            dsadc_mdec_b: 331,
            linear_current_threshold_count: 750,
            encoder_move_warn_counts: 128,
            status_flags_warn_mask: u32::MAX,
            scale_metadata_validated: true,
        }
    }

    fn resistance_axis_report(electrical_axis: u8) -> ResistanceAxisReport {
        ResistanceAxisReport {
            oid: 7,
            electrical_axis,
            phi_e_ext: if electrical_axis == 0 { 0 } else { 16_384 },
            r_count_slope_milli: 1010,
            intercept_count: 5,
            rmse_permille: 12,
            selected_mask: 0x00FF,
            excluded_point_mask: 0,
            selected_count: 8,
            signed_count_slope_milli: 1005,
            signed_asymmetry_permille: 10,
            drift_permille: 5,
            warning_flags: 0,
        }
    }

    fn resistance_run_report(status: u8) -> ResistanceRunReport {
        ResistanceRunReport {
            oid: 7,
            status,
            selected_r_count_slope_milli: 1010,
            gain_path_count_slope_milli: 0,
            warning_flags: 0,
            status_flags_or: 0,
            peak_abs_current_count: 1200,
            max_abs_steady_mean_current_count: 900,
            current_ceiling_count: 1600,
            power_stage_tripped: status != 0,
            pwm_maxcnt_readback: 3999,
            bbm_readback: 0x0909,
            dsadc_mdec_readback: 0x014B_014B,
            pwm_sv_chop_readback: 7,
        }
    }

    #[fixture_test]
    fn resistance_success_emits_profile_then_axes_then_run_last() {
        crate::state::report_seq::reset_report_seq();
        let mut sink = RecordingSink::default();

        emit_diagnostic_report(
            &mut sink,
            0,
            DiagnosticReport::ResistanceTest {
                profile: resistance_profile_report(),
                run: resistance_run_report(0),
                axes: Some([resistance_axis_report(0), resistance_axis_report(1)]),
            },
        );

        assert_eq!(
            sink.events,
            std::vec![
                Event::ResistanceProfile {
                    oid: 7,
                    report_seq: 0
                },
                Event::ResistanceAxis {
                    oid: 7,
                    report_seq: 0,
                    electrical_axis: 0,
                },
                Event::ResistanceAxis {
                    oid: 7,
                    report_seq: 0,
                    electrical_axis: 1,
                },
                Event::ResistanceDone {
                    oid: 7,
                    report_seq: 0,
                    status: 0,
                    peak_abs_current_count: 1200,
                    max_abs_steady_mean_current_count: 900,
                    current_ceiling_count: 1600,
                    power_stage_tripped: false,
                },
            ]
        );
    }

    #[fixture_test]
    fn resistance_failure_emits_profile_then_run_without_axes() {
        crate::state::report_seq::reset_report_seq();
        let mut sink = RecordingSink::default();

        emit_diagnostic_report(
            &mut sink,
            0,
            DiagnosticReport::ResistanceTest {
                profile: resistance_profile_report(),
                run: resistance_run_report(19),
                axes: None,
            },
        );

        assert_eq!(
            sink.events,
            std::vec![
                Event::ResistanceProfile {
                    oid: 7,
                    report_seq: 0
                },
                Event::ResistanceDone {
                    oid: 7,
                    report_seq: 0,
                    status: 19,
                    peak_abs_current_count: 1200,
                    max_abs_steady_mean_current_count: 900,
                    current_ceiling_count: 1600,
                    power_stage_tripped: true,
                },
            ]
        );
    }
}
