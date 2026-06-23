//! Shared active diagnostic reply emission.

use super::{
    CurrentStepReport, CurrentTorqueSampleReport, DiagnosticReport, ImpedanceFitReport,
    ImpedanceProfileReport, ImpedanceRunReport, ResistanceAxisReport, ResistanceProfileReport,
    ResistanceRunReport, VoltageStepReport,
};

/// Board-local active diagnostic reply sink.
pub trait DiagnosticReplySink {
    /// Emit a scalar current-step diagnostic result.
    fn foci_current_step_result(&mut self, report: CurrentStepReport);

    /// Emit a vector current-step diagnostic result.
    fn foci_current_vector_step_result(&mut self, report: CurrentStepReport);

    /// Emit a current torque sample summary result.
    fn foci_current_torque_sample_result(&mut self, report: CurrentTorqueSampleReport);

    /// Emit a current torque sample detail result.
    fn foci_current_torque_sample_detail_result(&mut self, report: &CurrentTorqueSampleReport);

    /// Emit an open-loop voltage-step diagnostic result.
    fn foci_voltage_step_result(&mut self, report: VoltageStepReport);

    /// Emit resistance-test board/profile metadata.
    fn foci_resistance_profile(&mut self, report: ResistanceProfileReport);

    /// Emit a resistance-test per-axis fit result.
    fn foci_resistance_axis(&mut self, report: ResistanceAxisReport);

    /// Emit the resistance-test per-run status/warnings/readbacks result.
    /// This is the diagnostic's terminal reply.
    fn foci_resistance_run(&mut self, report: ResistanceRunReport);

    /// Emit impedance-test board/profile metadata.
    fn foci_impedance_profile(&mut self, report: ImpedanceProfileReport);

    /// Emit one impedance-test per-point fit result.
    fn foci_impedance_fit(&mut self, report: ImpedanceFitReport);

    /// Emit the impedance-test per-run status/warnings result.
    /// This is the diagnostic's terminal reply.
    fn foci_impedance_run(&mut self, report: ImpedanceRunReport);
}

/// Emit active diagnostic replies for a completed diagnostic report.
pub fn emit_diagnostic_report<S: DiagnosticReplySink>(sink: &mut S, report: DiagnosticReport) {
    match report {
        DiagnosticReport::CurrentStep {
            vector_reply,
            report,
        } => {
            if vector_reply {
                sink.foci_current_vector_step_result(report);
            } else {
                sink.foci_current_step_result(report);
            }
        }
        DiagnosticReport::CurrentTorqueSample {
            report,
            emit_detail,
        } => {
            if emit_detail {
                sink.foci_current_torque_sample_detail_result(&report);
            }
            sink.foci_current_torque_sample_result(report);
        }
        DiagnosticReport::VoltageStep(report) => sink.foci_voltage_step_result(report),
        DiagnosticReport::ResistanceTest { profile, run, axes } => {
            sink.foci_resistance_profile(profile);
            if let Some([axis0, axis1]) = axes {
                sink.foci_resistance_axis(axis0);
                sink.foci_resistance_axis(axis1);
            }
            // Run reply last: it is the diagnostic's terminal/"done" signal.
            sink.foci_resistance_run(run);
        }
        DiagnosticReport::ImpedanceTest {
            profile,
            run,
            fits,
            detail,
        } => {
            sink.foci_impedance_profile(profile);
            if detail >= 1 {
                if let Some(fits) = fits {
                    for fit in fits {
                        sink.foci_impedance_fit(fit);
                    }
                }
            }
            // Run reply last: it is the diagnostic's terminal/"done" signal.
            sink.foci_impedance_run(run);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::vec::Vec;

    #[derive(Debug, Clone, PartialEq, Eq)]
    enum Event {
        CurrentStep {
            oid: u8,
            status: u8,
            target: i16,
        },
        CurrentVectorStep {
            oid: u8,
            status: u8,
            torque_target: i16,
            flux_target: i16,
        },
        TorqueDetail {
            oid: u8,
            target: i16,
            torque_error: i32,
            status_flags: u32,
        },
        TorqueSummary {
            oid: u8,
            status: u8,
            target: i16,
            status_flags: u32,
        },
        VoltageStep {
            oid: u8,
            status: u8,
            uq_ext: i16,
            status_flags: u32,
        },
        ResistanceProfile {
            oid: u8,
            profile_version: u16,
        },
        ResistanceAxis {
            oid: u8,
            electrical_axis: u8,
        },
        ResistanceDone {
            oid: u8,
            status: u8,
        },
        ImpedanceProfile {
            oid: u8,
            profile_version: u16,
        },
        ImpedanceFit {
            oid: u8,
            point_index: u8,
            residual_permille: u16,
            coverage_permille: u16,
            scalar_2f_mismatch_permille: u16,
        },
        ImpedanceDone {
            oid: u8,
            status: u8,
            status_flags_or: u32,
            warning_flags: u32,
        },
    }

    #[derive(Default)]
    struct RecordingSink {
        events: Vec<Event>,
    }

    impl DiagnosticReplySink for RecordingSink {
        fn foci_current_step_result(&mut self, report: CurrentStepReport) {
            self.events.push(Event::CurrentStep {
                oid: report.oid,
                status: report.status,
                target: report.torque_target,
            });
        }

        fn foci_current_vector_step_result(&mut self, report: CurrentStepReport) {
            self.events.push(Event::CurrentVectorStep {
                oid: report.oid,
                status: report.status,
                torque_target: report.torque_target,
                flux_target: report.flux_target,
            });
        }

        fn foci_current_torque_sample_result(&mut self, report: CurrentTorqueSampleReport) {
            self.events.push(Event::TorqueSummary {
                oid: report.oid,
                status: report.status,
                target: report.target,
                status_flags: report.status_flags,
            });
        }

        fn foci_current_torque_sample_detail_result(&mut self, report: &CurrentTorqueSampleReport) {
            self.events.push(Event::TorqueDetail {
                oid: report.oid,
                target: report.target,
                torque_error: report.torque_error,
                status_flags: report.status_flags,
            });
        }

        fn foci_voltage_step_result(&mut self, report: VoltageStepReport) {
            self.events.push(Event::VoltageStep {
                oid: report.oid,
                status: report.status,
                uq_ext: report.uq_ext,
                status_flags: report.status_flags,
            });
        }

        fn foci_resistance_profile(&mut self, report: ResistanceProfileReport) {
            self.events.push(Event::ResistanceProfile {
                oid: report.oid,
                profile_version: report.profile_version,
            });
        }

        fn foci_resistance_axis(&mut self, report: ResistanceAxisReport) {
            self.events.push(Event::ResistanceAxis {
                oid: report.oid,
                electrical_axis: report.electrical_axis,
            });
        }

        fn foci_resistance_run(&mut self, report: ResistanceRunReport) {
            self.events.push(Event::ResistanceDone {
                oid: report.oid,
                status: report.status,
            });
        }

        fn foci_impedance_profile(&mut self, report: ImpedanceProfileReport) {
            self.events.push(Event::ImpedanceProfile {
                oid: report.oid,
                profile_version: report.profile_version,
            });
        }

        fn foci_impedance_fit(&mut self, report: ImpedanceFitReport) {
            self.events.push(Event::ImpedanceFit {
                oid: report.oid,
                point_index: report.point_index,
                residual_permille: report.residual_permille,
                coverage_permille: report.coverage_permille,
                scalar_2f_mismatch_permille: report.scalar_2f_mismatch_permille,
            });
        }

        fn foci_impedance_run(&mut self, report: ImpedanceRunReport) {
            self.events.push(Event::ImpedanceDone {
                oid: report.oid,
                status: report.status,
                status_flags_or: report.status_flags_or,
                warning_flags: report.warning_flags,
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

    #[test]
    fn scalar_current_step_report_emits_current_step_reply() {
        let mut sink = RecordingSink::default();

        emit_diagnostic_report(
            &mut sink,
            DiagnosticReport::CurrentStep {
                vector_reply: false,
                report: current_step_report(),
            },
        );

        assert_eq!(
            sink.events,
            std::vec![Event::CurrentStep {
                oid: 7,
                status: 0,
                target: -123,
            }]
        );
    }

    #[test]
    fn vector_current_step_report_emits_vector_reply() {
        let mut sink = RecordingSink::default();

        emit_diagnostic_report(
            &mut sink,
            DiagnosticReport::CurrentStep {
                vector_reply: true,
                report: current_step_report(),
            },
        );

        assert_eq!(
            sink.events,
            std::vec![Event::CurrentVectorStep {
                oid: 7,
                status: 0,
                torque_target: -123,
                flux_target: 45,
            }]
        );
    }

    #[test]
    fn torque_sample_with_detail_emits_detail_before_summary() {
        let mut sink = RecordingSink::default();

        emit_diagnostic_report(
            &mut sink,
            DiagnosticReport::CurrentTorqueSample {
                report: torque_report(),
                emit_detail: true,
            },
        );

        assert_eq!(
            sink.events,
            std::vec![
                Event::TorqueDetail {
                    oid: 8,
                    target: 250,
                    torque_error: 20,
                    status_flags: 0x1234,
                },
                Event::TorqueSummary {
                    oid: 8,
                    status: 0,
                    target: 250,
                    status_flags: 0x1234,
                },
            ]
        );
    }

    #[test]
    fn torque_sample_without_detail_emits_summary_only() {
        let mut sink = RecordingSink::default();

        emit_diagnostic_report(
            &mut sink,
            DiagnosticReport::CurrentTorqueSample {
                report: torque_report(),
                emit_detail: false,
            },
        );

        assert_eq!(
            sink.events,
            std::vec![Event::TorqueSummary {
                oid: 8,
                status: 0,
                target: 250,
                status_flags: 0x1234,
            }]
        );
    }

    #[test]
    fn voltage_step_report_emits_voltage_reply() {
        let mut sink = RecordingSink::default();

        emit_diagnostic_report(&mut sink, DiagnosticReport::VoltageStep(voltage_report()));

        assert_eq!(
            sink.events,
            std::vec![Event::VoltageStep {
                oid: 9,
                status: 2,
                uq_ext: -300,
                status_flags: 0x5678,
            }]
        );
    }

    fn resistance_profile_report() -> ResistanceProfileReport {
        ResistanceProfileReport {
            oid: 7,
            profile_version: 1,
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
            profile_version: 1,
            selected_r_count_slope_milli: 1010,
            gain_path_count_slope_milli: 0,
            warning_flags: 0,
            status_flags_or: 0,
            pwm_maxcnt_readback: 3999,
            bbm_readback: 0x0909,
            dsadc_mdec_readback: 0x014B_014B,
            pwm_sv_chop_readback: 7,
        }
    }

    #[test]
    fn resistance_success_emits_profile_then_axes_then_run_last() {
        let mut sink = RecordingSink::default();

        emit_diagnostic_report(
            &mut sink,
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
                    profile_version: 1,
                },
                Event::ResistanceAxis {
                    oid: 7,
                    electrical_axis: 0,
                },
                Event::ResistanceAxis {
                    oid: 7,
                    electrical_axis: 1,
                },
                Event::ResistanceDone { oid: 7, status: 0 },
            ]
        );
    }

    #[test]
    fn resistance_failure_emits_profile_then_run_without_axes() {
        let mut sink = RecordingSink::default();

        emit_diagnostic_report(
            &mut sink,
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
                    profile_version: 1,
                },
                Event::ResistanceDone { oid: 7, status: 19 },
            ]
        );
    }

    fn impedance_profile_report() -> ImpedanceProfileReport {
        ImpedanceProfileReport {
            oid: 6,
            profile_version: 1,
            point_count: 2,
            sample_count: 128,
            sample_interval_us: 250,
            max_ud: 960,
            status_flags_fail_mask: 0x7000_4480,
            status_flags_warn_mask: 0,
            scale_metadata_validated: false,
        }
    }

    fn impedance_run_report(status: u8) -> ImpedanceRunReport {
        ImpedanceRunReport {
            oid: 6,
            status,
            profile_version: 1,
            selected_axis_max_count_per_ud_milli: 1234,
            selected_axis_min_count_per_ud_milli: 567,
            selected_axis_angle_electrical_counts: 99,
            repeatability_permille: 12,
            warning_flags: 0x24,
            status_flags_or: 0x55aa,
        }
    }

    fn impedance_fit_report(point_index: u8) -> ImpedanceFitReport {
        ImpedanceFitReport {
            oid: 6,
            point_index,
            frequency_millihz: if point_index == 0 { 25_000 } else { 50_000 },
            ud_abs: if point_index == 0 { 640 } else { 960 },
            axis_max_count_per_ud_milli: 1234,
            axis_min_count_per_ud_milli: 567,
            axis_angle_electrical_counts: 99,
            residual_permille: 10 + u16::from(point_index),
            coverage_permille: 900,
            scalar_2f_mismatch_permille: 20 + u16::from(point_index),
            warning_flags: 0x8,
        }
    }

    #[test]
    fn impedance_success_detail_zero_emits_profile_then_run_only() {
        let mut sink = RecordingSink::default();

        emit_diagnostic_report(
            &mut sink,
            DiagnosticReport::ImpedanceTest {
                profile: impedance_profile_report(),
                run: impedance_run_report(0),
                fits: Some([impedance_fit_report(0), impedance_fit_report(1)]),
                detail: 0,
            },
        );

        assert_eq!(
            sink.events,
            std::vec![
                Event::ImpedanceProfile {
                    oid: 6,
                    profile_version: 1,
                },
                Event::ImpedanceDone {
                    oid: 6,
                    status: 0,
                    status_flags_or: 0x55aa,
                    warning_flags: 0x24,
                },
            ]
        );
    }

    #[test]
    fn impedance_success_detail_one_emits_profile_fits_then_run_last() {
        let mut sink = RecordingSink::default();

        emit_diagnostic_report(
            &mut sink,
            DiagnosticReport::ImpedanceTest {
                profile: impedance_profile_report(),
                run: impedance_run_report(0),
                fits: Some([impedance_fit_report(0), impedance_fit_report(1)]),
                detail: 1,
            },
        );

        assert_eq!(
            sink.events,
            std::vec![
                Event::ImpedanceProfile {
                    oid: 6,
                    profile_version: 1,
                },
                Event::ImpedanceFit {
                    oid: 6,
                    point_index: 0,
                    residual_permille: 10,
                    coverage_permille: 900,
                    scalar_2f_mismatch_permille: 20,
                },
                Event::ImpedanceFit {
                    oid: 6,
                    point_index: 1,
                    residual_permille: 11,
                    coverage_permille: 900,
                    scalar_2f_mismatch_permille: 21,
                },
                Event::ImpedanceDone {
                    oid: 6,
                    status: 0,
                    status_flags_or: 0x55aa,
                    warning_flags: 0x24,
                },
            ]
        );
    }

    #[test]
    fn impedance_failure_emits_profile_then_run_without_fits() {
        let mut sink = RecordingSink::default();

        emit_diagnostic_report(
            &mut sink,
            DiagnosticReport::ImpedanceTest {
                profile: impedance_profile_report(),
                run: impedance_run_report(2),
                fits: None,
                detail: 2,
            },
        );

        assert_eq!(
            sink.events,
            std::vec![
                Event::ImpedanceProfile {
                    oid: 6,
                    profile_version: 1,
                },
                Event::ImpedanceDone {
                    oid: 6,
                    status: 2,
                    status_flags_or: 0x55aa,
                    warning_flags: 0x24,
                },
            ]
        );
    }
}
