//! Shared active diagnostic reply emission.

use super::{CurrentStepReport, CurrentTorqueSampleReport, DiagnosticReport, VoltageStepReport};

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
}
