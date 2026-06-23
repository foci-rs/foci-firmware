use super::*;
use crate::tmc::{
    tmc_current_step_test, tmc_current_torque_sample_test, tmc_current_vector_step_test,
    tmc_impedance_test, tmc_position_torque_offset_sample_test, tmc_voltage_step_test,
};
use crate::tmc_control::TmcControlRuntime;
use foci_tmc4671::registers::{
    ABN_DECODER_COUNT, ABN_DECODER_PPR, ADC_AGPI_A_RAW_ADC_VM_RAW, INTERIM_FOC_IQ_ID,
    INTERIM_FOC_UQ_UD_LIMITED, MODE_RAMP_MODE_MOTION, PID_ERROR_PID_FLUX_ERROR_SUM,
    PID_ERROR_PID_TORQUE_ERROR_SUM, PID_POSITION_ACTUAL, PID_POSITION_TARGET,
    PID_TORQUE_FLUX_ACTUAL, PID_TORQUE_FLUX_OFFSET, PID_TORQUE_FLUX_TARGET, PIDOUT_UQ_UD_LIMITS,
    STATUS_FLAGS, UQ_UD_EXT,
};
use foci_tmc4671::spi::Tmc4671Spi;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct MockSpiError;

impl embedded_hal_async::spi::Error for MockSpiError {
    fn kind(&self) -> embedded_hal_async::spi::ErrorKind {
        embedded_hal_async::spi::ErrorKind::Other
    }
}

#[derive(Default)]
struct DiagnosticMockSpi {
    writes: std::vec::Vec<(u8, u32)>,
    write_attempts: std::vec::Vec<(u8, u32)>,
    read_requests: std::vec::Vec<u8>,
    reads: std::collections::VecDeque<u32>,
    fail_writes_to: Option<u8>,
    fail_reads_from: Option<u8>,
}

impl DiagnosticMockSpi {
    fn with_read(mut self, value: u32) -> Self {
        self.reads.push_back(value);
        self
    }

    #[allow(dead_code)]
    fn fail_reads_from(mut self, addr: u8) -> Self {
        self.fail_reads_from = Some(addr);
        self
    }

    #[allow(dead_code)]
    fn fail_writes_to(mut self, addr: u8) -> Self {
        self.fail_writes_to = Some(addr);
        self
    }
}

impl embedded_hal_async::spi::ErrorType for DiagnosticMockSpi {
    type Error = MockSpiError;
}

impl embedded_hal_async::spi::SpiDevice<u8> for DiagnosticMockSpi {
    async fn transaction(
        &mut self,
        operations: &mut [embedded_hal_async::spi::Operation<'_, u8>],
    ) -> Result<(), Self::Error> {
        for op in operations {
            match op {
                embedded_hal_async::spi::Operation::Write(buf) if buf.len() == 5 => {
                    let addr = buf[0] & 0x7f;
                    let value = u32::from_be_bytes([buf[1], buf[2], buf[3], buf[4]]);
                    self.write_attempts.push((addr, value));
                    if self.fail_writes_to == Some(addr) {
                        return Err(MockSpiError);
                    }
                    self.writes.push((addr, value));
                }
                embedded_hal_async::spi::Operation::Write(buf) if buf.len() == 1 => {
                    let addr = buf[0];
                    self.read_requests.push(addr);
                    if self.fail_reads_from == Some(addr) {
                        return Err(MockSpiError);
                    }
                }
                embedded_hal_async::spi::Operation::Read(buf) => {
                    let value = self.reads.pop_front().unwrap_or(0);
                    buf.copy_from_slice(&value.to_be_bytes());
                }
                embedded_hal_async::spi::Operation::Write(_)
                | embedded_hal_async::spi::Operation::DelayNs(_) => {}
                embedded_hal_async::spi::Operation::Transfer(read, write) => {
                    read.copy_from_slice(write);
                }
                embedded_hal_async::spi::Operation::TransferInPlace(_) => {}
            }
        }
        Ok(())
    }
}

#[derive(Default)]
struct MockDelay {
    delays_ms: std::vec::Vec<u64>,
}

impl DiagnosticDelay for MockDelay {
    async fn delay_ms(&mut self, ms: u64) {
        self.delays_ms.push(ms);
    }
}

/// Board config fixture for diagnostics that need `DiagnosticContext::tmc_config`.
/// Mirrors a board's own `const TMC_CONFIG`; values match
/// `crate::config::TmcConfig::default_for_test`.
static TEST_TMC_CONFIG: crate::config::TmcConfig = crate::config::TmcConfig {
    pwm_maxcnt: 3999,
    tmc_clock_hz: 25_000_000,
    bbm_h: 9,
    bbm_l: 9,
    dsadc_mdec_a: 331,
    dsadc_mdec_b: 331,
    adc_i_select: 0x1800_0100,
    excitation_voltage: 512,
    didt_cycles: 5000,
    bandwidth_hz: 50.0,
    resistance_max_expected_current_count: 1600,
    resistance_scale_metadata_validated: true,
};

fn ready_runtime() -> TmcControlRuntime {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    runtime.set_motor_enabled_for_test(true);
    runtime.set_calibrated_for_test(true);
    runtime
}

#[test]
fn current_step_encoder_delta_wraps_by_counts_per_rev() {
    assert_eq!(current_step_encoder_delta(90, 10, 100), 20);
    assert_eq!(current_step_encoder_delta(10, 90, 100), -20);
    assert_eq!(current_step_encoder_delta(10, 40, 100), 30);
    assert_eq!(current_step_encoder_delta(40, 10, 100), -30);
    assert_eq!(current_step_encoder_delta(10, 90, 0), 80);
}

#[test]
fn pack_voltage_ext_preserves_signed_halves() {
    assert_eq!(pack_voltage_ext(0x1234, -2), 0x1234_fffe);
    assert_eq!(pack_voltage_ext(-1, 0x4567), 0xffff_4567);
}

#[test]
fn empty_reports_preserve_headers_and_zero_samples() {
    let torque =
        empty_current_torque_sample_report(7, CURRENT_STEP_STATUS_NOT_READY, -11, 22, 3, 44);
    assert_eq!(torque.oid, 7);
    assert_eq!(torque.status, CURRENT_STEP_STATUS_NOT_READY);
    assert_eq!(torque.target, -11);
    assert_eq!(torque.flux_target, 22);
    assert_eq!(torque.sample_delay_ms, 3);
    assert_eq!(torque.voltage_limit, 44);
    assert_eq!(torque.torque_sample, 0);
    assert_eq!(torque.status_flags, 0);

    let voltage = empty_voltage_step_report(8, CURRENT_STEP_STATUS_SPI, -33, 44, 5);
    assert_eq!(voltage.oid, 8);
    assert_eq!(voltage.status, CURRENT_STEP_STATUS_SPI);
    assert_eq!(voltage.uq_ext, -33);
    assert_eq!(voltage.ud_ext, 44);
    assert_eq!(voltage.sample_delay_ms, 5);
    assert_eq!(voltage.encoder_delta_after, 0);
}

#[test]
fn current_step_not_ready_returns_status_without_writes() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let mut delay = MockDelay::default();
        let mut tmc = Tmc4671Spi::new(DiagnosticMockSpi::default());
        let req = tmc_current_step_test(0, 7, 123, 4, 500);

        let outcome = run_current_step_request(
            req,
            DiagnosticContext {
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                tmc_config: &TEST_TMC_CONFIG,
                readiness: DiagnosticReadiness {
                    homing_active: false,
                    stepper_active: false,
                    saved_voltage_limit: 999,
                },
            },
        )
        .await;

        let DiagnosticOutcome::Handled(DiagnosticReport::CurrentStep {
            vector_reply,
            report,
        }) = outcome
        else {
            panic!("expected current step report");
        };
        assert!(!vector_reply);
        assert_eq!(report.status, CURRENT_STEP_STATUS_NOT_READY);
        assert_eq!(report.oid, 7);
        assert_eq!(report.torque_target, 123);
        assert!(tmc.into_inner().writes.is_empty());
        assert!(delay.delays_ms.is_empty());
    });
}

#[test]
fn current_vector_uses_vector_report_flag() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let mut delay = MockDelay::default();
        let mut tmc = Tmc4671Spi::new(DiagnosticMockSpi::default());
        let req = tmc_current_vector_step_test(0, 9, 11, -12, 4, 500);

        let outcome = run_current_step_request(
            req,
            DiagnosticContext {
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                tmc_config: &TEST_TMC_CONFIG,
                readiness: DiagnosticReadiness {
                    homing_active: true,
                    stepper_active: false,
                    saved_voltage_limit: 999,
                },
            },
        )
        .await;

        let DiagnosticOutcome::Handled(DiagnosticReport::CurrentStep {
            vector_reply,
            report,
        }) = outcome
        else {
            panic!("expected current vector report");
        };
        assert!(vector_reply);
        assert_eq!(report.status, CURRENT_STEP_STATUS_NOT_READY);
        assert_eq!(report.torque_target, 11);
        assert_eq!(report.flux_target, -12);
    });
}

#[test]
fn current_step_success_records_settle_delays_and_restores_limit() {
    pollster::block_on(async {
        let raw_spi = DiagnosticMockSpi::default()
            .with_read(0) // saved MODE_RAMP_MODE_MOTION
            .with_read(0xaaaa_bbbb) // PID_POSITION_ACTUAL
            .with_read(100) // ABN_DECODER_COUNT before
            .with_read(1000) // ABN_DECODER_PPR
            .with_read(0x0005_0000) // PID_TORQUE_FLUX_ACTUAL before
            .with_read(0x0009_000a) // PID_TORQUE_FLUX_ACTUAL during
            .with_read(0x000b_000c) // INTERIM_FOC_IQ_ID
            .with_read(0x000d_000e) // INTERIM_FOC_UQ_UD_LIMITED
            .with_read(0x0000_1234) // ADC_AGPI_A_RAW_ADC_VM_RAW
            .with_read(120) // ABN_DECODER_COUNT after
            .with_read(0x0006_0000); // PID_TORQUE_FLUX_ACTUAL after
        let mut tmc = Tmc4671Spi::new(raw_spi);
        let mut runtime = ready_runtime();
        let mut delay = MockDelay::default();

        let outcome = run_current_step_request(
            tmc_current_step_test(0, 7, 20, 3, 555),
            DiagnosticContext {
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                tmc_config: &TEST_TMC_CONFIG,
                readiness: DiagnosticReadiness {
                    homing_active: false,
                    stepper_active: false,
                    saved_voltage_limit: 777,
                },
            },
        )
        .await;

        let DiagnosticOutcome::Handled(DiagnosticReport::CurrentStep { report, .. }) = outcome
        else {
            panic!("expected current step report");
        };
        assert_eq!(report.status, 0);
        assert_eq!(report.encoder_before, 100);
        assert_eq!(report.encoder_after, 120);
        assert_eq!(report.encoder_delta, 20);
        assert_eq!(report.adc_vm_raw, 0x1234);
        assert_eq!(delay.delays_ms, std::vec![20, 3, 20]);

        let raw_spi = tmc.into_inner();
        let expected_writes = std::vec![
            (PID_TORQUE_FLUX_TARGET.addr(), 0),
            (PIDOUT_UQ_UD_LIMITS.addr(), 555),
            (
                PID_ERROR_PID_TORQUE_ERROR_SUM.addr.addr(),
                u32::from(PID_ERROR_PID_TORQUE_ERROR_SUM.sub_addr),
            ),
            (PID_ERROR_PID_TORQUE_ERROR_SUM.data.addr(), 0),
            (
                PID_ERROR_PID_FLUX_ERROR_SUM.addr.addr(),
                u32::from(PID_ERROR_PID_FLUX_ERROR_SUM.sub_addr),
            ),
            (PID_ERROR_PID_FLUX_ERROR_SUM.data.addr(), 0),
            (MODE_RAMP_MODE_MOTION.addr(), 0x8000_0001),
            (
                PID_TORQUE_FLUX_TARGET.addr(),
                crate::tmc::pack_torque_flux_target(20, 0),
            ),
            (
                INTERIM_FOC_IQ_ID.addr.addr(),
                u32::from(INTERIM_FOC_IQ_ID.sub_addr),
            ),
            (
                INTERIM_FOC_UQ_UD_LIMITED.addr.addr(),
                u32::from(INTERIM_FOC_UQ_UD_LIMITED.sub_addr),
            ),
            (
                ADC_AGPI_A_RAW_ADC_VM_RAW.addr.addr(),
                u32::from(ADC_AGPI_A_RAW_ADC_VM_RAW.sub_addr),
            ),
            (PID_TORQUE_FLUX_TARGET.addr(), 0),
            (PID_TORQUE_FLUX_TARGET.addr(), 0),
            (PIDOUT_UQ_UD_LIMITS.addr(), 777),
            (PID_POSITION_TARGET.addr(), 0xaaaa_bbbb),
            (MODE_RAMP_MODE_MOTION.addr(), 0x8000_0003),
        ];
        assert_eq!(raw_spi.write_attempts, expected_writes);
        assert_eq!(raw_spi.writes, raw_spi.write_attempts);
        assert_eq!(
            raw_spi.read_requests,
            std::vec![
                MODE_RAMP_MODE_MOTION.addr(),
                PID_POSITION_ACTUAL.addr(),
                ABN_DECODER_COUNT.addr(),
                ABN_DECODER_PPR.addr(),
                PID_TORQUE_FLUX_ACTUAL.addr(),
                PID_TORQUE_FLUX_ACTUAL.addr(),
                INTERIM_FOC_IQ_ID.data.addr(),
                INTERIM_FOC_UQ_UD_LIMITED.data.addr(),
                ADC_AGPI_A_RAW_ADC_VM_RAW.data.addr(),
                ABN_DECODER_COUNT.addr(),
                PID_TORQUE_FLUX_ACTUAL.addr(),
            ]
        );
    });
}

#[test]
fn torque_sample_not_ready_does_not_emit_detail() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let mut delay = MockDelay::default();
        let mut tmc = Tmc4671Spi::new(DiagnosticMockSpi::default());

        let outcome = run_current_torque_sample_request(
            tmc_current_torque_sample_test(0, 4, 22, 0, 3, 444),
            DiagnosticContext {
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                tmc_config: &TEST_TMC_CONFIG,
                readiness: DiagnosticReadiness {
                    homing_active: false,
                    stepper_active: false,
                    saved_voltage_limit: 777,
                },
            },
        )
        .await;

        let DiagnosticOutcome::Handled(DiagnosticReport::CurrentTorqueSample {
            report,
            emit_detail,
        }) = outcome
        else {
            panic!("expected torque sample report");
        };
        assert_eq!(report.status, CURRENT_STEP_STATUS_NOT_READY);
        assert!(!emit_detail);
    });
}

#[test]
fn position_offset_rejects_non_position_mode() {
    pollster::block_on(async {
        let mut runtime = ready_runtime();
        let mut delay = MockDelay::default();
        let raw_spi = DiagnosticMockSpi::default().with_read(0x8000_0001);
        let mut tmc = Tmc4671Spi::new(raw_spi);

        let outcome = run_position_torque_offset_sample_request(
            tmc_position_torque_offset_sample_test(0, 4, 33, 3, 444),
            DiagnosticContext {
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                tmc_config: &TEST_TMC_CONFIG,
                readiness: DiagnosticReadiness {
                    homing_active: false,
                    stepper_active: false,
                    saved_voltage_limit: 777,
                },
            },
        )
        .await;

        let DiagnosticOutcome::Handled(DiagnosticReport::CurrentTorqueSample {
            report,
            emit_detail,
        }) = outcome
        else {
            panic!("expected position offset report");
        };
        assert_eq!(report.status, CURRENT_STEP_STATUS_NOT_READY);
        assert!(!emit_detail);
        assert!(
            tmc.into_inner()
                .read_requests
                .contains(&MODE_RAMP_MODE_MOTION.addr())
        );
    });
}

#[test]
fn position_offset_spi_failure_attempts_offset_restore() {
    pollster::block_on(async {
        let raw_spi = DiagnosticMockSpi::default()
            .with_read(0x8000_0003) // saved mode
            .with_read(0x1234_5678) // saved offset
            .fail_writes_to(PIDOUT_UQ_UD_LIMITS.addr());
        let mut tmc = Tmc4671Spi::new(raw_spi);
        let mut runtime = ready_runtime();
        let mut delay = MockDelay::default();

        let outcome = run_position_torque_offset_sample_request(
            tmc_position_torque_offset_sample_test(0, 4, 33, 3, 444),
            DiagnosticContext {
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                tmc_config: &TEST_TMC_CONFIG,
                readiness: DiagnosticReadiness {
                    homing_active: false,
                    stepper_active: false,
                    saved_voltage_limit: 777,
                },
            },
        )
        .await;

        let DiagnosticOutcome::Handled(DiagnosticReport::CurrentTorqueSample { report, .. }) =
            outcome
        else {
            panic!("expected position offset report");
        };
        assert_eq!(report.status, CURRENT_STEP_STATUS_SPI);
        let raw_spi = tmc.into_inner();
        assert_eq!(
            raw_spi.write_attempts,
            std::vec![
                (PIDOUT_UQ_UD_LIMITS.addr(), 444),
                (PID_TORQUE_FLUX_OFFSET.addr(), 0x1234_5678),
                (PIDOUT_UQ_UD_LIMITS.addr(), 777),
                (MODE_RAMP_MODE_MOTION.addr(), 0x8000_0003),
            ]
        );
        assert_eq!(
            raw_spi.writes,
            std::vec![
                (PID_TORQUE_FLUX_OFFSET.addr(), 0x1234_5678),
                (MODE_RAMP_MODE_MOTION.addr(), 0x8000_0003),
            ]
        );
        assert_eq!(
            raw_spi.read_requests,
            std::vec![MODE_RAMP_MODE_MOTION.addr(), PID_TORQUE_FLUX_OFFSET.addr()]
        );
    });
}

#[test]
fn voltage_step_rejects_non_position_mode() {
    pollster::block_on(async {
        let raw_spi = DiagnosticMockSpi::default().with_read(0x8000_0001);
        let mut tmc = Tmc4671Spi::new(raw_spi);
        let mut runtime = ready_runtime();
        let mut delay = MockDelay::default();

        let outcome = run_voltage_step_request(
            tmc_voltage_step_test(0, 3, 44, -55, 6),
            DiagnosticContext {
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                tmc_config: &TEST_TMC_CONFIG,
                readiness: DiagnosticReadiness {
                    homing_active: false,
                    stepper_active: false,
                    saved_voltage_limit: 777,
                },
            },
        )
        .await;

        let DiagnosticOutcome::Handled(DiagnosticReport::VoltageStep(report)) = outcome else {
            panic!("expected voltage step report");
        };
        assert_eq!(report.status, CURRENT_STEP_STATUS_NOT_READY);
        assert_eq!(report.uq_ext, 44);
        assert_eq!(report.ud_ext, -55);
    });
}

#[test]
fn voltage_step_failure_clears_external_voltage_and_restores_mode() {
    pollster::block_on(async {
        let raw_spi = DiagnosticMockSpi::default()
            .with_read(0x8000_0003) // saved mode
            .with_read(100) // encoder before
            .with_read(1000) // ppr
            .with_read(0x0001_0000) // torque before
            .fail_writes_to(UQ_UD_EXT.addr());
        let mut tmc = Tmc4671Spi::new(raw_spi);
        let mut runtime = ready_runtime();
        let mut delay = MockDelay::default();

        let outcome = run_voltage_step_request(
            tmc_voltage_step_test(0, 3, 44, -55, 6),
            DiagnosticContext {
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                tmc_config: &TEST_TMC_CONFIG,
                readiness: DiagnosticReadiness {
                    homing_active: false,
                    stepper_active: false,
                    saved_voltage_limit: 777,
                },
            },
        )
        .await;

        let DiagnosticOutcome::Handled(DiagnosticReport::VoltageStep(report)) = outcome else {
            panic!("expected voltage step report");
        };
        assert_eq!(report.status, CURRENT_STEP_STATUS_SPI);
        let raw_spi = tmc.into_inner();
        assert_eq!(
            raw_spi.write_attempts,
            std::vec![
                (STATUS_FLAGS.addr(), 0),
                (UQ_UD_EXT.addr(), pack_voltage_ext(44, -55)),
                (MODE_RAMP_MODE_MOTION.addr(), 0),
                (UQ_UD_EXT.addr(), 0),
                (MODE_RAMP_MODE_MOTION.addr(), 0x8000_0003),
            ]
        );
        assert_eq!(
            raw_spi.writes,
            std::vec![
                (STATUS_FLAGS.addr(), 0),
                (MODE_RAMP_MODE_MOTION.addr(), 0),
                (MODE_RAMP_MODE_MOTION.addr(), 0x8000_0003),
            ]
        );
        assert_eq!(
            raw_spi.read_requests,
            std::vec![
                MODE_RAMP_MODE_MOTION.addr(),
                ABN_DECODER_COUNT.addr(),
                ABN_DECODER_PPR.addr(),
                PID_TORQUE_FLUX_ACTUAL.addr(),
            ]
        );
    });
}

#[test]
fn dispatcher_returns_not_diagnostic_for_normal_request() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let mut delay = MockDelay::default();
        let mut tmc = Tmc4671Spi::new(DiagnosticMockSpi::default());
        let req = crate::tmc::tmc_set_current(0, 1000);

        let outcome = handle_diagnostic_request(
            req.clone(),
            DiagnosticContext {
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                tmc_config: &TEST_TMC_CONFIG,
                readiness: DiagnosticReadiness {
                    homing_active: false,
                    stepper_active: false,
                    saved_voltage_limit: 777,
                },
            },
        )
        .await;

        let DiagnosticOutcome::NotDiagnostic(returned) = outcome else {
            panic!("expected NotDiagnostic");
        };
        assert_eq!(returned, req);
    });
}

#[derive(Debug, Clone, PartialEq, Eq)]
enum RecordedResistanceEvent {
    Profile { oid: u8 },
    Axis { oid: u8, electrical_axis: u8 },
    Done { oid: u8, status: u8 },
    ImpedanceProfile { oid: u8 },
    ImpedanceFit {
        oid: u8,
        point_index: u8,
        scalar_2f_mismatch_permille: u16,
        residual_permille: u16,
        coverage_permille: u16,
    },
    ImpedanceDone {
        oid: u8,
        status: u8,
        status_flags_or: u32,
        warning_flags: u32,
    },
}

#[derive(Default)]
struct RecordingDiagnosticSink {
    events: std::vec::Vec<RecordedResistanceEvent>,
}

impl DiagnosticReplySink for RecordingDiagnosticSink {
    fn foci_current_step_result(&mut self, _report: CurrentStepReport) {}
    fn foci_current_vector_step_result(&mut self, _report: CurrentStepReport) {}
    fn foci_current_torque_sample_result(&mut self, _report: CurrentTorqueSampleReport) {}
    fn foci_current_torque_sample_detail_result(&mut self, _report: &CurrentTorqueSampleReport) {}
    fn foci_voltage_step_result(&mut self, _report: VoltageStepReport) {}

    fn foci_resistance_profile(&mut self, report: ResistanceProfileReport) {
        self.events
            .push(RecordedResistanceEvent::Profile { oid: report.oid });
    }

    fn foci_resistance_axis(&mut self, report: ResistanceAxisReport) {
        self.events.push(RecordedResistanceEvent::Axis {
            oid: report.oid,
            electrical_axis: report.electrical_axis,
        });
    }

    fn foci_resistance_run(&mut self, report: ResistanceRunReport) {
        self.events.push(RecordedResistanceEvent::Done {
            oid: report.oid,
            status: report.status,
        });
    }

    fn foci_impedance_profile(&mut self, report: ImpedanceProfileReport) {
        self.events
            .push(RecordedResistanceEvent::ImpedanceProfile { oid: report.oid });
    }

    fn foci_impedance_fit(&mut self, report: ImpedanceFitReport) {
        self.events.push(RecordedResistanceEvent::ImpedanceFit {
            oid: report.oid,
            point_index: report.point_index,
            scalar_2f_mismatch_permille: report.scalar_2f_mismatch_permille,
            residual_permille: report.residual_permille,
            coverage_permille: report.coverage_permille,
        });
    }

    fn foci_impedance_run(&mut self, report: ImpedanceRunReport) {
        self.events.push(RecordedResistanceEvent::ImpedanceDone {
            oid: report.oid,
            status: report.status,
            status_flags_or: report.status_flags_or,
            warning_flags: report.warning_flags,
        });
    }
}

#[test]
fn resistance_test_request_dispatches_to_shared_runner() {
    pollster::block_on(async {
        // Global per-channel state (`motor_sentinel`) is shared across the
        // whole test binary; serialize against other suites that touch it.
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
            .lock()
            .await;
        let channel_state = crate::state::tmc::tmc_channel(0).unwrap();
        // Non-zero sentinel: ADC offsets are "ready", isolating the
        // `NotArmed` rejection this test exercises from the `NotAdcReady`
        // entry pre-check that runs first.
        channel_state
            .motor_sentinel
            .store(0x0002_0032, core::sync::atomic::Ordering::Relaxed);

        // Motor not enabled/calibrated: the shared runner's
        // `ExternallyArmed` entry pre-check rejects with `NotArmed` before
        // touching SPI. This proves the request reaches the shared runner
        // (rather than being silently dropped) without needing the full
        // register-level mock already covered by
        // `resistance_identification::tests`.
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let mut delay = MockDelay::default();
        let mut tmc = Tmc4671Spi::new(DiagnosticMockSpi::default());
        let req = crate::tmc::tmc_resistance_test(0, 7, 0);

        let outcome = handle_diagnostic_request(
            req,
            DiagnosticContext {
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                tmc_config: &TEST_TMC_CONFIG,
                readiness: DiagnosticReadiness {
                    homing_active: false,
                    stepper_active: false,
                    saved_voltage_limit: 777,
                },
            },
        )
        .await;

        channel_state
            .motor_sentinel
            .store(0, core::sync::atomic::Ordering::Relaxed);

        let DiagnosticOutcome::Handled(report) = outcome else {
            panic!("expected resistance test report");
        };

        let mut sink = RecordingDiagnosticSink::default();
        emit_diagnostic_report(&mut sink, report);

        assert!(
            sink.events
                .iter()
                .any(|event| matches!(event, RecordedResistanceEvent::Done { oid: 7, .. }))
        );
    });
}

#[test]
fn resistance_test_not_armed_reports_no_axes() {
    pollster::block_on(async {
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
            .lock()
            .await;
        let channel_state = crate::state::tmc::tmc_channel(0).unwrap();
        channel_state
            .motor_sentinel
            .store(0x0002_0032, core::sync::atomic::Ordering::Relaxed);

        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let mut delay = MockDelay::default();
        let mut tmc = Tmc4671Spi::new(DiagnosticMockSpi::default());
        let req = crate::tmc::tmc_resistance_test(0, 9, 0);

        let outcome = handle_diagnostic_request(
            req,
            DiagnosticContext {
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                tmc_config: &TEST_TMC_CONFIG,
                readiness: DiagnosticReadiness {
                    homing_active: false,
                    stepper_active: false,
                    saved_voltage_limit: 777,
                },
            },
        )
        .await;

        channel_state
            .motor_sentinel
            .store(0, core::sync::atomic::Ordering::Relaxed);

        let DiagnosticOutcome::Handled(DiagnosticReport::ResistanceTest { profile, run, axes }) =
            outcome
        else {
            panic!("expected resistance test report");
        };
        assert_eq!(profile.oid, 9);
        assert_eq!(
            run.status,
            crate::resistance_identification::ResistanceError::NotArmed.status_code()
        );
        assert!(axes.is_none());
    });
}

#[test]
fn resistance_test_rejects_during_homing_without_invoking_runner() {
    pollster::block_on(async {
        // Mirrors `current_step_not_ready_returns_status_without_writes`:
        // sibling diagnostics reject on `readiness.homing_active` /
        // `readiness.stepper_active` before touching SPI. The resistance-test
        // dispatch must do the same instead of only gating on
        // enabled+calibrated, or a resistance test can switch the motor to
        // open-loop voltage while homing or active stepping is in progress.
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
            .lock()
            .await;
        let channel_state = crate::state::tmc::tmc_channel(0).unwrap();
        // Non-zero sentinel and an enabled+calibrated runtime: every other
        // entry gate (`NotAdcReady`, `NotArmed`) is satisfied, isolating the
        // homing-readiness rejection this test exercises.
        channel_state
            .motor_sentinel
            .store(0x0002_0032, core::sync::atomic::Ordering::Relaxed);

        let mut runtime = ready_runtime();
        let mut delay = MockDelay::default();
        let mut tmc = Tmc4671Spi::new(DiagnosticMockSpi::default());
        let req = crate::tmc::tmc_resistance_test(0, 11, 0);

        let outcome = handle_diagnostic_request(
            req,
            DiagnosticContext {
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                tmc_config: &TEST_TMC_CONFIG,
                readiness: DiagnosticReadiness {
                    homing_active: true,
                    stepper_active: false,
                    saved_voltage_limit: 777,
                },
            },
        )
        .await;

        channel_state
            .motor_sentinel
            .store(0, core::sync::atomic::Ordering::Relaxed);

        let DiagnosticOutcome::Handled(DiagnosticReport::ResistanceTest { profile, run, axes }) =
            outcome
        else {
            panic!("expected resistance test report");
        };
        assert_eq!(profile.oid, 11);
        // Homing/active-stepping is a "busy, not ready" rejection, the same
        // semantics the sibling voltage_step/current_step diagnostics report
        // via `CURRENT_STEP_STATUS_NOT_READY` -- not `NotArmed`, which means
        // the motor itself is not armed/calibrated for open-loop voltage.
        assert_eq!(run.status, CURRENT_STEP_STATUS_NOT_READY);
        assert!(axes.is_none());
        assert!(
            tmc.into_inner().writes.is_empty(),
            "expected no SPI writes (no energizing) while homing is active"
        );
    });
}

#[test]
fn impedance_test_request_dispatches_to_shared_runner() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let mut delay = MockDelay::default();
        let mut tmc = Tmc4671Spi::new(DiagnosticMockSpi::default());
        let req = tmc_impedance_test(0, 7, 0);

        let outcome = handle_diagnostic_request(
            req,
            DiagnosticContext {
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                tmc_config: &TEST_TMC_CONFIG,
                readiness: DiagnosticReadiness {
                    homing_active: false,
                    stepper_active: false,
                    saved_voltage_limit: 777,
                },
            },
        )
        .await;

        let DiagnosticOutcome::Handled(DiagnosticReport::ImpedanceTest { run, .. }) = outcome
        else {
            panic!("expected impedance test report");
        };
        assert_eq!(run.oid, 7);
    });
}

#[test]
fn impedance_test_rejects_during_homing_without_invoking_runner() {
    pollster::block_on(async {
        let mut runtime = ready_runtime();
        let mut delay = MockDelay::default();
        let mut tmc = Tmc4671Spi::new(DiagnosticMockSpi::default());
        let req = tmc_impedance_test(0, 11, 1);

        let outcome = handle_diagnostic_request(
            req,
            DiagnosticContext {
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                tmc_config: &TEST_TMC_CONFIG,
                readiness: DiagnosticReadiness {
                    homing_active: true,
                    stepper_active: false,
                    saved_voltage_limit: 777,
                },
            },
        )
        .await;

        let DiagnosticOutcome::Handled(DiagnosticReport::ImpedanceTest { run, fits, .. }) = outcome
        else {
            panic!("expected impedance test report");
        };
        assert_eq!(run.status, CURRENT_STEP_STATUS_NOT_READY);
        assert!(fits.is_none());
        assert!(tmc.into_inner().writes.is_empty());
        assert!(delay.delays_ms.is_empty());
    });
}

#[test]
fn impedance_test_rejects_during_active_stepping_without_invoking_runner() {
    pollster::block_on(async {
        let mut runtime = ready_runtime();
        let mut delay = MockDelay::default();
        let mut tmc = Tmc4671Spi::new(DiagnosticMockSpi::default());
        let req = tmc_impedance_test(0, 12, 1);

        let outcome = handle_diagnostic_request(
            req,
            DiagnosticContext {
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                tmc_config: &TEST_TMC_CONFIG,
                readiness: DiagnosticReadiness {
                    homing_active: false,
                    stepper_active: true,
                    saved_voltage_limit: 777,
                },
            },
        )
        .await;

        let DiagnosticOutcome::Handled(DiagnosticReport::ImpedanceTest { run, fits, .. }) = outcome
        else {
            panic!("expected impedance test report");
        };
        assert_eq!(run.status, CURRENT_STEP_STATUS_NOT_READY);
        assert!(fits.is_none());
        assert!(tmc.into_inner().writes.is_empty());
        assert!(delay.delays_ms.is_empty());
    });
}

#[test]
fn impedance_test_not_ready_writes_nothing() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let mut delay = MockDelay::default();
        let mut tmc = Tmc4671Spi::new(DiagnosticMockSpi::default());
        let req = tmc_impedance_test(0, 13, 1);

        let outcome = handle_diagnostic_request(
            req,
            DiagnosticContext {
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                tmc_config: &TEST_TMC_CONFIG,
                readiness: DiagnosticReadiness {
                    homing_active: false,
                    stepper_active: false,
                    saved_voltage_limit: 777,
                },
            },
        )
        .await;

        let DiagnosticOutcome::Handled(DiagnosticReport::ImpedanceTest { run, fits, .. }) = outcome
        else {
            panic!("expected impedance test report");
        };
        assert_eq!(run.status, CURRENT_STEP_STATUS_NOT_READY);
        assert!(fits.is_none());
        assert!(tmc.into_inner().writes.is_empty());
    });
}

#[test]
fn impedance_runner_failure_emits_profile_then_run_without_fits() {
    pollster::block_on(async {
        let raw_spi = DiagnosticMockSpi::default().fail_reads_from(PIDOUT_UQ_UD_LIMITS.addr());
        let mut tmc = Tmc4671Spi::new(raw_spi);
        let mut runtime = ready_runtime();
        let mut delay = MockDelay::default();

        let outcome = handle_diagnostic_request(
            tmc_impedance_test(0, 14, 2),
            DiagnosticContext {
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                tmc_config: &TEST_TMC_CONFIG,
                readiness: DiagnosticReadiness {
                    homing_active: false,
                    stepper_active: false,
                    saved_voltage_limit: 777,
                },
            },
        )
        .await;

        let DiagnosticOutcome::Handled(report) = outcome else {
            panic!("expected impedance test report");
        };

        let mut sink = RecordingDiagnosticSink::default();
        emit_diagnostic_report(&mut sink, report);

        assert_eq!(
            sink.events,
            std::vec![
                RecordedResistanceEvent::ImpedanceProfile { oid: 14 },
                RecordedResistanceEvent::ImpedanceDone {
                    oid: 14,
                    status: CURRENT_STEP_STATUS_SPI,
                    status_flags_or: 0,
                    warning_flags: 0,
                },
            ]
        );
    });
}
