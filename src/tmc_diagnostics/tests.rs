use super::*;
use crate::tmc::{
    CurrentStepAxis, TorqueSampleKind, tmc_current_step_test, tmc_current_torque_sample_test,
    tmc_current_vector_step_test, tmc_position_torque_offset_sample_test, tmc_voltage_step_test,
};
use crate::tmc_control::TmcControlRuntime;
use foci_firmware_macros::fixture_test;
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
    register_values: std::collections::BTreeMap<u8, u32>,
    last_read_addr: Option<u8>,
    transaction_attempts: usize,
    fail_transaction: Option<usize>,
    fail_writes_to: Option<u8>,
    fail_reads_from: Option<u8>,
}

impl DiagnosticMockSpi {
    fn with_read(mut self, value: u32) -> Self {
        self.reads.push_back(value);
        self
    }

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
        self.transaction_attempts += 1;
        let fail_this_transaction = self.fail_transaction == Some(self.transaction_attempts);
        for op in operations {
            match op {
                embedded_hal_async::spi::Operation::Write(buf) if buf.len() == 5 => {
                    let addr = buf[0] & 0x7f;
                    let value = u32::from_be_bytes([buf[1], buf[2], buf[3], buf[4]]);
                    self.write_attempts.push((addr, value));
                    if fail_this_transaction || self.fail_writes_to == Some(addr) {
                        return Err(MockSpiError);
                    }
                    self.writes.push((addr, value));
                    self.register_values.insert(addr, value);
                }
                embedded_hal_async::spi::Operation::Write(buf) if buf.len() == 1 => {
                    let addr = buf[0];
                    self.read_requests.push(addr);
                    self.last_read_addr = Some(addr);
                    if fail_this_transaction || self.fail_reads_from == Some(addr) {
                        return Err(MockSpiError);
                    }
                }
                embedded_hal_async::spi::Operation::Read(buf) => {
                    let value = self.reads.pop_front().unwrap_or_else(|| {
                        self.last_read_addr
                            .and_then(|addr| self.register_values.get(&addr).copied())
                            .unwrap_or(0)
                    });
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
    delays_us: std::vec::Vec<u32>,
    now_us: core::cell::Cell<u64>,
    advance_on_delay: bool,
    advance_on_now_us: u64,
}

#[derive(Default)]
struct MockDiagnosticPowerStage {
    trip_calls: u8,
    motor_armed_at_trip: bool,
}

impl DiagnosticPowerStage for MockDiagnosticPowerStage {
    fn trip_power_stage(&mut self) {
        self.trip_calls = self.trip_calls.saturating_add(1);
        self.motor_armed_at_trip = crate::state::tmc::tmc_channel0()
            .motor_armed
            .load(core::sync::atomic::Ordering::Relaxed);
    }
}

impl DiagnosticDelay for MockDelay {
    async fn delay_ms(&mut self, ms: u64) {
        self.delays_ms.push(ms);
        if self.advance_on_delay {
            self.now_us
                .set(self.now_us.get().wrapping_add(ms.saturating_mul(1_000)));
        }
    }

    async fn delay_us(&mut self, us: u32) {
        self.delays_us.push(us);
        if self.advance_on_delay {
            self.now_us
                .set(self.now_us.get().wrapping_add(u64::from(us)));
        }
    }

    fn now_us(&self) -> u64 {
        let now_us = self.now_us.get();
        self.now_us.set(now_us.wrapping_add(self.advance_on_now_us));
        now_us
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
    electrical_model_theta_e_us: 160,
    current_validation_flux_max_encoder_delta_counts: 4,
    current_validation_torque_max_encoder_delta_counts: 16,
    resistance_max_expected_current_count: 1600,
    resistance_scale_metadata_validated: true,
    current_scale_ma_per_lsb: 1.155,
};

fn ready_runtime() -> TmcControlRuntime {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    runtime.set_motor_enabled_for_test(true);
    runtime.set_calibrated_for_test(true);
    runtime
}

#[test]
fn pack_voltage_ext_preserves_signed_halves() {
    assert_eq!(pack_voltage_ext(0x1234, -2), 0x1234_fffe);
    assert_eq!(pack_voltage_ext(-1, 0x4567), 0xffff_4567);
}

#[test]
fn empty_reports_preserve_headers_and_zero_samples() {
    let torque = empty_current_torque_sample_report(
        7,
        TorqueSampleKind::CurrentTorqueSample,
        CURRENT_STEP_STATUS_NOT_READY,
        -11,
        22,
        3,
        44,
    );
    assert_eq!(torque.oid, 7);
    assert_eq!(torque.kind, TorqueSampleKind::CurrentTorqueSample);
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

    let resistance = empty_resistance_run_report(9, CURRENT_STEP_STATUS_SPI);
    assert_eq!(resistance.oid, 9);
    assert_eq!(resistance.status, CURRENT_STEP_STATUS_SPI);
    assert_eq!(resistance.peak_abs_current_count, 0);
    assert_eq!(resistance.max_abs_steady_mean_current_count, 0);
    assert_eq!(resistance.current_ceiling_count, 0);
    assert!(!resistance.power_stage_tripped);
}

#[test]
fn failed_resistance_report_retains_current_evidence() {
    let mut current = crate::resistance_identification::ResistanceCurrentEvidence::new(1_600);
    current.observe_energized_sample(-1_800);
    current.record_completed_energized_mean(-900);
    current.mark_power_stage_tripped();

    let report = super::resistance::failed_run_report(
        7,
        crate::resistance_identification::ResistanceError::BackendIo.status_code(),
        current,
    );

    assert_eq!(report.oid, 7);
    assert_eq!(
        report.status,
        crate::resistance_identification::ResistanceError::BackendIo.status_code()
    );
    assert_eq!(report.peak_abs_current_count, 1_800);
    assert_eq!(report.max_abs_steady_mean_current_count, 900);
    assert_eq!(report.current_ceiling_count, 1_600);
    assert!(report.power_stage_tripped);
}

#[test]
fn commissioning_busy_diagnostic_returns_not_ready_without_hardware() {
    let request = crate::tmc::TmcRequest::new(
        0,
        crate::tmc::TmcRequestKind::CurrentStepTest {
            oid: 9,
            axis: crate::tmc::CurrentStepAxis::Torque,
            target: 64,
            duration_ms: 5,
            voltage_limit: 512,
        },
    );

    let outcome =
        super::busy_diagnostic_report(request, &crate::config::TmcConfig::default_for_test());

    let super::DiagnosticOutcome::Handled(super::DiagnosticReport::CurrentStep { reply, report }) =
        outcome
    else {
        panic!("expected scalar current-step busy report");
    };
    assert_eq!(
        reply,
        super::CurrentStepReply::Scalar(crate::tmc::CurrentStepAxis::Torque)
    );
    assert_eq!(report.oid, 9);
    assert_eq!(report.status, super::CURRENT_STEP_STATUS_NOT_READY);
    assert_eq!(report.torque_target, 64);
}

#[test]
fn current_step_not_ready_returns_status_without_writes() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let mut delay = MockDelay::default();
        let mut tmc = Tmc4671Spi::new(DiagnosticMockSpi::default());
        let req = tmc_current_step_test(0, 7, CurrentStepAxis::Torque, 123, 4, 500);

        let outcome = run_current_step_request(
            req,
            DiagnosticContext {
                host_liveness_expired: None,
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                power_stage: &mut MockDiagnosticPowerStage::default(),
                tmc_config: &TEST_TMC_CONFIG,
                feedback_config: crate::config::FeedbackConfig::abn_abn(),
                readiness: DiagnosticReadiness {
                    homing_active: false,
                    stepper_active: false,
                    saved_voltage_limit: 999,
                },
            },
        )
        .await;

        let DiagnosticOutcome::Handled(DiagnosticReport::CurrentStep { reply, report }) = outcome
        else {
            panic!("expected current step report");
        };
        assert_eq!(reply, CurrentStepReply::Scalar(CurrentStepAxis::Torque));
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
                host_liveness_expired: None,
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                power_stage: &mut MockDiagnosticPowerStage::default(),
                tmc_config: &TEST_TMC_CONFIG,
                feedback_config: crate::config::FeedbackConfig::abn_abn(),
                readiness: DiagnosticReadiness {
                    homing_active: true,
                    stepper_active: false,
                    saved_voltage_limit: 999,
                },
            },
        )
        .await;

        let DiagnosticOutcome::Handled(DiagnosticReport::CurrentStep { reply, report }) = outcome
        else {
            panic!("expected current vector report");
        };
        assert_eq!(reply, CurrentStepReply::Vector);
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
            tmc_current_step_test(0, 7, CurrentStepAxis::Torque, 20, 3, 555),
            DiagnosticContext {
                host_liveness_expired: None,
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                power_stage: &mut MockDiagnosticPowerStage::default(),
                tmc_config: &TEST_TMC_CONFIG,
                feedback_config: crate::config::FeedbackConfig::abn_abn(),
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
fn flux_current_step_drives_and_reports_flux_axis() {
    pollster::block_on(async {
        let raw_spi = DiagnosticMockSpi::default()
            .with_read(0) // saved MODE_RAMP_MODE_MOTION
            .with_read(0xaaaa_bbbb) // PID_POSITION_ACTUAL
            .with_read(100) // ABN_DECODER_COUNT before
            .with_read(1000) // ABN_DECODER_PPR
            .with_read(0x0005_fffe) // PID_TORQUE_FLUX_ACTUAL before
            .with_read(0x0009_0014) // PID_TORQUE_FLUX_ACTUAL during
            .with_read(0x000b_000c) // INTERIM_FOC_IQ_ID
            .with_read(0x000d_000e) // INTERIM_FOC_UQ_UD_LIMITED
            .with_read(0x0000_1234) // ADC_AGPI_A_RAW_ADC_VM_RAW
            .with_read(120) // ABN_DECODER_COUNT after
            .with_read(0x0006_fff9); // PID_TORQUE_FLUX_ACTUAL after
        let mut tmc = Tmc4671Spi::new(raw_spi);
        let mut runtime = ready_runtime();
        let mut delay = MockDelay::default();

        let outcome = run_current_step_request(
            tmc_current_step_test(0, 7, CurrentStepAxis::Flux, 20, 3, 555),
            DiagnosticContext {
                host_liveness_expired: None,
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                power_stage: &mut MockDiagnosticPowerStage::default(),
                tmc_config: &TEST_TMC_CONFIG,
                feedback_config: crate::config::FeedbackConfig::abn_abn(),
                readiness: DiagnosticReadiness {
                    homing_active: false,
                    stepper_active: false,
                    saved_voltage_limit: 777,
                },
            },
        )
        .await;

        let DiagnosticOutcome::Handled(DiagnosticReport::CurrentStep { reply, report }) = outcome
        else {
            panic!("expected current step report");
        };
        assert_eq!(reply, CurrentStepReply::Scalar(CurrentStepAxis::Flux));
        assert_eq!(report.status, 0);
        assert_eq!(report.torque_target, 20);
        assert_eq!(report.flux_target, 20);
        assert_eq!(report.torque_before, -2);
        assert_eq!(report.torque_during, 20);
        assert_eq!(report.torque_after, -7);
        assert_eq!(report.flux_during, 20);

        let raw_spi = tmc.into_inner();
        assert!(raw_spi.write_attempts.contains(&(
            PID_TORQUE_FLUX_TARGET.addr(),
            crate::tmc::pack_torque_flux_target(0, 20),
        )));
    });
}

#[test]
fn busy_flux_current_step_reports_flux_axis() {
    let outcome = busy_diagnostic_report(
        tmc_current_step_test(0, 7, CurrentStepAxis::Flux, 20, 30, 555),
        &TEST_TMC_CONFIG,
    );

    let DiagnosticOutcome::Handled(DiagnosticReport::CurrentStep { reply, report }) = outcome
    else {
        panic!("expected current step report");
    };
    assert_eq!(reply, CurrentStepReply::Scalar(CurrentStepAxis::Flux));
    assert_eq!(report.status, CURRENT_STEP_STATUS_NOT_READY);
}

#[test]
fn busy_current_vector_step_uses_vector_reply() {
    let outcome = busy_diagnostic_report(
        tmc_current_vector_step_test(0, 7, 20, -10, 30, 555),
        &TEST_TMC_CONFIG,
    );

    let DiagnosticOutcome::Handled(DiagnosticReport::CurrentStep { reply, .. }) = outcome else {
        panic!("expected current step report");
    };
    assert_eq!(reply, CurrentStepReply::Vector);
}

#[test]
fn busy_torque_samples_report_their_kind() {
    for (req, kind) in [
        (
            tmc_current_torque_sample_test(0, 4, 22, 0, 3, 444),
            TorqueSampleKind::CurrentTorqueSample,
        ),
        (
            tmc_position_torque_offset_sample_test(0, 4, 22, 3, 444),
            TorqueSampleKind::PositionTorqueOffset,
        ),
    ] {
        let DiagnosticOutcome::Handled(DiagnosticReport::CurrentTorqueSample(report)) =
            busy_diagnostic_report(req, &TEST_TMC_CONFIG)
        else {
            panic!("expected torque sample report");
        };
        assert_eq!(report.kind, kind);
        assert_eq!(report.status, CURRENT_STEP_STATUS_NOT_READY);
    }
}

#[test]
fn current_step_invalid_ppr_aborts_before_excitation() {
    pollster::block_on(async {
        let raw_spi = DiagnosticMockSpi::default()
            .with_read(0) // saved MODE_RAMP_MODE_MOTION
            .with_read(0xaaaa_bbbb) // PID_POSITION_ACTUAL
            .with_read(100) // ABN_DECODER_COUNT before
            .with_read(0); // ABN_DECODER_PPR -- invalid
        let mut tmc = Tmc4671Spi::new(raw_spi);
        let mut runtime = ready_runtime();
        let mut delay = MockDelay::default();

        let outcome = run_current_step_request(
            tmc_current_step_test(0, 7, CurrentStepAxis::Torque, 20, 3, 555),
            DiagnosticContext {
                host_liveness_expired: None,
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                power_stage: &mut MockDiagnosticPowerStage::default(),
                tmc_config: &TEST_TMC_CONFIG,
                feedback_config: crate::config::FeedbackConfig::abn_abn(),
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
        assert_eq!(report.status, CURRENT_STEP_STATUS_SPI);
        let raw_spi = tmc.into_inner();
        assert!(!raw_spi.write_attempts.contains(&(
            PID_TORQUE_FLUX_TARGET.addr(),
            crate::tmc::pack_torque_flux_target(20, 0),
        )));
    });
}

#[test]
fn current_step_exact_half_turn_aborts_via_post_measurement_error() {
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
            .with_read(600); // ABN_DECODER_COUNT after -- exact half turn from 100 at ppr 1000
        let mut tmc = Tmc4671Spi::new(raw_spi);
        let mut runtime = ready_runtime();
        let mut delay = MockDelay::default();

        let outcome = run_current_step_request(
            tmc_current_step_test(0, 7, CurrentStepAxis::Torque, 20, 3, 555),
            DiagnosticContext {
                host_liveness_expired: None,
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                power_stage: &mut MockDiagnosticPowerStage::default(),
                tmc_config: &TEST_TMC_CONFIG,
                feedback_config: crate::config::FeedbackConfig::abn_abn(),
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
        assert_eq!(report.status, CURRENT_STEP_STATUS_SPI);
    });
}

#[test]
fn current_step_host_liveness_expiry_reports_dedicated_status_and_restores() {
    pollster::block_on(async {
        // The pre-check reads (saved mode, saved position) are followed,
        // after the pre-settle delay succeeds, by encoder-before, encoder
        // PPR (must be non-zero), and torque-before -- all reached before
        // the main delay this test targets.
        let raw_spi = DiagnosticMockSpi::default()
            .with_read(0x8000_0003) // saved mode
            .with_read(555) // saved position actual
            .with_read(100) // encoder_before (ABN_DECODER_COUNT)
            .with_read(1000) // encoder PPR (ABN_DECODER_PPR) -- must be non-zero
            .with_read(0); // torque_before (PID_TORQUE_FLUX_ACTUAL)
        let mut tmc = Tmc4671Spi::new(raw_spi);
        let mut runtime = ready_runtime();
        let mut delay = MockDelay::default();
        // duration_ms=150 makes the main delay 2 slices ([100, 50]); the
        // pre-settle delay (CURRENT_STEP_SETTLE_MS=20, always 1 slice) is
        // let through so this test reaches the SECOND checked-delay call
        // site, not just the first, and exercises the final slice of a
        // genuinely multi-slice delay. Checks: pre-settle pre-loop (1),
        // pre-settle post-slice (2) -- both false, letting 20ms through --
        // main pre-loop (3), main post-slice-1/100ms (4) -- both false --
        // main post-slice-2/50ms, the FINAL slice (5) -- true.
        let mut calls = 0u32;
        let mut expire_during_main_delay_final_slice = || {
            calls += 1;
            calls >= 5
        };
        let req = tmc_current_step_test(0, 7, CurrentStepAxis::Torque, 123, 150, 500);

        let outcome = run_current_step_request(
            req,
            DiagnosticContext {
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                power_stage: &mut MockDiagnosticPowerStage::default(),
                tmc_config: &TEST_TMC_CONFIG,
                feedback_config: crate::config::FeedbackConfig::abn_abn(),
                readiness: DiagnosticReadiness {
                    homing_active: false,
                    stepper_active: false,
                    saved_voltage_limit: 777,
                },
                host_liveness_expired: Some(&mut expire_during_main_delay_final_slice),
            },
        )
        .await;

        let DiagnosticOutcome::Handled(DiagnosticReport::CurrentStep { report, .. }) = outcome
        else {
            panic!("expected current step report");
        };
        assert_eq!(report.status, CURRENT_STEP_STATUS_HOST_LIVENESS);
        // Proves the slicing actually ran inside this runner, not just
        // inside the shared helper's own unit tests: pre-settle (20ms,
        // one slice) plus two main-delay slices (100ms, 50ms) before
        // detection on the final one.
        assert_eq!(delay.delays_ms, std::vec![20, 100, 50]);

        // Same unconditional restore tail a genuine SPI failure already
        // triggers: torque/flux zeroed, voltage limit and mode restored.
        // The setup writes before the first checked delay (target zero,
        // voltage limit, integrator clears, mode) already happened for
        // real; assert only the trailing restore sequence, not the whole
        // vector, so this test doesn't also have to hardcode the
        // integrator sub-register write pairs.
        let raw_spi = tmc.into_inner();
        let restore_tail = &raw_spi.write_attempts[raw_spi.write_attempts.len() - 4..];
        assert_eq!(
            restore_tail,
            [
                (PID_TORQUE_FLUX_TARGET.addr(), 0),
                (PIDOUT_UQ_UD_LIMITS.addr(), 777),
                (PID_POSITION_TARGET.addr(), 555),
                (MODE_RAMP_MODE_MOTION.addr(), 0x8000_0003),
            ]
        );
    });
}

#[test]
fn current_step_pre_settle_host_liveness_expiry_reports_dedicated_status() {
    // Distinct from the test above: this one expires on the pre-settle
    // delay itself (the FIRST checked-delay call site), catching a
    // mistake where that specific call was left as a raw, unchecked
    // `delay.delay_ms(...)` while the main-duration one was correctly
    // converted -- the two call sites are independent substitutions and
    // a per-site regression at only one of them would not be caught by
    // testing only the other.
    pollster::block_on(async {
        let raw_spi = DiagnosticMockSpi::default()
            .with_read(0x8000_0003) // saved mode
            .with_read(555); // saved position actual
        let mut tmc = Tmc4671Spi::new(raw_spi);
        let mut runtime = ready_runtime();
        let mut delay = MockDelay::default();
        let mut expire_immediately = || true;
        let req = tmc_current_step_test(0, 7, CurrentStepAxis::Torque, 123, 150, 500);

        let outcome = run_current_step_request(
            req,
            DiagnosticContext {
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                power_stage: &mut MockDiagnosticPowerStage::default(),
                tmc_config: &TEST_TMC_CONFIG,
                feedback_config: crate::config::FeedbackConfig::abn_abn(),
                readiness: DiagnosticReadiness {
                    homing_active: false,
                    stepper_active: false,
                    saved_voltage_limit: 777,
                },
                host_liveness_expired: Some(&mut expire_immediately),
            },
        )
        .await;

        let DiagnosticOutcome::Handled(DiagnosticReport::CurrentStep { report, .. }) = outcome
        else {
            panic!("expected current step report");
        };
        assert_eq!(report.status, CURRENT_STEP_STATUS_HOST_LIVENESS);
        // No delay was ever recorded -- the very first liveness check,
        // before the pre-settle delay's first slice, already caught it.
        assert!(delay.delays_ms.is_empty());
    });
}

#[test]
fn current_step_post_settle_host_liveness_expiry_reports_dedicated_status() {
    // Third and last call site in this function. `DiagnosticMockSpi`'s
    // unstocked reads fall back to `register_values` (populated by prior
    // writes to that address) or 0 (see its `Operation::Read` arm) --
    // none of the INTERIM_* reads between the main delay and this one
    // gate on a specific value (they are only stored into report fields),
    // so no additional mock reads are needed beyond the same 5 this
    // function's other two tests already use.
    pollster::block_on(async {
        let raw_spi = DiagnosticMockSpi::default()
            .with_read(0x8000_0003) // saved mode
            .with_read(555) // saved position actual
            .with_read(100) // encoder_before (ABN_DECODER_COUNT)
            .with_read(1000) // encoder PPR (ABN_DECODER_PPR) -- must be non-zero
            .with_read(0); // torque_before (PID_TORQUE_FLUX_ACTUAL)
        let mut tmc = Tmc4671Spi::new(raw_spi);
        let mut runtime = ready_runtime();
        let mut delay = MockDelay::default();
        // Checks: pre-settle pre-loop/post-slice (1, 2), main pre-loop/
        // post-slice (3, 4) -- all false, letting both 20ms delays and
        // every intervening register read through -- post-settle
        // pre-loop (5) -- true.
        let mut calls = 0u32;
        let mut expire_at_post_settle = || {
            calls += 1;
            calls >= 5
        };
        let req = tmc_current_step_test(0, 7, CurrentStepAxis::Torque, 123, 20, 500);

        let outcome = run_current_step_request(
            req,
            DiagnosticContext {
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                power_stage: &mut MockDiagnosticPowerStage::default(),
                tmc_config: &TEST_TMC_CONFIG,
                feedback_config: crate::config::FeedbackConfig::abn_abn(),
                readiness: DiagnosticReadiness {
                    homing_active: false,
                    stepper_active: false,
                    saved_voltage_limit: 777,
                },
                host_liveness_expired: Some(&mut expire_at_post_settle),
            },
        )
        .await;

        let DiagnosticOutcome::Handled(DiagnosticReport::CurrentStep { report, .. }) = outcome
        else {
            panic!("expected current step report");
        };
        assert_eq!(report.status, CURRENT_STEP_STATUS_HOST_LIVENESS);
        // Pre-settle and main delays (20ms each, single-slice) both ran in
        // full; post-settle never started.
        assert_eq!(delay.delays_ms, std::vec![20, 20]);

        let raw_spi = tmc.into_inner();
        let restore_tail = &raw_spi.write_attempts[raw_spi.write_attempts.len() - 4..];
        assert_eq!(
            restore_tail,
            [
                (PID_TORQUE_FLUX_TARGET.addr(), 0),
                (PIDOUT_UQ_UD_LIMITS.addr(), 777),
                (PID_POSITION_TARGET.addr(), 555),
                (MODE_RAMP_MODE_MOTION.addr(), 0x8000_0003),
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
                host_liveness_expired: None,
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                power_stage: &mut MockDiagnosticPowerStage::default(),
                tmc_config: &TEST_TMC_CONFIG,
                feedback_config: crate::config::FeedbackConfig::abn_abn(),
                readiness: DiagnosticReadiness {
                    homing_active: false,
                    stepper_active: false,
                    saved_voltage_limit: 777,
                },
            },
        )
        .await;

        let DiagnosticOutcome::Handled(DiagnosticReport::CurrentTorqueSample(report)) = outcome
        else {
            panic!("expected torque sample report");
        };
        assert_eq!(report.status, CURRENT_STEP_STATUS_NOT_READY);
        assert_eq!(report.kind, TorqueSampleKind::CurrentTorqueSample);
    });
}

#[test]
fn current_torque_sample_invalid_ppr_aborts_before_excitation() {
    pollster::block_on(async {
        let raw_spi = DiagnosticMockSpi::default()
            .with_read(0) // saved MODE_RAMP_MODE_MOTION
            .with_read(0xaaaa_bbbb) // PID_POSITION_ACTUAL
            .with_read(100) // ABN_DECODER_COUNT before
            .with_read(0); // ABN_DECODER_PPR -- invalid
        let mut tmc = Tmc4671Spi::new(raw_spi);
        let mut runtime = ready_runtime();
        let mut delay = MockDelay::default();

        let outcome = run_current_torque_sample_request(
            tmc_current_torque_sample_test(0, 4, 22, 0, 3, 444),
            DiagnosticContext {
                host_liveness_expired: None,
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                power_stage: &mut MockDiagnosticPowerStage::default(),
                tmc_config: &TEST_TMC_CONFIG,
                feedback_config: crate::config::FeedbackConfig::abn_abn(),
                readiness: DiagnosticReadiness {
                    homing_active: false,
                    stepper_active: false,
                    saved_voltage_limit: 777,
                },
            },
        )
        .await;

        let DiagnosticOutcome::Handled(DiagnosticReport::CurrentTorqueSample(report)) = outcome
        else {
            panic!("expected torque sample report");
        };
        assert_eq!(report.status, CURRENT_STEP_STATUS_SPI);
        let raw_spi = tmc.into_inner();
        assert!(!raw_spi.write_attempts.contains(&(
            PID_TORQUE_FLUX_TARGET.addr(),
            crate::tmc::pack_torque_flux_target(22, 0),
        )));
    });
}

#[test]
fn current_torque_sample_exact_half_turn_aborts_via_post_measurement_error() {
    pollster::block_on(async {
        let raw_spi = DiagnosticMockSpi::default()
            .with_read(0) // saved MODE_RAMP_MODE_MOTION
            .with_read(0xaaaa_bbbb) // PID_POSITION_ACTUAL
            .with_read(100) // ABN_DECODER_COUNT before
            .with_read(1000) // ABN_DECODER_PPR
            .with_read(0) // PID_TORQUE_FLUX_ACTUAL before
            .with_read(0) // INTERIM_PIDIN_TARGET_TORQUE
            .with_read(0) // INTERIM_PIDIN_TARGET_FLUX
            .with_read(0) // INTERIM_PIDOUT_TARGET_TORQUE
            .with_read(0) // INTERIM_PIDOUT_TARGET_FLUX
            .with_read(0) // INTERIM_PID_TORQUE_TARGET_TORQUE_ACTUAL
            .with_read(0) // PID_ERROR_PID_TORQUE_ERROR
            .with_read(0) // PID_ERROR_PID_FLUX_ERROR
            .with_read(0) // PID_ERROR_PID_TORQUE_ERROR_SUM
            .with_read(0) // PID_ERROR_PID_FLUX_ERROR_SUM
            .with_read(0) // STATUS_FLAGS
            .with_read(600); // ABN_DECODER_COUNT sample -- exact half turn from 100 at ppr 1000
        let mut tmc = Tmc4671Spi::new(raw_spi);
        let mut runtime = ready_runtime();
        let mut delay = MockDelay::default();

        let outcome = run_current_torque_sample_request(
            tmc_current_torque_sample_test(0, 4, 22, 0, 3, 444),
            DiagnosticContext {
                host_liveness_expired: None,
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                power_stage: &mut MockDiagnosticPowerStage::default(),
                tmc_config: &TEST_TMC_CONFIG,
                feedback_config: crate::config::FeedbackConfig::abn_abn(),
                readiness: DiagnosticReadiness {
                    homing_active: false,
                    stepper_active: false,
                    saved_voltage_limit: 777,
                },
            },
        )
        .await;

        let DiagnosticOutcome::Handled(DiagnosticReport::CurrentTorqueSample(report)) = outcome
        else {
            panic!("expected torque sample report");
        };
        assert_eq!(report.status, CURRENT_STEP_STATUS_SPI);
    });
}

#[test]
fn current_torque_sample_success_reports_current_torque_sample_kind() {
    pollster::block_on(async {
        let raw_spi = DiagnosticMockSpi::default()
            .with_read(0) // saved MODE_RAMP_MODE_MOTION
            .with_read(0xaaaa_bbbb) // PID_POSITION_ACTUAL
            .with_read(100) // ABN_DECODER_COUNT before
            .with_read(1000) // ABN_DECODER_PPR
            .with_read(0x0005_0000) // PID_TORQUE_FLUX_ACTUAL before
            .with_read(0) // INTERIM_PIDIN_TARGET_TORQUE
            .with_read(0) // INTERIM_PIDIN_TARGET_FLUX
            .with_read(0) // INTERIM_PIDOUT_TARGET_TORQUE
            .with_read(0) // INTERIM_PIDOUT_TARGET_FLUX
            .with_read(0) // INTERIM_PID_TORQUE_TARGET_TORQUE_ACTUAL
            .with_read(0) // PID_ERROR_PID_TORQUE_ERROR
            .with_read(0) // PID_ERROR_PID_FLUX_ERROR
            .with_read(0) // PID_ERROR_PID_TORQUE_ERROR_SUM
            .with_read(0) // PID_ERROR_PID_FLUX_ERROR_SUM
            .with_read(0) // STATUS_FLAGS
            .with_read(110) // ABN_DECODER_COUNT sample
            .with_read(0x0009_000a) // PID_TORQUE_FLUX_ACTUAL sample
            .with_read(0x000b_000c) // INTERIM_FOC_IQ_ID
            .with_read(0x000d_000e) // INTERIM_FOC_UQ_UD
            .with_read(0) // INTERIM_FF_VELOCITY
            .with_read(0) // INTERIM_FF_TORQUE
            .with_read(0x000f_0010) // INTERIM_FOC_UQ_UD_LIMITED
            .with_read(0x0000_1234) // ADC_AGPI_A_RAW_ADC_VM_RAW
            .with_read(120) // ABN_DECODER_COUNT after
            .with_read(0x0006_0000); // PID_TORQUE_FLUX_ACTUAL after
        let mut tmc = Tmc4671Spi::new(raw_spi);
        let mut runtime = ready_runtime();
        let mut delay = MockDelay::default();

        let outcome = run_current_torque_sample_request(
            tmc_current_torque_sample_test(0, 4, 22, 0, 3, 444),
            DiagnosticContext {
                host_liveness_expired: None,
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                power_stage: &mut MockDiagnosticPowerStage::default(),
                tmc_config: &TEST_TMC_CONFIG,
                feedback_config: crate::config::FeedbackConfig::abn_abn(),
                readiness: DiagnosticReadiness {
                    homing_active: false,
                    stepper_active: false,
                    saved_voltage_limit: 777,
                },
            },
        )
        .await;

        let DiagnosticOutcome::Handled(DiagnosticReport::CurrentTorqueSample(report)) = outcome
        else {
            panic!("expected torque sample report");
        };
        assert_eq!(report.status, 0);
        assert_eq!(report.kind, TorqueSampleKind::CurrentTorqueSample);
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
                host_liveness_expired: None,
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                power_stage: &mut MockDiagnosticPowerStage::default(),
                tmc_config: &TEST_TMC_CONFIG,
                feedback_config: crate::config::FeedbackConfig::abn_abn(),
                readiness: DiagnosticReadiness {
                    homing_active: false,
                    stepper_active: false,
                    saved_voltage_limit: 777,
                },
            },
        )
        .await;

        let DiagnosticOutcome::Handled(DiagnosticReport::CurrentTorqueSample(report)) = outcome
        else {
            panic!("expected position offset report");
        };
        assert_eq!(report.status, CURRENT_STEP_STATUS_NOT_READY);
        assert_eq!(report.kind, TorqueSampleKind::PositionTorqueOffset);
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
                host_liveness_expired: None,
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                power_stage: &mut MockDiagnosticPowerStage::default(),
                tmc_config: &TEST_TMC_CONFIG,
                feedback_config: crate::config::FeedbackConfig::abn_abn(),
                readiness: DiagnosticReadiness {
                    homing_active: false,
                    stepper_active: false,
                    saved_voltage_limit: 777,
                },
            },
        )
        .await;

        let DiagnosticOutcome::Handled(DiagnosticReport::CurrentTorqueSample(report)) = outcome
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
fn position_torque_offset_sample_invalid_ppr_aborts_before_excitation() {
    pollster::block_on(async {
        let raw_spi = DiagnosticMockSpi::default()
            .with_read(0x8000_0003) // saved mode
            .with_read(0x1234_5678) // saved offset
            .with_read(100) // ABN_DECODER_COUNT before
            .with_read(0); // ABN_DECODER_PPR -- invalid
        let mut tmc = Tmc4671Spi::new(raw_spi);
        let mut runtime = ready_runtime();
        let mut delay = MockDelay::default();

        let outcome = run_position_torque_offset_sample_request(
            tmc_position_torque_offset_sample_test(0, 4, 33, 3, 444),
            DiagnosticContext {
                host_liveness_expired: None,
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                power_stage: &mut MockDiagnosticPowerStage::default(),
                tmc_config: &TEST_TMC_CONFIG,
                feedback_config: crate::config::FeedbackConfig::abn_abn(),
                readiness: DiagnosticReadiness {
                    homing_active: false,
                    stepper_active: false,
                    saved_voltage_limit: 777,
                },
            },
        )
        .await;

        let DiagnosticOutcome::Handled(DiagnosticReport::CurrentTorqueSample(report)) = outcome
        else {
            panic!("expected position offset report");
        };
        assert_eq!(report.status, CURRENT_STEP_STATUS_SPI);
        let raw_spi = tmc.into_inner();
        let offset_reg = u32::from(33i16 as u16) << 16;
        assert!(
            !raw_spi
                .write_attempts
                .contains(&(PID_TORQUE_FLUX_OFFSET.addr(), offset_reg))
        );
    });
}

#[test]
fn position_torque_offset_sample_exact_half_turn_aborts_via_post_measurement_error() {
    pollster::block_on(async {
        let raw_spi = DiagnosticMockSpi::default()
            .with_read(0x8000_0003) // saved mode
            .with_read(0x1234_5678) // saved offset
            .with_read(100) // ABN_DECODER_COUNT before
            .with_read(1000) // ABN_DECODER_PPR
            .with_read(0) // PID_TORQUE_FLUX_ACTUAL before
            .with_read(0) // INTERIM_PIDIN_TARGET_TORQUE
            .with_read(0) // INTERIM_PIDIN_TARGET_FLUX
            .with_read(0) // INTERIM_PIDOUT_TARGET_TORQUE
            .with_read(0) // INTERIM_PIDOUT_TARGET_FLUX
            .with_read(0) // INTERIM_PID_TORQUE_TARGET_TORQUE_ACTUAL
            .with_read(0) // PID_ERROR_PID_TORQUE_ERROR
            .with_read(0) // PID_ERROR_PID_FLUX_ERROR
            .with_read(0) // PID_ERROR_PID_TORQUE_ERROR_SUM
            .with_read(0) // PID_ERROR_PID_FLUX_ERROR_SUM
            .with_read(0) // STATUS_FLAGS
            .with_read(600); // ABN_DECODER_COUNT sample -- exact half turn from 100 at ppr 1000
        let mut tmc = Tmc4671Spi::new(raw_spi);
        let mut runtime = ready_runtime();
        let mut delay = MockDelay::default();

        let outcome = run_position_torque_offset_sample_request(
            tmc_position_torque_offset_sample_test(0, 4, 33, 3, 444),
            DiagnosticContext {
                host_liveness_expired: None,
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                power_stage: &mut MockDiagnosticPowerStage::default(),
                tmc_config: &TEST_TMC_CONFIG,
                feedback_config: crate::config::FeedbackConfig::abn_abn(),
                readiness: DiagnosticReadiness {
                    homing_active: false,
                    stepper_active: false,
                    saved_voltage_limit: 777,
                },
            },
        )
        .await;

        let DiagnosticOutcome::Handled(DiagnosticReport::CurrentTorqueSample(report)) = outcome
        else {
            panic!("expected position offset report");
        };
        assert_eq!(report.status, CURRENT_STEP_STATUS_SPI);
    });
}

#[test]
fn position_torque_offset_sample_success_reports_position_torque_offset_kind() {
    pollster::block_on(async {
        let raw_spi = DiagnosticMockSpi::default()
            .with_read(0x8000_0003) // saved MODE_RAMP_MODE_MOTION -- must be position mode
            .with_read(0x1234_5678) // saved PID_TORQUE_FLUX_OFFSET
            .with_read(100) // ABN_DECODER_COUNT before
            .with_read(1000) // ABN_DECODER_PPR
            .with_read(0x0005_0000) // PID_TORQUE_FLUX_ACTUAL before
            .with_read(0) // INTERIM_PIDIN_TARGET_TORQUE
            .with_read(0) // INTERIM_PIDIN_TARGET_FLUX
            .with_read(0) // INTERIM_PIDOUT_TARGET_TORQUE
            .with_read(0) // INTERIM_PIDOUT_TARGET_FLUX
            .with_read(0) // INTERIM_PID_TORQUE_TARGET_TORQUE_ACTUAL
            .with_read(0) // PID_ERROR_PID_TORQUE_ERROR
            .with_read(0) // PID_ERROR_PID_FLUX_ERROR
            .with_read(0) // PID_ERROR_PID_TORQUE_ERROR_SUM
            .with_read(0) // PID_ERROR_PID_FLUX_ERROR_SUM
            .with_read(0) // STATUS_FLAGS
            .with_read(110) // ABN_DECODER_COUNT sample
            .with_read(0x0009_000a) // PID_TORQUE_FLUX_ACTUAL sample
            .with_read(0x000b_000c) // INTERIM_FOC_IQ_ID
            .with_read(0x000d_000e) // INTERIM_FOC_UQ_UD
            .with_read(0) // INTERIM_FF_VELOCITY
            .with_read(0) // INTERIM_FF_TORQUE
            .with_read(0x000f_0010) // INTERIM_FOC_UQ_UD_LIMITED
            .with_read(0x0000_1234) // ADC_AGPI_A_RAW_ADC_VM_RAW
            .with_read(120) // ABN_DECODER_COUNT after
            .with_read(0x0006_0000); // PID_TORQUE_FLUX_ACTUAL after
        let mut tmc = Tmc4671Spi::new(raw_spi);
        let mut runtime = ready_runtime();
        let mut delay = MockDelay::default();

        let outcome = run_position_torque_offset_sample_request(
            tmc_position_torque_offset_sample_test(0, 4, 33, 3, 444),
            DiagnosticContext {
                host_liveness_expired: None,
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                power_stage: &mut MockDiagnosticPowerStage::default(),
                tmc_config: &TEST_TMC_CONFIG,
                feedback_config: crate::config::FeedbackConfig::abn_abn(),
                readiness: DiagnosticReadiness {
                    homing_active: false,
                    stepper_active: false,
                    saved_voltage_limit: 777,
                },
            },
        )
        .await;

        let DiagnosticOutcome::Handled(DiagnosticReport::CurrentTorqueSample(report)) = outcome
        else {
            panic!("expected position offset report");
        };
        assert_eq!(report.status, 0);
        assert_eq!(report.kind, TorqueSampleKind::PositionTorqueOffset);
    });
}

#[test]
fn current_torque_sample_host_liveness_expiry_reports_dedicated_status_and_restores() {
    pollster::block_on(async {
        // Same shape as current-step: after the pre-settle delay succeeds,
        // encoder-before, encoder PPR (non-zero), and torque-before are
        // read before the main delay this test targets.
        let raw_spi = DiagnosticMockSpi::default()
            .with_read(0x8000_0003) // saved mode
            .with_read(555) // saved position actual
            .with_read(100) // encoder_before (ABN_DECODER_COUNT)
            .with_read(1000) // encoder PPR (ABN_DECODER_PPR) -- must be non-zero
            .with_read(0); // torque_before (PID_TORQUE_FLUX_ACTUAL)
        let mut tmc = Tmc4671Spi::new(raw_spi);
        let mut runtime = ready_runtime();
        let mut delay = MockDelay::default();
        // sample_delay_ms=150 makes the main delay 2 slices ([100, 50]);
        // the pre-settle delay (20ms, 1 slice) is let through so this test
        // reaches the SECOND checked-delay call site and its final slice,
        // same reasoning as the current-step test in Task 3.
        let mut calls = 0u32;
        let mut expire_during_main_delay_final_slice = || {
            calls += 1;
            calls >= 5
        };
        let req = tmc_current_torque_sample_test(0, 7, 100, 50, 150, 500);

        let outcome = run_current_torque_sample_request(
            req,
            DiagnosticContext {
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                power_stage: &mut MockDiagnosticPowerStage::default(),
                tmc_config: &TEST_TMC_CONFIG,
                feedback_config: crate::config::FeedbackConfig::abn_abn(),
                readiness: DiagnosticReadiness {
                    homing_active: false,
                    stepper_active: false,
                    saved_voltage_limit: 777,
                },
                host_liveness_expired: Some(&mut expire_during_main_delay_final_slice),
            },
        )
        .await;

        let DiagnosticOutcome::Handled(DiagnosticReport::CurrentTorqueSample(report)) = outcome
        else {
            panic!("expected torque sample report");
        };
        assert_eq!(report.status, CURRENT_STEP_STATUS_HOST_LIVENESS);
        assert_eq!(delay.delays_ms, std::vec![20, 100, 50]);

        // Same restore tail as current-step: target zeroed, voltage limit
        // and position target restored, mode restored.
        let raw_spi = tmc.into_inner();
        let restore_tail = &raw_spi.write_attempts[raw_spi.write_attempts.len() - 4..];
        assert_eq!(
            restore_tail,
            [
                (PID_TORQUE_FLUX_TARGET.addr(), 0),
                (PIDOUT_UQ_UD_LIMITS.addr(), 777),
                (PID_POSITION_TARGET.addr(), 555),
                (MODE_RAMP_MODE_MOTION.addr(), 0x8000_0003),
            ]
        );
    });
}

#[test]
fn position_offset_host_liveness_expiry_reports_dedicated_status_and_restores() {
    pollster::block_on(async {
        // `run_position_torque_offset_sample_request` has no pre-settle
        // delay -- its first (and only, before the post-settle) checked
        // delay is the sample delay, reached only after reading saved
        // mode, saved offset, encoder-before, encoder PPR (must be
        // non-zero or EncoderDomain::new rejects it before the delay is
        // ever reached), and torque-before, in that order. All five reads
        // must be queued for the mock to get there.
        let raw_spi = DiagnosticMockSpi::default()
            .with_read(0x8000_0003) // saved mode
            .with_read(0x1234_5678) // saved offset
            .with_read(100) // encoder_before (ABN_DECODER_COUNT)
            .with_read(1000) // encoder PPR (ABN_DECODER_PPR) -- must be non-zero
            .with_read(0); // torque_before (PID_TORQUE_FLUX_ACTUAL)
        let mut tmc = Tmc4671Spi::new(raw_spi);
        let mut runtime = ready_runtime();
        let mut delay = MockDelay::default();
        // sample_delay_ms=150 is this function's FIRST checked delay (no
        // pre-settle here), so 2 slices ([100, 50]): main pre-loop (1),
        // post-slice-1/100ms (2), post-slice-2/50ms -- the final slice -- (3).
        let mut calls = 0u32;
        let mut expire_during_main_delay_final_slice = || {
            calls += 1;
            calls >= 3
        };

        let outcome = run_position_torque_offset_sample_request(
            tmc_position_torque_offset_sample_test(0, 4, 33, 150, 444),
            DiagnosticContext {
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                power_stage: &mut MockDiagnosticPowerStage::default(),
                tmc_config: &TEST_TMC_CONFIG,
                feedback_config: crate::config::FeedbackConfig::abn_abn(),
                readiness: DiagnosticReadiness {
                    homing_active: false,
                    stepper_active: false,
                    saved_voltage_limit: 777,
                },
                host_liveness_expired: Some(&mut expire_during_main_delay_final_slice),
            },
        )
        .await;

        let DiagnosticOutcome::Handled(DiagnosticReport::CurrentTorqueSample(report)) = outcome
        else {
            panic!("expected position offset report");
        };
        assert_eq!(report.status, CURRENT_STEP_STATUS_HOST_LIVENESS);
        assert_eq!(delay.delays_ms, std::vec![100, 50]);

        // The unconditional restore tail is 3 writes (verified against
        // the current source): saved offset restored, saved voltage
        // limit restored, saved mode restored -- in that order. The
        // 150ms retarget means, unlike the existing
        // position_offset_spi_failure_attempts_offset_restore test (whose
        // injected failure is the very FIRST write, so its expected
        // vector has no preceding setup writes to skip), this run's
        // setup writes (voltage limit, STATUS_FLAGS clear, the requested
        // offset) succeed for real before the delay fails -- assert only
        // the trailing 3-write restore sequence, not the whole vector.
        let raw_spi = tmc.into_inner();
        let restore_tail = &raw_spi.write_attempts[raw_spi.write_attempts.len() - 3..];
        assert_eq!(
            restore_tail,
            [
                (PID_TORQUE_FLUX_OFFSET.addr(), 0x1234_5678),
                (PIDOUT_UQ_UD_LIMITS.addr(), 777),
                (MODE_RAMP_MODE_MOTION.addr(), 0x8000_0003),
            ]
        );
    });
}

#[test]
fn current_torque_sample_pre_settle_host_liveness_expiry_reports_dedicated_status() {
    // Same reasoning as current_step_pre_settle_host_liveness_expiry_reports_dedicated_status
    // in Task 3: an independent call site (this function's own pre-settle
    // delay) needs its own regression coverage, since a mistake converting
    // only the main delay would not be caught by the test above.
    pollster::block_on(async {
        let raw_spi = DiagnosticMockSpi::default()
            .with_read(0x8000_0003) // saved mode
            .with_read(555); // saved position actual
        let mut tmc = Tmc4671Spi::new(raw_spi);
        let mut runtime = ready_runtime();
        let mut delay = MockDelay::default();
        let mut expire_immediately = || true;
        let req = tmc_current_torque_sample_test(0, 7, 100, 50, 150, 500);

        let outcome = run_current_torque_sample_request(
            req,
            DiagnosticContext {
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                power_stage: &mut MockDiagnosticPowerStage::default(),
                tmc_config: &TEST_TMC_CONFIG,
                feedback_config: crate::config::FeedbackConfig::abn_abn(),
                readiness: DiagnosticReadiness {
                    homing_active: false,
                    stepper_active: false,
                    saved_voltage_limit: 777,
                },
                host_liveness_expired: Some(&mut expire_immediately),
            },
        )
        .await;

        let DiagnosticOutcome::Handled(DiagnosticReport::CurrentTorqueSample(report)) = outcome
        else {
            panic!("expected torque sample report");
        };
        assert_eq!(report.status, CURRENT_STEP_STATUS_HOST_LIVENESS);
        assert!(delay.delays_ms.is_empty());
    });
}

#[test]
fn current_torque_sample_post_settle_host_liveness_expiry_reports_dedicated_status() {
    // Third call site, same reasoning and mock-exhaustion-defaults-to-0
    // argument as current_step_post_settle_host_liveness_expiry_reports_dedicated_status.
    pollster::block_on(async {
        let raw_spi = DiagnosticMockSpi::default()
            .with_read(0x8000_0003) // saved mode
            .with_read(555) // saved position actual
            .with_read(100) // encoder_before (ABN_DECODER_COUNT)
            .with_read(1000) // encoder PPR (ABN_DECODER_PPR) -- must be non-zero
            .with_read(0); // torque_before (PID_TORQUE_FLUX_ACTUAL)
        let mut tmc = Tmc4671Spi::new(raw_spi);
        let mut runtime = ready_runtime();
        let mut delay = MockDelay::default();
        let mut calls = 0u32;
        let mut expire_at_post_settle = || {
            calls += 1;
            calls >= 5
        };
        let req = tmc_current_torque_sample_test(0, 7, 100, 50, 20, 500);

        let outcome = run_current_torque_sample_request(
            req,
            DiagnosticContext {
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                power_stage: &mut MockDiagnosticPowerStage::default(),
                tmc_config: &TEST_TMC_CONFIG,
                feedback_config: crate::config::FeedbackConfig::abn_abn(),
                readiness: DiagnosticReadiness {
                    homing_active: false,
                    stepper_active: false,
                    saved_voltage_limit: 777,
                },
                host_liveness_expired: Some(&mut expire_at_post_settle),
            },
        )
        .await;

        let DiagnosticOutcome::Handled(DiagnosticReport::CurrentTorqueSample(report)) = outcome
        else {
            panic!("expected torque sample report");
        };
        assert_eq!(report.status, CURRENT_STEP_STATUS_HOST_LIVENESS);
        assert_eq!(delay.delays_ms, std::vec![20, 20]);

        let raw_spi = tmc.into_inner();
        let restore_tail = &raw_spi.write_attempts[raw_spi.write_attempts.len() - 4..];
        assert_eq!(
            restore_tail,
            [
                (PID_TORQUE_FLUX_TARGET.addr(), 0),
                (PIDOUT_UQ_UD_LIMITS.addr(), 777),
                (PID_POSITION_TARGET.addr(), 555),
                (MODE_RAMP_MODE_MOTION.addr(), 0x8000_0003),
            ]
        );
    });
}

#[test]
fn position_offset_post_settle_host_liveness_expiry_reports_dedicated_status() {
    // No pre-settle delay in this function, so this is its second and
    // last call site.
    pollster::block_on(async {
        let raw_spi = DiagnosticMockSpi::default()
            .with_read(0x8000_0003) // saved mode
            .with_read(0x1234_5678) // saved offset
            .with_read(100) // encoder_before (ABN_DECODER_COUNT)
            .with_read(1000) // encoder PPR (ABN_DECODER_PPR) -- must be non-zero
            .with_read(0); // torque_before (PID_TORQUE_FLUX_ACTUAL)
        let mut tmc = Tmc4671Spi::new(raw_spi);
        let mut runtime = ready_runtime();
        let mut delay = MockDelay::default();
        // Checks: main pre-loop/post-slice (1, 2) -- both false, letting
        // the 20ms sample delay and every intervening read through --
        // post-settle pre-loop (3) -- true.
        let mut calls = 0u32;
        let mut expire_at_post_settle = || {
            calls += 1;
            calls >= 3
        };

        let outcome = run_position_torque_offset_sample_request(
            tmc_position_torque_offset_sample_test(0, 4, 33, 20, 444),
            DiagnosticContext {
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                power_stage: &mut MockDiagnosticPowerStage::default(),
                tmc_config: &TEST_TMC_CONFIG,
                feedback_config: crate::config::FeedbackConfig::abn_abn(),
                readiness: DiagnosticReadiness {
                    homing_active: false,
                    stepper_active: false,
                    saved_voltage_limit: 777,
                },
                host_liveness_expired: Some(&mut expire_at_post_settle),
            },
        )
        .await;

        let DiagnosticOutcome::Handled(DiagnosticReport::CurrentTorqueSample(report)) = outcome
        else {
            panic!("expected position offset report");
        };
        assert_eq!(report.status, CURRENT_STEP_STATUS_HOST_LIVENESS);
        assert_eq!(delay.delays_ms, std::vec![20]);

        let raw_spi = tmc.into_inner();
        let restore_tail = &raw_spi.write_attempts[raw_spi.write_attempts.len() - 3..];
        assert_eq!(
            restore_tail,
            [
                (PID_TORQUE_FLUX_OFFSET.addr(), 0x1234_5678),
                (PIDOUT_UQ_UD_LIMITS.addr(), 777),
                (MODE_RAMP_MODE_MOTION.addr(), 0x8000_0003),
            ]
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
                host_liveness_expired: None,
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                power_stage: &mut MockDiagnosticPowerStage::default(),
                tmc_config: &TEST_TMC_CONFIG,
                feedback_config: crate::config::FeedbackConfig::abn_abn(),
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
                host_liveness_expired: None,
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                power_stage: &mut MockDiagnosticPowerStage::default(),
                tmc_config: &TEST_TMC_CONFIG,
                feedback_config: crate::config::FeedbackConfig::abn_abn(),
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
fn voltage_step_invalid_ppr_aborts_before_excitation() {
    pollster::block_on(async {
        let raw_spi = DiagnosticMockSpi::default()
            .with_read(0x8000_0003) // saved mode
            .with_read(100) // encoder before
            .with_read(0); // ppr -- invalid
        let mut tmc = Tmc4671Spi::new(raw_spi);
        let mut runtime = ready_runtime();
        let mut delay = MockDelay::default();

        let outcome = run_voltage_step_request(
            tmc_voltage_step_test(0, 3, 44, -55, 6),
            DiagnosticContext {
                host_liveness_expired: None,
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                power_stage: &mut MockDiagnosticPowerStage::default(),
                tmc_config: &TEST_TMC_CONFIG,
                feedback_config: crate::config::FeedbackConfig::abn_abn(),
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
        assert!(
            !raw_spi
                .write_attempts
                .contains(&(UQ_UD_EXT.addr(), pack_voltage_ext(44, -55)))
        );
    });
}

#[test]
fn voltage_step_exact_half_turn_aborts_via_post_measurement_error() {
    pollster::block_on(async {
        let raw_spi = DiagnosticMockSpi::default()
            .with_read(0x8000_0003) // saved mode
            .with_read(100) // encoder before
            .with_read(1000) // ppr
            .with_read(0x0001_0000) // torque before
            .with_read(600); // encoder sample -- exact half turn from 100 at ppr 1000
        let mut tmc = Tmc4671Spi::new(raw_spi);
        let mut runtime = ready_runtime();
        let mut delay = MockDelay::default();

        let outcome = run_voltage_step_request(
            tmc_voltage_step_test(0, 3, 44, -55, 6),
            DiagnosticContext {
                host_liveness_expired: None,
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                power_stage: &mut MockDiagnosticPowerStage::default(),
                tmc_config: &TEST_TMC_CONFIG,
                feedback_config: crate::config::FeedbackConfig::abn_abn(),
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
    });
}

#[test]
fn voltage_step_host_liveness_expiry_reports_dedicated_status_and_restores() {
    pollster::block_on(async {
        // `run_voltage_step_request`'s only checked delay before the
        // post-settle one is the sample delay, reached only after reading
        // saved mode (must equal the position-mode constant, 0x8000_0003),
        // encoder-before, encoder PPR (must be non-zero), and torque-before,
        // in that order.
        let raw_spi = DiagnosticMockSpi::default()
            .with_read(0x8000_0003) // saved mode
            .with_read(100) // encoder_before (ABN_DECODER_COUNT)
            .with_read(1000) // encoder PPR (ABN_DECODER_PPR) -- must be non-zero
            .with_read(0); // torque_before (PID_TORQUE_FLUX_ACTUAL)
        let mut tmc = Tmc4671Spi::new(raw_spi);
        let mut runtime = ready_runtime();
        let mut delay = MockDelay::default();
        // sample_delay_ms=150 makes this delay 2 slices ([100, 50]):
        // pre-loop (1), post-slice-1/100ms (2), post-slice-2/50ms -- the
        // final slice -- (3).
        let mut calls = 0u32;
        let mut expire_during_main_delay_final_slice = || {
            calls += 1;
            calls >= 3
        };
        let req = crate::tmc::tmc_voltage_step_test(0, 4, 100, -50, 150);

        let outcome = run_voltage_step_request(
            req,
            DiagnosticContext {
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                power_stage: &mut MockDiagnosticPowerStage::default(),
                tmc_config: &TEST_TMC_CONFIG,
                feedback_config: crate::config::FeedbackConfig::abn_abn(),
                readiness: DiagnosticReadiness {
                    homing_active: false,
                    stepper_active: false,
                    saved_voltage_limit: 777,
                },
                host_liveness_expired: Some(&mut expire_during_main_delay_final_slice),
            },
        )
        .await;

        let DiagnosticOutcome::Handled(DiagnosticReport::VoltageStep(report)) = outcome else {
            panic!("expected voltage step report");
        };
        assert_eq!(report.status, CURRENT_STEP_STATUS_HOST_LIVENESS);
        assert_eq!(delay.delays_ms, std::vec![100, 50]);

        // Restore tail on any mid-run failure: mode zeroed, external
        // voltage zeroed, mode restored to the saved value. The setup
        // writes before the delay (STATUS_FLAGS clear, the requested
        // UQ_UD_EXT, open-loop mode) succeed for real first, so assert
        // only the trailing 3-write restore sequence.
        let raw_spi = tmc.into_inner();
        let restore_tail = &raw_spi.write_attempts[raw_spi.write_attempts.len() - 3..];
        assert_eq!(
            restore_tail,
            [
                (MODE_RAMP_MODE_MOTION.addr(), 0),
                (UQ_UD_EXT.addr(), 0),
                (MODE_RAMP_MODE_MOTION.addr(), 0x8000_0003),
            ]
        );
    });
}

#[test]
fn voltage_step_post_settle_host_liveness_expiry_reports_dedicated_status() {
    // Second and last call site: the post-settle delay after the main
    // sample delay already covered above.
    pollster::block_on(async {
        let raw_spi = DiagnosticMockSpi::default()
            .with_read(0x8000_0003) // saved mode
            .with_read(100) // encoder_before (ABN_DECODER_COUNT)
            .with_read(1000) // encoder PPR (ABN_DECODER_PPR) -- must be non-zero
            .with_read(0); // torque_before (PID_TORQUE_FLUX_ACTUAL)
        let mut tmc = Tmc4671Spi::new(raw_spi);
        let mut runtime = ready_runtime();
        let mut delay = MockDelay::default();
        // Checks: main pre-loop/post-slice (1, 2) -- both false, letting
        // the 20ms sample delay and every intervening read through --
        // post-settle pre-loop (3) -- true.
        let mut calls = 0u32;
        let mut expire_at_post_settle = || {
            calls += 1;
            calls >= 3
        };
        let req = crate::tmc::tmc_voltage_step_test(0, 4, 100, -50, 20);

        let outcome = run_voltage_step_request(
            req,
            DiagnosticContext {
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                power_stage: &mut MockDiagnosticPowerStage::default(),
                tmc_config: &TEST_TMC_CONFIG,
                feedback_config: crate::config::FeedbackConfig::abn_abn(),
                readiness: DiagnosticReadiness {
                    homing_active: false,
                    stepper_active: false,
                    saved_voltage_limit: 777,
                },
                host_liveness_expired: Some(&mut expire_at_post_settle),
            },
        )
        .await;

        let DiagnosticOutcome::Handled(DiagnosticReport::VoltageStep(report)) = outcome else {
            panic!("expected voltage step report");
        };
        assert_eq!(report.status, CURRENT_STEP_STATUS_HOST_LIVENESS);
        assert_eq!(delay.delays_ms, std::vec![20]);

        let raw_spi = tmc.into_inner();
        let restore_tail = &raw_spi.write_attempts[raw_spi.write_attempts.len() - 3..];
        assert_eq!(
            restore_tail,
            [
                (MODE_RAMP_MODE_MOTION.addr(), 0),
                (UQ_UD_EXT.addr(), 0),
                (MODE_RAMP_MODE_MOTION.addr(), 0x8000_0003),
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
                host_liveness_expired: None,
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                power_stage: &mut MockDiagnosticPowerStage::default(),
                tmc_config: &TEST_TMC_CONFIG,
                feedback_config: crate::config::FeedbackConfig::abn_abn(),
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
}

#[derive(Default)]
struct RecordingDiagnosticSink {
    events: std::vec::Vec<RecordedResistanceEvent>,
}

impl DiagnosticReplySink for RecordingDiagnosticSink {
    fn foci_current_step_result(
        &mut self,
        _report_seq: u8,
        _axis: CurrentStepAxis,
        _report: CurrentStepReport,
    ) {
    }
    fn foci_current_vector_step_result(&mut self, _report_seq: u8, _report: CurrentStepReport) {}
    fn foci_current_torque_sample_pid_result(
        &mut self,
        _report_seq: u8,
        _report: &CurrentTorqueSampleReport,
    ) {
    }
    fn foci_current_torque_sample_result(
        &mut self,
        _report_seq: u8,
        _report: CurrentTorqueSampleReport,
    ) {
    }
    fn foci_current_torque_sample_detail_result(
        &mut self,
        _report_seq: u8,
        _report: &CurrentTorqueSampleReport,
    ) {
    }
    fn foci_voltage_step_result(&mut self, _report_seq: u8, _report: VoltageStepReport) {}
    fn foci_voltage_step_detail_result(&mut self, _report_seq: u8, _report: &VoltageStepReport) {}

    fn foci_resistance_profile(&mut self, _report_seq: u8, report: ResistanceProfileReport) {
        self.events
            .push(RecordedResistanceEvent::Profile { oid: report.oid });
    }

    fn foci_resistance_axis(&mut self, _report_seq: u8, report: ResistanceAxisReport) {
        self.events.push(RecordedResistanceEvent::Axis {
            oid: report.oid,
            electrical_axis: report.electrical_axis,
        });
    }

    fn foci_resistance_run(&mut self, _report_seq: u8, report: ResistanceRunReport) {
        self.events.push(RecordedResistanceEvent::Done {
            oid: report.oid,
            status: report.status,
        });
    }
}

#[fixture_test]
async fn resistance_test_request_dispatches_to_shared_runner() {
    // Global per-channel state (`motor_sentinel`) is shared across the
    // whole test binary; serialize against other suites that touch it.
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
            host_liveness_expired: None,
            tmc: &mut tmc,
            runtime: &mut runtime,
            delay: &mut delay,
            power_stage: &mut MockDiagnosticPowerStage::default(),
            tmc_config: &TEST_TMC_CONFIG,
            feedback_config: crate::config::FeedbackConfig::abn_abn(),
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
    emit_diagnostic_report(&mut sink, 0, report);

    assert!(
        sink.events
            .iter()
            .any(|event| matches!(event, RecordedResistanceEvent::Done { oid: 7, .. }))
    );
}

#[fixture_test]
fn resistance_trip_lowers_board_stage_before_clearing_shared_state() {
    let channel_state = crate::state::tmc::tmc_channel0();
    let stepper_state = crate::state::stepper::stepper_channel0();
    channel_state.reset();
    stepper_state.reset();
    channel_state
        .motor_armed
        .store(true, core::sync::atomic::Ordering::Relaxed);
    stepper_state
        .timer_active_mirror
        .store(true, core::sync::atomic::Ordering::Relaxed);

    let mut runtime = ready_runtime();
    let mut delay = MockDelay::default();
    let mut tmc = Tmc4671Spi::new(DiagnosticMockSpi::default());
    let mut power_stage = MockDiagnosticPowerStage::default();
    let mut ctx = DiagnosticContext {
        host_liveness_expired: None,
        tmc: &mut tmc,
        runtime: &mut runtime,
        delay: &mut delay,
        power_stage: &mut power_stage,
        tmc_config: &TEST_TMC_CONFIG,
        feedback_config: crate::config::FeedbackConfig::abn_abn(),
        readiness: DiagnosticReadiness {
            homing_active: false,
            stepper_active: false,
            saved_voltage_limit: 777,
        },
    };

    {
        let mut backend = super::resistance::DiagnosticResistanceBackend::new(&mut ctx);
        crate::resistance_identification::ResistanceBackend::set_enable_pin(&mut backend, true);
        crate::resistance_identification::ResistanceBackend::set_enable_pin(&mut backend, false);
    }
    assert_eq!(power_stage.trip_calls, 0);
    assert!(runtime.is_motor_enabled());
    assert!(runtime.is_calibrated());

    let mut ctx = DiagnosticContext {
        host_liveness_expired: None,
        tmc: &mut tmc,
        runtime: &mut runtime,
        delay: &mut delay,
        power_stage: &mut power_stage,
        tmc_config: &TEST_TMC_CONFIG,
        feedback_config: crate::config::FeedbackConfig::abn_abn(),
        readiness: DiagnosticReadiness {
            homing_active: false,
            stepper_active: false,
            saved_voltage_limit: 777,
        },
    };
    {
        let mut backend = super::resistance::DiagnosticResistanceBackend::new(&mut ctx);
        crate::resistance_identification::ResistanceBackend::trip_power_stage(&mut backend);
    }

    assert_eq!(power_stage.trip_calls, 1);
    assert!(power_stage.motor_armed_at_trip);
    assert!(!runtime.is_motor_enabled());
    assert!(!runtime.is_calibrated());
    assert!(
        !channel_state
            .motor_armed
            .load(core::sync::atomic::Ordering::Relaxed)
    );
    assert!(
        !stepper_state
            .timer_active_mirror
            .load(core::sync::atomic::Ordering::Relaxed)
    );
    channel_state.reset();
    stepper_state.reset();
}

#[fixture_test]
async fn resistance_test_not_armed_reports_no_axes() {
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
            host_liveness_expired: None,
            tmc: &mut tmc,
            runtime: &mut runtime,
            delay: &mut delay,
            power_stage: &mut MockDiagnosticPowerStage::default(),
            tmc_config: &TEST_TMC_CONFIG,
            feedback_config: crate::config::FeedbackConfig::abn_abn(),
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
}

#[fixture_test]
async fn resistance_test_uses_board_monotonic_for_sample_timing() {
    let channel_state = crate::state::tmc::tmc_channel(0).unwrap();
    channel_state
        .motor_sentinel
        .store(0x0002_0032, core::sync::atomic::Ordering::Relaxed);
    channel_state
        .encoder_ppr
        .store(1_000, core::sync::atomic::Ordering::Relaxed);

    let mut runtime = ready_runtime();
    let mut delay = MockDelay {
        advance_on_delay: true,
        // Each logical sample observes two 3 ms register-operation spans,
        // exceeding the 5 ms sample-start period's service deadline.
        advance_on_now_us: 3_000,
        ..MockDelay::default()
    };
    let raw_spi = DiagnosticMockSpi::default()
        .with_read(2_000)
        .with_read(0x0002_0032)
        .with_read(7)
        .with_read(u32::from(TEST_TMC_CONFIG.pwm_maxcnt))
        .with_read((u32::from(TEST_TMC_CONFIG.bbm_h) << 8) | u32::from(TEST_TMC_CONFIG.bbm_l))
        .with_read(
            (u32::from(TEST_TMC_CONFIG.dsadc_mdec_b) << 16)
                | u32::from(TEST_TMC_CONFIG.dsadc_mdec_a),
        )
        .with_read(0)
        .with_read(0)
        .with_read(0)
        .with_read(0);
    let mut tmc = Tmc4671Spi::new(raw_spi);
    let req = crate::tmc::tmc_resistance_test(0, 10, 0);

    let outcome = handle_diagnostic_request(
        req,
        DiagnosticContext {
            host_liveness_expired: None,
            tmc: &mut tmc,
            runtime: &mut runtime,
            delay: &mut delay,
            power_stage: &mut MockDiagnosticPowerStage::default(),
            tmc_config: &TEST_TMC_CONFIG,
            feedback_config: crate::config::FeedbackConfig::abn_abn(),
            readiness: DiagnosticReadiness {
                homing_active: false,
                stepper_active: false,
                saved_voltage_limit: 2_000,
            },
        },
    )
    .await;

    channel_state
        .motor_sentinel
        .store(0, core::sync::atomic::Ordering::Relaxed);
    channel_state
        .encoder_ppr
        .store(0, core::sync::atomic::Ordering::Relaxed);

    let DiagnosticOutcome::Handled(DiagnosticReport::ResistanceTest { run, axes, .. }) = outcome
    else {
        panic!("expected resistance test report");
    };
    assert_eq!(
        run.status,
        crate::resistance_identification::ResistanceError::TimingRejected.status_code()
    );
    assert!(axes.is_none());
    assert!(delay.now_us.get() > 0);
}

#[fixture_test]
async fn resistance_test_rejects_during_homing_without_invoking_runner() {
    // Mirrors `current_step_not_ready_returns_status_without_writes`:
    // sibling diagnostics reject on `readiness.homing_active` /
    // `readiness.stepper_active` before touching SPI. The resistance-test
    // dispatch must do the same instead of only gating on
    // enabled+calibrated, or a resistance test can switch the motor to
    // open-loop voltage while homing or active stepping is in progress.
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
            host_liveness_expired: None,
            tmc: &mut tmc,
            runtime: &mut runtime,
            delay: &mut delay,
            power_stage: &mut MockDiagnosticPowerStage::default(),
            tmc_config: &TEST_TMC_CONFIG,
            feedback_config: crate::config::FeedbackConfig::abn_abn(),
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
}

#[test]
fn resistance_backend_delay_ms_liveness_expiry_injects_one_shot_error_then_recovers() {
    pollster::block_on(async {
        let mut runtime = ready_runtime();
        let mut delay = MockDelay::default();
        let mut tmc = Tmc4671Spi::new(DiagnosticMockSpi::default());
        let mut power_stage = MockDiagnosticPowerStage::default();
        let mut calls = 0u32;
        // 150ms delay slices into [100, 50]. Checks: pre-loop (1), after
        // slice 1 (2), after slice 2/the final slice (3) -- expiring on the
        // 3rd check proves detection after the LAST slice, not just before
        // each one.
        let mut expire_after_final_slice = || {
            calls += 1;
            calls >= 3
        };
        let mut ctx = DiagnosticContext {
            tmc: &mut tmc,
            runtime: &mut runtime,
            delay: &mut delay,
            power_stage: &mut power_stage,
            tmc_config: &TEST_TMC_CONFIG,
            feedback_config: crate::config::FeedbackConfig::abn_abn(),
            readiness: DiagnosticReadiness {
                homing_active: false,
                stepper_active: false,
                saved_voltage_limit: 777,
            },
            host_liveness_expired: Some(&mut expire_after_final_slice),
        };
        let mut backend = super::resistance::DiagnosticResistanceBackend::new(&mut ctx);

        crate::resistance_identification::ResistanceBackend::delay_ms(&mut backend, 150).await;

        // The one-shot injection fires on exactly the next register call...
        let first = crate::resistance_identification::ResistanceBackend::write_register(
            &mut backend,
            0x01,
            0,
        )
        .await;

        // ...and is consumed: the restore sequence's subsequent writes are
        // not suppressed by the persistent host_liveness_aborted flag.
        let second = crate::resistance_identification::ResistanceBackend::write_register(
            &mut backend,
            0x01,
            5,
        )
        .await;

        // A third call proves the one-shot really doesn't re-arm itself --
        // not just that exactly one Err happened somewhere in the sequence.
        let third = crate::resistance_identification::ResistanceBackend::write_register(
            &mut backend,
            0x01,
            9,
        )
        .await;

        // `backend` (and the `ctx`/`delay` borrow it holds) is not used
        // again after this point, so `delay.delays_ms` can be read directly
        // below -- reading it any earlier, while `backend` is still going
        // to be used again, would not satisfy the borrow checker.
        let aborted = backend.host_liveness_aborted();

        assert!(aborted); // still true after two more calls -- persistent
        assert!(first.is_err());
        assert!(second.is_ok());
        assert!(third.is_ok());
        assert_eq!(delay.delays_ms, std::vec![100, 50]);
    });
}

#[test]
fn resistance_terminal_status_prefers_host_liveness_over_backend_error_status() {
    use crate::resistance_identification::ResistanceError;

    assert_eq!(
        super::resistance::terminal_status(ResistanceError::BackendIo, true),
        crate::autotune::CommissionError::HostLivenessExpired.status_code()
    );
}

#[test]
fn resistance_terminal_status_uses_error_status_when_not_liveness_aborted() {
    use crate::resistance_identification::ResistanceError;

    assert_eq!(
        super::resistance::terminal_status(ResistanceError::BackendIo, false),
        ResistanceError::BackendIo.status_code()
    );
}

#[fixture_test]
async fn resistance_test_request_host_liveness_expiry_reports_dedicated_status() {
    let channel_state = crate::state::tmc::tmc_channel(0).unwrap();
    channel_state
        .motor_sentinel
        .store(0x0002_0032, core::sync::atomic::Ordering::Relaxed);

    let raw_spi = DiagnosticMockSpi::default()
        .with_read(1600) // ReadSavedLimit: PIDOUT_UQ_UD_LIMITS >= max_abs_ud()
        .with_read(0x0002_0032) // ReadMotorType: TwoPhase, 50 pole pairs
        .with_read(7) // Setup(4) verify_read: PWM_SV_CHOP
        .with_read(3999) // Setup(5) verify_read: PWM_MAXCNT
        .with_read(2313) // Setup(6) verify_read: PWM_BBM_H_BBM_L = (9<<8)|9
        .with_read(21_692_747) // Setup(7) verify_read: DSADC_MDEC_B_MDEC_A = (331<<16)|331
        .with_read(0) // Setup(8): STATUS_FLAGS baseline
        .with_read(0); // Setup(10): STATUS_FLAGS, no actionable bits
    let mut tmc = Tmc4671Spi::new(raw_spi);
    let mut runtime = ready_runtime();
    let mut delay = MockDelay::default();
    let mut power_stage = MockDiagnosticPowerStage::default();
    // Checks: pre-loop (1), post-slice-1/100ms (2), post-slice-2/100ms (3),
    // post-slice-3/50ms -- the FINAL slice of zero_settle_ms's [100,100,50]
    // -- (4).
    let mut calls = 0u32;
    let mut expire_during_settle_final_slice = || {
        calls += 1;
        calls >= 4
    };
    let ctx = DiagnosticContext {
        tmc: &mut tmc,
        runtime: &mut runtime,
        delay: &mut delay,
        power_stage: &mut power_stage,
        tmc_config: &TEST_TMC_CONFIG,
        feedback_config: crate::config::FeedbackConfig::abn_abn(),
        readiness: DiagnosticReadiness {
            homing_active: false,
            stepper_active: false,
            saved_voltage_limit: 777,
        },
        host_liveness_expired: Some(&mut expire_during_settle_final_slice),
    };

    let outcome = super::resistance::run_resistance_test_request(
        crate::tmc::tmc_resistance_test(0, 7, 0),
        ctx,
    )
    .await;

    channel_state
        .motor_sentinel
        .store(0, core::sync::atomic::Ordering::Relaxed);

    let DiagnosticOutcome::Handled(DiagnosticReport::ResistanceTest { run, .. }) = outcome else {
        panic!("expected resistance test report");
    };

    assert_eq!(
        run.status,
        crate::autotune::CommissionError::HostLivenessExpired.status_code()
    );
    assert_eq!(delay.delays_ms, std::vec![100, 100, 50]);
    // The four-write safe-state restore ran in full, with real values.
    let raw_spi = tmc.into_inner();
    let restore_tail = &raw_spi.write_attempts[raw_spi.write_attempts.len() - 4..];
    assert_eq!(
        restore_tail,
        [
            (
                foci_tmc4671::registers::MODE_RAMP_MODE_MOTION.addr(),
                foci_tmc4671::registers::MotionMode::Stopped as u32
            ),
            (foci_tmc4671::registers::UQ_UD_EXT.addr(), 0),
            (foci_tmc4671::registers::PWM_SV_CHOP.addr(), 0),
            (foci_tmc4671::registers::PIDOUT_UQ_UD_LIMITS.addr(), 1600),
        ]
    );
}

#[fixture_test]
async fn resistance_test_request_genuine_spi_failure_reports_generic_status() {
    // Identical setup to the liveness test, no liveness signal at all --
    // instead, fail the PHI_E_EXT write SelectAxis(1) makes. This is after
    // Setup(0) has already set `energized = true`, so the shared runner's
    // `fail()` path (not the entry-pre-check `complete_without_restore`
    // path) runs, exercising the same four-write restore for a genuine,
    // non-liveness reason.
    let channel_state = crate::state::tmc::tmc_channel(0).unwrap();
    channel_state
        .motor_sentinel
        .store(0x0002_0032, core::sync::atomic::Ordering::Relaxed);

    let raw_spi = DiagnosticMockSpi::default()
        .with_read(1600)
        .with_read(0x0002_0032)
        .with_read(7)
        .with_read(3999)
        .with_read(2313)
        .with_read(21_692_747)
        .with_read(0)
        .with_read(0)
        .fail_writes_to(foci_tmc4671::registers::PHI_E_EXT.addr());
    let mut tmc = Tmc4671Spi::new(raw_spi);
    let mut runtime = ready_runtime();
    let mut delay = MockDelay::default();
    let mut power_stage = MockDiagnosticPowerStage::default();
    let ctx = DiagnosticContext {
        tmc: &mut tmc,
        runtime: &mut runtime,
        delay: &mut delay,
        power_stage: &mut power_stage,
        tmc_config: &TEST_TMC_CONFIG,
        feedback_config: crate::config::FeedbackConfig::abn_abn(),
        readiness: DiagnosticReadiness {
            homing_active: false,
            stepper_active: false,
            saved_voltage_limit: 777,
        },
        host_liveness_expired: None,
    };

    let outcome = super::resistance::run_resistance_test_request(
        crate::tmc::tmc_resistance_test(0, 7, 0),
        ctx,
    )
    .await;

    channel_state
        .motor_sentinel
        .store(0, core::sync::atomic::Ordering::Relaxed);

    let DiagnosticOutcome::Handled(DiagnosticReport::ResistanceTest { run, .. }) = outcome else {
        panic!("expected resistance test report");
    };

    // Proves the two failure modes are actually distinguishable at the
    // full report level: this status must differ from
    // CommissionError::HostLivenessExpired.status_code() above.
    assert_eq!(
        run.status,
        crate::resistance_identification::ResistanceError::BackendIo.status_code()
    );
    assert_ne!(
        run.status,
        crate::autotune::CommissionError::HostLivenessExpired.status_code()
    );
    // Restore still ran, for a completely different reason.
    let raw_spi = tmc.into_inner();
    let restore_tail = &raw_spi.write_attempts[raw_spi.write_attempts.len() - 4..];
    assert_eq!(
        restore_tail,
        [
            (
                foci_tmc4671::registers::MODE_RAMP_MODE_MOTION.addr(),
                foci_tmc4671::registers::MotionMode::Stopped as u32
            ),
            (foci_tmc4671::registers::UQ_UD_EXT.addr(), 0),
            (foci_tmc4671::registers::PWM_SV_CHOP.addr(), 0),
            (foci_tmc4671::registers::PIDOUT_UQ_UD_LIMITS.addr(), 1600),
        ]
    );
}
