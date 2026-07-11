//! Test infrastructure for commission_dispatch: mock CommissioningBackend
//! with injectable SPI failures, register-write log, delay log, and
//! enable-pin tracking.

use crate::commissioning::backend::{
    CommissionBoardConfig, CommissionEncoderRead, CommissionMotorControl, CommissionRegisterIo,
    CommissionTiming, CommissionTrace,
};
use crate::config::TmcConfig;
use foci_tmc4671::registers::Register;
use heapless::Vec;

/// Synthetic SPI error for test injection.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct MockSpiError;

/// A single SPI operation recorded by `MockBackend`.
#[derive(Debug, Clone, Copy)]
pub enum SpiOp {
    Read,
    Write(u8, u32),
    /// `set_enable_pin` call, recorded in the same ordered log as register
    /// writes so tests can assert relative ordering (e.g. a register must be
    /// written before the enable pin is asserted).
    EnablePin(bool),
}

/// Mock implementation of `CommissioningBackend` for host-side unit tests.
///
/// Stores register state in a linear map keyed by register address, records
/// every SPI operation, and supports per-address failure injection for both
/// reads and writes.
///
/// Do not implement the `Commission*Access` helper traits for this type.
/// `MockBackend` intentionally keeps explicit capability impls so tests can
/// inject register-map mismatches, SPI failures, and delay observations without
/// colliding with production blanket impls.
pub struct MockBackend {
    /// Current register values, keyed by TMC4671 address byte.
    pub registers: heapless::LinearMap<u8, u32, 64>,
    /// Ordered log of every SPI read/write issued during the test.
    pub spi_log: Vec<SpiOp, 384>,
    /// Ordered trace events recorded by the commissioning engine.
    pub trace_events: Vec<crate::trace::events::CommissionTraceEvent, 64>,
    /// Current state of the TMC4671 enable pin.
    pub enable_pin: bool,
    /// Log of every `delay_us` call, in order.
    pub delay_us_log: Vec<u32, 32>,
    /// If `Some(addr)`, `write_register` returns `Err(MockSpiError)` for that address.
    pub fail_on_write: Option<u8>,
    /// If `Some(addr)`, `read_register` returns `Err(MockSpiError)` for that address.
    pub fail_on_read: Option<u8>,
    /// Last value passed to `store_motor_sentinel`. None until first call.
    pub last_motor_sentinel_stored: core::cell::Cell<Option<u32>>,
    /// When Some((addr, value)), `read_register(reg)` for that addr returns
    /// `value` instead of consulting `registers`. Used by verify tests to
    /// inject a mismatch.
    pub read_override: Option<(u8, u32)>,
    /// Value returned by `elapsed_us()`.
    pub elapsed_us_value: u64,
    /// Value returned by `is_shutdown_requested()`.
    pub shutdown_requested: bool,
    /// Value returned by `current_limit()`.
    pub current_limit_value: u32,
    /// Value returned by `pwm_maxcnt()`.
    pub pwm_maxcnt_value: u16,
    /// Value returned by `bbm_register()`.
    pub bbm_register_value: u32,
    /// Value returned by `dsadc_mdec_register()`.
    pub dsadc_mdec_register_value: u32,
    /// Value returned by `excitation_voltage()`.
    pub excitation_voltage_value: u16,
    /// Value returned by `step_width()`.
    pub step_width_value: u32,
    /// Encoder quadrature counts per mechanical revolution.
    pub mechanical_encoder_quadrature_counts_per_rev_value: u32,
    /// Value returned by `adc_offsets_ready()`.
    pub adc_offsets_ready_value: bool,
    /// Value returned by `operational_velocity_limit()`.
    pub operational_velocity_limit_value: u32,
    /// Value returned by `operational_voltage_limit()`.
    pub operational_voltage_limit_value: u32,
    /// Value returned by `tmc_config()`.
    pub tmc_config: TmcConfig,
    /// Optional torque-filter override returned by `configured_torque_filter_hz()`.
    pub configured_torque_filter_hz: Option<u16>,
    /// Optional flux-filter override returned by `configured_flux_filter_hz()`.
    pub configured_flux_filter_hz: Option<u16>,
    /// Encoder position returned by the sampled-count accessor.
    pub mechanical_encoder_position_counts_value: u32,
    /// Sequential values to return for consecutive `PID_POSITION_ACTUAL` reads.
    ///
    /// When non-empty, each `read_register(PID_POSITION_ACTUAL)` call pops and
    /// returns the front value instead of consulting `registers`. Used by
    /// stability-check tests that need two reads to return different values.
    pub read_position_actual_sequence: Vec<u32, 8>,
    /// Current index into `read_position_actual_sequence`.
    pub read_position_actual_pos: usize,
    /// Sequential values to return for consecutive `ABN_DECODER_COUNT` reads.
    pub abn_decoder_count_sequence: Vec<u32, 16>,
    /// Current index into `abn_decoder_count_sequence`.
    pub abn_decoder_count_pos: usize,
    /// Sequential values to return for consecutive `PID_TORQUE_FLUX_ACTUAL` reads.
    pub pid_torque_flux_actual_sequence: Vec<u32, 128>,
    /// Current index into `pid_torque_flux_actual_sequence`.
    pub pid_torque_flux_actual_pos: usize,
    /// Sequential values to return for consecutive `OPENLOOP_PHI` reads.
    pub openloop_phi_sequence: Vec<u32, 128>,
    /// Current index into `openloop_phi_sequence`.
    pub openloop_phi_pos: usize,
    /// Elapsed-time increment applied for each scripted `OPENLOOP_PHI` read.
    pub openloop_phi_elapsed_step_us: u64,
    /// If true, every SPI write asserts that step processing has already been gated off.
    pub assert_motor_armed_false_on_write: bool,
    /// If true, `read_register(ADC_IWY_IUX)` reports a phase current
    /// proportional 1:1 to the most recent `UQ_UD_EXT` write (packed into
    /// both axis halves), and `read_register(STATUS_FLAGS)` reports `0`.
    /// Used by the shared resistance-identification runner's integration
    /// tests to script a clean linear current response without needing a
    /// dedicated fake backend.
    pub script_resistance_current_proportional_to_ud: bool,
    /// Last value written to `UQ_UD_EXT`, tracked only when
    /// `script_resistance_current_proportional_to_ud` is set.
    last_uq_ud_ext_write: u32,
}

impl MockBackend {
    /// Construct a `MockBackend` with sensible defaults for unit tests.
    pub fn new() -> Self {
        Self {
            registers: heapless::LinearMap::new(),
            spi_log: Vec::new(),
            trace_events: Vec::new(),
            enable_pin: false,
            delay_us_log: Vec::new(),
            fail_on_write: None,
            fail_on_read: None,
            last_motor_sentinel_stored: core::cell::Cell::new(None),
            read_override: None,
            elapsed_us_value: 0,
            shutdown_requested: false,
            current_limit_value: 0,
            pwm_maxcnt_value: 4095,
            bbm_register_value: 0,
            dsadc_mdec_register_value: 0,
            excitation_voltage_value: 1024,
            step_width_value: 0,
            mechanical_encoder_quadrature_counts_per_rev_value: 4000,
            adc_offsets_ready_value: true,
            operational_velocity_limit_value: 500_000,
            operational_voltage_limit_value:
                crate::state::tmc::DEFAULT_OPERATIONAL_PIDOUT_UQ_UD_LIMIT,
            tmc_config: TmcConfig::default_for_test(),
            configured_torque_filter_hz: None,
            configured_flux_filter_hz: None,
            mechanical_encoder_position_counts_value: 0,
            read_position_actual_sequence: Vec::new(),
            read_position_actual_pos: 0,
            abn_decoder_count_sequence: Vec::new(),
            abn_decoder_count_pos: 0,
            pid_torque_flux_actual_sequence: Vec::new(),
            pid_torque_flux_actual_pos: 0,
            openloop_phi_sequence: Vec::new(),
            openloop_phi_pos: 0,
            openloop_phi_elapsed_step_us: 0,
            assert_motor_armed_false_on_write: false,
            script_resistance_current_proportional_to_ud: false,
            last_uq_ud_ext_write: 0,
        }
    }

    /// Set sequential `ABN_DECODER_COUNT` values for read-register tests.
    pub fn set_abn_decoder_count_sequence(&mut self, values: &[u32]) {
        self.abn_decoder_count_sequence.clear();
        for &value in values {
            let _ = self.abn_decoder_count_sequence.push(value);
        }
        self.abn_decoder_count_pos = 0;
    }

    /// Seed `ABN_DECODER_COUNT` with a signed encoder-count bit pattern.
    pub fn set_abn_decoder_count(&mut self, count: i32) {
        let _ = self.registers.insert(
            foci_tmc4671::registers::ABN_DECODER_COUNT.addr(),
            count as u32,
        );
    }

    /// Set sequential `PID_TORQUE_FLUX_ACTUAL` values for read-register tests.
    pub fn set_pid_torque_flux_actual_sequence(&mut self, values: &[u32]) {
        self.pid_torque_flux_actual_sequence.clear();
        for &value in values {
            let _ = self.pid_torque_flux_actual_sequence.push(value);
        }
        self.pid_torque_flux_actual_pos = 0;
    }

    /// Set sequential `OPENLOOP_PHI` values for read-register tests.
    pub fn set_openloop_phi_sequence(&mut self, values: &[u32], elapsed_step_us: u64) {
        self.openloop_phi_sequence.clear();
        for &value in values {
            let _ = self.openloop_phi_sequence.push(value);
        }
        self.openloop_phi_pos = 0;
        self.openloop_phi_elapsed_step_us = elapsed_step_us;
    }

    /// Write a register through the mock's register-I/O trait implementation.
    pub async fn write_register(&mut self, reg: Register, val: u32) -> Result<(), MockSpiError> {
        <Self as CommissionRegisterIo>::write_register(self, reg, val).await
    }
}

impl CommissionRegisterIo for MockBackend {
    type Error = MockSpiError;

    async fn read_register(&mut self, reg: Register) -> Result<u32, Self::Error> {
        let addr = reg.addr();
        self.spi_log.push(SpiOp::Read).ok();
        if self.fail_on_read == Some(addr) {
            return Err(MockSpiError);
        }
        if let Some((addr_override, val)) = self.read_override
            && addr_override == addr
        {
            return Ok(val);
        }
        if addr == foci_tmc4671::registers::PID_POSITION_ACTUAL.addr()
            && self.read_position_actual_pos < self.read_position_actual_sequence.len()
        {
            let val = self.read_position_actual_sequence[self.read_position_actual_pos];
            self.read_position_actual_pos += 1;
            return Ok(val);
        }
        if addr == foci_tmc4671::registers::ABN_DECODER_COUNT.addr()
            && self.abn_decoder_count_pos < self.abn_decoder_count_sequence.len()
        {
            let val = self.abn_decoder_count_sequence[self.abn_decoder_count_pos];
            self.abn_decoder_count_pos += 1;
            return Ok(val);
        }
        if addr == foci_tmc4671::registers::PID_TORQUE_FLUX_ACTUAL.addr()
            && self.pid_torque_flux_actual_pos < self.pid_torque_flux_actual_sequence.len()
        {
            let val = self.pid_torque_flux_actual_sequence[self.pid_torque_flux_actual_pos];
            self.pid_torque_flux_actual_pos += 1;
            return Ok(val);
        }
        if addr == foci_tmc4671::registers::OPENLOOP_PHI.addr()
            && self.openloop_phi_pos < self.openloop_phi_sequence.len()
        {
            let val = self.openloop_phi_sequence[self.openloop_phi_pos];
            self.openloop_phi_pos += 1;
            self.elapsed_us_value = self
                .elapsed_us_value
                .saturating_add(self.openloop_phi_elapsed_step_us);
            return Ok(val);
        }
        if self.script_resistance_current_proportional_to_ud {
            if addr == foci_tmc4671::registers::ADC_IWY_IUX.addr() {
                // Pack the same proportional current into both axis halves so
                // either electrical axis channel reads the intended value.
                let half = (self.last_uq_ud_ext_write & 0xffff) as u16 as u32;
                return Ok((half << 16) | half);
            }
            if addr == foci_tmc4671::registers::STATUS_FLAGS.addr() {
                return Ok(0);
            }
        }
        Ok(*self.registers.get(&addr).unwrap_or(&0))
    }

    async fn write_register(&mut self, reg: Register, val: u32) -> Result<(), Self::Error> {
        let addr = reg.addr();
        if self.assert_motor_armed_false_on_write {
            assert!(
                !crate::state::tmc::tmc_channel0()
                    .motor_armed
                    .load(core::sync::atomic::Ordering::Relaxed),
                "dispatch SPI write ran while step processing was armed"
            );
        }
        if self.script_resistance_current_proportional_to_ud
            && addr == foci_tmc4671::registers::UQ_UD_EXT.addr()
        {
            self.last_uq_ud_ext_write = val;
        }
        self.spi_log.push(SpiOp::Write(addr, val)).ok();
        if self.fail_on_write == Some(addr) {
            return Err(MockSpiError);
        }
        let _ = self.registers.insert(addr, val);
        Ok(())
    }

    async fn write_register_verified(
        &mut self,
        reg: Register,
        val: u32,
    ) -> Result<(), Self::Error> {
        self.write_register(reg, val).await?;
        let actual = self.read_register(reg).await?;
        if actual == val {
            Ok(())
        } else {
            Err(MockSpiError)
        }
    }

    async fn read_register_verified(
        &mut self,
        reg: Register,
        expected: u32,
    ) -> Result<u32, Self::Error> {
        let actual = self.read_register(reg).await?;
        if actual == expected {
            Ok(actual)
        } else {
            Err(MockSpiError)
        }
    }
}

impl CommissionMotorControl for MockBackend {
    fn store_motor_sentinel(&self, value: u32) {
        self.last_motor_sentinel_stored.set(Some(value));
    }

    fn set_enable_pin(&mut self, state: bool) {
        self.enable_pin = state;
        self.spi_log.push(SpiOp::EnablePin(state)).ok();
    }
}

impl CommissionEncoderRead for MockBackend {
    async fn read_mechanical_encoder_position_counts(&mut self) -> u32 {
        self.mechanical_encoder_position_counts_value
    }

    async fn try_read_mechanical_encoder_position_counts(&mut self) -> Result<u32, Self::Error> {
        let addr = foci_tmc4671::registers::ABN_DECODER_COUNT.addr();
        self.spi_log.push(SpiOp::Read).ok();
        if self.fail_on_read == Some(addr) {
            return Err(MockSpiError);
        }
        Ok(self.mechanical_encoder_position_counts_value)
    }
}

impl CommissionTiming for MockBackend {
    fn elapsed_us(&self) -> u64 {
        self.elapsed_us_value
    }

    fn is_shutdown_requested(&self) -> bool {
        self.shutdown_requested
    }

    async fn delay_us(&mut self, us: u32) {
        self.delay_us_log.push(us).ok();
    }
}

impl CommissionBoardConfig for MockBackend {
    fn current_limit(&self) -> u32 {
        self.current_limit_value
    }

    fn pwm_maxcnt(&self) -> u16 {
        self.pwm_maxcnt_value
    }

    fn bbm_register(&self) -> u32 {
        self.bbm_register_value
    }

    fn dsadc_mdec_register(&self) -> u32 {
        self.dsadc_mdec_register_value
    }

    fn excitation_voltage(&self) -> u16 {
        self.excitation_voltage_value
    }

    fn tmc_config(&self) -> &TmcConfig {
        &self.tmc_config
    }

    fn configured_torque_filter_hz(&self) -> Option<u16> {
        self.configured_torque_filter_hz
    }

    fn configured_flux_filter_hz(&self) -> Option<u16> {
        self.configured_flux_filter_hz
    }

    fn step_width(&self) -> u32 {
        self.step_width_value
    }

    fn mechanical_encoder_quadrature_counts_per_rev(&self) -> u32 {
        self.mechanical_encoder_quadrature_counts_per_rev_value
    }

    fn adc_offsets_ready(&self) -> bool {
        self.adc_offsets_ready_value
    }

    fn operational_velocity_limit(&self) -> u32 {
        self.operational_velocity_limit_value
    }

    fn operational_voltage_limit(&self) -> u32 {
        self.operational_voltage_limit_value
    }
}

impl CommissionTrace for MockBackend {
    fn record_commission_trace_event(&mut self, event: crate::trace::events::CommissionTraceEvent) {
        self.trace_events.push(event).ok();
    }
}

/// Mutex for tests that mutate firmware-shared statics
/// (channel motor-armed / in-flight flags, and the gain/filter atomics in
/// `state::tmc`).
///
/// Tests acquire this guard before any read-modify-write sequence on a shared
/// static, and hold it for the duration of the assertion phase. Cargo runs
/// tests in parallel by default; without serialization, dirty static state
/// from one test would flake another running concurrently.
///
/// Usage:
/// ```ignore
/// let _guard = SHARED_STATE_MUTEX.lock().await;
/// crate::state::tmc::tmc_channel0().motor_armed.store(false, Ordering::Relaxed);
/// // ... test body ...
/// ```
///
/// Use Tokio's async mutex so async tests can intentionally hold the guard
/// across awaited firmware calls without blocking an executor thread.
pub static SHARED_STATE_MUTEX: tokio::sync::Mutex<()> = tokio::sync::Mutex::const_new(());

/// Build locals for a fresh dispatch state.
///
/// Returns `(calibrated, motor_enabled, stored_gains)` with zero/false/None
/// defaults. Tests borrow these as `&mut` to construct a `DispatchState`
/// (added in Task 11+) per call site.
pub fn fresh_state_locals() -> (bool, bool, Option<crate::pid::InnerLoopGains>) {
    (false, false, None)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mock_backend_satisfies_split_backend_traits() {
        fn assert_register_io<T: crate::commissioning::backend::CommissionRegisterIo>() {}
        fn assert_motor_control<T: crate::commissioning::backend::CommissionMotorControl>() {}
        fn assert_encoder_read<T: crate::commissioning::backend::CommissionEncoderRead>() {}
        fn assert_timing<T: crate::commissioning::backend::CommissionTiming>() {}
        fn assert_board_config<T: crate::commissioning::backend::CommissionBoardConfig>() {}
        fn assert_trace<T: crate::commissioning::backend::CommissionTrace>() {}
        fn assert_backend<T: crate::commissioning::backend::CommissioningBackend>() {}

        assert_register_io::<MockBackend>();
        assert_motor_control::<MockBackend>();
        assert_encoder_read::<MockBackend>();
        assert_timing::<MockBackend>();
        assert_board_config::<MockBackend>();
        assert_trace::<MockBackend>();
        assert_backend::<MockBackend>();
    }

    #[test]
    fn mock_store_motor_sentinel_writes_field() {
        let backend = MockBackend::new();
        backend.store_motor_sentinel(0x0002_0032);
        assert_eq!(backend.last_motor_sentinel_stored.get(), Some(0x0002_0032));
    }

    #[test]
    fn mock_write_register_verified_succeeds_on_match() {
        pollster::block_on(async {
            let mut backend = MockBackend::new();
            let result = backend
                .write_register_verified(foci_tmc4671::registers::PID_POSITION_TARGET, 42)
                .await;
            assert!(result.is_ok());
            assert_eq!(
                backend
                    .registers
                    .get(&foci_tmc4671::registers::PID_POSITION_TARGET.addr())
                    .copied(),
                Some(42),
            );
        });
    }

    #[test]
    fn mock_write_register_verified_returns_error_on_mismatch() {
        pollster::block_on(async {
            let mut backend = MockBackend::new();
            backend.read_override =
                Some((foci_tmc4671::registers::PID_POSITION_TARGET.addr(), 999));
            let result = backend
                .write_register_verified(foci_tmc4671::registers::PID_POSITION_TARGET, 42)
                .await;
            assert!(result.is_err(), "expected verify mismatch");
        });
    }

    #[test]
    fn mock_read_register_verified_succeeds_on_match() {
        pollster::block_on(async {
            let mut backend = MockBackend::new();
            let _ = backend
                .registers
                .insert(foci_tmc4671::registers::PID_POSITION_TARGET.addr(), 42);

            let result = backend
                .read_register_verified(foci_tmc4671::registers::PID_POSITION_TARGET, 42)
                .await;

            assert_eq!(result, Ok(42));
        });
    }

    #[test]
    fn mock_read_register_verified_returns_error_on_mismatch() {
        pollster::block_on(async {
            let mut backend = MockBackend::new();
            let _ = backend
                .registers
                .insert(foci_tmc4671::registers::PID_POSITION_TARGET.addr(), 999);

            let result = backend
                .read_register_verified(foci_tmc4671::registers::PID_POSITION_TARGET, 42)
                .await;

            assert!(result.is_err(), "expected verify mismatch");
        });
    }

    #[test]
    fn mock_try_read_mechanical_encoder_position_counts_propagates_read_failure() {
        pollster::block_on(async {
            let mut backend = MockBackend::new();
            backend.fail_on_read = Some(foci_tmc4671::registers::ABN_DECODER_COUNT.addr());

            let result = backend.try_read_mechanical_encoder_position_counts().await;

            assert_eq!(result, Err(MockSpiError));
        });
    }

    #[test]
    fn mock_mechanical_encoder_accessors_report_explicit_units() {
        pollster::block_on(async {
            let mut backend = MockBackend::new();
            backend.mechanical_encoder_position_counts_value = 1_234;
            backend.mechanical_encoder_quadrature_counts_per_rev_value = 8_000;

            assert_eq!(
                backend.read_mechanical_encoder_position_counts().await,
                1_234
            );
            assert_eq!(
                backend.mechanical_encoder_quadrature_counts_per_rev(),
                8_000
            );
        });
    }
}
