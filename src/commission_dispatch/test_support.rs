//! Test infrastructure for commission_dispatch: mock CommissioningBackend
//! with injectable SPI failures, register-write log, delay log, and
//! enable-pin tracking.

use crate::commissioning::types::CommissioningBackend;
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
}

/// Mock implementation of `CommissioningBackend` for host-side unit tests.
///
/// Stores register state in a linear map keyed by register address, records
/// every SPI operation, and supports per-address failure injection for both
/// reads and writes.
pub struct MockBackend {
    /// Current register values, keyed by TMC4671 address byte.
    pub registers: heapless::LinearMap<u8, u32, 64>,
    /// Ordered log of every SPI read/write issued during the test.
    pub spi_log: Vec<SpiOp, 128>,
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
    /// Expected ABN_DECODER_PPR quadrature count for readback tests.
    pub encoder_quadrature_ppr_value: u32,
    /// Value returned by `operational_velocity_limit()`.
    pub operational_velocity_limit_value: u32,
    /// Value returned by `operational_voltage_limit()`.
    pub operational_voltage_limit_value: u32,
    /// Value returned by `tmc_config()`.
    pub tmc_config: TmcConfig,
    /// Value returned by `read_encoder_count()`.
    pub encoder_count_value: u32,
    /// Sequential values to return for consecutive `PID_POSITION_ACTUAL` reads.
    ///
    /// When non-empty, each `read_register(PID_POSITION_ACTUAL)` call pops and
    /// returns the front value instead of consulting `registers`. Used by
    /// stability-check tests that need two reads to return different values.
    pub read_position_actual_sequence: Vec<u32, 8>,
    /// Current index into `read_position_actual_sequence`.
    pub read_position_actual_pos: usize,
    /// If true, every SPI write asserts that step processing has already been gated off.
    pub assert_motor_armed_false_on_write: bool,
}

impl MockBackend {
    /// Construct a `MockBackend` with sensible defaults for unit tests.
    pub fn new() -> Self {
        Self {
            registers: heapless::LinearMap::new(),
            spi_log: Vec::new(),
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
            encoder_quadrature_ppr_value: 4000,
            operational_velocity_limit_value: 500_000,
            operational_voltage_limit_value:
                crate::state::tmc::DEFAULT_OPERATIONAL_PIDOUT_UQ_UD_LIMIT,
            tmc_config: TmcConfig::default_for_test(),
            encoder_count_value: 0,
            read_position_actual_sequence: Vec::new(),
            read_position_actual_pos: 0,
            assert_motor_armed_false_on_write: false,
        }
    }
}

impl CommissioningBackend for MockBackend {
    type Error = MockSpiError;

    fn synthesize_verify_error(&self, _expected: u32, _actual: u32) -> Self::Error {
        MockSpiError
    }

    fn store_motor_sentinel(&self, value: u32) {
        self.last_motor_sentinel_stored.set(Some(value));
    }

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
        self.spi_log.push(SpiOp::Write(addr, val)).ok();
        if self.fail_on_write == Some(addr) {
            return Err(MockSpiError);
        }
        let _ = self.registers.insert(addr, val);
        Ok(())
    }

    fn set_enable_pin(&mut self, state: bool) {
        self.enable_pin = state;
    }

    async fn read_encoder_count(&mut self) -> u32 {
        self.encoder_count_value
    }

    async fn try_read_encoder_count(&mut self) -> Result<u32, Self::Error> {
        let addr = foci_tmc4671::registers::ABN_DECODER_COUNT.addr();
        self.spi_log.push(SpiOp::Read).ok();
        if self.fail_on_read == Some(addr) {
            return Err(MockSpiError);
        }
        Ok(self.encoder_count_value)
    }

    fn elapsed_us(&self) -> u64 {
        self.elapsed_us_value
    }

    fn is_shutdown_requested(&self) -> bool {
        self.shutdown_requested
    }

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

    fn step_width(&self) -> u32 {
        self.step_width_value
    }

    fn encoder_quadrature_ppr(&self) -> u32 {
        self.encoder_quadrature_ppr_value
    }

    fn operational_velocity_limit(&self) -> u32 {
        self.operational_velocity_limit_value
    }

    fn operational_voltage_limit(&self) -> u32 {
        self.operational_voltage_limit_value
    }

    async fn delay_us(&mut self, us: u32) {
        self.delay_us_log.push(us).ok();
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
}
