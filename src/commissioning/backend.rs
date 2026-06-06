//! Hardware backend capability traits for commissioning.
//!
//! The commissioning engine and dispatch code mostly use the umbrella
//! [`CommissioningBackend`] bound. The smaller traits keep the capability
//! surface explicit and allow later helpers to depend on narrower behavior
//! without changing board semantics.

use crate::config::TmcConfig;
use embedded_hal_async::spi::{ErrorType, SpiDevice};
use foci_tmc4671::registers::{ABN_DECODER_COUNT, Register};
use foci_tmc4671::spi::{SpiError, Tmc4671Spi};

/// Register I/O capability for TMC4671 commissioning operations.
#[allow(async_fn_in_trait)]
pub trait CommissionRegisterIo {
    /// SPI/hardware error type.
    type Error;

    /// Read a TMC4671 register.
    async fn read_register(&mut self, reg: Register) -> Result<u32, Self::Error>;

    /// Write a TMC4671 register.
    async fn write_register(&mut self, reg: Register, val: u32) -> Result<(), Self::Error>;

    /// Write a register, read it back, and compare against `val`.
    ///
    /// Production board implementations must preserve the `Tmc4671Spi`
    /// retry policy. Test backends may implement single-attempt behavior.
    async fn write_register_verified(&mut self, reg: Register, val: u32)
    -> Result<(), Self::Error>;

    /// Read a register and compare against `expected`.
    ///
    /// Production board implementations must preserve the `Tmc4671Spi`
    /// retry policy. Test backends may implement single-attempt behavior.
    async fn read_register_verified(
        &mut self,
        reg: Register,
        expected: u32,
    ) -> Result<u32, Self::Error>;
}

/// Explicit motor lifecycle side effects used by commissioning.
pub trait CommissionMotorControl {
    /// Store the captured `MOTOR_TYPE_N_POLE_PAIRS` sentinel.
    ///
    /// Called by AdcCal immediately after the `MOTOR_TYPE_N_POLE_PAIRS`
    /// write succeeds. Used by `arm_motor_electrical` to detect silent
    /// chip resets. The default is a no-op for mocks that do not care
    /// about motor lifecycle state.
    fn store_motor_sentinel(&self, _value: u32) {}

    /// Set the TMC4671 enable pin state.
    fn set_enable_pin(&mut self, state: bool);
}

/// Encoder sampling capability used by inner and outer commissioning.
#[allow(async_fn_in_trait)]
pub trait CommissionEncoderRead: CommissionRegisterIo {
    /// Read the current ABN encoder count.
    async fn read_encoder_count(&mut self) -> u32;

    /// Read the current ABN encoder count and preserve backend read failures.
    async fn try_read_encoder_count(&mut self) -> Result<u32, Self::Error> {
        Ok(self.read_encoder_count().await)
    }
}

/// Cooperative timing and shutdown checks for the commissioning engine.
#[allow(async_fn_in_trait)]
pub trait CommissionTiming {
    /// Microseconds elapsed since an arbitrary monotonic reference point.
    ///
    /// Used for deadline-driven sub-state timing within `step()`. Must be
    /// strictly monotonic, never decreasing or wrapping, across a
    /// commissioning session.
    ///
    /// Returning `u64` avoids any realistic wraparound: `u64` microseconds
    /// covers roughly 584,942 years. Board implementations should use a
    /// hardware-backed monotonic to preserve microsecond precision for
    /// sub-ms sampling loops in electrical identification.
    fn elapsed_us(&self) -> u64;

    /// Whether a shutdown has been requested.
    fn is_shutdown_requested(&self) -> bool;

    /// Cooperative microsecond delay.
    ///
    /// Used by the OuterFailed stability check and any other firmware-side
    /// wait that must yield to higher-priority RTIC tasks. `u32`
    /// microseconds gives roughly 71 minutes of range, far more than any
    /// commissioning wait.
    ///
    /// Implementations should override this with the board's RTIC
    /// monotonic. The default panics because a missing override is a bug:
    /// firmware-side cooperative waits would not yield without a real wait.
    async fn delay_us(&mut self, _us: u32) {
        panic!(
            "CommissioningBackend impl missing delay_us override; \
             firmware-side cooperative waits would not yield"
        );
    }
}

/// Board and channel configuration values consumed by commissioning.
pub trait CommissionBoardConfig {
    /// Read the configured current limit register value.
    fn current_limit(&self) -> u32;

    /// PWM_MAXCNT register value for the board.
    fn pwm_maxcnt(&self) -> u16;

    /// PWM_BBM_H_BBM_L register value.
    fn bbm_register(&self) -> u32;

    /// DSADC_MDEC_B_MDEC_A register value.
    fn dsadc_mdec_register(&self) -> u32;

    /// Test excitation voltage magnitude.
    fn excitation_voltage(&self) -> u16;

    /// Board-specific TMC4671 configuration.
    fn tmc_config(&self) -> &TmcConfig;

    /// TMC4671 STEP_WIDTH value.
    fn step_width(&self) -> u32;

    /// Configured ABN decoder quadrature count per mechanical revolution.
    fn encoder_quadrature_ppr(&self) -> u32;

    /// Configured operational PID_VELOCITY_LIMIT value.
    fn operational_velocity_limit(&self) -> u32;

    /// Configured operational PIDOUT_UQ_UD_LIMITS value.
    fn operational_voltage_limit(&self) -> u32;
}

/// Commissioning trace emission capability.
pub trait CommissionTrace {
    /// Record a transport-neutral commissioning trace event.
    fn record_commission_trace_event(&mut self, _event: crate::trace_events::CommissionTraceEvent) {
    }
}

/// Opt-in access to a production TMC4671 SPI driver for shared backend impls.
///
/// Do not implement this helper for `MockBackend`; the mock provides explicit
/// capability impls for register-map behavior and failure injection.
pub trait CommissionTmcSpiAccess {
    /// Concrete board SPI device type.
    type Spi: SpiDevice<u8>;

    /// Mutable access to the board-owned TMC4671 SPI driver.
    fn tmc_spi(&mut self) -> &mut Tmc4671Spi<Self::Spi>;
}

/// Opt-in access to a production TMC channel state for shared backend impls.
pub trait CommissionTmcStateAccess {
    /// TMC channel state associated with this commissioning backend.
    fn tmc_state(&self) -> &'static crate::state::tmc::TmcChannelState;
}

/// Opt-in enable-pin hook for shared motor-control backend impls.
pub trait CommissionEnablePinAccess {
    /// Drive the board-owned TMC4671 enable pin.
    fn set_commission_enable_pin(&mut self, state: bool);
}

/// Opt-in board configuration access for shared backend impls.
pub trait CommissionConfigAccess {
    /// Board-specific TMC4671 configuration.
    fn commission_tmc_config(&self) -> &'static TmcConfig;
}

/// Opt-in timing hooks for shared commissioning timing impls.
#[allow(async_fn_in_trait)]
pub trait CommissionTimingAccess {
    /// Microseconds elapsed since the board monotonic epoch.
    fn commission_elapsed_us(&self) -> u64;

    /// Whether a shutdown has been requested.
    fn commission_is_shutdown_requested(&self) -> bool;

    /// Cooperative microsecond delay.
    async fn commission_delay_us(&mut self, us: u32);
}

/// Opt-in trace hook for shared commissioning trace impls.
pub trait CommissionTraceAccess {
    /// Record a transport-neutral commissioning trace event.
    fn record_commission_trace_event_impl(
        &mut self,
        _event: crate::trace_events::CommissionTraceEvent,
    ) {
    }
}

impl<T> CommissionRegisterIo for T
where
    T: CommissionTmcSpiAccess,
{
    type Error = SpiError<<T::Spi as ErrorType>::Error>;

    async fn read_register(&mut self, reg: Register) -> Result<u32, Self::Error> {
        self.tmc_spi().read_register(reg).await
    }

    async fn write_register(&mut self, reg: Register, val: u32) -> Result<(), Self::Error> {
        self.tmc_spi().write_register(reg, val).await
    }

    async fn write_register_verified(
        &mut self,
        reg: Register,
        val: u32,
    ) -> Result<(), Self::Error> {
        self.tmc_spi().write_register_verified(reg, val).await
    }

    async fn read_register_verified(
        &mut self,
        reg: Register,
        expected: u32,
    ) -> Result<u32, Self::Error> {
        self.tmc_spi().read_register_verified(reg, expected).await
    }
}

impl<T> CommissionEncoderRead for T
where
    T: CommissionTmcSpiAccess,
{
    async fn read_encoder_count(&mut self) -> u32 {
        self.tmc_spi()
            .read_register(ABN_DECODER_COUNT)
            .await
            .unwrap_or(0)
    }

    async fn try_read_encoder_count(&mut self) -> Result<u32, Self::Error> {
        self.tmc_spi().read_register(ABN_DECODER_COUNT).await
    }
}

impl<T> CommissionMotorControl for T
where
    T: CommissionTmcStateAccess + CommissionEnablePinAccess,
{
    fn store_motor_sentinel(&self, value: u32) {
        self.tmc_state()
            .motor_sentinel
            .store(value, core::sync::atomic::Ordering::Relaxed);
    }

    fn set_enable_pin(&mut self, state: bool) {
        self.set_commission_enable_pin(state);
    }
}

impl<T> CommissionBoardConfig for T
where
    T: CommissionTmcStateAccess + CommissionConfigAccess,
{
    fn current_limit(&self) -> u32 {
        self.tmc_state()
            .current_limit_reg
            .load(core::sync::atomic::Ordering::Relaxed)
    }

    fn pwm_maxcnt(&self) -> u16 {
        self.commission_tmc_config().pwm_maxcnt
    }

    fn bbm_register(&self) -> u32 {
        self.commission_tmc_config().bbm_register()
    }

    fn dsadc_mdec_register(&self) -> u32 {
        self.commission_tmc_config().dsadc_mdec_register()
    }

    fn excitation_voltage(&self) -> u16 {
        self.commission_tmc_config().excitation_voltage
    }

    fn tmc_config(&self) -> &TmcConfig {
        self.commission_tmc_config()
    }

    fn step_width(&self) -> u32 {
        self.tmc_state()
            .step_width_value
            .load(core::sync::atomic::Ordering::Relaxed)
            .max(0) as u32
    }

    fn encoder_quadrature_ppr(&self) -> u32 {
        self.tmc_state()
            .encoder_ppr
            .load(core::sync::atomic::Ordering::Relaxed)
            .saturating_mul(4)
    }

    fn operational_velocity_limit(&self) -> u32 {
        self.tmc_state()
            .velocity_limit_cfg
            .load(core::sync::atomic::Ordering::Relaxed)
    }

    fn operational_voltage_limit(&self) -> u32 {
        self.tmc_state()
            .voltage_limit_cfg
            .load(core::sync::atomic::Ordering::Relaxed)
    }
}

impl<T> CommissionTiming for T
where
    T: CommissionTimingAccess,
{
    fn elapsed_us(&self) -> u64 {
        self.commission_elapsed_us()
    }

    fn is_shutdown_requested(&self) -> bool {
        self.commission_is_shutdown_requested()
    }

    async fn delay_us(&mut self, us: u32) {
        self.commission_delay_us(us).await;
    }
}

impl<T> CommissionTrace for T
where
    T: CommissionTraceAccess,
{
    fn record_commission_trace_event(&mut self, event: crate::trace_events::CommissionTraceEvent) {
        self.record_commission_trace_event_impl(event);
    }
}

/// Complete commissioning backend capability bundle.
pub trait CommissioningBackend:
    CommissionRegisterIo
    + CommissionMotorControl
    + CommissionEncoderRead
    + CommissionTiming
    + CommissionBoardConfig
    + CommissionTrace
{
}

impl<T> CommissioningBackend for T where
    T: CommissionRegisterIo
        + CommissionMotorControl
        + CommissionEncoderRead
        + CommissionTiming
        + CommissionBoardConfig
        + CommissionTrace
{
}
