//! Hardware backend capability traits for commissioning.
//!
//! The commissioning engine and dispatch code mostly use the umbrella
//! [`CommissioningBackend`] bound. The smaller traits keep the capability
//! surface explicit and allow later helpers to depend on narrower behavior
//! without changing board semantics.

use crate::config::TmcConfig;
use foci_tmc4671::registers::Register;

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
    /// The default implementation performs one attempt with no retry.
    /// Real board implementations should override this to trampoline to
    /// `Tmc4671Spi::write_register_verified`, which applies the configured
    /// `RetryPolicy`.
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
            Err(self.synthesize_verify_error(val, actual))
        }
    }

    /// Read a register and compare against `expected`.
    ///
    /// The default implementation performs one attempt with no retry.
    /// Real board implementations should override this to trampoline to
    /// `Tmc4671Spi::read_register_verified`, which applies the configured
    /// `RetryPolicy`.
    async fn read_register_verified(
        &mut self,
        reg: Register,
        expected: u32,
    ) -> Result<u32, Self::Error> {
        let actual = self.read_register(reg).await?;
        if actual == expected {
            Ok(actual)
        } else {
            Err(self.synthesize_verify_error(expected, actual))
        }
    }

    /// Build a verification error for the default verified helpers.
    fn synthesize_verify_error(&self, expected: u32, actual: u32) -> Self::Error;
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
