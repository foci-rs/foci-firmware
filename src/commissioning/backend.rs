//! Hardware backend capability traits for commissioning.
//!
//! The commissioning engine and dispatch code mostly use the umbrella
//! [`CommissioningBackend`] bound. The smaller traits keep the capability
//! surface explicit and allow later helpers to depend on narrower behavior
//! without changing board semantics.

use core::convert::Infallible;
use core::marker::PhantomData;

use crate::config::TmcConfig;
use embedded_hal::digital::OutputPin;
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

    /// Host-configured torque filter cutoff, including explicit zero disable.
    fn configured_torque_filter_hz(&self) -> Option<u16> {
        None
    }

    /// Host-configured flux filter cutoff, including explicit zero disable.
    fn configured_flux_filter_hz(&self) -> Option<u16> {
        None
    }

    /// TMC4671 STEP_WIDTH value.
    fn step_width(&self) -> u32;

    /// Configured ABN decoder quadrature count per mechanical revolution.
    fn encoder_quadrature_ppr(&self) -> u32;

    /// Encoder counts per mechanical revolution.
    ///
    /// Used by the shared resistance runner to wrap the per-window encoder-move
    /// delta. Identical to [`Self::encoder_quadrature_ppr`]; provided under the
    /// runner-facing name expected by the `ResistanceBackend` adapter.
    fn encoder_counts_per_rev(&self) -> u32 {
        self.encoder_quadrature_ppr()
    }

    /// Whether ADC current-offset calibration has been established.
    ///
    /// Backs the resistance runner's `adc_offsets_ready` entry pre-check. True
    /// once the commissioning ADC-calibration phase has captured the motor
    /// sentinel, which the engine always does before electrical identification
    /// runs the sweep.
    fn adc_offsets_ready(&self) -> bool;

    /// Configured operational PID_VELOCITY_LIMIT value.
    fn operational_velocity_limit(&self) -> u32;

    /// Configured operational PIDOUT_UQ_UD_LIMITS value.
    fn operational_voltage_limit(&self) -> u32;
}

/// Commissioning trace emission capability.
pub trait CommissionTrace {
    /// Record a transport-neutral commissioning trace event.
    fn record_commission_trace_event(
        &mut self,
        _event: crate::trace::events::CommissionTraceEvent,
    ) {
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
        _event: crate::trace::events::CommissionTraceEvent,
    ) {
    }
}

/// Board-specific hooks used by [`SharedCommissioningBackend`].
#[allow(async_fn_in_trait)]
pub trait CommissionBackendHooks {
    /// Board-specific TMC4671 configuration.
    fn commission_tmc_config() -> &'static TmcConfig;

    /// Microseconds elapsed since the board monotonic epoch.
    fn commission_elapsed_us() -> u64;

    /// Whether a shutdown has been requested.
    fn commission_is_shutdown_requested() -> bool;

    /// Cooperative microsecond delay.
    async fn commission_delay_us(us: u32);

    /// Record a transport-neutral commissioning trace event.
    fn record_commission_trace_event(
        _channel: u8,
        _event: crate::trace::events::CommissionTraceEvent,
    ) {
    }
}

/// Shared production commissioning backend core used by board crates.
pub struct SharedCommissioningBackend<'a, Spi, Pin, Hooks> {
    channel: u8,
    tmc: &'a mut Tmc4671Spi<Spi>,
    enable_pin: &'a mut Pin,
    tmc_state: &'static crate::state::tmc::TmcChannelState,
    _hooks: PhantomData<fn() -> Hooks>,
}

impl<'a, Spi, Pin, Hooks> SharedCommissioningBackend<'a, Spi, Pin, Hooks> {
    /// Create a shared commissioning backend from board-owned hardware handles.
    #[must_use]
    pub fn new(
        channel: u8,
        tmc: &'a mut Tmc4671Spi<Spi>,
        enable_pin: &'a mut Pin,
        tmc_state: &'static crate::state::tmc::TmcChannelState,
    ) -> Self {
        Self {
            channel,
            tmc,
            enable_pin,
            tmc_state,
            _hooks: PhantomData,
        }
    }
}

impl<'a, Spi, Pin, Hooks> CommissionTmcSpiAccess for SharedCommissioningBackend<'a, Spi, Pin, Hooks>
where
    Spi: SpiDevice<u8>,
{
    type Spi = Spi;

    fn tmc_spi(&mut self) -> &mut Tmc4671Spi<Self::Spi> {
        self.tmc
    }
}

impl<'a, Spi, Pin, Hooks> CommissionTmcStateAccess
    for SharedCommissioningBackend<'a, Spi, Pin, Hooks>
{
    fn tmc_state(&self) -> &'static crate::state::tmc::TmcChannelState {
        self.tmc_state
    }
}

impl<'a, Spi, Pin, Hooks> CommissionEnablePinAccess
    for SharedCommissioningBackend<'a, Spi, Pin, Hooks>
where
    Pin: OutputPin<Error = Infallible>,
{
    fn set_commission_enable_pin(&mut self, state: bool) {
        if state {
            let _ = self.enable_pin.set_high();
        } else {
            let _ = self.enable_pin.set_low();
        }
    }
}

impl<'a, Spi, Pin, Hooks> CommissionConfigAccess for SharedCommissioningBackend<'a, Spi, Pin, Hooks>
where
    Hooks: CommissionBackendHooks,
{
    fn commission_tmc_config(&self) -> &'static TmcConfig {
        Hooks::commission_tmc_config()
    }
}

impl<'a, Spi, Pin, Hooks> CommissionTimingAccess for SharedCommissioningBackend<'a, Spi, Pin, Hooks>
where
    Hooks: CommissionBackendHooks,
{
    fn commission_elapsed_us(&self) -> u64 {
        Hooks::commission_elapsed_us()
    }

    fn commission_is_shutdown_requested(&self) -> bool {
        Hooks::commission_is_shutdown_requested()
    }

    async fn commission_delay_us(&mut self, us: u32) {
        Hooks::commission_delay_us(us).await;
    }
}

impl<'a, Spi, Pin, Hooks> CommissionTraceAccess for SharedCommissioningBackend<'a, Spi, Pin, Hooks>
where
    Hooks: CommissionBackendHooks,
{
    fn record_commission_trace_event_impl(
        &mut self,
        event: crate::trace::events::CommissionTraceEvent,
    ) {
        Hooks::record_commission_trace_event(self.channel, event);
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

    fn configured_torque_filter_hz(&self) -> Option<u16> {
        self.tmc_state()
            .torque_filter_hz
            .load(core::sync::atomic::Ordering::Relaxed)
    }

    fn configured_flux_filter_hz(&self) -> Option<u16> {
        self.tmc_state()
            .flux_filter_hz
            .load(core::sync::atomic::Ordering::Relaxed)
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

    fn adc_offsets_ready(&self) -> bool {
        // The ADC-calibration phase captures a non-zero motor sentinel before
        // any later phase energizes. Electrical identification (which runs the
        // resistance sweep) is sequenced strictly after that phase, so a
        // non-zero sentinel reflects that ADC offsets have been established.
        self.tmc_state()
            .motor_sentinel
            .load(core::sync::atomic::Ordering::Relaxed)
            != 0
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
    fn record_commission_trace_event(&mut self, event: crate::trace::events::CommissionTraceEvent) {
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

#[cfg(test)]
mod tests {
    use super::*;

    use core::convert::Infallible;
    use core::sync::atomic::{
        AtomicBool, AtomicU8, AtomicU16, AtomicU32, AtomicU64, Ordering::Relaxed,
    };

    use embedded_hal_async::spi::Operation;
    use foci_tmc4671::registers::PID_POSITION_TARGET;
    use std::sync::Mutex;
    use std::vec::Vec;

    use crate::commissioning::types::EngineKind;
    use crate::trace::events::{
        CommissionTraceEvent, EVENT_COMMISSION_DIAGNOSTIC_WARNING, TraceEventLevel,
    };

    static TEST_LOCK: Mutex<()> = Mutex::new(());
    static TEST_TMC_STATE: crate::state::tmc::TmcChannelState =
        crate::state::tmc::TmcChannelState::new();
    static TEST_CONFIG: TmcConfig = TmcConfig {
        pwm_maxcnt: 3999,
        tmc_clock_hz: 25_000_000,
        bbm_h: 10,
        bbm_l: 10,
        dsadc_mdec_a: 331,
        dsadc_mdec_b: 331,
        adc_i_select: 0x1800_0100,
        excitation_voltage: 512,
        bandwidth_hz: 50.0,
        electrical_model_theta_e_us: 160,
        current_validation_flux_max_encoder_delta_counts: 4,
        current_validation_torque_max_encoder_delta_counts: 16,
        resistance_max_expected_current_count: 1600,
        resistance_scale_metadata_validated: false,
    };
    static ELAPSED_US: AtomicU64 = AtomicU64::new(0);
    static SHUTDOWN_REQUESTED: AtomicBool = AtomicBool::new(false);
    static LAST_DELAY_US: AtomicU32 = AtomicU32::new(0);
    static LAST_TRACE_CHANNEL: AtomicU8 = AtomicU8::new(u8::MAX);
    static LAST_TRACE_EVENT_ID: AtomicU16 = AtomicU16::new(0);

    #[derive(Default)]
    struct RecordingSpi {
        writes: Vec<(u8, u32)>,
    }

    impl embedded_hal_async::spi::ErrorType for RecordingSpi {
        type Error = Infallible;
    }

    impl embedded_hal_async::spi::SpiDevice<u8> for RecordingSpi {
        async fn transaction(
            &mut self,
            operations: &mut [Operation<'_, u8>],
        ) -> Result<(), Self::Error> {
            for operation in operations {
                match operation {
                    Operation::Write(buf) if buf.len() == 5 && (buf[0] & 0x80) != 0 => {
                        let addr = buf[0] & 0x7F;
                        let value = u32::from_be_bytes([buf[1], buf[2], buf[3], buf[4]]);
                        self.writes.push((addr, value));
                    }
                    Operation::Read(buf) => {
                        buf.fill(0);
                    }
                    _ => {}
                }
            }
            Ok(())
        }
    }

    #[derive(Default)]
    struct RecordingPin {
        is_high: bool,
    }

    impl embedded_hal::digital::ErrorType for RecordingPin {
        type Error = Infallible;
    }

    impl embedded_hal::digital::OutputPin for RecordingPin {
        fn set_low(&mut self) -> Result<(), Self::Error> {
            self.is_high = false;
            Ok(())
        }

        fn set_high(&mut self) -> Result<(), Self::Error> {
            self.is_high = true;
            Ok(())
        }
    }

    struct RecordingHooks;

    impl CommissionBackendHooks for RecordingHooks {
        fn commission_tmc_config() -> &'static TmcConfig {
            &TEST_CONFIG
        }

        fn commission_elapsed_us() -> u64 {
            ELAPSED_US.load(Relaxed)
        }

        fn commission_is_shutdown_requested() -> bool {
            SHUTDOWN_REQUESTED.load(Relaxed)
        }

        async fn commission_delay_us(us: u32) {
            LAST_DELAY_US.store(us, Relaxed);
        }

        fn record_commission_trace_event(channel: u8, event: CommissionTraceEvent) {
            LAST_TRACE_CHANNEL.store(channel, Relaxed);
            LAST_TRACE_EVENT_ID.store(event.event_id, Relaxed);
        }
    }

    fn reset_hooks() {
        ELAPSED_US.store(0, Relaxed);
        SHUTDOWN_REQUESTED.store(false, Relaxed);
        LAST_DELAY_US.store(0, Relaxed);
        LAST_TRACE_CHANNEL.store(u8::MAX, Relaxed);
        LAST_TRACE_EVENT_ID.store(0, Relaxed);
    }

    #[test]
    fn shared_backend_satisfies_commissioning_backend_bundle() {
        fn assert_backend<T: CommissioningBackend>() {}

        assert_backend::<
            SharedCommissioningBackend<'static, RecordingSpi, RecordingPin, RecordingHooks>,
        >();
    }

    #[test]
    fn shared_backend_delegates_access_traits() {
        let _guard = TEST_LOCK
            .lock()
            .expect("commissioning backend test lock is available");
        reset_hooks();

        pollster::block_on(async {
            ELAPSED_US.store(123_456, Relaxed);
            SHUTDOWN_REQUESTED.store(true, Relaxed);

            let mut tmc = Tmc4671Spi::new(RecordingSpi::default());
            let mut enable_pin = RecordingPin::default();
            let trace_event = CommissionTraceEvent {
                level: TraceEventLevel::Warn,
                event_id: EVENT_COMMISSION_DIAGNOSTIC_WARNING,
                engine_kind: EngineKind::Commission,
                payload: [7, 8],
                payload_count: 2,
            };

            {
                let mut backend = SharedCommissioningBackend::<
                    RecordingSpi,
                    RecordingPin,
                    RecordingHooks,
                >::new(
                    7, &mut tmc, &mut enable_pin, &TEST_TMC_STATE
                );

                assert!(core::ptr::eq(backend.tmc_state(), &TEST_TMC_STATE));
                assert_eq!(
                    backend.commission_tmc_config().pwm_maxcnt,
                    TEST_CONFIG.pwm_maxcnt
                );
                assert_eq!(backend.commission_elapsed_us(), 123_456);
                assert!(backend.commission_is_shutdown_requested());

                backend.commission_delay_us(250).await;
                backend.record_commission_trace_event_impl(trace_event);
                backend.set_commission_enable_pin(true);
                backend.set_commission_enable_pin(false);
                backend
                    .tmc_spi()
                    .write_register(PID_POSITION_TARGET, 0x0102_0304)
                    .await
                    .expect("recording SPI write succeeds");
            }

            assert!(!enable_pin.is_high);
            assert_eq!(LAST_DELAY_US.load(Relaxed), 250);
            assert_eq!(LAST_TRACE_CHANNEL.load(Relaxed), 7);
            assert_eq!(
                LAST_TRACE_EVENT_ID.load(Relaxed),
                EVENT_COMMISSION_DIAGNOSTIC_WARNING
            );

            let spi = tmc.into_inner();
            assert_eq!(
                spi.writes,
                std::vec![(PID_POSITION_TARGET.addr(), 0x0102_0304)]
            );
        });
    }
}
