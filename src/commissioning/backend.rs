//! Hardware backend capability traits for commissioning.
//!
//! The commissioning engine and dispatch code mostly use the umbrella
//! [`CommissioningBackend`] bound. The smaller traits keep the capability
//! surface explicit and allow later helpers to depend on narrower behavior
//! without changing board semantics.

use core::convert::Infallible;
use core::marker::PhantomData;

use crate::config::TmcConfig;
use crate::tmc_control::pending_config::{PendingConfigItem, PendingConfigSink};
use embedded_hal::digital::OutputPin;
use embedded_hal_async::spi::{ErrorType, SpiDevice};
use foci_tmc4671::current_limit::PidTorqueFluxLimit;
use foci_tmc4671::registers::{ABN_DECODER_COUNT, MotorTypeNPolePairs, Register};
use foci_tmc4671::spi::{SpiError, Tmc4671Spi};

/// Register I/O capability for TMC4671 commissioning operations.
#[allow(async_fn_in_trait)]
pub trait CommissionRegisterIo {
    /// SPI/hardware error type.
    type Error;

    /// Read a TMC4671 register.
    async fn read_register(&mut self, reg: Register) -> Result<u32, Self::Error>;

    /// Read one register for a fixed-rate capture observation.
    async fn read_capture_register(&mut self, reg: Register) -> Result<u32, Self::Error> {
        self.read_register(reg).await
    }

    /// Read two TMC4671 registers for one capture observation.
    ///
    /// The default preserves ordinary async behavior for test and alternate
    /// backends. Production shared backends override this with two blocking,
    /// separately CS-framed transactions to avoid DMA wakeup overhead.
    async fn read_register_pair(
        &mut self,
        first: Register,
        second: Register,
    ) -> Result<(u32, u32), Self::Error> {
        let first = self.read_register(first).await?;
        let second = self.read_register(second).await?;
        Ok((first, second))
    }

    /// Write a TMC4671 register.
    async fn write_register(&mut self, reg: Register, val: u32) -> Result<(), Self::Error>;

    /// Write a register, read it back, and compare against `val`.
    ///
    /// Production board implementations and production-semantic test backends
    /// must preserve the `Tmc4671Spi` retry policy.
    async fn write_register_verified(&mut self, reg: Register, val: u32)
    -> Result<(), Self::Error>;

    /// Read a register and compare against `expected`.
    ///
    /// Production board implementations and production-semantic test backends
    /// must preserve the `Tmc4671Spi` retry policy.
    async fn read_register_verified(
        &mut self,
        reg: Register,
        expected: u32,
    ) -> Result<u32, Self::Error>;
}

/// Result of one existing register await with measured monotonic wall time.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TimedRegisterIo<T> {
    /// Unchanged result returned by the wrapped register operation.
    pub result: T,
    /// Time spent inside the register-operation await, in microseconds.
    pub wall_us: u32,
    /// Whether elapsed-time subtraction or narrowing could not be represented.
    pub overflowed: bool,
}

/// Read a register and measure wall time around the existing backend await.
pub async fn timed_read_register<B>(
    backend: &mut B,
    reg: Register,
) -> TimedRegisterIo<Result<u32, B::Error>>
where
    B: CommissioningBackend,
{
    let start_us = backend.elapsed_us();
    let result = backend.read_register(reg).await;
    let end_us = backend.elapsed_us();
    let (wall_us, overflowed) = measured_wall_us(start_us, end_us);
    TimedRegisterIo {
        result,
        wall_us,
        overflowed,
    }
}

/// Read two registers as one backend operation and measure its wall time.
pub async fn timed_read_register_pair<B>(
    backend: &mut B,
    first: Register,
    second: Register,
) -> TimedRegisterIo<Result<(u32, u32), B::Error>>
where
    B: CommissioningBackend,
{
    let start_us = backend.elapsed_us();
    let result = backend.read_register_pair(first, second).await;
    let end_us = backend.elapsed_us();
    let (wall_us, overflowed) = measured_wall_us(start_us, end_us);
    TimedRegisterIo {
        result,
        wall_us,
        overflowed,
    }
}

/// Read one capture register and measure its wall time.
pub async fn timed_read_capture_register<B>(
    backend: &mut B,
    reg: Register,
) -> TimedRegisterIo<Result<u32, B::Error>>
where
    B: CommissioningBackend,
{
    let start_us = backend.elapsed_us();
    let result = backend.read_capture_register(reg).await;
    let end_us = backend.elapsed_us();
    let (wall_us, overflowed) = measured_wall_us(start_us, end_us);
    TimedRegisterIo {
        result,
        wall_us,
        overflowed,
    }
}

/// Write a register and measure wall time around the existing backend await.
pub async fn timed_write_register<B>(
    backend: &mut B,
    reg: Register,
    value: u32,
) -> TimedRegisterIo<Result<(), B::Error>>
where
    B: CommissioningBackend,
{
    let start_us = backend.elapsed_us();
    let result = backend.write_register(reg, value).await;
    let end_us = backend.elapsed_us();
    let (wall_us, overflowed) = measured_wall_us(start_us, end_us);
    TimedRegisterIo {
        result,
        wall_us,
        overflowed,
    }
}

fn measured_wall_us(start_us: u64, end_us: u64) -> (u32, bool) {
    let Some(wall_us) = end_us.checked_sub(start_us) else {
        return (0, true);
    };
    match u32::try_from(wall_us) {
        Ok(wall_us) => (wall_us, false),
        Err(_) => (u32::MAX, true),
    }
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

    /// True when a fault is latched, a disable is pending, or the STATUS pin
    /// is asserted. Checked before every non-bootstrap `STATUS_FLAGS` clear
    /// and before every enable-pin assertion outside the calibration phases.
    fn interlock_tripped(&self) -> bool;

    /// Record a motor lifecycle flag transition at the exact mutation site.
    ///
    /// Default no-op: production backends do not need an interleaved
    /// lifecycle log. Test backends override this to capture the relative
    /// order of lifecycle transitions against SPI/GPIO effects.
    fn record_lifecycle(
        &mut self,
        _flag: crate::tmc_control::lifecycle::LifecycleFlag,
        _value: bool,
    ) {
    }

    /// Persist the outcome of a safe-state restore pass.
    ///
    /// Called once at the end of every `restore_safe_state_via_backend`
    /// pass. `timestamp_ms` is a board-monotonic millisecond timestamp
    /// taken when the pass completed. Default no-op: a backend that does
    /// not need to retain this (e.g. a mock that doesn't exercise the
    /// re-arm gate) can ignore it.
    fn record_safe_state_report(
        &mut self,
        _report: crate::tmc_safe_state::SafeStateRestoreReport,
        _timestamp_ms: u32,
    ) {
    }
}

/// Encoder sampling capability used by inner and outer commissioning.
#[allow(async_fn_in_trait)]
pub trait CommissionEncoderRead: CommissionRegisterIo {
    /// Read the current ABN encoder position, in quadrature counts.
    async fn read_mechanical_encoder_position_counts(&mut self) -> u32;

    /// Read the current ABN encoder position in counts and preserve read failures.
    async fn try_read_mechanical_encoder_position_counts(&mut self) -> Result<u32, Self::Error> {
        Ok(self.read_mechanical_encoder_position_counts().await)
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

/// Combine the board-owned shutdown signals `CommissionTiming::is_shutdown_requested`
/// should report as true: an explicit disable request, an expired
/// host-liveness lease, or an outer shutdown/fault already in progress.
pub const fn shutdown_requested_from(
    disable_signal: bool,
    lease_expired: bool,
    shutdown_active: bool,
    fault_active: bool,
) -> bool {
    disable_signal || lease_expired || shutdown_active || fault_active
}

/// Board and channel configuration values consumed by commissioning.
pub trait CommissionBoardConfig {
    /// Read the configured torque/flux current limit.
    fn current_limit(&self) -> PidTorqueFluxLimit;

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

    /// This channel's feedback wiring (which encoder drives each TMC4671
    /// selection register).
    fn feedback_config(&self) -> crate::config::FeedbackConfig;

    /// Host-configured torque filter cutoff, including explicit zero disable.
    fn configured_torque_filter_hz(&self) -> Option<u16> {
        None
    }

    /// Host-configured flux filter cutoff, including explicit zero disable.
    fn configured_flux_filter_hz(&self) -> Option<u16> {
        None
    }

    /// Published TMC4671 position units per physical emitted pulse.
    ///
    /// Returns `None` until the complete motion scale has been configured.
    fn configured_physical_step_width(&self) -> Option<u32>;

    /// Encoder counts per mechanical revolution.
    fn mechanical_encoder_quadrature_counts_per_rev(&self) -> u32;

    /// Configured motor electrical pole-pair count.
    ///
    /// Decoded from the chip-verified `MOTOR_TYPE_N_POLE_PAIRS` sentinel
    /// captured by ADC calibration, not a check-specific hardcoded constant.
    /// Zero means the sentinel has not been established yet (motor
    /// configuration not calibrated); callers must treat that as a hard
    /// failure rather than falling back to a default pole-pair count.
    fn motor_pole_pairs(&self) -> u16;

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

    /// Record one characterization-only raw velocity observation.
    #[cfg(feature = "trace")]
    fn record_velocity_trace_observation(
        &mut self,
        _observation: crate::trace::stream::VelocityTraceObservation,
    ) {
    }

    /// Record one canonical velocity-sweep evidence record for offline audit.
    fn record_velocity_sweep_trace_record(
        &mut self,
        _record: crate::ankyra::velocity_sweep_encoding::VelocitySweepCanonicalRecord,
    ) {
    }
}

/// Connect-time configuration drain capability.
///
/// Config-time setters record their value in `TmcChannelState` instead of
/// enqueueing an SPI request, so the recorded value reaches the chip only
/// when something drains it. Arming needs that drain to have happened before
/// the motor is energized, which is why the capability sits in the
/// [`CommissioningBackend`] bundle rather than only on the board control tick.
#[allow(async_fn_in_trait)]
pub trait CommissionPendingConfig {
    /// Push every recorded-but-unapplied configuration item to the TMC4671.
    ///
    /// # Errors
    ///
    /// Returns the [`PendingConfigItem`] whose write failed. That item's
    /// pending bit stays set so a later drain retries it.
    async fn drain_pending_config(
        &mut self,
        sink: PendingConfigSink<'_>,
    ) -> Result<(), PendingConfigItem>;
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

    /// This channel's feedback wiring.
    fn commission_feedback_config(&self) -> crate::config::FeedbackConfig;
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

    /// Record one characterization-only raw velocity observation.
    #[cfg(feature = "trace")]
    fn record_velocity_trace_observation_impl(
        &mut self,
        _observation: crate::trace::stream::VelocityTraceObservation,
    ) {
    }

    /// Record one canonical velocity-sweep evidence record for offline audit.
    #[cfg(feature = "trace")]
    fn record_velocity_sweep_trace_record_impl(
        &mut self,
        _record: crate::ankyra::velocity_sweep_encoding::VelocitySweepCanonicalRecord,
    ) {
    }
}

/// Board-specific hooks used by [`SharedCommissioningBackend`].
#[allow(async_fn_in_trait)]
pub trait CommissionBackendHooks {
    /// Board-specific TMC4671 configuration.
    fn commission_tmc_config() -> &'static TmcConfig;

    /// Board-specific per-channel feedback wiring, indexed by channel.
    fn commission_feedback_configs() -> &'static [crate::config::FeedbackConfig];

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

    /// Record one characterization-only raw velocity observation.
    #[cfg(feature = "trace")]
    fn record_velocity_trace_observation(
        _channel: u8,
        _observation: crate::trace::stream::VelocityTraceObservation,
    ) {
    }

    /// Record one canonical velocity-sweep evidence record for offline audit.
    #[cfg(feature = "trace")]
    fn record_velocity_sweep_trace_record(
        _channel: u8,
        _record: crate::ankyra::velocity_sweep_encoding::VelocitySweepCanonicalRecord,
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
        self.tmc_state()
            .enable_pin_asserted
            .store(state, core::sync::atomic::Ordering::Relaxed);
    }
}

impl<'a, Spi, Pin, Hooks> CommissionConfigAccess for SharedCommissioningBackend<'a, Spi, Pin, Hooks>
where
    Hooks: CommissionBackendHooks,
{
    fn commission_tmc_config(&self) -> &'static TmcConfig {
        Hooks::commission_tmc_config()
    }

    fn commission_feedback_config(&self) -> crate::config::FeedbackConfig {
        Hooks::commission_feedback_configs()[self.channel as usize]
    }
}

impl<'a, Spi, Pin, Hooks> CommissionPendingConfig
    for SharedCommissioningBackend<'a, Spi, Pin, Hooks>
where
    Spi: SpiDevice<u8>,
    Hooks: CommissionBackendHooks,
{
    async fn drain_pending_config(
        &mut self,
        sink: PendingConfigSink<'_>,
    ) -> Result<(), PendingConfigItem> {
        crate::tmc_control::pending_config::drain_pending_config(
            self.tmc,
            self.tmc_state,
            *Hooks::commission_tmc_config(),
            sink,
        )
        .await
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

    #[cfg(feature = "trace")]
    fn record_velocity_trace_observation_impl(
        &mut self,
        observation: crate::trace::stream::VelocityTraceObservation,
    ) {
        Hooks::record_velocity_trace_observation(self.channel, observation);
    }

    #[cfg(feature = "trace")]
    fn record_velocity_sweep_trace_record_impl(
        &mut self,
        record: crate::ankyra::velocity_sweep_encoding::VelocitySweepCanonicalRecord,
    ) {
        Hooks::record_velocity_sweep_trace_record(self.channel, record);
    }
}

impl<T> CommissionRegisterIo for T
where
    T: CommissionTmcSpiAccess,
    T::Spi: embedded_hal::spi::SpiDevice<u8>,
{
    type Error = SpiError<<T::Spi as ErrorType>::Error>;

    async fn read_register(&mut self, reg: Register) -> Result<u32, Self::Error> {
        self.tmc_spi().read_register(reg).await
    }

    async fn read_capture_register(&mut self, reg: Register) -> Result<u32, Self::Error> {
        self.tmc_spi().read_register_blocking(reg)
    }

    async fn read_register_pair(
        &mut self,
        first: Register,
        second: Register,
    ) -> Result<(u32, u32), Self::Error> {
        self.tmc_spi().read_register_pair_blocking(first, second)
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
    T::Spi: embedded_hal::spi::SpiDevice<u8>,
{
    async fn read_mechanical_encoder_position_counts(&mut self) -> u32 {
        self.tmc_spi()
            .read_register(ABN_DECODER_COUNT)
            .await
            .unwrap_or(0)
    }

    async fn try_read_mechanical_encoder_position_counts(&mut self) -> Result<u32, Self::Error> {
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

    fn interlock_tripped(&self) -> bool {
        crate::tmc_control::interlock::fault_interlock_tripped(self.tmc_state())
    }

    fn record_safe_state_report(
        &mut self,
        report: crate::tmc_safe_state::SafeStateRestoreReport,
        timestamp_ms: u32,
    ) {
        use core::sync::atomic::Ordering::Relaxed;

        let tmc = self.tmc_state();
        let last_pass_failed = report.failed_writes > 0;
        // Every restore pass leaves the compound latch set: a clean pass
        // alone does not confirm the chip is safe to arm, only a fresh
        // sentinel pass does (see arm_motor_electrical). Only that sentinel
        // success clears safe_state_incomplete.
        tmc.safe_state_incomplete.store(true, Relaxed);
        tmc.safe_state_last_pass_failed
            .store(last_pass_failed, Relaxed);
        tmc.safe_state_failed_writes
            .store(report.failed_writes, Relaxed);
        tmc.safe_state_first_failed_register
            .store(report.first_failed_register.unwrap_or(0xFF), Relaxed);
        tmc.safe_state_first_failed_sub_addr
            .store(report.first_failed_sub_addr.unwrap_or(0xFF), Relaxed);
        tmc.safe_state_failure_timestamp_ms
            .store(timestamp_ms, Relaxed);
        tmc.safe_state_pass_count.store(
            tmc.safe_state_pass_count.load(Relaxed).saturating_add(1),
            Relaxed,
        );
    }
}

impl<T> CommissionBoardConfig for T
where
    T: CommissionTmcStateAccess + CommissionConfigAccess,
{
    fn current_limit(&self) -> PidTorqueFluxLimit {
        self.tmc_state()
            .load_current_limit(core::sync::atomic::Ordering::Relaxed)
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

    fn feedback_config(&self) -> crate::config::FeedbackConfig {
        self.commission_feedback_config()
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

    fn configured_physical_step_width(&self) -> Option<u32> {
        self.tmc_state().physical_step_width()
    }

    fn mechanical_encoder_quadrature_counts_per_rev(&self) -> u32 {
        self.tmc_state()
            .encoder_ppr
            .load(core::sync::atomic::Ordering::Relaxed)
            .saturating_mul(4)
    }

    fn motor_pole_pairs(&self) -> u16 {
        MotorTypeNPolePairs::from(
            self.tmc_state()
                .motor_sentinel
                .load(core::sync::atomic::Ordering::Relaxed),
        )
        .n_pole_pairs()
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

    #[cfg(feature = "trace")]
    fn record_velocity_trace_observation(
        &mut self,
        observation: crate::trace::stream::VelocityTraceObservation,
    ) {
        self.record_velocity_trace_observation_impl(observation);
    }

    #[cfg(feature = "trace")]
    fn record_velocity_sweep_trace_record(
        &mut self,
        record: crate::ankyra::velocity_sweep_encoding::VelocitySweepCanonicalRecord,
    ) {
        self.record_velocity_sweep_trace_record_impl(record);
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
    + CommissionPendingConfig
{
}

impl<T> CommissioningBackend for T where
    T: CommissionRegisterIo
        + CommissionMotorControl
        + CommissionEncoderRead
        + CommissionTiming
        + CommissionBoardConfig
        + CommissionTrace
        + CommissionPendingConfig
{
}

#[cfg(test)]
mod tests {
    use super::*;

    use core::convert::Infallible;
    #[cfg(feature = "trace")]
    use core::sync::atomic::AtomicI32;
    use core::sync::atomic::{
        AtomicBool, AtomicU8, AtomicU16, AtomicU32, AtomicU64, Ordering::Relaxed,
    };

    use embedded_hal_async::spi::Operation;
    use foci_firmware_macros::fixture_test;
    use foci_tmc4671::registers::PID_POSITION_TARGET;
    use std::sync::Mutex;
    use std::vec::Vec;

    use crate::commissioning::types::EngineKind;
    use crate::trace::events::{
        CommissionTraceEvent, EVENT_COMMISSION_DIAGNOSTIC_WARNING, TraceEventLevel,
    };

    #[test]
    fn shutdown_requested_from_false_when_all_clear() {
        assert!(!shutdown_requested_from(false, false, false, false));
    }

    #[test]
    fn shutdown_requested_from_true_when_disable_signal_set() {
        assert!(shutdown_requested_from(true, false, false, false));
    }

    #[test]
    fn shutdown_requested_from_true_when_lease_expired() {
        assert!(shutdown_requested_from(false, true, false, false));
    }

    #[test]
    fn shutdown_requested_from_true_when_shutdown_active() {
        assert!(shutdown_requested_from(false, false, true, false));
    }

    #[test]
    fn shutdown_requested_from_true_when_fault_active() {
        assert!(shutdown_requested_from(false, false, false, true));
    }

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
        electrical_model_theta_e_us: 160,
        current_validation_flux_max_encoder_delta_counts: 4,
        current_validation_torque_max_encoder_delta_counts: 16,
        resistance_max_expected_current_count: 1600,
        resistance_scale_metadata_validated: false,
        current_scale_ma_per_lsb: 1.155,
    };
    static TEST_FEEDBACK_CONFIGS: [crate::config::FeedbackConfig; 8] = {
        let mut configs = [crate::config::FeedbackConfig::abn_abn(); 8];
        configs[3] = crate::config::FeedbackConfig {
            phi_e_source: crate::config::FeedbackSource::Analog,
            position_source: crate::config::FeedbackSource::Abn,
        };
        configs
    };
    static ELAPSED_US: AtomicU64 = AtomicU64::new(0);
    static SHUTDOWN_REQUESTED: AtomicBool = AtomicBool::new(false);
    static LAST_DELAY_US: AtomicU32 = AtomicU32::new(0);
    static LAST_TRACE_CHANNEL: AtomicU8 = AtomicU8::new(u8::MAX);
    static LAST_TRACE_EVENT_ID: AtomicU16 = AtomicU16::new(0);
    #[cfg(feature = "trace")]
    static LAST_VELOCITY_DELTA: AtomicI32 = AtomicI32::new(0);

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

    impl embedded_hal::spi::SpiDevice<u8> for RecordingSpi {
        fn transaction(&mut self, operations: &mut [Operation<'_, u8>]) -> Result<(), Self::Error> {
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

        fn commission_feedback_configs() -> &'static [crate::config::FeedbackConfig] {
            &TEST_FEEDBACK_CONFIGS
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

        #[cfg(feature = "trace")]
        fn record_velocity_trace_observation(
            channel: u8,
            observation: crate::trace::stream::VelocityTraceObservation,
        ) {
            LAST_TRACE_CHANNEL.store(channel, Relaxed);
            LAST_VELOCITY_DELTA.store(observation.delta_counts, Relaxed);
        }
    }

    fn reset_hooks() {
        ELAPSED_US.store(0, Relaxed);
        SHUTDOWN_REQUESTED.store(false, Relaxed);
        LAST_DELAY_US.store(0, Relaxed);
        LAST_TRACE_CHANNEL.store(u8::MAX, Relaxed);
        LAST_TRACE_EVENT_ID.store(0, Relaxed);
        #[cfg(feature = "trace")]
        LAST_VELOCITY_DELTA.store(0, Relaxed);
        TEST_TMC_STATE.motor_sentinel.store(0, Relaxed);
        TEST_TMC_STATE.safe_state_incomplete.store(false, Relaxed);
        TEST_TMC_STATE
            .safe_state_last_pass_failed
            .store(false, Relaxed);
        TEST_TMC_STATE.safe_state_failed_writes.store(0, Relaxed);
        TEST_TMC_STATE
            .safe_state_first_failed_register
            .store(0xFF, Relaxed);
        TEST_TMC_STATE
            .safe_state_first_failed_sub_addr
            .store(0xFF, Relaxed);
        TEST_TMC_STATE
            .safe_state_failure_timestamp_ms
            .store(0, Relaxed);
        TEST_TMC_STATE.safe_state_pass_count.store(0, Relaxed);
    }

    #[fixture_test]
    fn set_commission_enable_pin_updates_enable_pin_asserted() {
        let _guard = TEST_LOCK
            .lock()
            .expect("commissioning backend test lock is available");
        reset_hooks();
        TEST_TMC_STATE.enable_pin_asserted.store(false, Relaxed);

        let mut tmc = Tmc4671Spi::new(RecordingSpi::default());
        let mut enable_pin = RecordingPin::default();
        let mut backend =
            SharedCommissioningBackend::<RecordingSpi, RecordingPin, RecordingHooks>::new(
                0,
                &mut tmc,
                &mut enable_pin,
                &TEST_TMC_STATE,
            );

        backend.set_commission_enable_pin(true);
        assert!(TEST_TMC_STATE.enable_pin_asserted.load(Relaxed));

        backend.set_commission_enable_pin(false);
        assert!(!TEST_TMC_STATE.enable_pin_asserted.load(Relaxed));
    }

    #[fixture_test]
    fn shared_backend_persists_safe_state_report_into_tmc_state() {
        let _guard = TEST_LOCK
            .lock()
            .expect("commissioning backend test lock is available");
        reset_hooks();

        let mut tmc = Tmc4671Spi::new(RecordingSpi::default());
        let mut enable_pin = RecordingPin::default();
        let mut backend =
            SharedCommissioningBackend::<RecordingSpi, RecordingPin, RecordingHooks>::new(
                0,
                &mut tmc,
                &mut enable_pin,
                &TEST_TMC_STATE,
            );

        let report = crate::tmc_safe_state::SafeStateRestoreReport {
            failed_writes: 2,
            first_failed_register: Some(0x1A),
            first_failed_sub_addr: Some(6),
        };
        backend.record_safe_state_report(report, 12_345);

        assert!(TEST_TMC_STATE.safe_state_incomplete.load(Relaxed));
        assert!(TEST_TMC_STATE.safe_state_last_pass_failed.load(Relaxed));
        assert_eq!(TEST_TMC_STATE.safe_state_failed_writes.load(Relaxed), 2);
        assert_eq!(
            TEST_TMC_STATE
                .safe_state_first_failed_register
                .load(Relaxed),
            0x1A
        );
        assert_eq!(
            TEST_TMC_STATE
                .safe_state_first_failed_sub_addr
                .load(Relaxed),
            6
        );
        assert_eq!(
            TEST_TMC_STATE.safe_state_failure_timestamp_ms.load(Relaxed),
            12_345
        );
        assert_eq!(TEST_TMC_STATE.safe_state_pass_count.load(Relaxed), 1);

        backend.record_safe_state_report(report, 12_346);
        assert_eq!(
            TEST_TMC_STATE.safe_state_pass_count.load(Relaxed),
            2,
            "every completed pass counts, not just failed ones"
        );
    }

    #[fixture_test]
    fn shared_backend_clean_report_leaves_incomplete_latch_set_but_clears_last_pass_failed() {
        let _guard = TEST_LOCK
            .lock()
            .expect("commissioning backend test lock is available");
        reset_hooks();
        TEST_TMC_STATE.safe_state_incomplete.store(false, Relaxed);
        TEST_TMC_STATE
            .safe_state_last_pass_failed
            .store(true, Relaxed);
        TEST_TMC_STATE.safe_state_pass_count.store(3, Relaxed);

        let mut tmc = Tmc4671Spi::new(RecordingSpi::default());
        let mut enable_pin = RecordingPin::default();
        let mut backend =
            SharedCommissioningBackend::<RecordingSpi, RecordingPin, RecordingHooks>::new(
                0,
                &mut tmc,
                &mut enable_pin,
                &TEST_TMC_STATE,
            );

        let report = crate::tmc_safe_state::SafeStateRestoreReport {
            failed_writes: 0,
            first_failed_register: None,
            first_failed_sub_addr: None,
        };
        backend.record_safe_state_report(report, 0);

        assert!(
            TEST_TMC_STATE.safe_state_incomplete.load(Relaxed),
            "only arm_motor_electrical's sentinel success may clear the compound latch"
        );
        assert!(
            !TEST_TMC_STATE.safe_state_last_pass_failed.load(Relaxed),
            "a clean pass should clear the last-pass-failed flag"
        );
        assert_eq!(
            TEST_TMC_STATE.safe_state_pass_count.load(Relaxed),
            4,
            "a clean pass still counts as a completed restore attempt"
        );
    }

    #[fixture_test]
    fn shared_backend_decodes_motor_pole_pairs_from_sentinel() {
        let _guard = TEST_LOCK
            .lock()
            .expect("commissioning backend test lock is available");
        reset_hooks();

        let sentinel: u32 = foci_tmc4671::registers::MotorTypeNPolePairs::new()
            .with_n_pole_pairs(50)
            .into();
        TEST_TMC_STATE.motor_sentinel.store(sentinel, Relaxed);

        let mut tmc = Tmc4671Spi::new(RecordingSpi::default());
        let mut enable_pin = RecordingPin::default();
        let backend = SharedCommissioningBackend::<RecordingSpi, RecordingPin, RecordingHooks>::new(
            1,
            &mut tmc,
            &mut enable_pin,
            &TEST_TMC_STATE,
        );

        assert_eq!(backend.motor_pole_pairs(), 50);
    }

    #[fixture_test]
    fn shared_backend_reads_feedback_config_for_its_own_channel() {
        let _guard = TEST_LOCK
            .lock()
            .expect("commissioning backend test lock is available");
        reset_hooks();

        let mut tmc = Tmc4671Spi::new(RecordingSpi::default());
        let mut enable_pin = RecordingPin::default();
        let backend = SharedCommissioningBackend::<RecordingSpi, RecordingPin, RecordingHooks>::new(
            3,
            &mut tmc,
            &mut enable_pin,
            &TEST_TMC_STATE,
        );

        // Channel 3's TEST_FEEDBACK_CONFIGS entry is deliberately distinct from
        // every other channel's (see Step 4) — this only passes if the lookup is
        // genuinely indexed by `self.channel`, not a hard-coded index or a
        // channel-blind constant.
        assert_eq!(
            backend.feedback_config(),
            crate::config::FeedbackConfig {
                phi_e_source: crate::config::FeedbackSource::Analog,
                position_source: crate::config::FeedbackSource::Abn,
            }
        );
    }

    #[fixture_test]
    fn shared_backend_reports_zero_pole_pairs_for_uncalibrated_sentinel() {
        let _guard = TEST_LOCK
            .lock()
            .expect("commissioning backend test lock is available");
        reset_hooks();

        let mut tmc = Tmc4671Spi::new(RecordingSpi::default());
        let mut enable_pin = RecordingPin::default();
        let backend = SharedCommissioningBackend::<RecordingSpi, RecordingPin, RecordingHooks>::new(
            1,
            &mut tmc,
            &mut enable_pin,
            &TEST_TMC_STATE,
        );

        assert_eq!(backend.motor_pole_pairs(), 0);
    }

    #[fixture_test]
    fn shared_backend_drains_pending_config_through_the_real_drain() {
        use foci_tmc4671::registers::PID_TORQUE_FLUX_LIMITS;

        let _guard = TEST_LOCK
            .lock()
            .expect("commissioning backend test lock is available");
        reset_hooks();

        pollster::block_on(async {
            TEST_TMC_STATE.run_current_ma_cfg.store(1_000, Relaxed);
            TEST_TMC_STATE
                .config_apply_pending
                .store(crate::state::tmc::PENDING_CURRENT, Relaxed);

            let mut tmc = Tmc4671Spi::new(RecordingSpi::default());
            let mut enable_pin = RecordingPin::default();
            {
                let mut backend = SharedCommissioningBackend::<
                    RecordingSpi,
                    RecordingPin,
                    RecordingHooks,
                >::new(
                    0, &mut tmc, &mut enable_pin, &TEST_TMC_STATE
                );
                let mut stored_inner_gains = None;
                backend
                    .drain_pending_config(PendingConfigSink {
                        stored_inner_gains: &mut stored_inner_gains,
                    })
                    .await
                    .expect("a recording SPI accepts every drain write");
            }

            let expected = crate::tmc_control::current_limit::current_limit_from_run_ma(
                1_000,
                TEST_CONFIG.current_scale_ma_per_lsb,
            )
            .expect("1000 mA at the board scale is representable")
            .register_value();
            let spi = tmc.into_inner();
            assert_eq!(
                spi.writes,
                std::vec![(PID_TORQUE_FLUX_LIMITS.addr(), expected)],
                "the delegation must reach the real drain, not report success without writing"
            );
            assert_eq!(TEST_TMC_STATE.config_apply_pending.load(Relaxed), 0);

            TEST_TMC_STATE.run_current_ma_cfg.store(0, Relaxed);
        });
    }

    #[test]
    fn shared_backend_satisfies_commissioning_backend_bundle() {
        fn assert_backend<T: CommissioningBackend>() {}

        assert_backend::<
            SharedCommissioningBackend<'static, RecordingSpi, RecordingPin, RecordingHooks>,
        >();
    }

    #[fixture_test]
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

    #[cfg(feature = "trace")]
    #[test]
    fn shared_backend_forwards_velocity_trace_observations() {
        let _guard = TEST_LOCK
            .lock()
            .expect("commissioning backend test lock is available");
        reset_hooks();
        let mut tmc = Tmc4671Spi::new(RecordingSpi::default());
        let mut enable_pin = RecordingPin::default();
        let mut backend =
            SharedCommissioningBackend::<RecordingSpi, RecordingPin, RecordingHooks>::new(
                3,
                &mut tmc,
                &mut enable_pin,
                &TEST_TMC_STATE,
            );

        backend.record_velocity_trace_observation_impl(
            crate::trace::stream::VelocityTraceObservation {
                delta_counts: -7,
                ..Default::default()
            },
        );

        assert_eq!(LAST_TRACE_CHANNEL.load(Relaxed), 3);
        assert_eq!(LAST_VELOCITY_DELTA.load(Relaxed), -7);
    }
}
