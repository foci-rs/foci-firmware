//! Shared register-sequenced resistance-identification runner.
//!
//! This is the only production code path that energizes the motor for
//! resistance identification. Commissioning drives it with
//! [`ResistanceRunMode::CommissioningManagedEnable`] (the managed-enable
//! exception that owns `set_enable_pin`); the `FOCI_RESISTANCE_TEST` diagnostic
//! drives it with [`ResistanceRunMode::ExternallyArmed`] over a motor that the
//! normal lifecycle path has already armed and enabled.
//!
//! Safety invariants enforced here:
//!
//! - `PIDOUT_UQ_UD_LIMITS` is set to the profile's full sweep authority
//!   (`profile.max_abs_ud()`) before any open-loop voltage command and before
//!   the chopper is enabled, in both run modes, so no sweep point is silently
//!   clamped. The pre-existing value is only a restore baseline, not an
//!   authority cap, in [`ResistanceRunMode::CommissioningManagedEnable`]
//!   (commissioning's ADC calibration phase leaves `PIDOUT_UQ_UD_LIMITS=1024`
//!   behind as such a baseline). In [`ResistanceRunMode::ExternallyArmed`] the
//!   pre-existing value is the motor's operational voltage authority, so a
//!   pre-sweep check rejects with [`ResistanceError::OutOfBoundsParameter`]
//!   when it cannot cover the sweep, instead of energizing past it.
//! - `PWM_SV_CHOP=7` is written only during energized operation; the safe-state
//!   restore sets it back to `0`.
//! - In [`ResistanceRunMode::ExternallyArmed`] the runner never asserts the
//!   enable pin; it rejects with [`ResistanceError::NotArmed`] when the motor is
//!   not already armed for open-loop voltage.
//! - Every exit path past the entry pre-checks and pre-energize validations
//!   ([`ResistanceError::OutOfBoundsParameter`], [`ResistanceError::InvalidMotorType`],
//!   [`ResistanceError::UnsupportedProfile`]) restores safe state through one
//!   shared helper, so no `Err` return or `?` failure can leave the power
//!   stage energized. The pre-energize validations themselves run before any
//!   energizing register write, so a rejection there must not write the
//!   restore sequence over an externally armed motor.

use super::profile::ResistanceProfile;
use super::types::{ResistanceError, ResistanceRunResult};

pub use super::state::{ResistancePoll, ResistanceRunner, ResistanceTimingEvidence};

/// How the runner manages the TMC4671 enable pin during the sweep.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum ResistanceRunMode {
    /// Commissioning owns the enable pin and may assert it for the sweep.
    CommissioningManagedEnable,
    /// The motor is already armed and enabled by the normal lifecycle path;
    /// the runner must not touch the enable pin.
    ExternallyArmed,
}

/// Error returned by backend register I/O during resistance identification.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum ResistanceBackendError {
    /// SPI transaction failed.
    Spi,
    /// The motor is not armed for open-loop voltage.
    NotArmed,
}

/// Board glue the runner needs to sequence a resistance run.
///
/// The diagnostic adapter implements [`set_enable_pin`](Self::set_enable_pin)
/// as a no-op and uses [`ResistanceRunMode::ExternallyArmed`]; the
/// commissioning adapter manages the enable pin and uses
/// [`ResistanceRunMode::CommissioningManagedEnable`].
#[allow(async_fn_in_trait)]
pub trait ResistanceBackend {
    /// Read a raw TMC4671 register by 7-bit address.
    async fn read_register(&mut self, addr: u8) -> Result<u32, ResistanceBackendError>;
    /// Write a raw TMC4671 register by 7-bit address.
    async fn write_register(&mut self, addr: u8, value: u32) -> Result<(), ResistanceBackendError>;
    /// Delay for the given number of milliseconds.
    async fn delay_ms(&mut self, ms: u64);
    /// Monotonic microsecond timestamp.
    fn elapsed_us(&self) -> u64;
    /// Set the TMC4671 enable pin. No-op in [`ResistanceRunMode::ExternallyArmed`].
    fn set_enable_pin(&mut self, enabled: bool);
    /// Whether the motor is already armed and enabled for open-loop voltage.
    fn is_armed_for_open_loop_voltage(&self) -> bool;
    /// The board's resistance-identification profile.
    fn profile(&self) -> ResistanceProfile;
    /// Encoder counts per revolution, used to wrap the encoder-move delta.
    fn encoder_counts_per_rev(&self) -> u32;
    /// Whether ADC current-offset calibration is ready.
    fn adc_offsets_ready(&self) -> bool;
}

/// Run the shared resistance-identification sweep.
///
/// Sequences power-stage setup, a fixed two-axis voltage sweep with signed
/// anchors, the firmware-owned fixed-point fit, and an unconditional safe-state
/// restore. The motor is energized only between the bounded-limit/chopper setup
/// and the restore.
///
/// # Errors
///
/// - [`ResistanceError::NotAdcReady`] when ADC offsets are not ready.
/// - [`ResistanceError::NotArmed`] in [`ResistanceRunMode::ExternallyArmed`]
///   when the motor is not armed for open-loop voltage.
/// - [`ResistanceError::OutOfBoundsParameter`] in
///   [`ResistanceRunMode::ExternallyArmed`] when the saved operational voltage
///   limit cannot cover the profile's full sweep authority.
/// - [`ResistanceError::InvalidMotorType`] when `MOTOR_TYPE` is not the
///   two-phase stepper type.
/// - [`ResistanceError::UnsupportedProfile`] when the fixed profile would exceed
///   the per-window or total energized-exposure budget.
/// - [`ResistanceError::WrongPowerStage`] when the power-stage readback does not
///   match the profile.
/// - [`ResistanceError::PeakCurrentExceeded`] when an energized window's mean
///   current exceeds the profile peak.
/// - [`ResistanceError::BackendIo`] when a backend register transaction fails.
/// - Any fit/evaluation error produced by the resumable runner.
///
/// All errors except the entry pre-checks and pre-energize validations
/// ([`ResistanceError::NotAdcReady`], [`ResistanceError::NotArmed`],
/// [`ResistanceError::OutOfBoundsParameter`], [`ResistanceError::InvalidMotorType`],
/// and [`ResistanceError::UnsupportedProfile`]) restore safe TMC4671 state
/// before returning.
pub async fn run_resistance_identification<B: ResistanceBackend>(
    backend: &mut B,
    mode: ResistanceRunMode,
) -> Result<ResistanceRunResult, ResistanceError> {
    let mut logical_now_us = backend.elapsed_us();
    let profile = backend.profile();
    let mut runner = ResistanceRunner::new(mode, profile, logical_now_us)?;
    loop {
        match runner.poll(backend, logical_now_us).await {
            ResistancePoll::ReadyNow => {}
            ResistancePoll::At(deadline_us) => {
                let delay_us = deadline_us.saturating_sub(logical_now_us);
                backend.delay_ms(delay_us.div_ceil(1_000)).await;
                logical_now_us = backend.elapsed_us().max(deadline_us);
            }
            ResistancePoll::Complete(result) => return result,
        }
    }
}
