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

use foci_tmc4671::registers::{
    ABN_DECODER_COUNT, ADC_IWY_IUX, MODE_RAMP_MODE_MOTION, MotionMode, STATUS_FLAGS, UQ_UD_EXT,
};

use super::profile::ResistanceProfile;
use super::types::{ElectricalAxis, ResistanceError, ResistanceRunResult, ResistanceWarningFlags};

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

/// Number of samples averaged per window. Kept small and fixed; the windows
/// settle for `settle_ms`/`zero_settle_ms` before sampling.
const SAMPLES_PER_WINDOW: u8 = 8;

/// Result of one sampled window.
///
/// Carries only the mean physical phase current; the window's `STATUS_FLAGS` and
/// encoder motion are folded into the run-wide accumulator and the warning flags
/// inside [`capture_window`], so they are not carried out here.
#[allow(dead_code, reason = "removed with the remaining legacy sweep helpers")]
struct WindowMean {
    /// Mean physical phase-current count for the axis channel.
    phase_current_count: i32,
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
/// - Any fit/evaluation error from [`fit_resistance_axis`] or
///   [`evaluate_resistance_result`].
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
                logical_now_us = deadline_us;
            }
            ResistancePoll::Complete(result) => return result,
        }
    }
}

/// Capture one window: settle, sample `SAMPLES_PER_WINDOW` times, and return the
/// mean physical phase current (selected per axis). Folds the sampled status
/// flags into the run-wide accumulator and raises the `ENCODER_MOVED` and
/// `STATUS_FLAGS` warning flags as appropriate.
#[allow(dead_code, reason = "removed with the remaining legacy sweep helpers")]
async fn capture_window<B: ResistanceBackend>(
    backend: &mut B,
    profile: &ResistanceProfile,
    axis: ElectricalAxis,
    ud: i16,
    settle_ms: u16,
    status_flags_or: &mut u32,
    warning_flags: &mut ResistanceWarningFlags,
) -> Result<WindowMean, ResistanceError> {
    let energized = ud != 0;

    if energized {
        write(backend, UQ_UD_EXT.addr(), pack_ud(ud)).await?;
        write(
            backend,
            MODE_RAMP_MODE_MOTION.addr(),
            MotionMode::UqUdExt as u32,
        )
        .await?;
    } else {
        write(
            backend,
            MODE_RAMP_MODE_MOTION.addr(),
            MotionMode::Stopped as u32,
        )
        .await?;
        write(backend, UQ_UD_EXT.addr(), 0).await?;
    }

    // Capture the encoder before the dwell so the move detector spans the full
    // energized settling and sampling interval.
    let encoder_before = read(backend, ABN_DECODER_COUNT.addr()).await?;

    backend.delay_ms(u64::from(settle_ms)).await;

    let mut current_sum: i32 = 0;
    let mut window_status_or: u32 = 0;
    for _ in 0..SAMPLES_PER_WINDOW {
        let packed = read(backend, ADC_IWY_IUX.addr()).await?;
        current_sum += i32::from(phase_current_for_axis(axis, packed));
        window_status_or |= read(backend, STATUS_FLAGS.addr()).await?;
        backend
            .delay_ms(u64::from(profile.sample_interval_ms))
            .await;
    }
    let phase_current_count = current_sum / i32::from(SAMPLES_PER_WINDOW);

    let encoder_after = read(backend, ABN_DECODER_COUNT.addr()).await?;
    let encoder_delta_count = wrapped_encoder_delta(
        encoder_before,
        encoder_after,
        backend.encoder_counts_per_rev(),
    );

    *status_flags_or |= window_status_or;
    if window_status_or & profile.status_flags_warn_mask != 0 {
        warning_flags.insert(ResistanceWarningFlags::STATUS_FLAGS);
    }
    if energized && encoder_delta_count.unsigned_abs() > u32::from(profile.encoder_move_warn_counts)
    {
        warning_flags.insert(ResistanceWarningFlags::ENCODER_MOVED);
    }

    Ok(WindowMean {
        phase_current_count,
    })
}

/// Read a register, mapping backend I/O failures to [`ResistanceError::BackendIo`].
async fn read<B: ResistanceBackend>(backend: &mut B, addr: u8) -> Result<u32, ResistanceError> {
    backend
        .read_register(addr)
        .await
        .map_err(|_| ResistanceError::BackendIo)
}

/// Write a register, mapping backend I/O failures to [`ResistanceError::BackendIo`].
async fn write<B: ResistanceBackend>(
    backend: &mut B,
    addr: u8,
    value: u32,
) -> Result<(), ResistanceError> {
    backend
        .write_register(addr, value)
        .await
        .map_err(|_| ResistanceError::BackendIo)
}

/// Pack a signed `UD_EXT` value with `UQ_EXT=0` into the `UQ_UD_EXT` word.
fn pack_ud(ud: i16) -> u32 {
    u32::from(ud as u16)
}

/// Select the physical phase-current channel for the electrical axis.
///
/// `ADC_IWY_IUX` packs IWY in the upper half and IUX in the lower half. Axis 0
/// (PHI_E_EXT=0, coil A) uses IUX; axis 1 (PHI_E_EXT=16384, coil B) uses IWY.
fn phase_current_for_axis(axis: ElectricalAxis, packed: u32) -> i16 {
    if axis.index == 0 {
        crate::tmc::decode_signed_lower_half(packed)
    } else {
        crate::tmc::decode_signed_upper_half(packed)
    }
}

/// Wrap an encoder delta into `[-counts_per_rev/2, counts_per_rev/2]`.
fn wrapped_encoder_delta(before: u32, after: u32, counts_per_rev: u32) -> i32 {
    let raw = (after as i32).wrapping_sub(before as i32);
    let counts_per_rev = counts_per_rev as i32;
    if counts_per_rev <= 0 {
        return raw;
    }
    let half = counts_per_rev / 2;
    if raw > half {
        raw - counts_per_rev
    } else if raw < -half {
        raw + counts_per_rev
    } else {
        raw
    }
}
