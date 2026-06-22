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
//! - Every exit path past the entry pre-checks restores safe state through one
//!   shared helper, so no `Err` return or `?` failure can leave the power stage
//!   energized.

use foci_tmc4671::registers::{
    ABN_DECODER_COUNT, ADC_IWY_IUX, DSADC_MDEC_B_MDEC_A, MODE_RAMP_MODE_MOTION,
    MOTOR_TYPE_N_POLE_PAIRS, MotionMode, MotorType, MotorTypeNPolePairs, PHI_E_EXT,
    PHI_E_SELECTION, PIDOUT_UQ_UD_LIMITS, PWM_BBM_H_BBM_L, PWM_MAXCNT, PWM_SV_CHOP, STATUS_FLAGS,
    UQ_UD_EXT,
};

use super::profile::ResistanceProfile;
use super::types::{
    ElectricalAxis, ResistanceError, ResistancePoint, ResistanceRunResult, ResistanceWarningFlags,
    SignedAnchorEvidence,
};
use crate::resistance_identification::fit::{evaluate_resistance_result, fit_resistance_axis};

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
/// All errors except the two entry pre-checks ([`ResistanceError::NotAdcReady`]
/// and [`ResistanceError::NotArmed`]) restore safe TMC4671 state before
/// returning.
pub async fn run_resistance_identification<B: ResistanceBackend>(
    backend: &mut B,
    mode: ResistanceRunMode,
) -> Result<ResistanceRunResult, ResistanceError> {
    // Entry pre-checks. These run before any register is written, so a
    // rejection here cannot leave the power stage in a disturbed state and
    // must not write a safe-state restore over an externally armed motor.
    if !backend.adc_offsets_ready() {
        return Err(ResistanceError::NotAdcReady);
    }
    if mode == ResistanceRunMode::ExternallyArmed && !backend.is_armed_for_open_loop_voltage() {
        return Err(ResistanceError::NotArmed);
    }

    // Save the pre-existing voltage limit so the restore can put it back. From
    // here on every exit path funnels through `restore_safe_state`.
    let saved_voltage_limit = match backend.read_register(PIDOUT_UQ_UD_LIMITS.addr()).await {
        Ok(value) => value,
        Err(_) => return Err(ResistanceError::BackendIo),
    };

    let profile = backend.profile();

    // Run the energizing body, then unconditionally restore safe state. The
    // body's `Err` is preserved; a restore failure is reported only when the
    // body itself succeeded.
    let body_result = run_energized_body(backend, mode, &profile, saved_voltage_limit).await;
    let restore_result = restore_safe_state(backend, mode, saved_voltage_limit).await;

    match (body_result, restore_result) {
        (Ok(result), Ok(())) => Ok(result),
        (Ok(_), Err(())) => Err(ResistanceError::BackendIo),
        (Err(error), _) => Err(error),
    }
}

/// Energizing body. Any early return here is caught by the unconditional
/// restore in [`run_resistance_identification`].
async fn run_energized_body<B: ResistanceBackend>(
    backend: &mut B,
    mode: ResistanceRunMode,
    profile: &ResistanceProfile,
    saved_voltage_limit: u32,
) -> Result<ResistanceRunResult, ResistanceError> {
    // In `ExternallyArmed` mode the saved limit is the motor's operational
    // voltage authority, not a restore baseline (commissioning's ADC
    // calibration intentionally leaves PIDOUT_UQ_UD_LIMITS=1024 as such a
    // baseline; see `step_adc_calibrate`). The sweep must never exceed that
    // authority for a diagnostic run over an externally armed motor, so
    // reject here, before any energizing register write, rather than silently
    // clamping sweep points below the requested UD. This is a pre-sweep
    // rejection like the motor-type and exposure-budget checks below it, so
    // the unconditional restore in `run_resistance_identification` still
    // restores safe state on this exit.
    if mode == ResistanceRunMode::ExternallyArmed
        && saved_voltage_limit < u32::from(profile.max_abs_ud().unsigned_abs())
    {
        return Err(ResistanceError::OutOfBoundsParameter);
    }

    // Verify the typed/masked MOTOR_TYPE through the register wrapper.
    let motor_type_raw = read(backend, MOTOR_TYPE_N_POLE_PAIRS.addr()).await?;
    let motor_type = MotorTypeNPolePairs::from(motor_type_raw).motor_type();
    if motor_type != MotorType::TwoPhase as u8 {
        return Err(ResistanceError::InvalidMotorType);
    }

    // Static exposure-budget gate. Proves by construction that the fixed sweep
    // stays within the thermal budget before any energizing.
    let largest_window_exposure = (profile.max_abs_ud() as u64).pow(2) * profile.settle_ms as u64;
    if profile.exposure_per_axis() > profile.max_total_exposure
        || largest_window_exposure > profile.max_window_exposure
    {
        return Err(ResistanceError::UnsupportedProfile);
    }

    // Power-stage setup. The sweep voltage limit is set to the profile's full
    // sweep authority (not clamped against the saved baseline) before the
    // chopper is enabled and before any open-loop voltage command, so no
    // sweep point is silently clamped. `ExternallyArmed` mode already
    // rejected above when the saved operational limit could not cover this;
    // `CommissioningManagedEnable` always uses the full authority because the
    // saved value there is only a restore baseline, not an authority cap.
    write(
        backend,
        MODE_RAMP_MODE_MOTION.addr(),
        MotionMode::Stopped as u32,
    )
    .await?;
    write(backend, UQ_UD_EXT.addr(), 0).await?;
    write(
        backend,
        PIDOUT_UQ_UD_LIMITS.addr(),
        u32::from(profile.max_abs_ud().unsigned_abs()),
    )
    .await?;
    write(backend, PWM_SV_CHOP.addr(), profile.pwm_sv_chop).await?;

    // Power-stage readback verification.
    let pwm_sv_chop = read(backend, PWM_SV_CHOP.addr()).await?;
    let pwm_maxcnt = read(backend, PWM_MAXCNT.addr()).await?;
    let pwm_bbm = read(backend, PWM_BBM_H_BBM_L.addr()).await?;
    let dsadc = read(backend, DSADC_MDEC_B_MDEC_A.addr()).await?;
    if pwm_sv_chop != profile.pwm_sv_chop
        || pwm_maxcnt != u32::from(profile.pwm_maxcnt)
        || pwm_bbm != ((u32::from(profile.bbm_h) << 8) | u32::from(profile.bbm_l))
        || dsadc != ((u32::from(profile.dsadc_mdec_b) << 16) | u32::from(profile.dsadc_mdec_a))
    {
        return Err(ResistanceError::WrongPowerStage);
    }

    // Initialize the status-flags accumulator from the pre-sweep snapshot.
    let mut status_flags_or = read(backend, STATUS_FLAGS.addr()).await?;

    // Only assert the enable pin under the commissioning managed-enable
    // exception. ExternallyArmed relies on the already-enabled lifecycle path.
    if mode == ResistanceRunMode::CommissioningManagedEnable {
        backend.set_enable_pin(true);
    }

    let mut warning_flags = ResistanceWarningFlags::NONE;

    let axis0 = measure_axis(
        backend,
        profile,
        profile.axes[0],
        &mut status_flags_or,
        &mut warning_flags,
    )
    .await?;
    let axis1 = measure_axis(
        backend,
        profile,
        profile.axes[1],
        &mut status_flags_or,
        &mut warning_flags,
    )
    .await?;

    let mut result = evaluate_resistance_result(profile, axis0, axis1, status_flags_or)?;
    result.warning_flags.insert(warning_flags);
    Ok(result)
}

/// Measure one electrical axis: zero/energized windows, signed anchors, and the
/// fixed-point fit.
async fn measure_axis<B: ResistanceBackend>(
    backend: &mut B,
    profile: &ResistanceProfile,
    axis: ElectricalAxis,
    status_flags_or: &mut u32,
    warning_flags: &mut ResistanceWarningFlags,
) -> Result<crate::resistance_identification::types::ResistanceAxisResult, ResistanceError> {
    // Select the open-loop external electrical angle for this axis.
    write(backend, PHI_E_SELECTION.addr(), 1).await?;
    write(backend, PHI_E_EXT.addr(), u32::from(axis.phi_e_ext)).await?;

    // Initial zero window for offset tracking.
    let initial_zero = capture_window(
        backend,
        profile,
        axis,
        0,
        profile.zero_settle_ms,
        status_flags_or,
        warning_flags,
    )
    .await?;

    let mut points = [ResistancePoint {
        requested_ud: 0,
        voltage_delta_count: 0,
        current_delta_count: 0,
        selected: false,
    }; 8];

    let mut first_anchor_current: Option<i32> = None;

    // Rolling leading zero, advanced to each point's trailing zero so every
    // point is bracketed against its ADJACENT zero windows (prior trailing and
    // own trailing). Bracketing against the global initial zero would bias late
    // points against a stale start-of-sweep offset and defeat the per-point
    // trailing zeros' drift compensation.
    let mut leading_zero = initial_zero.phase_current_count;

    for (index, &ud) in profile.ud_points.iter().enumerate() {
        let energized = capture_window(
            backend,
            profile,
            axis,
            ud,
            profile.settle_ms,
            status_flags_or,
            warning_flags,
        )
        .await?;

        check_peak_current(profile, energized.phase_current_count)?;

        // Adjacent zero window after the energized point.
        let trailing_zero = capture_window(
            backend,
            profile,
            axis,
            0,
            profile.zero_settle_ms,
            status_flags_or,
            warning_flags,
        )
        .await?;

        // Zero-subtract against the mean of the adjacent bracketing zero
        // windows (prior trailing and own trailing).
        let zero_current = (leading_zero + trailing_zero.phase_current_count) / 2;
        let current_delta_count = energized.phase_current_count - zero_current;

        // The first positive anchor must feed `build_signed_anchor` with the
        // same zero-subtraction as every other point and the negative anchor,
        // otherwise the signed slope, asymmetry, and drift mix raw and
        // zero-subtracted currents.
        if ud == profile.signed_anchor_ud {
            first_anchor_current = Some(current_delta_count);
        }

        points[index] = ResistancePoint {
            requested_ud: ud,
            voltage_delta_count: i32::from(ud),
            current_delta_count,
            selected: false,
        };

        // Advance the rolling bracket to this point's trailing zero.
        leading_zero = trailing_zero.phase_current_count;
    }

    // Repeated positive anchor near the end of the axis for drift evidence.
    let repeated_anchor = capture_window(
        backend,
        profile,
        axis,
        profile.signed_anchor_ud,
        profile.settle_ms,
        status_flags_or,
        warning_flags,
    )
    .await?;
    check_peak_current(profile, repeated_anchor.phase_current_count)?;

    let repeated_zero = capture_window(
        backend,
        profile,
        axis,
        0,
        profile.zero_settle_ms,
        status_flags_or,
        warning_flags,
    )
    .await?;

    // Bracket the repeated anchor against its adjacent zeros (the last sweep
    // point's trailing zero and the repeated anchor's own trailing zero), then
    // advance the rolling bracket past it.
    let repeated_zero_mean = (leading_zero + repeated_zero.phase_current_count) / 2;
    let repeated_zero_subtracted = repeated_anchor.phase_current_count - repeated_zero_mean;
    leading_zero = repeated_zero.phase_current_count;

    // One negative signed anchor.
    let negative_anchor = capture_window(
        backend,
        profile,
        axis,
        -profile.signed_anchor_ud,
        profile.settle_ms,
        status_flags_or,
        warning_flags,
    )
    .await?;
    check_peak_current(profile, negative_anchor.phase_current_count)?;

    let negative_zero = capture_window(
        backend,
        profile,
        axis,
        0,
        profile.zero_settle_ms,
        status_flags_or,
        warning_flags,
    )
    .await?;

    // Bracket the negative anchor against its adjacent zeros (the repeated
    // anchor's trailing zero and the negative anchor's own trailing zero).
    let negative_zero_mean = (leading_zero + negative_zero.phase_current_count) / 2;
    let negative_zero_subtracted = negative_anchor.phase_current_count - negative_zero_mean;

    // The positive anchor is always the zero-subtracted value: either the first
    // matching sweep point or, if none matched, the zero-subtracted repeated
    // anchor. It must never be a raw energized current.
    let positive_anchor_current = first_anchor_current.unwrap_or(repeated_zero_subtracted);

    let signed_anchor = build_signed_anchor(
        positive_anchor_current,
        negative_zero_subtracted,
        repeated_zero_subtracted,
        profile.signed_anchor_ud,
    );

    fit_resistance_axis(axis, profile, &points, Some(signed_anchor))
}

/// Capture one window: settle, sample `SAMPLES_PER_WINDOW` times, and return the
/// mean physical phase current (selected per axis). Folds the sampled status
/// flags into the run-wide accumulator and raises the `ENCODER_MOVED` and
/// `STATUS_FLAGS` warning flags as appropriate.
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

/// Abort with [`ResistanceError::PeakCurrentExceeded`] when an energized
/// window's mean physical current exceeds the profile peak.
fn check_peak_current(
    profile: &ResistanceProfile,
    current_count: i32,
) -> Result<(), ResistanceError> {
    if current_count.unsigned_abs() > u32::from(profile.max_expected_current_count) {
        Err(ResistanceError::PeakCurrentExceeded)
    } else {
        Ok(())
    }
}

/// Build the signed-anchor evidence from the zero-subtracted anchor currents.
fn build_signed_anchor(
    positive_current_count: i32,
    negative_current_count: i32,
    repeated_positive_current_count: i32,
    anchor_ud: i16,
) -> SignedAnchorEvidence {
    // Signed slope across the +/-UD anchor pair, scaled to milli-units.
    let voltage_span = i64::from(anchor_ud) * 2;
    let current_span = i64::from(positive_current_count) - i64::from(negative_current_count);
    let signed_count_slope_milli = if current_span != 0 {
        ((voltage_span * 1000) / current_span) as i32
    } else {
        0
    };

    // Asymmetry: |+I| vs |-I| relative to the larger magnitude.
    let pos_abs = positive_current_count.unsigned_abs();
    let neg_abs = negative_current_count.unsigned_abs();
    let max_abs = pos_abs.max(neg_abs).max(1);
    let asymmetry_permille =
        ((u64::from(pos_abs.abs_diff(neg_abs)) * 1000) / u64::from(max_abs)).min(1000) as u16;

    // Drift: repeated positive anchor vs first positive anchor.
    let first_abs = positive_current_count.unsigned_abs().max(1);
    let drift_abs = positive_current_count.abs_diff(repeated_positive_current_count);
    let drift_permille = ((u64::from(drift_abs) * 1000) / u64::from(first_abs)).min(1000) as u16;

    SignedAnchorEvidence {
        positive_current_count,
        negative_current_count,
        signed_count_slope_milli,
        asymmetry_permille,
        repeated_positive_current_count,
        drift_permille,
    }
}

/// Restore safe TMC4671 state. Always called before the runner returns past the
/// entry pre-checks. Sets motion mode stopped, clears the external voltage,
/// disables the chopper, restores the saved voltage limit, and drops the enable
/// pin only under the commissioning managed-enable exception.
async fn restore_safe_state<B: ResistanceBackend>(
    backend: &mut B,
    mode: ResistanceRunMode,
    saved_voltage_limit: u32,
) -> Result<(), ()> {
    let mut failed = false;
    let writes = [
        (MODE_RAMP_MODE_MOTION.addr(), MotionMode::Stopped as u32),
        (UQ_UD_EXT.addr(), 0),
        (PWM_SV_CHOP.addr(), 0),
        (PIDOUT_UQ_UD_LIMITS.addr(), saved_voltage_limit),
    ];
    for (addr, value) in writes {
        if backend.write_register(addr, value).await.is_err() {
            failed = true;
        }
    }

    if mode == ResistanceRunMode::CommissioningManagedEnable {
        backend.set_enable_pin(false);
    }

    if failed { Err(()) } else { Ok(()) }
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
