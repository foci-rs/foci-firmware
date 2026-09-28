//! TMC4671 command handlers: `tmc_init`, `tmc_read_register`,
//! `tmc_write_register`, `tmc_set_current`, `tmc_set_motion_scale`,
//! `tmc_set_encoder_dir`, `tmc_set_flux_filter`, `tmc_set_pid_gains`,
//! `tmc_set_position_filter`, `tmc_set_position_gains`,
//! `tmc_set_torque_filter`, `tmc_set_velocity_feedforward_rpm`,
//! `tmc_set_velocity_filter`, `tmc_set_velocity_limit`,
//! `tmc_set_voltage_limit`, `tmc_current_torque_sample_test`,
//! `tmc_position_torque_offset_sample_test`, `tmc_voltage_step_test`,
//! `tmc_resistance_test`, `tmc_set_auto_calibrate_on_enable`, `tmc_test_spin`.
//!
//! Handler bodies bind to the capability traits declared in
//! [`crate::context`]; the board crate wires up the trait impls on its
//! own `FirmwareCtx`.

use ankyra::prelude::klipper_command;
use core::sync::atomic::Ordering;
use foci_tmc4671::pid::{PidIGain, PidPGain};

use crate::context::{ShutdownRecovery, TmcHost};
use crate::shutdown::ShutdownReason;
use crate::state::tmc::TmcChannelState;

pub(crate) const MAX_CURRENT_STEP_TARGET: i16 = 1_000;
const MIN_CURRENT_STEP_DURATION_MS: u16 = 20;
pub(crate) const MAX_CURRENT_STEP_DURATION_MS: u16 = 200;
const MIN_CURRENT_SAMPLE_DELAY_MS: u16 = 1;
pub(crate) const MAX_CURRENT_SAMPLE_DELAY_MS: u16 = MAX_CURRENT_STEP_DURATION_MS;
const MAX_VOLTAGE_STEP_EXT: i16 = 1_024;
const MIN_CURRENT_STEP_VOLTAGE_LIMIT: u16 =
    crate::state::tmc::MIN_OPERATIONAL_PIDOUT_UQ_UD_LIMIT as u16;
pub(crate) const MAX_CURRENT_STEP_VOLTAGE_LIMIT: u16 = 29_000;

/// Trigger TMC4671 initialization sequence (dev-only).
#[cfg(feature = "dev")]
#[klipper_command]
pub fn tmc_init(ctx: &mut dyn TmcWriteHost, oid: u8) {
    let Some((channel, _state)) = foci_tmc_state_or_shutdown(ctx, oid) else {
        return;
    };
    let _ = ctx.enqueue_request(crate::tmc::tmc_init(channel));
}

/// Test motor rotation by sweeping PHI_E_EXT through N revolutions (dev-only).
#[cfg(feature = "dev")]
#[klipper_command]
pub fn tmc_test_spin(ctx: &mut dyn TmcWriteHost, oid: u8, revolutions: u8) {
    let Some((channel, _state)) = foci_tmc_state_or_shutdown(ctx, oid) else {
        return;
    };
    let _ = ctx.enqueue_request(crate::tmc::tmc_test_spin(channel, revolutions));
}

/// Write a TMC4671 register via SPI (dev-only).
#[cfg(feature = "dev")]
#[klipper_command]
pub fn tmc_write_register(ctx: &mut dyn TmcWriteHost, oid: u8, addr: u8, value: u32) {
    let Some((channel, _state)) = foci_tmc_state_or_shutdown(ctx, oid) else {
        return;
    };
    let _ = ctx.enqueue_request(crate::tmc::tmc_write_register(channel, addr, value));
}

/// Read a TMC4671 register via SPI (dev-only).
///
/// The response is sent asynchronously as a `tmc_register_value` reply.
/// If the queue is full the request is dropped silently; the Python
/// query_command timeout handles the missing reply naturally.
#[cfg(feature = "dev")]
#[klipper_command]
pub fn tmc_read_register(ctx: &mut dyn TmcWriteHost, oid: u8, addr: u8) {
    let Some((channel, _state)) = foci_tmc_state_or_shutdown(ctx, oid) else {
        return;
    };
    if ctx
        .enqueue_request(crate::tmc::tmc_read_register(channel, oid, addr))
        .is_err()
    {
        defmt::warn!("tmc_read_register dropped: addr={:#04x}", addr);
    }
}

/// Set motor run current. The firmware converts milliamps to
/// `PID_TORQUE_FLUX_LIMITS` register counts using the board-specific
/// current scaling factor.
///
/// Recorded in channel state and applied by the TMC task on its control
/// tick. See [`TmcChannelState::config_apply_pending`] for why this does
/// not enqueue.
#[klipper_command]
pub fn tmc_set_current(ctx: &mut dyn TmcWriteHost, oid: u8, run_ma: u32) {
    let Some((_channel, state)) = foci_tmc_state_or_shutdown(ctx, oid) else {
        return;
    };
    state.run_current_ma_cfg.store(run_ma, Ordering::Relaxed);
    state
        .config_apply_pending
        .fetch_or(crate::state::tmc::PENDING_CURRENT, Ordering::Release);
}

/// Configure independent encoder-feedback and planner-motion scales while idle.
#[klipper_command]
pub fn tmc_set_motion_scale(
    ctx: &mut dyn TmcWriteHost,
    oid: u8,
    channel: u8,
    encoder_ppr: u32,
    planner_steps_per_rev: u32,
) {
    let Some((channel, _state)) = foci_tmc_state_matching_or_shutdown(ctx, oid, channel) else {
        return;
    };
    let req = match crate::tmc::tmc_set_encoder_hw(channel, oid, encoder_ppr, planner_steps_per_rev)
    {
        Ok(req) => req,
        Err(error) => {
            defmt::error!("tmc_set_motion_scale rejected: {}", error);
            ctx.report_motion_scale_rejection(oid, error.into());
            shutdown_request_validation(ctx);
            return;
        }
    };
    if let Err(rejection) = ctx.motion_scale_idle(channel) {
        defmt::error!("tmc_set_motion_scale requires disarmed motor, empty queue, inactive timer");
        ctx.report_motion_scale_rejection(oid, rejection);
        shutdown_request_validation(ctx);
        return;
    }
    if ctx.enqueue_request(req).is_err() {
        ctx.report_motion_scale_rejection(
            oid,
            crate::motion_scale::MotionScaleRejection::RequestQueueFull,
        );
        if ctx.enter_shutdown() {
            let clock = ctx.now();
            ctx.send_shutdown(ShutdownReason::TmcSetMotionScaleLostQueueFull, clock);
        }
    }
}

/// Set ABN encoder count direction. `invert=1` flips the count direction
/// so the position PID has correct (negative) feedback. Depends on
/// motor/encoder wiring — not board-specific.
///
/// Recorded in channel state and applied by the TMC task on its control
/// tick. See [`TmcChannelState::config_apply_pending`] for why this does
/// not enqueue.
#[klipper_command]
pub fn tmc_set_encoder_dir(ctx: &mut dyn TmcWriteHost, oid: u8, channel: u8, invert: u8) {
    let Some((_channel, state)) = foci_tmc_state_matching_or_shutdown(ctx, oid, channel) else {
        return;
    };
    state
        .encoder_invert_cfg
        .store(Some(invert != 0), Ordering::Relaxed);
    state
        .config_apply_pending
        .fetch_or(crate::state::tmc::PENDING_ENCODER_DIR, Ordering::Release);
}

/// Set inner loop PI gains from host (explicit override or saved values).
///
/// Recorded in channel state and applied by the TMC task on its control
/// tick. See [`TmcChannelState::config_apply_pending`] for why this does
/// not enqueue.
#[klipper_command]
pub fn tmc_set_pid_gains(
    ctx: &mut dyn TmcWriteHost,
    oid: u8,
    flux_p: u16,
    flux_i: u16,
    torque_p: u16,
    torque_i: u16,
) {
    let Some((_channel, state)) = foci_tmc_state_or_shutdown(ctx, oid) else {
        return;
    };
    let (Ok(flux_p), Ok(flux_i), Ok(torque_p), Ok(torque_i)) = (
        PidPGain::new(flux_p),
        PidIGain::new(flux_i),
        PidPGain::new(torque_p),
        PidIGain::new(torque_i),
    ) else {
        shutdown_request_validation(ctx);
        return;
    };
    state.store_pending_inner_gains(
        crate::pid::InnerLoopGains {
            flux_p,
            flux_i,
            torque_p,
            torque_i,
        },
        Ordering::Relaxed,
    );
    state
        .config_apply_pending
        .fetch_or(crate::state::tmc::PENDING_INNER_GAINS, Ordering::Release);
}

/// Set velocity biquad filter cutoff frequency from host config.
///
/// Recorded in channel state and applied by the TMC task on its control tick.
/// See [`TmcChannelState::config_apply_pending`] for why this does not enqueue.
#[klipper_command]
pub fn tmc_set_velocity_filter(ctx: &mut dyn TmcWriteHost, oid: u8, filter_hz: u16) {
    let Some((_channel, state)) = foci_tmc_state_or_shutdown(ctx, oid) else {
        return;
    };
    state
        .velocity_filter_hz
        .store(Some(filter_hz), Ordering::Relaxed);
    state
        .config_apply_pending
        .fetch_or(crate::state::tmc::PENDING_FILTERS, Ordering::Release);
}

/// Set torque biquad filter cutoff frequency from host config.
///
/// Recorded in channel state and applied by the TMC task on its control
/// tick. See [`TmcChannelState::config_apply_pending`].
#[klipper_command]
pub fn tmc_set_torque_filter(ctx: &mut dyn TmcWriteHost, oid: u8, filter_hz: u16) {
    let Some((_channel, state)) = foci_tmc_state_or_shutdown(ctx, oid) else {
        return;
    };
    state
        .torque_filter_hz
        .store(Some(filter_hz), Ordering::Relaxed);
    state
        .config_apply_pending
        .fetch_or(crate::state::tmc::PENDING_FILTERS, Ordering::Release);
}

/// Set position target biquad filter cutoff frequency from host config.
///
/// Recorded in channel state and applied by the TMC task on its control
/// tick. See [`TmcChannelState::config_apply_pending`].
#[klipper_command]
pub fn tmc_set_position_filter(ctx: &mut dyn TmcWriteHost, oid: u8, filter_hz: u16) {
    let Some((_channel, state)) = foci_tmc_state_or_shutdown(ctx, oid) else {
        return;
    };
    state
        .position_filter_hz
        .store(Some(filter_hz), Ordering::Relaxed);
    state
        .config_apply_pending
        .fetch_or(crate::state::tmc::PENDING_FILTERS, Ordering::Release);
}

/// Set flux biquad filter cutoff frequency from host config.
///
/// Recorded in channel state and applied by the TMC task on its control
/// tick. See [`TmcChannelState::config_apply_pending`].
#[klipper_command]
pub fn tmc_set_flux_filter(ctx: &mut dyn TmcWriteHost, oid: u8, filter_hz: u16) {
    let Some((_channel, state)) = foci_tmc_state_or_shutdown(ctx, oid) else {
        return;
    };
    state
        .flux_filter_hz
        .store(Some(filter_hz), Ordering::Relaxed);
    state
        .config_apply_pending
        .fetch_or(crate::state::tmc::PENDING_FILTERS, Ordering::Release);
}

/// Set position/velocity PI gains from host config.
///
/// Recorded in channel state and applied by the TMC task on its control
/// tick. See [`TmcChannelState::config_apply_pending`] for why this does
/// not enqueue.
#[klipper_command]
pub fn tmc_set_position_gains(
    ctx: &mut dyn TmcWriteHost,
    oid: u8,
    position_p: u16,
    position_i: u16,
    velocity_p: u16,
    velocity_i: u16,
) {
    let Some((_channel, state)) = foci_tmc_state_or_shutdown(ctx, oid) else {
        return;
    };
    let (Ok(position_p), Ok(position_i), Ok(velocity_p), Ok(velocity_i)) = (
        PidPGain::new(position_p),
        PidIGain::new(position_i),
        PidPGain::new(velocity_p),
        PidIGain::new(velocity_i),
    ) else {
        shutdown_request_validation(ctx);
        return;
    };
    state.store_pending_outer_gains(
        crate::pid::OuterLoopGains {
            position_p,
            position_i,
            velocity_p,
            velocity_i,
        },
        Ordering::Relaxed,
    );
    state
        .config_apply_pending
        .fetch_or(crate::state::tmc::PENDING_OUTER_GAINS, Ordering::Release);
}

/// Enable or disable RPM-domain velocity feedforward from host config.
#[klipper_command]
pub fn tmc_set_velocity_feedforward_rpm(
    ctx: &mut dyn ShutdownRecovery,
    oid: u8,
    enable: u8,
    gain_permille: u16,
) {
    let Some((_channel, state)) = foci_tmc_state_or_shutdown(ctx, oid) else {
        return;
    };
    state
        .velocity_feedforward
        .store(enable != 0, Ordering::Relaxed);
    state
        .velocity_feedforward_gain_permille
        .store(gain_permille, Ordering::Relaxed);
    defmt::info!(
        "Velocity feedforward: {} gain_permille={}",
        enable != 0,
        gain_permille
    );
}

/// Enable or disable acceleration feedforward from a live diagnostic command.
#[klipper_command]
pub fn tmc_set_accel_feedforward(
    ctx: &mut dyn ShutdownRecovery,
    oid: u8,
    enable: u8,
    accel_gain_permille: u16,
    decel_gain_permille: u16,
) {
    let Some((_channel, state)) = foci_tmc_state_or_shutdown(ctx, oid) else {
        return;
    };
    state
        .accel_feedforward
        .store(enable != 0, Ordering::Relaxed);
    state
        .accel_feedforward_accel_gain_permille
        .store(accel_gain_permille, Ordering::Relaxed);
    state
        .accel_feedforward_decel_gain_permille
        .store(decel_gain_permille, Ordering::Relaxed);
    defmt::info!(
        "Acceleration feedforward: {} accel_gain_permille={} decel_gain_permille={}",
        enable != 0,
        accel_gain_permille,
        decel_gain_permille
    );
}

/// Enable or disable diagnostic position-target lead from a live command.
#[klipper_command]
pub fn tmc_set_position_lead(
    ctx: &mut dyn ShutdownRecovery,
    oid: u8,
    enable: u8,
    gain_permille: u16,
    max_counts: u16,
) {
    let Some((_channel, state)) = foci_tmc_state_or_shutdown(ctx, oid) else {
        return;
    };
    state.position_lead.store(enable != 0, Ordering::Relaxed);
    state
        .position_lead_gain_permille
        .store(gain_permille, Ordering::Relaxed);
    state
        .position_lead_max_counts
        .store(max_counts, Ordering::Relaxed);
    defmt::info!(
        "Position lead: {} gain_permille={} max_counts={}",
        enable != 0,
        gain_permille,
        max_counts
    );
}

/// Enable or disable runtime commutation phase advance from a live command.
#[klipper_command]
pub fn tmc_set_phase_advance(
    ctx: &mut dyn ShutdownRecovery,
    oid: u8,
    enable: u8,
    gain_ppm: i32,
    max_counts: u16,
    deadband: u16,
) {
    let Some((_channel, state)) = foci_tmc_state_or_shutdown(ctx, oid) else {
        return;
    };
    state.phase_advance.store(enable != 0, Ordering::Relaxed);
    state
        .phase_advance_gain_ppm
        .store(gain_ppm, Ordering::Relaxed);
    state.phase_advance_max_counts.store(
        max_counts.min(crate::tmc::PHASE_ADVANCE_MAX_SAFE_COUNTS),
        Ordering::Relaxed,
    );
    state
        .phase_advance_deadband
        .store(deadband, Ordering::Relaxed);
    defmt::info!(
        "Phase advance: {} gain_ppm={} max_counts={} deadband={}",
        enable != 0,
        gain_ppm,
        max_counts,
        deadband
    );
}

/// Set PID velocity limit from host config.
///
/// Caps position PID output to prevent integrator-driven velocity runaway.
/// Firmware default: 500,000. Override via `pid_velocity_limit` in
/// `printer.cfg`.
#[klipper_command]
pub fn tmc_set_velocity_limit(ctx: &mut dyn ShutdownRecovery, oid: u8, limit: u32) {
    let Some((_channel, state)) = foci_tmc_state_or_shutdown(ctx, oid) else {
        return;
    };
    state.velocity_limit_cfg.store(limit, Ordering::Relaxed);
    defmt::info!("Velocity limit: {}", limit);
}

/// Set PIDOUT_UQ_UD_LIMITS from a live diagnostic command.
#[klipper_command]
pub fn tmc_set_voltage_limit(ctx: &mut dyn TmcWriteHost, oid: u8, voltage_limit: u32) {
    let Some((_channel, state)) = foci_tmc_state_or_shutdown(ctx, oid) else {
        return;
    };
    if !(crate::state::tmc::MIN_OPERATIONAL_PIDOUT_UQ_UD_LIMIT
        ..=crate::state::tmc::MAX_OPERATIONAL_PIDOUT_UQ_UD_LIMIT)
        .contains(&voltage_limit)
    {
        if ctx.enter_shutdown() {
            let clock = ctx.now();
            ctx.send_shutdown(ShutdownReason::TmcRequestValidationFailed, clock);
        }
        return;
    }
    state
        .voltage_limit_cfg
        .store(voltage_limit, Ordering::Relaxed);
    state
        .config_apply_pending
        .fetch_or(crate::state::tmc::PENDING_VOLTAGE_LIMIT, Ordering::Release);
}

/// Run a bounded current-loop step diagnostic.
///
/// This command only queues a fixed diagnostic sequence. The board task
/// rejects it unless the motor is already enabled and calibrated by the
/// production path.
#[klipper_command]
pub fn tmc_current_step_test(
    ctx: &mut dyn TmcWriteHost,
    oid: u8,
    axis: u8,
    target: i16,
    duration_ms: u16,
    voltage_limit: u16,
) {
    let Some((channel, _state)) = foci_tmc_state_or_shutdown(ctx, oid) else {
        return;
    };
    let Some(axis) = crate::tmc::CurrentStepAxis::from_u8(axis) else {
        if ctx.enter_shutdown() {
            let clock = ctx.now();
            ctx.send_shutdown(ShutdownReason::TmcRequestValidationFailed, clock);
        }
        return;
    };
    if target.unsigned_abs() > MAX_CURRENT_STEP_TARGET as u16
        || !(MIN_CURRENT_STEP_DURATION_MS..=MAX_CURRENT_STEP_DURATION_MS).contains(&duration_ms)
        || !(MIN_CURRENT_STEP_VOLTAGE_LIMIT..=MAX_CURRENT_STEP_VOLTAGE_LIMIT)
            .contains(&voltage_limit)
    {
        if ctx.enter_shutdown() {
            let clock = ctx.now();
            ctx.send_shutdown(ShutdownReason::TmcRequestValidationFailed, clock);
        }
        return;
    }
    if ctx
        .enqueue_request(crate::tmc::tmc_current_step_test(
            channel,
            oid,
            axis,
            target,
            duration_ms,
            voltage_limit,
        ))
        .is_err()
        && ctx.enter_shutdown()
    {
        let clock = ctx.now();
        ctx.send_shutdown(ShutdownReason::TmcRequestValidationFailed, clock);
    }
}

/// Run a bounded current-vector step diagnostic.
///
/// This command only queues a fixed diagnostic sequence. The board task
/// rejects it unless the motor is already enabled and calibrated by the
/// production path.
#[klipper_command]
pub fn tmc_current_vector_step_test(
    ctx: &mut dyn TmcWriteHost,
    oid: u8,
    torque_target: i16,
    flux_target: i16,
    duration_ms: u16,
    voltage_limit: u16,
) {
    let Some((channel, _state)) = foci_tmc_state_or_shutdown(ctx, oid) else {
        return;
    };
    if torque_target.unsigned_abs() > MAX_CURRENT_STEP_TARGET as u16
        || flux_target.unsigned_abs() > MAX_CURRENT_STEP_TARGET as u16
        || !(MIN_CURRENT_STEP_DURATION_MS..=MAX_CURRENT_STEP_DURATION_MS).contains(&duration_ms)
        || !(MIN_CURRENT_STEP_VOLTAGE_LIMIT..=MAX_CURRENT_STEP_VOLTAGE_LIMIT)
            .contains(&voltage_limit)
    {
        if ctx.enter_shutdown() {
            let clock = ctx.now();
            ctx.send_shutdown(ShutdownReason::TmcRequestValidationFailed, clock);
        }
        return;
    }
    if ctx
        .enqueue_request(crate::tmc::tmc_current_vector_step_test(
            channel,
            oid,
            torque_target,
            flux_target,
            duration_ms,
            voltage_limit,
        ))
        .is_err()
        && ctx.enter_shutdown()
    {
        let clock = ctx.now();
        ctx.send_shutdown(ShutdownReason::TmcRequestValidationFailed, clock);
    }
}

/// Run a bounded torque-current pulse and sample it before the old dwell floor.
///
/// This command only queues a fixed diagnostic sequence. The board task
/// rejects it unless the motor is already enabled and calibrated by the
/// production path.
#[klipper_command]
pub fn tmc_current_torque_sample_test(
    ctx: &mut dyn TmcWriteHost,
    oid: u8,
    target: i16,
    flux_target: i16,
    sample_delay_ms: u16,
    voltage_limit: u16,
) {
    let Some((channel, _state)) = foci_tmc_state_or_shutdown(ctx, oid) else {
        return;
    };
    if target.unsigned_abs() > MAX_CURRENT_STEP_TARGET as u16
        || flux_target.unsigned_abs() > MAX_CURRENT_STEP_TARGET as u16
        || !(MIN_CURRENT_SAMPLE_DELAY_MS..=MAX_CURRENT_SAMPLE_DELAY_MS).contains(&sample_delay_ms)
        || !(MIN_CURRENT_STEP_VOLTAGE_LIMIT..=MAX_CURRENT_STEP_VOLTAGE_LIMIT)
            .contains(&voltage_limit)
    {
        if ctx.enter_shutdown() {
            let clock = ctx.now();
            ctx.send_shutdown(ShutdownReason::TmcRequestValidationFailed, clock);
        }
        return;
    }
    if ctx
        .enqueue_request(crate::tmc::tmc_current_torque_sample_test(
            channel,
            oid,
            target,
            flux_target,
            sample_delay_ms,
            voltage_limit,
        ))
        .is_err()
        && ctx.enter_shutdown()
    {
        let clock = ctx.now();
        ctx.send_shutdown(ShutdownReason::TmcRequestValidationFailed, clock);
    }
}

/// Run a bounded position-mode torque-offset pulse and sample it.
///
/// This command only queues a fixed diagnostic sequence. The board task rejects
/// it unless the motor is already enabled, calibrated, idle, outside a homing
/// move, and still in production position mode.
#[klipper_command]
pub fn tmc_position_torque_offset_sample_test(
    ctx: &mut dyn TmcWriteHost,
    oid: u8,
    target: i16,
    sample_delay_ms: u16,
    voltage_limit: u16,
) {
    let Some((channel, _state)) = foci_tmc_state_or_shutdown(ctx, oid) else {
        return;
    };
    if target.unsigned_abs() > MAX_CURRENT_STEP_TARGET as u16
        || !(MIN_CURRENT_SAMPLE_DELAY_MS..=MAX_CURRENT_SAMPLE_DELAY_MS).contains(&sample_delay_ms)
        || !(MIN_CURRENT_STEP_VOLTAGE_LIMIT..=MAX_CURRENT_STEP_VOLTAGE_LIMIT)
            .contains(&voltage_limit)
    {
        if ctx.enter_shutdown() {
            let clock = ctx.now();
            ctx.send_shutdown(ShutdownReason::TmcRequestValidationFailed, clock);
        }
        return;
    }
    if ctx
        .enqueue_request(crate::tmc::tmc_position_torque_offset_sample_test(
            channel,
            oid,
            target,
            sample_delay_ms,
            voltage_limit,
        ))
        .is_err()
        && ctx.enter_shutdown()
    {
        let clock = ctx.now();
        ctx.send_shutdown(ShutdownReason::TmcRequestValidationFailed, clock);
    }
}

/// Run a bounded open-loop voltage-vector pulse and sample it.
///
/// This command only queues a fixed diagnostic sequence. The board task rejects
/// it unless the motor is already enabled, calibrated, idle, outside a homing
/// move, and still in production position mode.
#[klipper_command]
pub fn tmc_voltage_step_test(
    ctx: &mut dyn TmcWriteHost,
    oid: u8,
    uq_ext: i16,
    ud_ext: i16,
    sample_delay_ms: u16,
) {
    let Some((channel, _state)) = foci_tmc_state_or_shutdown(ctx, oid) else {
        return;
    };
    if uq_ext.unsigned_abs() > MAX_VOLTAGE_STEP_EXT as u16
        || ud_ext.unsigned_abs() > MAX_VOLTAGE_STEP_EXT as u16
        || !(MIN_CURRENT_SAMPLE_DELAY_MS..=MAX_CURRENT_SAMPLE_DELAY_MS).contains(&sample_delay_ms)
    {
        if ctx.enter_shutdown() {
            let clock = ctx.now();
            ctx.send_shutdown(ShutdownReason::TmcRequestValidationFailed, clock);
        }
        return;
    }
    if ctx
        .enqueue_request(crate::tmc::tmc_voltage_step_test(
            channel,
            oid,
            uq_ext,
            ud_ext,
            sample_delay_ms,
        ))
        .is_err()
        && ctx.enter_shutdown()
    {
        let clock = ctx.now();
        ctx.send_shutdown(ShutdownReason::TmcRequestValidationFailed, clock);
    }
}

/// Run the shared resistance-identification sweep over an already-armed motor.
///
/// This command only queues a fixed diagnostic sequence. The board task
/// rejects it unless the motor is already enabled, calibrated, and ADC
/// offsets are ready; it does not touch commissioning state.
#[klipper_command]
pub fn tmc_resistance_test(ctx: &mut dyn TmcWriteHost, oid: u8, detail: u8) {
    let Some((channel, _state)) = foci_tmc_state_or_shutdown(ctx, oid) else {
        return;
    };
    if ctx
        .enqueue_request(crate::tmc::tmc_resistance_test(channel, oid, detail))
        .is_err()
        && ctx.enter_shutdown()
    {
        let clock = ctx.now();
        ctx.send_shutdown(ShutdownReason::TmcDiagnosticLostQueueFull, clock);
    }
}

/// Set whether a raw enable may trigger firmware-side auto-calibration.
#[klipper_command]
pub fn tmc_set_auto_calibrate_on_enable(ctx: &mut dyn ShutdownRecovery, oid: u8, enable: u8) {
    let Some((_channel, state)) = foci_tmc_state_or_shutdown(ctx, oid) else {
        return;
    };
    state
        .auto_calibrate_on_enable_allowed
        .store(enable != 0, Ordering::Relaxed);
    defmt::info!("Auto-calibrate on enable: {}", enable != 0);
}

/// Composite bound for handlers that both enqueue TMC requests and may
/// trigger shutdown on queue-full errors.
///
/// `&mut dyn Trait1 + Trait2` is not a valid trait-object type in stable
/// Rust for two non-auto traits; this named supertrait is the dyn-safe way
/// to compose `ShutdownRecovery` (shutdown + clock) and `TmcHost` (request
/// enqueue), the same pattern as [`crate::context::LifecycleReset`].
pub trait TmcWriteHost: ShutdownRecovery + TmcHost {}
impl<T: ?Sized + ShutdownRecovery + TmcHost> TmcWriteHost for T {}

fn foci_tmc_state_or_shutdown(
    ctx: &mut dyn ShutdownRecovery,
    oid: u8,
) -> Option<(u8, &'static TmcChannelState)> {
    match ctx.state().stepper_channel(oid) {
        Some(channel) => {
            tmc_state_for_channel_or_shutdown(ctx, channel).map(|state| (channel, state))
        }
        None => {
            shutdown_invalid_tmc_channel(ctx);
            None
        }
    }
}

fn foci_tmc_state_matching_or_shutdown(
    ctx: &mut dyn ShutdownRecovery,
    oid: u8,
    requested_channel: u8,
) -> Option<(u8, &'static TmcChannelState)> {
    let (channel, state) = foci_tmc_state_or_shutdown(ctx, oid)?;
    if channel != requested_channel {
        shutdown_invalid_tmc_channel(ctx);
        return None;
    }
    Some((channel, state))
}

fn tmc_state_for_channel_or_shutdown(
    ctx: &mut dyn ShutdownRecovery,
    channel: u8,
) -> Option<&'static TmcChannelState> {
    match crate::state::tmc::tmc_channel(channel) {
        Some(state) => Some(state),
        None => {
            shutdown_invalid_tmc_channel(ctx);
            None
        }
    }
}

fn shutdown_invalid_tmc_channel(ctx: &mut dyn ShutdownRecovery) {
    if ctx.enter_shutdown() {
        let clock = ctx.now();
        ctx.send_shutdown(ShutdownReason::InvalidTmcChannel, clock);
    }
}

fn shutdown_request_validation(ctx: &mut dyn ShutdownRecovery) {
    if ctx.enter_shutdown() {
        let clock = ctx.now();
        ctx.send_shutdown(ShutdownReason::TmcRequestValidationFailed, clock);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::context::{BoardChannels, ClockSource, QueueFull, StateAccess, TmcHost};
    use crate::shutdown::ShutdownReason;
    use core::sync::atomic::Ordering::Relaxed;
    use foci_klipper::commands::stepper::config_stepper_channel;
    use foci_klipper::state::State;
    use std::cell::RefCell;

    struct MockCtx {
        state: State,
        channel_count: u8,
        last_request: RefCell<Option<crate::tmc::TmcRequest>>,
        last_shutdown: Option<ShutdownReason>,
        motor_disarmed: bool,
        queue_empty: bool,
        timer_inactive: bool,
        accept_request: bool,
        motion_scale_rejections: std::vec::Vec<crate::motion_scale::MotionScaleRejection>,
    }

    impl MockCtx {
        fn new(channel_count: u8) -> Self {
            Self {
                state: State::new(),
                channel_count,
                last_request: RefCell::new(None),
                last_shutdown: None,
                motor_disarmed: true,
                queue_empty: true,
                timer_inactive: true,
                accept_request: true,
                motion_scale_rejections: std::vec::Vec::new(),
            }
        }
    }

    impl StateAccess for MockCtx {
        fn state(&mut self) -> &mut State {
            &mut self.state
        }
    }

    impl ClockSource for MockCtx {
        fn now(&self) -> u32 {
            123
        }
    }

    impl ShutdownRecovery for MockCtx {
        fn enter_shutdown(&mut self) -> bool {
            true
        }

        fn reset_to_bootloader(&mut self) -> ! {
            panic!("reset_to_bootloader not used in tests")
        }

        fn send_shutdown(&mut self, reason: ShutdownReason, _clock: u32) {
            self.last_shutdown = Some(reason);
        }
    }

    impl BoardChannels for MockCtx {
        fn tmc_channel_count(&self) -> u8 {
            self.channel_count
        }

        fn step_pin_channel(&self, _pin: u8) -> Option<u8> {
            None
        }

        fn dir_pin_channel(&self, _pin: u8) -> Option<u8> {
            None
        }

        fn enable_pin_channel(&self, _pin: u32) -> Option<u8> {
            None
        }
    }

    impl TmcHost for MockCtx {
        fn enqueue_request(&self, req: crate::tmc::TmcRequest) -> Result<(), QueueFull> {
            if !self.accept_request {
                return Err(QueueFull);
            }
            self.last_request.replace(Some(req));
            Ok(())
        }

        fn motion_scale_idle(
            &self,
            _channel: u8,
        ) -> Result<(), crate::motion_scale::MotionScaleRejection> {
            if !self.motor_disarmed {
                return Err(crate::motion_scale::MotionScaleRejection::MotorArmed);
            }
            if !self.queue_empty {
                return Err(crate::motion_scale::MotionScaleRejection::QueueNotEmpty);
            }
            if !self.timer_inactive {
                return Err(crate::motion_scale::MotionScaleRejection::TimerActive);
            }
            Ok(())
        }

        fn motion_quiescent(&self) -> bool {
            self.motion_scale_idle(0).is_ok()
        }

        fn report_motion_scale_rejection(
            &mut self,
            _oid: u8,
            rejection: crate::motion_scale::MotionScaleRejection,
        ) {
            self.motion_scale_rejections.push(rejection);
        }
    }

    #[test]
    fn tmc_set_current_records_config_without_queueing() {
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
        let mut ctx = MockCtx::new(2);
        let mut sender = ();
        config_stepper_channel(ctx.state(), 10, 1).unwrap();
        let tmc_state = crate::state::tmc::tmc_channel(1).unwrap();
        tmc_state.run_current_ma_cfg.store(0, Relaxed);
        tmc_state.config_apply_pending.store(0, Relaxed);

        tmc_set_current(&mut ctx, &mut sender, 10, 800);

        assert_eq!(tmc_state.run_current_ma_cfg.load(Relaxed), 800);
        assert_ne!(
            tmc_state.config_apply_pending.load(Relaxed) & crate::state::tmc::PENDING_CURRENT,
            0
        );
        assert!(ctx.last_request.borrow().is_none());
        assert_eq!(ctx.last_shutdown, None);
    }

    #[test]
    fn tmc_set_current_cannot_shut_down_on_a_full_queue() {
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
        let mut ctx = MockCtx::new(2);
        // A full request channel is exactly the connect-time condition that
        // shut the MCU down; recording config must be immune to it.
        ctx.accept_request = false;
        let mut sender = ();
        config_stepper_channel(ctx.state(), 10, 1).unwrap();
        let tmc_state = crate::state::tmc::tmc_channel(1).unwrap();
        tmc_state.run_current_ma_cfg.store(0, Relaxed);
        tmc_state.config_apply_pending.store(0, Relaxed);

        tmc_set_current(&mut ctx, &mut sender, 10, 1500);

        assert_eq!(tmc_state.run_current_ma_cfg.load(Relaxed), 1500);
        assert_ne!(
            tmc_state.config_apply_pending.load(Relaxed) & crate::state::tmc::PENDING_CURRENT,
            0
        );
        assert!(ctx.last_request.borrow().is_none());
        assert_eq!(ctx.last_shutdown, None);
    }

    #[test]
    fn tmc_set_encoder_dir_records_config_without_queueing() {
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
        let mut ctx = MockCtx::new(2);
        let mut sender = ();
        config_stepper_channel(ctx.state(), 10, 1).unwrap();
        let tmc_state = crate::state::tmc::tmc_channel(1).unwrap();
        tmc_state.encoder_invert_cfg.store(None, Relaxed);
        tmc_state.config_apply_pending.store(0, Relaxed);

        tmc_set_encoder_dir(&mut ctx, &mut sender, 10, 1, 1);

        assert_eq!(tmc_state.encoder_invert_cfg.load(Relaxed), Some(true));
        assert_ne!(
            tmc_state.config_apply_pending.load(Relaxed) & crate::state::tmc::PENDING_ENCODER_DIR,
            0
        );
        assert!(ctx.last_request.borrow().is_none());
        assert_eq!(ctx.last_shutdown, None);
    }

    #[test]
    fn tmc_set_encoder_dir_cannot_shut_down_on_a_full_queue() {
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
        let mut ctx = MockCtx::new(2);
        // A full request channel is exactly the connect-time condition that
        // shut the MCU down; recording config must be immune to it.
        ctx.accept_request = false;
        let mut sender = ();
        config_stepper_channel(ctx.state(), 10, 1).unwrap();
        let tmc_state = crate::state::tmc::tmc_channel(1).unwrap();
        tmc_state.encoder_invert_cfg.store(None, Relaxed);
        tmc_state.config_apply_pending.store(0, Relaxed);

        tmc_set_encoder_dir(&mut ctx, &mut sender, 10, 1, 0);

        assert_eq!(tmc_state.encoder_invert_cfg.load(Relaxed), Some(false));
        assert_ne!(
            tmc_state.config_apply_pending.load(Relaxed) & crate::state::tmc::PENDING_ENCODER_DIR,
            0
        );
        assert!(ctx.last_request.borrow().is_none());
        assert_eq!(ctx.last_shutdown, None);
    }

    #[test]
    fn tmc_set_pid_gains_records_config_without_queueing() {
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
        let mut ctx = MockCtx::new(2);
        let mut sender = ();
        config_stepper_channel(ctx.state(), 10, 1).unwrap();
        let tmc_state = crate::state::tmc::tmc_channel(1).unwrap();
        tmc_state.config_apply_pending.store(0, Relaxed);

        tmc_set_pid_gains(&mut ctx, &mut sender, 10, 0x7fff, 0x7fff, 0x7fff, 0x7fff);

        let gains = tmc_state.load_pending_inner_gains(Relaxed).unwrap();
        assert_eq!(gains.flux_p.raw(), 0x7fff);
        assert_eq!(gains.flux_i.raw(), 0x7fff);
        assert_eq!(gains.torque_p.raw(), 0x7fff);
        assert_eq!(gains.torque_i.raw(), 0x7fff);
        assert_ne!(
            tmc_state.config_apply_pending.load(Relaxed) & crate::state::tmc::PENDING_INNER_GAINS,
            0
        );
        assert!(ctx.last_request.borrow().is_none());
        assert_eq!(ctx.last_shutdown, None);
    }

    #[test]
    fn tmc_set_pid_gains_cannot_shut_down_on_a_full_queue() {
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
        let mut ctx = MockCtx::new(2);
        // A full request channel is exactly the connect-time condition that
        // shut the MCU down; recording config must be immune to it.
        ctx.accept_request = false;
        let mut sender = ();
        config_stepper_channel(ctx.state(), 10, 1).unwrap();
        let tmc_state = crate::state::tmc::tmc_channel(1).unwrap();
        tmc_state.config_apply_pending.store(0, Relaxed);

        tmc_set_pid_gains(&mut ctx, &mut sender, 10, 21, 22, 23, 24);

        let gains = tmc_state.load_pending_inner_gains(Relaxed).unwrap();
        assert_eq!(gains.flux_p.raw(), 21);
        assert_eq!(gains.flux_i.raw(), 22);
        assert_eq!(gains.torque_p.raw(), 23);
        assert_eq!(gains.torque_i.raw(), 24);
        assert_ne!(
            tmc_state.config_apply_pending.load(Relaxed) & crate::state::tmc::PENDING_INNER_GAINS,
            0
        );
        assert!(ctx.last_request.borrow().is_none());
        assert_eq!(ctx.last_shutdown, None);
    }

    #[test]
    fn tmc_set_pid_gains_rejects_negative_encoding() {
        let mut rejected = MockCtx::new(2);
        let mut sender = ();
        config_stepper_channel(rejected.state(), 10, 1).unwrap();

        tmc_set_pid_gains(
            &mut rejected,
            &mut sender,
            10,
            0x8000,
            0x7fff,
            0x7fff,
            0x7fff,
        );

        assert!(rejected.last_request.borrow().is_none());
        assert_eq!(
            rejected.last_shutdown,
            Some(ShutdownReason::TmcRequestValidationFailed)
        );
    }

    #[test]
    fn tmc_set_position_gains_records_pending_without_publishing() {
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
        let mut ctx = MockCtx::new(2);
        let mut sender = ();
        config_stepper_channel(ctx.state(), 10, 1).unwrap();
        let tmc_state = crate::state::tmc::tmc_channel(1).unwrap();
        let confirmed = crate::pid::DEFAULT_OUTER_GAINS;
        tmc_state.store_outer_gains(confirmed, Relaxed);
        tmc_state.config_apply_pending.store(0, Relaxed);

        tmc_set_position_gains(&mut ctx, &mut sender, 10, 256, 3, 768, 4);

        let pending = tmc_state.load_pending_outer_gains(Relaxed).unwrap();
        assert_eq!(pending.position_p.raw(), 256);
        assert_eq!(pending.position_i.raw(), 3);
        assert_eq!(pending.velocity_p.raw(), 768);
        assert_eq!(pending.velocity_i.raw(), 4);
        assert_ne!(
            tmc_state.config_apply_pending.load(Relaxed) & crate::state::tmc::PENDING_OUTER_GAINS,
            0
        );
        assert_eq!(
            tmc_state.load_outer_gains(Relaxed),
            Ok(confirmed),
            "handler must never publish unconfirmed gains"
        );
        assert!(ctx.last_request.borrow().is_none());
        assert_eq!(ctx.last_shutdown, None);
    }

    #[test]
    fn tmc_set_position_gains_queue_full_does_not_shut_down_and_preserves_confirmed_gains() {
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
        let mut ctx = MockCtx::new(2);
        // A full request channel is exactly the connect-time condition that
        // used to shut the MCU down; recording config must be immune to it.
        ctx.accept_request = false;
        let mut sender = ();
        config_stepper_channel(ctx.state(), 10, 1).unwrap();
        let tmc_state = crate::state::tmc::tmc_channel(1).unwrap();
        let confirmed = crate::pid::test_outer_gains(641, 1, 1153, 2);
        tmc_state.store_outer_gains(confirmed, Relaxed);
        tmc_state.config_apply_pending.store(0, Relaxed);

        tmc_set_position_gains(&mut ctx, &mut sender, 10, 256, 3, 768, 4);

        let pending = tmc_state.load_pending_outer_gains(Relaxed).unwrap();
        assert_eq!(pending.position_p.raw(), 256);
        assert_eq!(pending.position_i.raw(), 3);
        assert_eq!(pending.velocity_p.raw(), 768);
        assert_eq!(pending.velocity_i.raw(), 4);
        assert_ne!(
            tmc_state.config_apply_pending.load(Relaxed) & crate::state::tmc::PENDING_OUTER_GAINS,
            0
        );
        assert_eq!(tmc_state.load_outer_gains(Relaxed), Ok(confirmed));
        assert!(ctx.last_request.borrow().is_none());
        assert_eq!(ctx.last_shutdown, None);
    }

    #[test]
    fn tmc_set_position_gains_accepts_maximum_and_rejects_negative_encoding() {
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
        let tmc_state = crate::state::tmc::tmc_channel(1).unwrap();
        let confirmed = crate::pid::DEFAULT_OUTER_GAINS;
        tmc_state.store_outer_gains(confirmed, Relaxed);

        let mut accepted = MockCtx::new(2);
        let mut sender = ();
        config_stepper_channel(accepted.state(), 10, 1).unwrap();
        tmc_set_position_gains(
            &mut accepted,
            &mut sender,
            10,
            0x7fff,
            0x7fff,
            0x7fff,
            0x7fff,
        );
        let pending = tmc_state.load_pending_outer_gains(Relaxed).unwrap();
        assert_eq!(pending.position_p.raw(), 0x7fff);
        assert_eq!(pending.position_i.raw(), 0x7fff);
        assert_eq!(pending.velocity_p.raw(), 0x7fff);
        assert_eq!(pending.velocity_i.raw(), 0x7fff);
        assert!(accepted.last_request.borrow().is_none());
        assert_eq!(accepted.last_shutdown, None);

        let mut rejected = MockCtx::new(2);
        config_stepper_channel(rejected.state(), 10, 1).unwrap();
        tmc_set_position_gains(
            &mut rejected,
            &mut sender,
            10,
            0x7fff,
            0x7fff,
            0x8000,
            0x7fff,
        );

        assert!(rejected.last_request.borrow().is_none());
        assert_eq!(
            rejected.last_shutdown,
            Some(ShutdownReason::TmcRequestValidationFailed)
        );
        assert_eq!(tmc_state.load_outer_gains(Relaxed), Ok(confirmed));
    }

    #[test]
    fn tmc_set_velocity_filter_records_config_without_queueing() {
        use core::sync::atomic::Ordering::Relaxed;

        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
        let mut ctx = MockCtx::new(2);
        let mut sender = ();
        config_stepper_channel(ctx.state(), 10, 1).unwrap();
        let tmc_state = crate::state::tmc::tmc_channel(1).unwrap();
        tmc_state.velocity_filter_hz.store(None, Relaxed);
        tmc_state.config_apply_pending.store(0, Relaxed);

        tmc_set_velocity_filter(&mut ctx, &mut sender, 10, 200);

        // The cutoff is recorded in channel state...
        assert_eq!(tmc_state.velocity_filter_hz.load(Relaxed), Some(200));
        // ...and the filter bit is flagged for the TMC task to apply on its
        // control tick...
        assert_ne!(
            tmc_state.config_apply_pending.load(Relaxed) & crate::state::tmc::PENDING_FILTERS,
            0
        );
        // ...without consuming a slot on the bounded request channel.
        assert!(ctx.last_request.borrow().is_none());
        assert_eq!(ctx.last_shutdown, None);
    }

    #[test]
    fn tmc_set_voltage_limit_records_config_without_queueing() {
        use core::sync::atomic::Ordering::Relaxed;

        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
        let mut ctx = MockCtx::new(2);
        // A full channel is the connect-time condition that shut the MCU down.
        ctx.accept_request = false;
        let mut sender = ();
        config_stepper_channel(ctx.state(), 10, 1).unwrap();
        let tmc_state = crate::state::tmc::tmc_channel(1).unwrap();
        tmc_state.voltage_limit_cfg.store(16_000, Relaxed);
        tmc_state.config_apply_pending.store(0, Relaxed);

        tmc_set_voltage_limit(&mut ctx, &mut sender, 10, 29_000);

        assert_eq!(tmc_state.voltage_limit_cfg.load(Relaxed), 29_000);
        assert_ne!(
            tmc_state.config_apply_pending.load(Relaxed) & crate::state::tmc::PENDING_VOLTAGE_LIMIT,
            0
        );
        assert!(ctx.last_request.borrow().is_none());
        assert_eq!(ctx.last_shutdown, None);
    }

    #[test]
    fn tmc_set_velocity_filter_cannot_shut_down_on_a_full_queue() {
        use core::sync::atomic::Ordering::Relaxed;

        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
        let mut ctx = MockCtx::new(2);
        // A full request channel is exactly the connect-time condition that
        // shut the MCU down; recording config must be immune to it.
        ctx.accept_request = false;
        let mut sender = ();
        config_stepper_channel(ctx.state(), 10, 1).unwrap();
        let tmc_state = crate::state::tmc::tmc_channel(1).unwrap();
        tmc_state.velocity_filter_hz.store(None, Relaxed);

        tmc_set_velocity_filter(&mut ctx, &mut sender, 10, 200);

        assert_eq!(tmc_state.velocity_filter_hz.load(Relaxed), Some(200));
        assert_eq!(ctx.last_shutdown, None);
    }

    #[test]
    fn tmc_set_voltage_limit_updates_saved_state_before_worker_runs() {
        use core::sync::atomic::Ordering::Relaxed;

        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
        let mut ctx = MockCtx::new(2);
        let mut sender = ();
        config_stepper_channel(ctx.state(), 10, 1).unwrap();
        let tmc_state = crate::state::tmc::tmc_channel(1).unwrap();
        tmc_state.voltage_limit_cfg.store(16_000, Relaxed);

        tmc_set_voltage_limit(&mut ctx, &mut sender, 10, 20_000);

        // Recorded in channel state for the TMC task to apply on its control
        // tick; config setters no longer consume a request-channel slot.
        assert_eq!(tmc_state.voltage_limit_cfg.load(Relaxed), 20_000);
        assert_ne!(
            tmc_state.config_apply_pending.load(Relaxed) & crate::state::tmc::PENDING_VOLTAGE_LIMIT,
            0
        );
        assert!(ctx.last_request.borrow().is_none());
        assert_eq!(ctx.last_shutdown, None);
    }

    #[test]
    fn tmc_set_voltage_limit_accepts_raw_chip_minimum() {
        use core::sync::atomic::Ordering::Relaxed;

        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
        let mut ctx = MockCtx::new(2);
        let mut sender = ();
        config_stepper_channel(ctx.state(), 10, 1).unwrap();
        let tmc_state = crate::state::tmc::tmc_channel(1).unwrap();
        tmc_state.voltage_limit_cfg.store(16_000, Relaxed);

        tmc_set_voltage_limit(&mut ctx, &mut sender, 10, 0);

        // Raw chip minimum is accepted, not rejected as out of range.
        assert_eq!(tmc_state.voltage_limit_cfg.load(Relaxed), 0);
        assert_ne!(
            tmc_state.config_apply_pending.load(Relaxed) & crate::state::tmc::PENDING_VOLTAGE_LIMIT,
            0
        );
        assert_eq!(ctx.last_shutdown, None);
    }

    #[test]
    fn tmc_current_step_test_routes_by_stepper_oid() {
        let mut ctx = MockCtx::new(2);
        let mut sender = ();
        config_stepper_channel(ctx.state(), 10, 1).unwrap();

        tmc_current_step_test(&mut ctx, &mut sender, 10, 0, 250, 80, 12_000);

        let req = ctx.last_request.borrow().clone().unwrap();
        assert_eq!(req.channel, 1);
        assert!(matches!(
            req.kind,
            crate::tmc::TmcRequestKind::CurrentStepTest {
                oid: 10,
                axis: crate::tmc::CurrentStepAxis::Torque,
                target: 250,
                duration_ms: 80,
                voltage_limit: 12_000,
            }
        ));
        assert_eq!(ctx.last_shutdown, None);
    }

    #[test]
    fn tmc_current_vector_step_test_routes_by_stepper_oid() {
        let mut ctx = MockCtx::new(2);
        let mut sender = ();
        config_stepper_channel(ctx.state(), 10, 1).unwrap();

        tmc_current_vector_step_test(&mut ctx, &mut sender, 10, 0, 250, 80, 12_000);

        let req = ctx.last_request.borrow().clone().unwrap();
        assert_eq!(req.channel, 1);
        assert!(matches!(
            req.kind,
            crate::tmc::TmcRequestKind::CurrentVectorStepTest {
                oid: 10,
                torque_target: 0,
                flux_target: 250,
                duration_ms: 80,
                voltage_limit: 12_000,
            }
        ));
        assert_eq!(ctx.last_shutdown, None);
    }

    #[test]
    fn tmc_current_torque_sample_test_routes_by_stepper_oid() {
        let mut ctx = MockCtx::new(2);
        let mut sender = ();
        config_stepper_channel(ctx.state(), 10, 1).unwrap();

        tmc_current_torque_sample_test(&mut ctx, &mut sender, 10, 500, -125, 100, 29_000);

        let req = ctx.last_request.borrow().clone().unwrap();
        assert_eq!(req.channel, 1);
        assert!(matches!(
            req.kind,
            crate::tmc::TmcRequestKind::CurrentTorqueSampleTest {
                oid: 10,
                target: 500,
                flux_target: -125,
                sample_delay_ms: 100,
                voltage_limit: 29_000,
            }
        ));
        assert_eq!(ctx.last_shutdown, None);
    }

    #[test]
    fn tmc_current_torque_sample_test_rejects_zero_delay() {
        let mut ctx = MockCtx::new(2);
        let mut sender = ();
        config_stepper_channel(ctx.state(), 10, 1).unwrap();

        tmc_current_torque_sample_test(&mut ctx, &mut sender, 10, 500, 0, 0, 29_000);

        assert!(ctx.last_request.borrow().is_none());
        assert_eq!(
            ctx.last_shutdown,
            Some(crate::shutdown::ShutdownReason::TmcRequestValidationFailed)
        );
    }

    #[test]
    fn tmc_position_torque_offset_sample_test_routes_by_stepper_oid() {
        let mut ctx = MockCtx::new(2);
        let mut sender = ();
        config_stepper_channel(ctx.state(), 10, 1).unwrap();

        tmc_position_torque_offset_sample_test(&mut ctx, &mut sender, 10, 500, 2, 29_000);

        assert_eq!(
            ctx.last_request.borrow().as_ref().unwrap().kind,
            crate::tmc::TmcRequestKind::PositionTorqueOffsetSampleTest {
                oid: 10,
                target: 500,
                sample_delay_ms: 2,
                voltage_limit: 29_000,
            }
        );
    }

    #[test]
    fn tmc_voltage_step_test_routes_by_stepper_oid() {
        let mut ctx = MockCtx::new(2);
        let mut sender = ();
        config_stepper_channel(ctx.state(), 10, 1).unwrap();

        tmc_voltage_step_test(&mut ctx, &mut sender, 10, 512, -256, 2);

        assert_eq!(
            ctx.last_request.borrow().as_ref().unwrap().kind,
            crate::tmc::TmcRequestKind::VoltageStepTest {
                oid: 10,
                uq_ext: 512,
                ud_ext: -256,
                sample_delay_ms: 2,
            }
        );
    }

    #[test]
    fn tmc_voltage_step_test_rejects_large_voltage() {
        let mut ctx = MockCtx::new(2);
        let mut sender = ();
        config_stepper_channel(ctx.state(), 10, 1).unwrap();

        tmc_voltage_step_test(&mut ctx, &mut sender, 10, 1025, 0, 2);

        assert!(ctx.last_request.borrow().is_none());
        assert_eq!(
            ctx.last_shutdown,
            Some(crate::shutdown::ShutdownReason::TmcRequestValidationFailed)
        );
    }

    #[test]
    fn tmc_resistance_test_routes_by_stepper_oid() {
        let mut ctx = MockCtx::new(2);
        let mut sender = ();
        config_stepper_channel(ctx.state(), 10, 1).unwrap();

        tmc_resistance_test(&mut ctx, &mut sender, 10, 0);

        assert_eq!(
            ctx.last_request.borrow().as_ref().unwrap().kind,
            crate::tmc::TmcRequestKind::ResistanceTest { oid: 10, detail: 0 }
        );
    }

    #[test]
    fn tmc_resistance_test_rejects_unknown_oid() {
        let mut ctx = MockCtx::new(2);
        let mut sender = ();

        tmc_resistance_test(&mut ctx, &mut sender, 99, 0);

        assert!(ctx.last_request.borrow().is_none());
        assert_eq!(
            ctx.last_shutdown,
            Some(crate::shutdown::ShutdownReason::InvalidTmcChannel)
        );
    }

    #[test]
    fn tmc_current_vector_step_test_rejects_excessive_flux_target() {
        let mut ctx = MockCtx::new(2);
        let mut sender = ();
        config_stepper_channel(ctx.state(), 10, 1).unwrap();

        tmc_current_vector_step_test(&mut ctx, &mut sender, 10, 0, 1_001, 80, 12_000);

        assert!(ctx.last_request.borrow().is_none());
        assert_eq!(
            ctx.last_shutdown,
            Some(crate::shutdown::ShutdownReason::TmcRequestValidationFailed)
        );
    }

    #[test]
    fn tmc_current_step_test_rejects_excessive_target() {
        let mut ctx = MockCtx::new(2);
        let mut sender = ();
        config_stepper_channel(ctx.state(), 10, 1).unwrap();

        tmc_current_step_test(&mut ctx, &mut sender, 10, 0, 1_001, 80, 12_000);

        assert!(ctx.last_request.borrow().is_none());
        assert_eq!(
            ctx.last_shutdown,
            Some(crate::shutdown::ShutdownReason::TmcRequestValidationFailed)
        );
    }

    #[test]
    fn tmc_current_step_test_rejects_too_long_duration() {
        let mut ctx = MockCtx::new(2);
        let mut sender = ();
        config_stepper_channel(ctx.state(), 10, 1).unwrap();

        tmc_current_step_test(&mut ctx, &mut sender, 10, 0, 250, 201, 12_000);

        assert!(ctx.last_request.borrow().is_none());
        assert_eq!(
            ctx.last_shutdown,
            Some(crate::shutdown::ShutdownReason::TmcRequestValidationFailed)
        );
    }

    #[test]
    fn tmc_set_motion_scale_enqueues_complete_scale_without_mutating_state() {
        let mut ctx = MockCtx::new(2);
        let mut sender = ();
        config_stepper_channel(ctx.state(), 10, 1).unwrap();
        let tmc_state = crate::state::tmc::tmc_channel(1).unwrap();
        let before_configured = tmc_state.motion_scale_configured();

        tmc_set_motion_scale(&mut ctx, &mut sender, 10, 1, 1000, 3200);

        assert_eq!(tmc_state.motion_scale_configured(), before_configured);
        let req = ctx.last_request.borrow().clone().unwrap();
        assert_eq!(req.channel, 1);
        assert!(matches!(
            req.kind,
            crate::tmc::TmcRequestKind::SetMotionScale {
                oid: 10,
                encoder_ppr: 1000,
                planner_steps_per_rev: 3200,
            }
        ));
        assert_eq!(ctx.last_shutdown, None);
    }

    #[test]
    fn tmc_set_motion_scale_rejects_invalid_or_busy_requests_without_mutation() {
        use foci_klipper::pulse_mapper::MAX_PLANNER_STEPS_PER_REV;

        let mut ctx = MockCtx::new(2);
        let mut sender = ();
        config_stepper_channel(ctx.state(), 10, 1).unwrap();
        let tmc_state = crate::state::tmc::tmc_channel(1).unwrap();
        let before_configured = tmc_state.motion_scale_configured();

        for (encoder_ppr, planner_steps_per_rev, expected) in [
            (
                0,
                3200,
                crate::motion_scale::MotionScaleRejection::EncoderPprZero,
            ),
            (
                u32::MAX / 4 + 1,
                3200,
                crate::motion_scale::MotionScaleRejection::EncoderPprTooLarge,
            ),
            (
                1000,
                0,
                crate::motion_scale::MotionScaleRejection::PlannerStepsPerRevZero,
            ),
            (
                1000,
                MAX_PLANNER_STEPS_PER_REV + 1,
                crate::motion_scale::MotionScaleRejection::PlannerStepsPerRevTooLarge,
            ),
        ] {
            tmc_set_motion_scale(
                &mut ctx,
                &mut sender,
                10,
                1,
                encoder_ppr,
                planner_steps_per_rev,
            );
            assert!(ctx.last_request.borrow().is_none());
            assert_eq!(tmc_state.motion_scale_configured(), before_configured);
            assert_eq!(ctx.motion_scale_rejections.last(), Some(&expected));
        }

        for (motor_disarmed, queue_empty, timer_inactive, expected) in [
            (
                false,
                true,
                true,
                crate::motion_scale::MotionScaleRejection::MotorArmed,
            ),
            (
                true,
                false,
                true,
                crate::motion_scale::MotionScaleRejection::QueueNotEmpty,
            ),
            (
                true,
                true,
                false,
                crate::motion_scale::MotionScaleRejection::TimerActive,
            ),
        ] {
            ctx.motor_disarmed = motor_disarmed;
            ctx.queue_empty = queue_empty;
            ctx.timer_inactive = timer_inactive;
            tmc_set_motion_scale(&mut ctx, &mut sender, 10, 1, 1000, 3200);
            assert!(ctx.last_request.borrow().is_none());
            assert_eq!(ctx.motion_scale_rejections.last(), Some(&expected));
        }

        ctx.motor_disarmed = true;
        ctx.queue_empty = true;
        ctx.timer_inactive = true;
        ctx.accept_request = false;
        tmc_set_motion_scale(&mut ctx, &mut sender, 10, 1, 1000, 3200);
        assert_eq!(tmc_state.motion_scale_configured(), before_configured);
        assert_eq!(
            ctx.motion_scale_rejections.last(),
            Some(&crate::motion_scale::MotionScaleRejection::RequestQueueFull)
        );
    }

    #[test]
    fn tmc_set_motion_scale_queue_full_reports_error_without_enqueue_or_mutation() {
        let mut ctx = MockCtx::new(2);
        let mut sender = ();
        config_stepper_channel(ctx.state(), 10, 1).unwrap();
        let tmc_state = crate::state::tmc::tmc_channel(1).unwrap();
        let stepper_state = crate::state::stepper::stepper_channel(1).unwrap();
        let before_configured = tmc_state.motion_scale_configured();
        let before_encoder_ppr = tmc_state.encoder_ppr();
        let before_encoder_counts_per_rev = tmc_state.encoder_counts_per_rev();
        let before_planner_steps_per_rev = stepper_state.planner_steps_per_rev(tmc_state);
        let before_tmc_grid = tmc_state.tmc_grid();
        let before_physical_step_width = tmc_state.physical_step_width();
        ctx.accept_request = false;

        tmc_set_motion_scale(&mut ctx, &mut sender, 10, 1, 1000, 3200);

        assert!(ctx.last_request.borrow().is_none());
        assert_eq!(
            ctx.last_shutdown,
            Some(ShutdownReason::TmcSetMotionScaleLostQueueFull)
        );
        assert_eq!(tmc_state.motion_scale_configured(), before_configured);
        assert_eq!(tmc_state.encoder_ppr(), before_encoder_ppr);
        assert_eq!(
            tmc_state.encoder_counts_per_rev(),
            before_encoder_counts_per_rev
        );
        assert_eq!(
            stepper_state.planner_steps_per_rev(tmc_state),
            before_planner_steps_per_rev
        );
        assert_eq!(tmc_state.tmc_grid(), before_tmc_grid);
        assert_eq!(tmc_state.physical_step_width(), before_physical_step_width);
    }

    #[test]
    fn tmc_set_motion_scale_rejects_channel_mismatch_for_stepper_oid() {
        let mut ctx = MockCtx::new(2);
        let mut sender = ();
        config_stepper_channel(ctx.state(), 10, 1).unwrap();

        tmc_set_motion_scale(&mut ctx, &mut sender, 10, 0, 1000, 3200);

        assert!(ctx.last_request.borrow().is_none());
        assert_eq!(ctx.last_shutdown, Some(ShutdownReason::InvalidTmcChannel));
    }

    #[test]
    fn tmc_set_velocity_feedforward_rpm_stores_gain_permille() {
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
        let mut ctx = MockCtx::new(2);
        let mut sender = ();
        config_stepper_channel(ctx.state(), 10, 1).unwrap();
        let tmc_state = crate::state::tmc::tmc_channel(1).unwrap();

        tmc_set_velocity_feedforward_rpm(&mut ctx, &mut sender, 10, 1, 1_450);

        assert!(tmc_state.velocity_feedforward.load(Relaxed));
        assert_eq!(
            tmc_state.velocity_feedforward_gain_permille.load(Relaxed),
            1_450
        );

        tmc_set_velocity_feedforward_rpm(&mut ctx, &mut sender, 10, 0, 1_000);

        assert!(!tmc_state.velocity_feedforward.load(Relaxed));
        assert_eq!(
            tmc_state.velocity_feedforward_gain_permille.load(Relaxed),
            1_000
        );
    }

    #[test]
    fn provider_registers_the_rpm_velocity_feedforward_command() {
        let source = include_str!("../provider.rs");

        assert_eq!(
            source
                .matches("crate::ankyra::handlers::tmc::tmc_set_velocity_feedforward_rpm,")
                .count(),
            2
        );
        assert!(!source.contains("crate::ankyra::handlers::tmc::tmc_set_velocity_feedforward,"));
    }

    #[test]
    fn tmc_set_velocity_filter_leaves_voltage_limit_bit_clear() {
        use core::sync::atomic::Ordering::Relaxed;

        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
        let mut ctx = MockCtx::new(2);
        let mut sender = ();
        config_stepper_channel(ctx.state(), 10, 1).unwrap();
        let tmc_state = crate::state::tmc::tmc_channel(1).unwrap();
        tmc_state.velocity_filter_hz.store(None, Relaxed);
        tmc_state.config_apply_pending.store(0, Relaxed);

        tmc_set_velocity_filter(&mut ctx, &mut sender, 10, 200);

        let pending = tmc_state.config_apply_pending.load(Relaxed);
        assert_ne!(
            pending & crate::state::tmc::PENDING_FILTERS,
            0,
            "filter bit should be set after calling filter setter"
        );
        assert_eq!(
            pending & crate::state::tmc::PENDING_VOLTAGE_LIMIT,
            0,
            "voltage limit bit should remain clear after calling filter setter"
        );
    }

    /// Regression guard for the whole connect-time config allowlist: exactly
    /// the nine handlers below must be immune to `QueueFull` (they never
    /// touch the TMC request queue at all), and `tmc_set_motion_scale` must
    /// remain the one exception that still enqueues and shuts down on
    /// `QueueFull`. Individual handlers each have their own focused test
    /// from the task that converted them; this test exists so a future
    /// handler that forgets to convert (or a regression that reintroduces
    /// an enqueue) fails here even if every per-handler test still passes.
    #[test]
    fn connect_time_config_handlers_are_immune_to_queue_full() {
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
        assert_eq!(crate::state::tmc::CONNECT_TIME_QUEUE_SLOTS_PER_CHANNEL, 1);

        let mut ctx = MockCtx::new(2);
        ctx.accept_request = false;
        let mut sender = ();
        config_stepper_channel(ctx.state(), 10, 1).unwrap();

        tmc_set_voltage_limit(&mut ctx, &mut sender, 10, 20_000);
        tmc_set_velocity_filter(&mut ctx, &mut sender, 10, 200);
        tmc_set_torque_filter(&mut ctx, &mut sender, 10, 200);
        tmc_set_position_filter(&mut ctx, &mut sender, 10, 200);
        tmc_set_flux_filter(&mut ctx, &mut sender, 10, 200);
        tmc_set_current(&mut ctx, &mut sender, 10, 800);
        tmc_set_encoder_dir(&mut ctx, &mut sender, 10, 1, 1);
        tmc_set_pid_gains(&mut ctx, &mut sender, 10, 21, 22, 23, 24);
        tmc_set_position_gains(&mut ctx, &mut sender, 10, 256, 3, 768, 4);

        assert!(
            ctx.last_request.borrow().is_none(),
            "connect-time config handlers must never enqueue, even under QueueFull"
        );
        assert_eq!(
            ctx.last_shutdown, None,
            "connect-time config handlers must never shut down on QueueFull"
        );

        tmc_set_motion_scale(&mut ctx, &mut sender, 10, 1, 1000, 3200);

        assert!(ctx.last_request.borrow().is_none());
        assert_eq!(
            ctx.last_shutdown,
            Some(ShutdownReason::TmcSetMotionScaleLostQueueFull),
            "tmc_set_motion_scale is the one connect-time handler that still \
             enqueues and must still shut down on QueueFull"
        );
    }

    /// Every connect-time handler must raise its pending bit with `Release`.
    ///
    /// Raising the bit is what publishes the value stored just above it, and
    /// the drain claims the bit with `Acquire` before reading that value.
    /// `Acquire` only orders against a `Release` on the writing side, so a
    /// relaxed raise leaves the pair unsynchronised: the store and the raise
    /// may be reordered against each other, and a drain that claims the bit
    /// in between applies the previous value while clearing the bit that
    /// pointed at the new one -- the lost update this mechanism exists to
    /// prevent.
    ///
    /// Guarded at the source because the property is invisible to a host
    /// test: x86 will not reorder these stores, so a handler added with
    /// relaxed ordering passes every behavioural test and only misbehaves on
    /// the Cortex-M targets this firmware ships to.
    #[test]
    fn connect_time_config_handlers_raise_their_pending_bit_with_release_ordering() {
        let source = include_str!("tmc.rs");
        let handlers = source
            .split("#[cfg(test)]")
            .next()
            .expect("the module has a non-test half");

        let raises: std::vec::Vec<&str> = handlers
            .split(';')
            .filter(|statement| statement.contains("config_apply_pending"))
            .filter(|statement| statement.contains("fetch_or("))
            .collect();

        assert_eq!(
            raises.len(),
            9,
            "expected one pending-bit raise per connect-time config handler"
        );
        for statement in raises {
            assert!(
                statement.contains("Ordering::Release"),
                "a pending bit raised with anything weaker than Release does \
                 not publish the value it points at: {}",
                statement.trim()
            );
        }
    }
}
