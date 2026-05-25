//! TMC4671 command handlers: `tmc_init`, `tmc_read_register`,
//! `tmc_write_register`, `tmc_set_current`, `tmc_set_encoder`,
//! `tmc_set_encoder_dir`, `tmc_set_flux_filter`, `tmc_set_pid_gains`,
//! `tmc_set_position_filter`, `tmc_set_position_gains`,
//! `tmc_set_torque_filter`, `tmc_set_velocity_feedforward`,
//! `tmc_set_velocity_filter`, `tmc_set_velocity_limit`,
//! `tmc_set_voltage_limit`, `tmc_current_torque_sample_test`,
//! `tmc_position_torque_offset_sample_test`, `tmc_voltage_step_test`,
//! `tmc_set_auto_calibrate_on_enable`, `tmc_test_spin`.
//!
//! Lifted from `boards/openffboard-fw/src/ankyra/handlers/tmc.rs` in
//! Task 16 of the foci-firmware handler-lift plan. Handler bodies bind to
//! the capability traits declared in [`crate::context`]; the board crate
//! wires up the trait impls on its own `FirmwareCtx`.
//!
//! See the handler audit at
//! `docs/notes/foci-firmware-lift-handler-audit.md` for the exact
//! capability bound used by each handler.

use ankyra::prelude::klipper_command;
use core::sync::atomic::Ordering;

use crate::context::{ShutdownRecovery, TmcHost};
use crate::shutdown::ShutdownReason;
use crate::state::tmc::TmcChannelState;

const MAX_CURRENT_STEP_TARGET: i16 = 1_000;
const MIN_CURRENT_STEP_DURATION_MS: u16 = 20;
const MAX_CURRENT_STEP_DURATION_MS: u16 = 200;
const MIN_CURRENT_SAMPLE_DELAY_MS: u16 = 1;
const MAX_CURRENT_SAMPLE_DELAY_MS: u16 = 20;
const MAX_VOLTAGE_STEP_EXT: i16 = 1_024;
const MIN_CURRENT_STEP_VOLTAGE_LIMIT: u16 =
    crate::state::tmc::MIN_OPERATIONAL_PIDOUT_UQ_UD_LIMIT as u16;
const MAX_CURRENT_STEP_VOLTAGE_LIMIT: u16 = 29_000;

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
#[klipper_command]
pub fn tmc_set_current(ctx: &mut dyn TmcWriteHost, oid: u8, run_ma: u32) {
    let Some((channel, _state)) = foci_tmc_state_or_shutdown(ctx, oid) else {
        return;
    };
    if ctx
        .enqueue_request(crate::tmc::tmc_set_current(channel, run_ma))
        .is_err()
        && ctx.enter_shutdown()
    {
        let clock = ctx.now();
        ctx.send_shutdown(ShutdownReason::TmcSetCurrentLostQueueFull, clock);
    }
}

/// Configure ABN encoder resolution and compute step width.
///
/// The closed-loop position PID currently operates in `phi_m_abn`
/// mechanical-angle space (`POSITION_SELECTION=9`), so STEP_WIDTH is one
/// mechanical revolution divided by the quadrature encoder count:
/// `step_width = 65536 / quadrature_ppr`.
#[klipper_command]
pub fn tmc_set_encoder(ctx: &mut dyn TmcWriteHost, oid: u8, channel: u8, ppr: u32) {
    let Some((channel, state)) = foci_tmc_state_matching_or_shutdown(ctx, oid, channel) else {
        return;
    };
    let (req, step_width) = crate::tmc::tmc_set_encoder_hw(channel, ppr, 1);
    state.step_width_value.store(step_width, Ordering::Relaxed);
    defmt::info!("channel={} ppr={} step_width={}", channel, ppr, step_width);
    if ctx.enqueue_request(req).is_err() && ctx.enter_shutdown() {
        let clock = ctx.now();
        ctx.send_shutdown(ShutdownReason::TmcSetEncoderLostQueueFull, clock);
    }
}

/// Set ABN encoder count direction. `invert=1` flips the count direction
/// so the position PID has correct (negative) feedback. Depends on
/// motor/encoder wiring — not board-specific.
#[klipper_command]
pub fn tmc_set_encoder_dir(ctx: &mut dyn TmcWriteHost, oid: u8, channel: u8, invert: u8) {
    let Some((channel, _state)) = foci_tmc_state_matching_or_shutdown(ctx, oid, channel) else {
        return;
    };
    if ctx
        .enqueue_request(crate::tmc::tmc_set_encoder_dir(channel, invert != 0))
        .is_err()
        && ctx.enter_shutdown()
    {
        let clock = ctx.now();
        ctx.send_shutdown(ShutdownReason::TmcSetEncoderDirLostQueueFull, clock);
    }
}

/// Set inner loop PI gains from host (explicit override or saved values).
#[klipper_command]
pub fn tmc_set_pid_gains(
    ctx: &mut dyn TmcWriteHost,
    oid: u8,
    flux_p: u16,
    flux_i: u16,
    torque_p: u16,
    torque_i: u16,
) {
    let Some((channel, _state)) = foci_tmc_state_or_shutdown(ctx, oid) else {
        return;
    };
    if ctx
        .enqueue_request(crate::tmc::tmc_set_pid_gains(
            channel, flux_p, flux_i, torque_p, torque_i,
        ))
        .is_err()
        && ctx.enter_shutdown()
    {
        let clock = ctx.now();
        ctx.send_shutdown(ShutdownReason::TmcSetPidGainsLostQueueFull, clock);
    }
}

/// Set velocity biquad filter cutoff frequency from host config.
#[klipper_command]
pub fn tmc_set_velocity_filter(ctx: &mut dyn ShutdownRecovery, oid: u8, filter_hz: u16) {
    let Some((_channel, state)) = foci_tmc_state_or_shutdown(ctx, oid) else {
        return;
    };
    state.velocity_filter_hz.store(filter_hz, Ordering::Relaxed);
    defmt::info!("Velocity filter: {} Hz", filter_hz);
}

/// Set torque biquad filter cutoff frequency from host config.
#[klipper_command]
pub fn tmc_set_torque_filter(ctx: &mut dyn ShutdownRecovery, oid: u8, filter_hz: u16) {
    let Some((_channel, state)) = foci_tmc_state_or_shutdown(ctx, oid) else {
        return;
    };
    state.torque_filter_hz.store(filter_hz, Ordering::Relaxed);
    defmt::info!("Torque filter: {} Hz", filter_hz);
}

/// Set position target biquad filter cutoff frequency from host config.
#[klipper_command]
pub fn tmc_set_position_filter(ctx: &mut dyn ShutdownRecovery, oid: u8, filter_hz: u16) {
    let Some((_channel, state)) = foci_tmc_state_or_shutdown(ctx, oid) else {
        return;
    };
    state.position_filter_hz.store(filter_hz, Ordering::Relaxed);
    defmt::info!("Position filter: {} Hz", filter_hz);
}

/// Set flux biquad filter cutoff frequency from host config.
#[klipper_command]
pub fn tmc_set_flux_filter(ctx: &mut dyn ShutdownRecovery, oid: u8, filter_hz: u16) {
    let Some((_channel, state)) = foci_tmc_state_or_shutdown(ctx, oid) else {
        return;
    };
    state.flux_filter_hz.store(filter_hz, Ordering::Relaxed);
    defmt::info!("Flux filter: {} Hz", filter_hz);
}

/// Set position/velocity PI gains from host config.
#[klipper_command]
pub fn tmc_set_position_gains(
    ctx: &mut dyn TmcWriteHost,
    oid: u8,
    position_p: u16,
    position_i: u16,
    velocity_p: u16,
    velocity_i: u16,
) {
    let Some((channel, state)) = foci_tmc_state_or_shutdown(ctx, oid) else {
        return;
    };
    state.position_p_value.store(position_p, Ordering::Relaxed);
    state.position_i_value.store(position_i, Ordering::Relaxed);
    state.velocity_p_value.store(velocity_p, Ordering::Relaxed);
    state.velocity_i_value.store(velocity_i, Ordering::Relaxed);
    if ctx
        .enqueue_request(crate::tmc::tmc_set_position_gains(
            channel, position_p, position_i, velocity_p, velocity_i,
        ))
        .is_err()
        && ctx.enter_shutdown()
    {
        let clock = ctx.now();
        ctx.send_shutdown(ShutdownReason::TmcSetPositionGainsLostQueueFull, clock);
    }
}

/// Enable or disable velocity feedforward from host config.
#[klipper_command]
pub fn tmc_set_velocity_feedforward(
    ctx: &mut dyn ShutdownRecovery,
    oid: u8,
    enable: u8,
    multiplier: u16,
) {
    let Some((_channel, state)) = foci_tmc_state_or_shutdown(ctx, oid) else {
        return;
    };
    state
        .velocity_feedforward
        .store(enable != 0, Ordering::Relaxed);
    state
        .velocity_feedforward_multiplier
        .store(multiplier, Ordering::Relaxed);
    defmt::info!(
        "Velocity feedforward: {} multiplier={}",
        enable != 0,
        multiplier
    );
}

/// Enable or disable velocity-surface transient feedforward from a live command.
#[klipper_command]
pub fn tmc_set_velocity_transient_feedforward(
    ctx: &mut dyn ShutdownRecovery,
    oid: u8,
    enable: u8,
    lead_time_us: u16,
    gain_permille: u16,
    max_offset: u16,
    rate_hz: u16,
) {
    let Some((_channel, state)) = foci_tmc_state_or_shutdown(ctx, oid) else {
        return;
    };
    let rate_hz = crate::tmc::normalize_velocity_transient_rate_hz(rate_hz);
    state
        .velocity_transient_feedforward
        .store(enable != 0, Ordering::Relaxed);
    state
        .velocity_transient_lead_time_us
        .store(lead_time_us, Ordering::Relaxed);
    state
        .velocity_transient_gain_permille
        .store(gain_permille, Ordering::Relaxed);
    state
        .velocity_transient_max_offset
        .store(max_offset, Ordering::Relaxed);
    state
        .velocity_transient_rate_hz
        .store(rate_hz, Ordering::Relaxed);
    defmt::info!(
        "Velocity transient feedforward: {} lead_time_us={} gain_permille={} max_offset={} rate_hz={}",
        enable != 0,
        lead_time_us,
        gain_permille,
        max_offset,
        rate_hz
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
    let Some((channel, state)) = foci_tmc_state_or_shutdown(ctx, oid) else {
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
    if ctx
        .enqueue_request(crate::tmc::tmc_set_voltage_limit(channel, voltage_limit))
        .is_err()
        && ctx.enter_shutdown()
    {
        let clock = ctx.now();
        ctx.send_shutdown(ShutdownReason::TmcSetVoltageLimitLostQueueFull, clock);
    }
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
    target: i16,
    duration_ms: u16,
    voltage_limit: u16,
) {
    let Some((channel, _state)) = foci_tmc_state_or_shutdown(ctx, oid) else {
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
/// enqueue). Mirrors the `LifecycleReset` pattern introduced in Task 12.
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::context::{BoardChannels, ClockSource, QueueFull, StateAccess, TmcHost};
    use crate::shutdown::ShutdownReason;
    use foci_klipper::commands::stepper::config_stepper_channel;
    use foci_klipper::state::State;
    use std::cell::RefCell;

    struct MockCtx {
        state: State,
        channel_count: u8,
        last_request: RefCell<Option<crate::tmc::TmcRequest>>,
        last_shutdown: Option<ShutdownReason>,
    }

    impl MockCtx {
        fn new(channel_count: u8) -> Self {
            Self {
                state: State::new(),
                channel_count,
                last_request: RefCell::new(None),
                last_shutdown: None,
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
            self.last_request.replace(Some(req));
            Ok(())
        }
    }

    #[test]
    fn tmc_set_current_routes_by_stepper_oid() {
        let mut ctx = MockCtx::new(2);
        let mut sender = ();
        config_stepper_channel(ctx.state(), 10, 1).unwrap();

        tmc_set_current(&mut ctx, &mut sender, 10, 800);

        let req = ctx.last_request.borrow().clone().unwrap();
        assert_eq!(req.channel, 1);
        assert!(matches!(
            req.kind,
            crate::tmc::TmcRequestKind::SetCurrent { run_ma: 800 }
        ));
        assert_eq!(ctx.last_shutdown, None);
    }

    #[test]
    fn tmc_set_position_gains_updates_saved_state_before_worker_runs() {
        use core::sync::atomic::Ordering::Relaxed;

        let mut ctx = MockCtx::new(2);
        let mut sender = ();
        config_stepper_channel(ctx.state(), 10, 1).unwrap();
        let tmc_state = crate::state::tmc::tmc_channel(1).unwrap();
        tmc_state.position_p_value.store(640, Relaxed);
        tmc_state.position_i_value.store(0, Relaxed);
        tmc_state.velocity_p_value.store(1152, Relaxed);
        tmc_state.velocity_i_value.store(0, Relaxed);

        tmc_set_position_gains(&mut ctx, &mut sender, 10, 256, 3, 768, 4);

        assert_eq!(tmc_state.position_p_value.load(Relaxed), 256);
        assert_eq!(tmc_state.position_i_value.load(Relaxed), 3);
        assert_eq!(tmc_state.velocity_p_value.load(Relaxed), 768);
        assert_eq!(tmc_state.velocity_i_value.load(Relaxed), 4);
        let req = ctx.last_request.borrow().clone().unwrap();
        assert_eq!(req.channel, 1);
        assert!(matches!(
            req.kind,
            crate::tmc::TmcRequestKind::SetPositionGains {
                position_p: 256,
                position_i: 3,
                velocity_p: 768,
                velocity_i: 4,
            }
        ));
        assert_eq!(ctx.last_shutdown, None);
    }

    #[test]
    fn tmc_set_voltage_limit_updates_saved_state_before_worker_runs() {
        use core::sync::atomic::Ordering::Relaxed;

        let mut ctx = MockCtx::new(2);
        let mut sender = ();
        config_stepper_channel(ctx.state(), 10, 1).unwrap();
        let tmc_state = crate::state::tmc::tmc_channel(1).unwrap();
        tmc_state.voltage_limit_cfg.store(16_000, Relaxed);

        tmc_set_voltage_limit(&mut ctx, &mut sender, 10, 20_000);

        assert_eq!(tmc_state.voltage_limit_cfg.load(Relaxed), 20_000);
        let req = ctx.last_request.borrow().clone().unwrap();
        assert_eq!(req.channel, 1);
        assert!(matches!(
            req.kind,
            crate::tmc::TmcRequestKind::SetVoltageLimit {
                voltage_limit: 20_000,
            }
        ));
        assert_eq!(ctx.last_shutdown, None);
    }

    #[test]
    fn tmc_current_step_test_routes_by_stepper_oid() {
        let mut ctx = MockCtx::new(2);
        let mut sender = ();
        config_stepper_channel(ctx.state(), 10, 1).unwrap();

        tmc_current_step_test(&mut ctx, &mut sender, 10, 250, 80, 12_000);

        let req = ctx.last_request.borrow().clone().unwrap();
        assert_eq!(req.channel, 1);
        assert!(matches!(
            req.kind,
            crate::tmc::TmcRequestKind::CurrentStepTest {
                oid: 10,
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

        tmc_current_torque_sample_test(&mut ctx, &mut sender, 10, 500, -125, 5, 29_000);

        let req = ctx.last_request.borrow().clone().unwrap();
        assert_eq!(req.channel, 1);
        assert!(matches!(
            req.kind,
            crate::tmc::TmcRequestKind::CurrentTorqueSampleTest {
                oid: 10,
                target: 500,
                flux_target: -125,
                sample_delay_ms: 5,
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

        tmc_current_step_test(&mut ctx, &mut sender, 10, 1_001, 80, 12_000);

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

        tmc_current_step_test(&mut ctx, &mut sender, 10, 250, 201, 12_000);

        assert!(ctx.last_request.borrow().is_none());
        assert_eq!(
            ctx.last_shutdown,
            Some(crate::shutdown::ShutdownReason::TmcRequestValidationFailed)
        );
    }

    #[test]
    fn tmc_set_encoder_stores_mechanical_step_width() {
        use core::sync::atomic::Ordering::Relaxed;

        let mut ctx = MockCtx::new(2);
        let mut sender = ();
        config_stepper_channel(ctx.state(), 10, 1).unwrap();
        let tmc_state = crate::state::tmc::tmc_channel(1).unwrap();
        tmc_state.step_width_value.store(819, Relaxed);

        tmc_set_encoder(&mut ctx, &mut sender, 10, 1, 1000);

        assert_eq!(tmc_state.step_width_value.load(Relaxed), 16);
        let req = ctx.last_request.borrow().clone().unwrap();
        assert_eq!(req.channel, 1);
        assert!(matches!(
            req.kind,
            crate::tmc::TmcRequestKind::SetEncoder { ppr: 1000 }
        ));
        assert_eq!(ctx.last_shutdown, None);
    }

    #[test]
    fn tmc_set_encoder_rejects_channel_mismatch_for_stepper_oid() {
        let mut ctx = MockCtx::new(2);
        let mut sender = ();
        config_stepper_channel(ctx.state(), 10, 1).unwrap();

        tmc_set_encoder(&mut ctx, &mut sender, 10, 0, 1000);

        assert!(ctx.last_request.borrow().is_none());
        assert_eq!(ctx.last_shutdown, Some(ShutdownReason::InvalidTmcChannel));
    }
}
