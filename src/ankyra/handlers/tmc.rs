//! TMC4671 command handlers: `tmc_init`, `tmc_read_register`,
//! `tmc_write_register`, `tmc_set_current`, `tmc_set_encoder`,
//! `tmc_set_encoder_dir`, `tmc_set_flux_filter`, `tmc_set_pid_gains`,
//! `tmc_set_position_filter`, `tmc_set_position_gains`,
//! `tmc_set_torque_filter`, `tmc_set_velocity_feedforward`,
//! `tmc_set_velocity_filter`, `tmc_set_velocity_limit`,
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

use crate::context::{BoardChannels, ShutdownRecovery, TmcHost};
use crate::shutdown::ShutdownReason;
use crate::state::tmc::TmcChannelState;

/// Bind a FOCI TMC driver OID to a configured stepper and board channel.
#[klipper_command]
pub fn config_foci_tmc(ctx: &mut dyn FociConfigHost, oid: u8, stepper_oid: u8, channel: u8) {
    let channel_count = ctx.tmc_channel_count();
    if let Err(e) = foci_klipper::commands::foci::config_foci_tmc(
        ctx.state(),
        oid,
        stepper_oid,
        channel,
        channel_count,
    ) {
        defmt::error!("config_foci_tmc: {}", e);
        if ctx.enter_shutdown() {
            let clock = ctx.now();
            ctx.send_shutdown(ShutdownReason::ConfigFociTmcFailed, clock);
        }
    }
}

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
/// Uses `tmc_set_encoder_hw` to compute the step width from PPR and
/// pole pairs, then stores it in the channel runtime state for
/// the TMC4671 STEP_WIDTH register. The TMC4671 position PID operates
/// on phi_e, not encoder counts:
///   `step_width = 65536 * pole_pairs / quadrature_ppr`
#[klipper_command]
pub fn tmc_set_encoder(ctx: &mut dyn TmcWriteHost, oid: u8, channel: u8, ppr: u32) {
    let Some((channel, state)) = foci_tmc_state_matching_or_shutdown(ctx, oid, channel) else {
        return;
    };
    let (req, step_width) = crate::tmc::tmc_set_encoder_hw(channel, ppr, 50);
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
    let Some((channel, _state)) = foci_tmc_state_or_shutdown(ctx, oid) else {
        return;
    };
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
pub fn tmc_set_velocity_feedforward(ctx: &mut dyn ShutdownRecovery, oid: u8, enable: u8) {
    let Some((_channel, state)) = foci_tmc_state_or_shutdown(ctx, oid) else {
        return;
    };
    state
        .velocity_feedforward
        .store(enable != 0, Ordering::Relaxed);
    defmt::info!("Velocity feedforward: {}", enable != 0);
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

/// Composite bound for `config_foci_tmc`: needs shutdown recovery and the
/// board's TMC channel count.
pub trait FociConfigHost: ShutdownRecovery + BoardChannels {}
impl<T: ?Sized + ShutdownRecovery + BoardChannels> FociConfigHost for T {}

fn foci_tmc_state_or_shutdown(
    ctx: &mut dyn ShutdownRecovery,
    oid: u8,
) -> Option<(u8, &'static TmcChannelState)> {
    match ctx.state().foci_channel_for_oid(oid) {
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
    use foci_klipper::commands::foci::config_stepper_channel;
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
    fn config_foci_tmc_registers_matching_channel() {
        let mut ctx = MockCtx::new(2);
        let mut sender = ();
        config_stepper_channel(ctx.state(), 10, 1).unwrap();

        config_foci_tmc(&mut ctx, &mut sender, 20, 10, 1);

        assert_eq!(ctx.state.foci_channel_for_oid(20), Some(1));
        assert_eq!(ctx.last_shutdown, None);
    }

    #[test]
    fn config_foci_tmc_rejects_channel_outside_board_range() {
        let mut ctx = MockCtx::new(1);
        let mut sender = ();
        config_stepper_channel(ctx.state(), 10, 1).unwrap();

        config_foci_tmc(&mut ctx, &mut sender, 20, 10, 1);

        assert_eq!(ctx.state.foci_channel_for_oid(20), None);
        assert_eq!(ctx.last_shutdown, Some(ShutdownReason::ConfigFociTmcFailed));
    }

    #[test]
    fn tmc_set_current_routes_by_foci_oid() {
        let mut ctx = MockCtx::new(2);
        let mut sender = ();
        config_stepper_channel(ctx.state(), 10, 1).unwrap();
        foci_klipper::commands::foci::config_foci_tmc(ctx.state(), 20, 10, 1, 2).unwrap();

        tmc_set_current(&mut ctx, &mut sender, 20, 800);

        let req = ctx.last_request.borrow().clone().unwrap();
        assert_eq!(req.channel, 1);
        assert!(matches!(
            req.kind,
            crate::tmc::TmcRequestKind::SetCurrent { run_ma: 800 }
        ));
        assert_eq!(ctx.last_shutdown, None);
    }

    #[test]
    fn tmc_set_encoder_rejects_channel_mismatch_for_foci_oid() {
        let mut ctx = MockCtx::new(2);
        let mut sender = ();
        config_stepper_channel(ctx.state(), 10, 1).unwrap();
        foci_klipper::commands::foci::config_foci_tmc(ctx.state(), 20, 10, 1, 2).unwrap();

        tmc_set_encoder(&mut ctx, &mut sender, 20, 0, 1000);

        assert!(ctx.last_request.borrow().is_none());
        assert_eq!(ctx.last_shutdown, Some(ShutdownReason::InvalidTmcChannel));
    }
}
