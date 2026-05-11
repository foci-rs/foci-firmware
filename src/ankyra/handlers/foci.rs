//! FOCI-specific command handlers: `foci_selftest`, `foci_calibrate`,
//! `foci_dump_registers`, `foci_trace_info`, `foci_trace_fetch`,
//! `foci_commission`.
//!
//! Lifted out of `boards/openffboard-fw/src/ankyra/handlers/foci.rs` in
//! Task 17 of the foci-firmware handler-lift plan. Handler bodies now
//! bind to the [`TmcHost`] capability trait declared in
//! [`crate::context`]; the board crate wires up the trait impl on its
//! own `FirmwareCtx`. The seventh FOCI handler (`foci_tune`) stays
//! board-local per the handler audit at
//! `docs/notes/foci-firmware-lift-handler-audit.md` (line 461) because
//! its implementation calls `foci_tune_inner`, which emits replies via
//! the board-local ankyra transport sender.
//!
//! Failure replies for commissioning/calibration are emitted
//! synchronously so the host always sees a terminal reply even when the
//! request cannot be enqueued; progress/success replies arrive later
//! from the `tmc_control` task.

use ankyra::prelude::klipper_command;
use core::sync::atomic::Ordering::Relaxed;

use crate::context::{ShutdownRecovery, TmcHost};
use crate::shutdown::ShutdownReason;
use crate::state::tmc::TmcChannelState;

/// Trigger TMC4671 self-test diagnostic via the commissioning engine.
///
/// Only one commissioning operation may be in flight at a time.
/// Concurrent requests are rejected with `CommissionError::AlreadyRunning`.
#[klipper_command]
pub fn foci_selftest(ctx: &mut dyn FociRuntimeHost, oid: u8) {
    let Some((channel, state)) = foci_tmc_state_or_shutdown(ctx, oid) else {
        return;
    };
    if state.commission_in_flight.swap(true, Relaxed) {
        defmt::warn!("foci_selftest rejected: commission in flight");
        ::ankyra::klipper_reply!(
            crate::ankyra::replies::FociSelftestDone,
            oid: u8 = oid,
            status: u8 = crate::autotune::CommissionError::AlreadyRunning.status_code()
        );
        return;
    }
    if ctx
        .enqueue_request(crate::tmc::tmc_selftest(channel, oid))
        .is_err()
    {
        state.commission_in_flight.store(false, Relaxed);
        ::ankyra::klipper_reply!(
            crate::ankyra::replies::FociSelftestDone,
            oid: u8 = oid,
            status: u8 = crate::autotune::CommissionError::QueueFull.status_code()
        );
    }
}

/// Calibrate the TMC4671: ADC offset calibration, encoder alignment,
/// and closed-loop entry via the commissioning engine.
///
/// Must be called after `tmc_set_encoder` populates the channel step width.
/// Rejected if the step width is unset, or if another commissioning
/// operation is already in flight.
#[klipper_command]
pub fn foci_calibrate(ctx: &mut dyn FociRuntimeHost, oid: u8) {
    let Some((channel, state)) = foci_tmc_state_or_shutdown(ctx, oid) else {
        return;
    };
    // Pre-flight: channel step width must have been populated by a prior
    // tmc_set_encoder command. If it hasn't, fail before touching hardware
    // — otherwise calibration would succeed but the motor would silently
    // refuse to respond to step pulses. The engine's ClosedLoopEntry
    // phase also checks this via backend.step_width().
    if state.step_width_value.load(Relaxed) <= 0 {
        defmt::error!(
            "foci_calibrate rejected: STEP_WIDTH not configured (host must call tmc_set_encoder first)"
        );
        ::ankyra::klipper_reply!(
            crate::ankyra::replies::FociCalibrateResult,
            oid: u8 = oid,
            status: u8 = crate::calibration::CalibrationError::ConfigFault.status_code(),
            adc_i0: u16 = 0,
            adc_i1: u16 = 0,
            encoder_count: u32 = 0
        );
        return;
    }
    if state.commission_in_flight.swap(true, Relaxed) {
        defmt::warn!("foci_calibrate rejected: commission in flight");
        ::ankyra::klipper_reply!(
            crate::ankyra::replies::FociCalibrateResult,
            oid: u8 = oid,
            status: u8 = crate::calibration::CalibrationError::InternalError.status_code(),
            adc_i0: u16 = 0,
            adc_i1: u16 = 0,
            encoder_count: u32 = 0
        );
        return;
    }
    if ctx
        .enqueue_request(crate::tmc::tmc_calibrate(channel, oid))
        .is_err()
    {
        state.commission_in_flight.store(false, Relaxed);
        ::ankyra::klipper_reply!(
            crate::ankyra::replies::FociCalibrateResult,
            oid: u8 = oid,
            status: u8 = crate::calibration::CalibrationError::InternalError.status_code(),
            adc_i0: u16 = 0,
            adc_i1: u16 = 0,
            encoder_count: u32 = 0
        );
    }
}

/// Dump all TMC4671 registers via klipper_output protocol.
///
/// The firmware reads registers internally and streams results as
/// `foci_dump_value` replies terminated by `foci_dump_done`. Replaces
/// Python-side register reads. If the queue is full the request is
/// dropped silently; the Python wait loop timeout handles the missing
/// reply naturally.
#[klipper_command]
pub fn foci_dump_registers(ctx: &mut dyn FociRuntimeHost, oid: u8) {
    let Some((channel, _state)) = foci_tmc_state_or_shutdown(ctx, oid) else {
        return;
    };
    if ctx
        .enqueue_request(crate::tmc::tmc_dump_registers(channel, oid))
        .is_err()
    {
        defmt::warn!("foci_dump_registers dropped");
    }
}

/// Query trace buffer metadata.
///
/// On queue-full the request is dropped silently; the Python wait loop
/// timeout handles the missing reply naturally.
#[klipper_command]
pub fn foci_trace_info(ctx: &mut dyn FociRuntimeHost, oid: u8) {
    let Some((channel, _state)) = foci_tmc_state_or_shutdown(ctx, oid) else {
        return;
    };
    if ctx
        .enqueue_request(crate::tmc::tmc_trace_info(channel, oid))
        .is_err()
    {
        defmt::warn!("trace_info: TMC channel full");
    }
}

/// Fetch a trace sample by offset. `generation` must match current generation.
///
/// On queue-full the request is dropped silently; the Python wait loop
/// timeout handles the missing reply naturally.
#[klipper_command]
pub fn foci_trace_fetch(ctx: &mut dyn FociRuntimeHost, oid: u8, offset: u16, generation: u8) {
    let Some((channel, _state)) = foci_tmc_state_or_shutdown(ctx, oid) else {
        return;
    };
    if ctx
        .enqueue_request(crate::tmc::tmc_trace_fetch(
            channel, oid, offset, generation,
        ))
        .is_err()
    {
        defmt::warn!("trace_fetch: TMC channel full");
    }
}

/// Start Stage 1 commissioning (diagnostics + current tune + closed-loop entry).
///
/// Only one commissioning operation may be in flight at a time.
/// Concurrent requests are rejected with `CommissionError::AlreadyRunning`.
#[klipper_command]
pub fn foci_commission(ctx: &mut dyn FociRuntimeHost, oid: u8, profile: u8) {
    let Some((channel, state)) = foci_tmc_state_or_shutdown(ctx, oid) else {
        return;
    };
    if state.commission_in_flight.swap(true, Relaxed) {
        ::ankyra::klipper_reply!(
            crate::ankyra::replies::FociCommissionPhase,
            oid: u8 = oid,
            phase: u8 = 0,
            status: u8 = crate::autotune::CommissionError::AlreadyRunning.status_code()
        );
        return;
    }
    if ctx
        .enqueue_request(crate::tmc::tmc_commission(channel, oid, profile))
        .is_err()
    {
        state.commission_in_flight.store(false, Relaxed);
        ::ankyra::klipper_reply!(
            crate::ankyra::replies::FociCommissionPhase,
            oid: u8 = oid,
            phase: u8 = 0,
            status: u8 = crate::autotune::CommissionError::QueueFull.status_code()
        );
    }
}

/// Composite bound for runtime FOCI handlers: they enqueue TMC requests
/// and enter shutdown when an OID has no configured stepper channel.
pub trait FociRuntimeHost: ShutdownRecovery + TmcHost {}
impl<T: ?Sized + ShutdownRecovery + TmcHost> FociRuntimeHost for T {}

fn foci_tmc_state_or_shutdown(
    ctx: &mut dyn ShutdownRecovery,
    oid: u8,
) -> Option<(u8, &'static TmcChannelState)> {
    match ctx.state().stepper_channel(oid) {
        Some(channel) => match crate::state::tmc::tmc_channel(channel) {
            Some(state) => Some((channel, state)),
            None => {
                shutdown_invalid_tmc_channel(ctx);
                None
            }
        },
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
