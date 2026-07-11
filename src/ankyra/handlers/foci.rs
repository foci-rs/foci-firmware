//! FOCI-specific command handlers: `foci_selftest`, `foci_calibrate`,
//! `foci_dump_registers`, `foci_commission`.
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

const ADC_VM_OFFSET_STATUS_OK: u8 = 0;
const ADC_VM_OFFSET_STATUS_NOT_READY: u8 = 1;

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
/// Must be called after `tmc_set_motion_scale` populates the channel step width.
/// Rejected if the step width is unset, or if another commissioning
/// operation is already in flight.
#[klipper_command]
pub fn foci_calibrate(ctx: &mut dyn FociRuntimeHost, oid: u8) {
    let Some((channel, state)) = foci_tmc_state_or_shutdown(ctx, oid) else {
        return;
    };
    // Pre-flight: the physical pulse width must have been populated by a prior
    // tmc_set_motion_scale command. If it hasn't, fail before touching hardware
    // — otherwise calibration would succeed but the motor would silently
    // refuse to respond to step pulses. The engine's ClosedLoopEntry
    // phase also checks the named configured physical width.
    if state.physical_step_width().is_none() {
        defmt::error!(
            "foci_calibrate rejected: STEP_WIDTH not configured (call tmc_set_motion_scale first)"
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

/// Return the cached ADC_VM mode-5 startup offset for voltage decoding.
///
/// This query is intentionally passive: it does not touch TMC registers or
/// enqueue a measurement. Board startup captures the offset before any motor
/// enable path; the host reads that cached evidence during connect.
#[klipper_command]
pub fn foci_adc_vm_offset(ctx: &mut dyn FociRuntimeHost, oid: u8) {
    let Some((_channel, state)) = foci_tmc_state_or_shutdown(ctx, oid) else {
        return;
    };
    let Some(offset) = state.adc_vm_offset_snapshot() else {
        ::ankyra::klipper_reply!(
            crate::ankyra::replies::FociAdcVmOffsetResult,
            oid: u8 = oid,
            offset_raw: u16 = 0,
            sample_count: u8 = 0,
            status: u8 = ADC_VM_OFFSET_STATUS_NOT_READY
        );
        return;
    };
    ::ankyra::klipper_reply!(
        crate::ankyra::replies::FociAdcVmOffsetResult,
        oid: u8 = oid,
        offset_raw: u16 = offset.offset_raw,
        sample_count: u8 = offset.sample_count,
        status: u8 = ADC_VM_OFFSET_STATUS_OK
    );
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

        fn motion_scale_idle(&self, _channel: u8) -> bool {
            true
        }
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    struct AdcVmOffsetReply {
        oid: u8,
        offset_raw: u16,
        sample_count: u8,
        status: u8,
    }

    struct MockSender {
        adc_vm_offsets: std::vec::Vec<AdcVmOffsetReply>,
    }

    impl MockSender {
        fn new() -> Self {
            Self {
                adc_vm_offsets: std::vec::Vec::new(),
            }
        }
    }

    impl ::ankyra::SendReply<crate::ankyra::replies::FociAdcVmOffsetResult> for MockSender {
        fn send(&mut self, payload: crate::ankyra::replies::FociAdcVmOffsetResult) {
            self.adc_vm_offsets.push(AdcVmOffsetReply {
                oid: payload.oid,
                offset_raw: payload.offset_raw,
                sample_count: payload.sample_count,
                status: payload.status,
            });
        }
    }

    #[test]
    fn foci_adc_vm_offset_returns_cached_startup_value() {
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
        let mut ctx = MockCtx::new(2);
        let mut sender = MockSender::new();
        config_stepper_channel(ctx.state(), 10, 1).unwrap();
        let tmc_state = crate::state::tmc::tmc_channel(1).unwrap();
        tmc_state.store_adc_vm_offset(33_662, 8);

        foci_adc_vm_offset(&mut ctx, &mut sender, 10);

        assert_eq!(
            sender.adc_vm_offsets,
            [AdcVmOffsetReply {
                oid: 10,
                offset_raw: 33_662,
                sample_count: 8,
                status: 0,
            }]
        );
        assert!(ctx.last_request.borrow().is_none());
        assert_eq!(ctx.last_shutdown, None);
    }

    #[test]
    fn foci_adc_vm_offset_reports_not_ready_without_cached_value() {
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
        let mut ctx = MockCtx::new(2);
        let mut sender = MockSender::new();
        config_stepper_channel(ctx.state(), 10, 1).unwrap();
        let tmc_state = crate::state::tmc::tmc_channel(1).unwrap();
        tmc_state.store_adc_vm_offset(0, 0);

        foci_adc_vm_offset(&mut ctx, &mut sender, 10);

        assert_eq!(
            sender.adc_vm_offsets,
            [AdcVmOffsetReply {
                oid: 10,
                offset_raw: 0,
                sample_count: 0,
                status: 1,
            }]
        );
        assert!(ctx.last_request.borrow().is_none());
        assert_eq!(ctx.last_shutdown, None);
    }
}
