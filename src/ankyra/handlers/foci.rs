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

const STACK_WATERMARK_STATUS_OK: u8 = 0;
const STACK_WATERMARK_STATUS_NOT_PAINTED: u8 = 1;
const STACK_WATERMARK_STATUS_LOWER_BOUND: u8 = 2;
const STACK_WATERMARK_STATUS_MOTION_ACTIVE: u8 = 3;

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
    // Surface any panic/HardFault or interrupted breakaway campaign recovered
    // from no-init RAM on the previous boot, once, riding the dump the host
    // already issues on connect. A completed breadcrumb remains useful context
    // for a fault but is quiet on its own.
    let fault = crate::diagnostic::take_last_panic();
    let breadcrumb = crate::diagnostic::take_last_boot_breakaway_breadcrumb();
    if fault.is_some() || breadcrumb.is_some_and(|record| record.interrupted) {
        let (fault_kind, line, pc, file_hash) = fault.map_or((0, 0, 0, 0), |record| {
            (record.kind, record.line, record.pc, record.file_hash)
        });
        let (breakaway_checkpoint, breakaway_interrupted) = breadcrumb.map_or((0, 0), |record| {
            (record.checkpoint as u8, u8::from(record.interrupted))
        });
        ::ankyra::klipper_reply!(
            crate::ankyra::replies::FociLastBootDiagnostic,
            oid: u8 = oid,
            fault_kind: u8 = fault_kind,
            line: u32 = line,
            pc: u32 = pc,
            file_hash: u32 = file_hash,
            breakaway_checkpoint: u8 = breakaway_checkpoint,
            breakaway_interrupted: u8 = breakaway_interrupted
        );
    }
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

/// Report the MSP stack high-water mark from the boot-time sentinel paint.
///
/// Host-triggered by design: the reply describes the deepest point the stack
/// has reached since boot, so it must be issued after the workload under test
/// rather than sampled automatically. The scan masks interrupts, so this is not
/// something to poll and not something to issue during motion.
///
/// The scan masks interrupts, which would delay the step timer and the position
/// tick, so it is refused with `status = 3` unless every channel is disarmed,
/// its step queue empty, and its step timer idle. That quiescence check is
/// handed to the scan rather than made here, so it runs inside the same
/// critical section that masks the walk; deciding it out here would leave a
/// window for an interrupt to start motion between the answer and the mask.
///
/// `status` is otherwise 0 when the scan reached the first overwritten word, 1
/// on a board that never painted, and 2 when the scan window ran out with every
/// word still holding the sentinel, which makes `stack_unused_bytes` a lower
/// bound on headroom rather than a measurement of it. See
/// [`crate::stack_watermark`].
#[klipper_command]
pub fn foci_stack_watermark(ctx: &mut dyn TmcHost, oid: u8) {
    use crate::stack_watermark::StackScan;

    let (unused_bytes, painted_bytes, status) =
        match crate::stack_watermark::scan_admitted_by(|| ctx.motion_quiescent()) {
            StackScan::Refused => (0, 0, STACK_WATERMARK_STATUS_MOTION_ACTIVE),
            StackScan::NotPainted => (0, 0, STACK_WATERMARK_STATUS_NOT_PAINTED),
            StackScan::Measured(usage) => {
                let status = if usage.watermark_found {
                    STACK_WATERMARK_STATUS_OK
                } else {
                    STACK_WATERMARK_STATUS_LOWER_BOUND
                };
                defmt::info!(
                    "stack_unused_bytes={=u32} painted_bytes={=u32} status={=u8}",
                    usage.unused_bytes,
                    usage.painted_bytes,
                    status
                );
                (usage.unused_bytes, usage.painted_bytes, status)
            }
        };
    ::ankyra::klipper_reply!(
        crate::ankyra::replies::FociStackWatermarkResult,
        oid: u8 = oid,
        stack_unused_bytes: u32 = unused_bytes,
        painted_bytes: u32 = painted_bytes,
        status: u8 = status
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
        motion_quiescent: bool,
    }

    impl MockCtx {
        fn new(channel_count: u8) -> Self {
            Self {
                state: State::new(),
                channel_count,
                last_request: RefCell::new(None),
                last_shutdown: None,
                motion_quiescent: true,
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

        fn motion_scale_idle(
            &self,
            _channel: u8,
        ) -> Result<(), crate::motion_scale::MotionScaleRejection> {
            Ok(())
        }

        fn motion_quiescent(&self) -> bool {
            self.motion_quiescent
        }

        fn report_motion_scale_rejection(
            &mut self,
            _oid: u8,
            _rejection: crate::motion_scale::MotionScaleRejection,
        ) {
        }
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    struct AdcVmOffsetReply {
        oid: u8,
        offset_raw: u16,
        sample_count: u8,
        status: u8,
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    struct StackWatermarkReply {
        oid: u8,
        stack_unused_bytes: u32,
        painted_bytes: u32,
        status: u8,
    }

    struct MockSender {
        adc_vm_offsets: std::vec::Vec<AdcVmOffsetReply>,
        last_boot_diagnostics: std::vec::Vec<crate::ankyra::replies::FociLastBootDiagnostic>,
        stack_watermark_payloads: std::vec::Vec<crate::ankyra::replies::FociStackWatermarkResult>,
    }

    impl MockSender {
        fn new() -> Self {
            Self {
                adc_vm_offsets: std::vec::Vec::new(),
                last_boot_diagnostics: std::vec::Vec::new(),
                stack_watermark_payloads: std::vec::Vec::new(),
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

    impl ::ankyra::SendReply<crate::ankyra::replies::FociLastBootDiagnostic> for MockSender {
        fn send(&mut self, payload: crate::ankyra::replies::FociLastBootDiagnostic) {
            self.last_boot_diagnostics.push(payload);
        }
    }

    impl ::ankyra::SendReply<crate::ankyra::replies::FociStackWatermarkResult> for MockSender {
        fn send(&mut self, payload: crate::ankyra::replies::FociStackWatermarkResult) {
            self.stack_watermark_payloads.push(payload);
        }
    }

    fn prepare_dump_test() -> (MockCtx, MockSender) {
        while crate::diagnostic::take_last_panic().is_some() {}
        crate::diagnostic::set_breakaway_breadcrumb_raw_for_test(0);
        let mut ctx = MockCtx::new(2);
        config_stepper_channel(ctx.state(), 10, 1).unwrap();
        (ctx, MockSender::new())
    }

    fn run_dump_test(
        fault: Option<crate::diagnostic::LastPanic>,
        breadcrumb: Option<(crate::diagnostic::BreakawayCheckpoint, bool)>,
    ) -> std::vec::Vec<crate::ankyra::replies::FociLastBootDiagnostic> {
        let (mut ctx, mut sender) = prepare_dump_test();
        if let Some(fault) = fault {
            crate::diagnostic::set_last_panic(fault);
        }
        if let Some((checkpoint, interrupted)) = breadcrumb {
            if interrupted {
                crate::diagnostic::checkpoint_breakaway_campaign(checkpoint);
            } else {
                crate::diagnostic::complete_breakaway_campaign(checkpoint);
            }
        }
        crate::diagnostic::recover_last_boot_breakaway_breadcrumb();
        foci_dump_registers(&mut ctx, &mut sender, 10);
        sender.last_boot_diagnostics
    }

    #[test]
    fn foci_dump_registers_combines_fault_and_breadcrumb_by_actionability() {
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
        let replies = run_dump_test(
            Some(crate::diagnostic::LastPanic {
                kind: crate::diagnostic::last_panic_kind::RUST_PANIC,
                line: 412,
                pc: 0,
                file_hash: 0x1234_5678,
            }),
            None,
        );
        let [reply] = replies.as_slice() else {
            panic!("panic must emit one diagnostic");
        };
        assert_eq!(
            (reply.fault_kind, reply.line, reply.file_hash),
            (
                crate::diagnostic::last_panic_kind::RUST_PANIC,
                412,
                0x1234_5678
            )
        );
        assert_eq!(
            (reply.breakaway_checkpoint, reply.breakaway_interrupted),
            (0, 0)
        );

        let replies = run_dump_test(
            None,
            Some((crate::diagnostic::BreakawayCheckpoint::Capture, true)),
        );
        let [reply] = replies.as_slice() else {
            panic!("interrupted campaign must emit one diagnostic");
        };
        assert_eq!((reply.fault_kind, reply.breakaway_checkpoint), (0, 5));
        assert_eq!(reply.breakaway_interrupted, 1);

        let replies = run_dump_test(
            Some(crate::diagnostic::LastPanic {
                kind: crate::diagnostic::last_panic_kind::HARD_FAULT,
                line: 0,
                pc: 0x0801_2345,
                file_hash: 0,
            }),
            Some((
                crate::diagnostic::BreakawayCheckpoint::StageCHandedOff,
                false,
            )),
        );
        let [reply] = replies.as_slice() else {
            panic!("fault must carry completed campaign context");
        };
        assert_eq!((reply.fault_kind, reply.pc), (2, 0x0801_2345));
        assert_eq!(
            (reply.breakaway_checkpoint, reply.breakaway_interrupted),
            (8, 0)
        );

        assert!(
            run_dump_test(
                None,
                Some((
                    crate::diagnostic::BreakawayCheckpoint::CampaignTerminalHandled,
                    false,
                )),
            )
            .is_empty()
        );
    }

    #[test]
    fn foci_dump_registers_consumes_last_boot_diagnostic_once() {
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
        let (mut ctx, mut sender) = prepare_dump_test();
        crate::diagnostic::begin_breakaway_campaign();
        crate::diagnostic::recover_last_boot_breakaway_breadcrumb();

        foci_dump_registers(&mut ctx, &mut sender, 10);
        foci_dump_registers(&mut ctx, &mut sender, 10);

        assert_eq!(sender.last_boot_diagnostics.len(), 1);
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

    /// Drive `foci_stack_watermark` the way the transport does: encode `oid`
    /// as Klipper VLQ, hand the bytes to the generated dispatch wrapper so the
    /// argument is decoded off the wire, then re-decode the emitted reply's
    /// encoded payload field by field. A descriptor typo or a field-order or
    /// field-type drift fails here, where calling the handler directly and
    /// reading the Rust struct back would not notice.
    fn dispatch_stack_watermark_over_the_wire(oid: u8) -> StackWatermarkReply {
        dispatch_stack_watermark_with_motion(oid, true)
    }

    fn dispatch_stack_watermark_with_motion(oid: u8, quiescent: bool) -> StackWatermarkReply {
        use ankyra::encoding::{Readable, Writable};

        let mut request = ankyra::ScratchOutput::<8>::new();
        oid.write(&mut request);
        let request = request.result().to_vec();

        let mut ctx = MockCtx::new(2);
        ctx.motion_quiescent = quiescent;
        let mut sender = MockSender::new();
        let mut frame: &[u8] = &request;
        super::__ankyra_dispatch_foci_stack_watermark(&mut frame, &mut ctx, &mut sender)
            .expect("dispatch must decode the request");
        assert!(frame.is_empty(), "dispatch left request bytes unread");

        let [reply] = sender.stack_watermark_payloads.as_slice() else {
            panic!("expected exactly one reply");
        };
        let mut payload = ankyra::ScratchOutput::<64>::new();
        reply.write(&mut payload);
        let payload = payload.result().to_vec();

        let mut cursor: &[u8] = &payload;
        let decoded = StackWatermarkReply {
            oid: u8::read(&mut cursor).expect("oid"),
            stack_unused_bytes: u32::read(&mut cursor).expect("stack_unused_bytes"),
            painted_bytes: u32::read(&mut cursor).expect("painted_bytes"),
            status: u8::read(&mut cursor).expect("status"),
        };
        assert!(cursor.is_empty(), "reply carried unexpected trailing bytes");
        decoded
    }

    #[test]
    fn foci_stack_watermark_wire_formats_match_the_klipper_dictionary() {
        assert_eq!(
            super::__ANKYRA_FORMAT_command_foci_stack_watermark,
            "foci_stack_watermark oid=%c"
        );
        let reply = crate::ankyra::replies::__ankyra_descriptor_FociStackWatermarkResult();
        assert_eq!(reply.protocol_name(), "foci_stack_watermark_result");
        assert_eq!(
            reply.message_format(),
            "foci_stack_watermark_result oid=%c stack_unused_bytes=%u painted_bytes=%u status=%c"
        );
    }

    #[test]
    fn foci_stack_watermark_round_trips_the_surviving_sentinel_run() {
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
        let sentinel = crate::stack_watermark::STACK_PAINT_SENTINEL;
        let words: &'static [u32] = std::vec::Vec::leak(std::vec![
            sentinel,
            sentinel,
            sentinel,
            0xDEAD_BEEF,
            sentinel,
        ]);
        // SAFETY: the span is leaked, so it outlives every later read.
        unsafe { crate::stack_watermark::set_painted_region_for_test(words) };

        assert_eq!(
            dispatch_stack_watermark_over_the_wire(10),
            StackWatermarkReply {
                oid: 10,
                stack_unused_bytes: 12,
                painted_bytes: 20,
                status: 0,
            }
        );
    }

    #[test]
    fn foci_stack_watermark_round_trips_a_lower_bound() {
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
        let sentinel = crate::stack_watermark::STACK_PAINT_SENTINEL;
        let words: &'static [u32] = std::vec::Vec::leak(std::vec![sentinel; 3]);
        // SAFETY: the span is leaked, so it outlives every later read.
        unsafe { crate::stack_watermark::set_painted_region_for_test(words) };

        assert_eq!(
            dispatch_stack_watermark_over_the_wire(11),
            StackWatermarkReply {
                oid: 11,
                stack_unused_bytes: 12,
                painted_bytes: 12,
                status: 2,
            }
        );
    }

    #[test]
    fn foci_stack_watermark_refuses_to_mask_interrupts_while_motion_is_live() {
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
        let sentinel = crate::stack_watermark::STACK_PAINT_SENTINEL;
        let words: &'static [u32] = std::vec::Vec::leak(std::vec![sentinel, 0xDEAD_BEEF]);
        // SAFETY: the span is leaked, so it outlives every later read.
        unsafe { crate::stack_watermark::set_painted_region_for_test(words) };

        assert_eq!(
            dispatch_stack_watermark_with_motion(12, false),
            StackWatermarkReply {
                oid: 12,
                stack_unused_bytes: 0,
                painted_bytes: 0,
                status: 3,
            }
        );
    }

    #[test]
    fn foci_stack_watermark_round_trips_an_unpainted_board() {
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
        crate::stack_watermark::clear_painted_region_for_test();

        assert_eq!(
            dispatch_stack_watermark_over_the_wire(7),
            StackWatermarkReply {
                oid: 7,
                stack_unused_bytes: 0,
                painted_bytes: 0,
                status: 1,
            }
        );
    }
}
