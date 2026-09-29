//! Shared TMC context boundary and send-only enqueue policy.

use super::{QueueFull, StateAccess};

/// TMC4671 request bus. Send-only; the `tmc_control` task on the
/// board side consumes from the channel and performs the actual
/// register writes.
pub trait TmcHost: StateAccess {
    fn enqueue_request(&self, req: crate::tmc::TmcRequest) -> Result<(), QueueFull>;

    /// Capture read-only idle admission for motion-scale configuration.
    fn motion_scale_idle(
        &self,
        channel: u8,
    ) -> Result<(), crate::motion_scale::MotionScaleRejection>;

    /// Report whether no channel is armed, queued, or stepping.
    ///
    /// Board-implemented because only the board knows its channel count and
    /// owns its step queues. Gates work that must not run while interrupt-driven
    /// motion is live, so an implementation that cannot tell must answer
    /// `false`.
    fn motion_quiescent(&self) -> bool;

    /// Board-specific RMS milliamp ceiling for run and homing current.
    ///
    /// Board-implemented; mirrors `TmcConfig::max_run_current_ma`.
    fn max_run_current_ma(&self) -> u32;

    /// Emit a host-visible reason before rejecting motion-scale configuration.
    fn report_motion_scale_rejection(
        &mut self,
        oid: u8,
        rejection: crate::motion_scale::MotionScaleRejection,
    );
}

/// Board hook for the RTIC TMC request sender.
pub trait TmcRequestQueueBoard {
    /// Try to send one request to the board TMC control task.
    fn try_send_tmc_request(&self, req: crate::tmc::TmcRequest) -> bool;
}

/// Enqueue one TMC request through the board RTIC sender hook.
///
/// This helper is intentionally send-only. It does not inspect request kind
/// and does not reserve commissioning admission.
pub fn enqueue_tmc_request<B: TmcRequestQueueBoard + ?Sized>(
    board: &B,
    req: crate::tmc::TmcRequest,
) -> Result<(), QueueFull> {
    if board.try_send_tmc_request(req) {
        Ok(())
    } else {
        Err(QueueFull)
    }
}

#[cfg(test)]
mod tests {
    use core::cell::{Cell, RefCell};

    use super::*;
    use crate::tmc::{TmcRequest, TmcRequestKind};

    struct MockTmcQueue {
        accept: Cell<bool>,
        send_count: Cell<u8>,
        last_request: RefCell<Option<TmcRequest>>,
    }

    impl MockTmcQueue {
        fn new(accept: bool) -> Self {
            Self {
                accept: Cell::new(accept),
                send_count: Cell::new(0),
                last_request: RefCell::new(None),
            }
        }
    }

    impl TmcRequestQueueBoard for MockTmcQueue {
        fn try_send_tmc_request(&self, req: TmcRequest) -> bool {
            self.send_count.set(self.send_count.get().wrapping_add(1));
            self.last_request.replace(Some(req));
            self.accept.get()
        }
    }

    #[test]
    fn enqueue_tmc_request_returns_ok_when_board_accepts() {
        let board = MockTmcQueue::new(true);
        let req = TmcRequest::new(0, TmcRequestKind::SetCurrent { run_ma: 1000 });

        assert_eq!(enqueue_tmc_request(&board, req.clone()), Ok(()));

        assert_eq!(board.send_count.get(), 1);
        assert_eq!(board.last_request.borrow().as_ref(), Some(&req));
    }

    #[test]
    fn enqueue_tmc_request_reports_queue_full_when_board_rejects() {
        let board = MockTmcQueue::new(false);
        let req = TmcRequest::new(0, TmcRequestKind::DumpRegisters { oid: 7 });

        assert_eq!(enqueue_tmc_request(&board, req.clone()), Err(QueueFull));

        assert_eq!(board.send_count.get(), 1);
        assert_eq!(board.last_request.borrow().as_ref(), Some(&req));
    }

    #[test]
    fn enqueue_tmc_request_sends_commissioning_and_enable_requests_without_special_cases() {
        let requests = [
            TmcRequest::new(0, TmcRequestKind::Calibrate { oid: 1 }),
            TmcRequest::new(0, TmcRequestKind::Commission { oid: 2, profile: 1 }),
            TmcRequest::new(
                0,
                TmcRequestKind::Tune {
                    oid: 3,
                    profile: 1,
                    mode: 1,
                    inner_lambda_us: 20,
                    theta_e_us: 30,
                    current_bandwidth_hz: 1000,
                    inner_warning_flags: 0,
                    requested_velocity_mrev_s: 6000,
                    machine_velocity_ceiling_mrev_s: 10_000,
                    requested_velocity_source: crate::commissioning::outer::velocity::sweep_plan::RequestedVelocitySource::Defaulted,
                    max_stroke_travel_mrev: 1000,
                    settle_travel_reserve_mrev: 100,
                    negative_position_headroom_mrev: 2000,
                    positive_position_headroom_mrev: 2000,
                    max_duration_ms: 3000,
                    homing_speed_mrev_s: 4_375,
                    max_accel_mrev_s2: 200_000,
                },
            ),
            TmcRequest::new(0, TmcRequestKind::SelfTest { oid: 4 }),
            TmcRequest::new(0, TmcRequestKind::SetEnable(true)),
        ];
        let board = MockTmcQueue::new(true);

        for req in requests {
            assert_eq!(enqueue_tmc_request(&board, req), Ok(()));
        }

        assert_eq!(board.send_count.get(), 5);
    }

    #[test]
    fn enqueue_tmc_request_rejects_enable_true_only_when_board_rejects() {
        let board = MockTmcQueue::new(false);

        assert_eq!(
            enqueue_tmc_request(&board, TmcRequest::new(0, TmcRequestKind::SetEnable(true))),
            Err(QueueFull)
        );

        assert_eq!(board.send_count.get(), 1);
    }
}
