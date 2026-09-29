//! Lifecycle command handlers: status, recovery and diagnostics.

use ankyra::prelude::klipper_command;

use crate::analog_in::reset_all_analog_channels;
use crate::ankyra::replies::{Clock, Config, Uptime};
use crate::context::{ClockSource, LifecycleReset, ShutdownRecovery, StateAccess};
use crate::shutdown::ShutdownReason;
use crate::state::lifecycle::{
    HAS_RECEIVED_VALID_CLOCK, LAST_HOST_LIVENESS_TICKS, REPORT_PENDING, SHUTDOWN_ACTIVE,
    STATS_SEND_TIME, STATS_SEND_TIME_HIGH,
};
use crate::state::stepper::reset_all_stepper_channels;
use crate::state::tmc::{TMC_QUEUE_FLUSH, reset_all_tmc_channels};
use crate::state::trsync::{
    ENDSTOP_PIN_IDS, HOMING_REENTER_PENDING, HOMING_STATE, HOMING_TRIGGERED,
};

#[klipper_command(in_shutdown)]
pub fn get_clock(ctx: &mut dyn ClockSource) {
    use core::sync::atomic::Ordering::Relaxed;

    let clock = ctx.now();
    LAST_HOST_LIVENESS_TICKS.store(clock, Relaxed);
    HAS_RECEIVED_VALID_CLOCK.store(true, Relaxed);
    ::ankyra::klipper_reply!(Clock, clock: u32 = clock);
}

#[klipper_command(in_shutdown)]
pub fn get_uptime(ctx: &mut dyn ClockSource) {
    use core::sync::atomic::Ordering::Relaxed;
    let cur = ctx.now();
    let high = STATS_SEND_TIME_HIGH.load(Relaxed) + u32::from(cur < STATS_SEND_TIME.load(Relaxed));
    ::ankyra::klipper_reply!(
        Uptime,
        high: u32 = high,
        clock: u32 = cur,
    );
}

#[klipper_command(in_shutdown)]
pub fn get_config(ctx: &mut dyn StateAccess) {
    let (is_config, crc, is_shutdown, move_count) =
        foci_klipper::commands::lifecycle::get_config(ctx.state());
    ::ankyra::klipper_reply!(
        Config,
        is_config: bool = is_config,
        crc: u32 = crc,
        is_shutdown: bool = is_shutdown,
        move_count: u16 = move_count,
    );
}

/// Clear shutdown state; outside shutdown this triggers a shutdown instead.
#[klipper_command(in_shutdown)]
pub fn clear_shutdown(ctx: &mut dyn ShutdownRecovery) {
    match foci_klipper::commands::lifecycle::clear_shutdown(ctx.state()) {
        Ok(()) => {
            SHUTDOWN_ACTIVE.store(false, core::sync::atomic::Ordering::Relaxed);
        }
        Err(_) => {
            if ctx.enter_shutdown() {
                let clock = ctx.now();
                ctx.send_shutdown(ShutdownReason::ShutdownClearedWhenNotShutdown, clock);
            }
        }
    }
}

/// Full config reset: clear all OIDs, steppers, and shutdown state. Only available during shutdown.
#[klipper_command(in_shutdown)]
pub fn config_reset(ctx: &mut dyn LifecycleReset) {
    use core::sync::atomic::Ordering::Relaxed;
    if !ctx.state().mcu_state.is_shutdown() {
        if ctx.enter_shutdown() {
            let clock = ctx.now();
            ctx.send_shutdown(ShutdownReason::ConfigResetOnlyAvailableWhenShutdown, clock);
        }
        return;
    }
    foci_klipper::commands::lifecycle::config_reset(ctx.state());
    ctx.reset_all();
    REPORT_PENDING.store(false, Relaxed);
    HOMING_TRIGGERED.store(0xFF, Relaxed);
    HOMING_STATE.store(0, Relaxed);
    HOMING_REENTER_PENDING.store(false, Relaxed);
    for pin_id in &ENDSTOP_PIN_IDS {
        pin_id.store(0xFF, Relaxed);
    }
    reset_all_tmc_channels();
    reset_all_stepper_channels();
    reset_all_analog_channels();
    STATS_SEND_TIME.store(0, Relaxed);
    STATS_SEND_TIME_HIGH.store(0, Relaxed);
    TMC_QUEUE_FLUSH.store(true, Relaxed);
}

#[klipper_command]
pub fn finalize_config(ctx: &mut dyn StateAccess, crc: u32) {
    foci_klipper::commands::lifecycle::finalize_config(ctx.state(), crc);
    defmt::info!("Config finalized, CRC: {:#010x}", crc);
}

#[klipper_command]
pub fn allocate_oids(_ctx: &mut dyn StateAccess, _count: u8) {
    foci_klipper::commands::lifecycle::allocate_oids(_count);
}

#[klipper_command(in_shutdown)]
pub fn emergency_stop(ctx: &mut dyn ShutdownRecovery) {
    if ctx.enter_shutdown() {
        let clock = ctx.now();
        ctx.send_shutdown(ShutdownReason::CommandRequest, clock);
    }
}

/// Reset the MCU, bypassing Katapult straight to the application (used by `FIRMWARE_RESTART`).
#[klipper_command(in_shutdown)]
pub fn reset(ctx: &mut dyn ShutdownRecovery) {
    ctx.reset_to_bootloader();
}

#[klipper_command(in_shutdown)]
pub fn debug_nop(_ctx: &mut dyn StateAccess) {}

#[cfg(test)]
mod tests {
    use core::sync::atomic::Ordering::Relaxed;
    use std::sync::Mutex;

    use foci_klipper::shutdown::McuState;
    use foci_klipper::state::State;

    use super::*;

    static TEST_LOCK: Mutex<()> = Mutex::new(());

    struct MockCtx {
        state: State,
        last_shutdown: Option<ShutdownReason>,
    }

    struct MockSender;

    impl ::ankyra::SendReply<Clock> for MockSender {
        fn send(&mut self, _payload: Clock) {}
    }

    impl MockCtx {
        fn new() -> Self {
            Self {
                state: State::new(),
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
            crate::helpers::lifecycle::enter_shutdown(&mut self.state)
        }

        fn reset_to_bootloader(&mut self) -> ! {
            panic!("reset_to_bootloader not used in tests")
        }

        fn send_shutdown(&mut self, reason: ShutdownReason, _clock: u32) {
            self.last_shutdown = Some(reason);
        }
    }

    impl crate::context::TrsyncHost for MockCtx {
        fn reset_all(&mut self) {}
        fn register_trsync(&mut self, _oid: u8) -> Result<(), ()> {
            panic!("register_trsync not used in this test")
        }
        fn trsync_arm(
            &mut self,
            _oid: u8,
            _report_clock: u32,
            _report_ticks: u32,
            _expire_reason: u8,
        ) -> Result<(), ()> {
            panic!("trsync_arm not used in this test")
        }
        fn trsync_set_timeout(&mut self, _oid: u8, _clock: u32) -> Result<(), ()> {
            panic!("trsync_set_timeout not used in this test")
        }
        fn trsync_trigger(&mut self, _oid: u8, _reason: u8) -> Result<u8, ()> {
            panic!("trsync_trigger not used in this test")
        }
        fn register_endstop(&mut self, _oid: u8) -> Result<usize, ()> {
            panic!("register_endstop not used in this test")
        }
        fn endstop_start_homing(
            &mut self,
            _oid: u8,
            _request: crate::context::trsync::HomingRequest,
        ) -> Result<(), ()> {
            panic!("endstop_start_homing not used in this test")
        }
        fn endstop_query(&mut self, _oid: u8) -> Result<(bool, u32, u8), ()> {
            panic!("endstop_query not used in this test")
        }
        fn link_stepper_stop(&mut self, _stepper_idx: usize, _trsync_oid: u8) -> Result<u32, ()> {
            panic!("link_stepper_stop not used in this test")
        }
        fn notify_homing_start(&mut self) -> Result<(), ()> {
            Ok(())
        }
    }

    #[test]
    fn invalid_clear_shutdown_leaves_shutdown_active_set() {
        let _guard = TEST_LOCK.lock().unwrap();
        SHUTDOWN_ACTIVE.store(false, Relaxed);

        let mut ctx = MockCtx::new();
        let mut sender = ();
        ctx.state.mcu_state = McuState::Running;

        clear_shutdown(&mut ctx, &mut sender);

        assert!(ctx.state.mcu_state.is_shutdown());
        assert_eq!(
            ctx.last_shutdown,
            Some(ShutdownReason::ShutdownClearedWhenNotShutdown)
        );
        assert!(
            SHUTDOWN_ACTIVE.load(Relaxed),
            "an invalid clear_shutdown call entered shutdown but then cleared the shared flag"
        );
    }

    #[test]
    fn valid_clear_shutdown_clears_shutdown_active() {
        let _guard = TEST_LOCK.lock().unwrap();
        SHUTDOWN_ACTIVE.store(true, Relaxed);

        let mut ctx = MockCtx::new();
        let mut sender = ();
        ctx.state.mcu_state = McuState::Shutdown { reason: 0 };

        clear_shutdown(&mut ctx, &mut sender);

        assert!(!ctx.state.mcu_state.is_shutdown());
        assert_eq!(ctx.last_shutdown, None);
        assert!(!SHUTDOWN_ACTIVE.load(Relaxed));
    }

    #[test]
    fn get_clock_records_liveness_and_receipt() {
        let _guard = TEST_LOCK.lock().unwrap();
        LAST_HOST_LIVENESS_TICKS.store(0, Relaxed);
        HAS_RECEIVED_VALID_CLOCK.store(false, Relaxed);

        let mut ctx = MockCtx::new();
        let mut sender = MockSender;
        get_clock(&mut ctx, &mut sender);

        assert_eq!(LAST_HOST_LIVENESS_TICKS.load(Relaxed), 123);
        assert!(HAS_RECEIVED_VALID_CLOCK.load(Relaxed));
    }

    #[test]
    fn config_reset_drops_a_queued_homing_re_entry() {
        use crate::state::trsync::HOMING_REENTER_PENDING;

        let _guard = TEST_LOCK.lock().unwrap();
        let _shared_guard =
            crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
        HOMING_STATE.store(3, Relaxed);
        HOMING_REENTER_PENDING.store(true, Relaxed);

        let mut ctx = MockCtx::new();
        ctx.state.mcu_state = McuState::Shutdown { reason: 0 };
        let mut sender = ();
        config_reset(&mut ctx, &mut sender);

        assert_eq!(HOMING_STATE.load(Relaxed), 0);
        assert!(
            !HOMING_REENTER_PENDING.load(Relaxed),
            "a re-entry queued before the reset must not re-arm homing afterwards"
        );
    }

    #[test]
    fn config_reset_clears_a_previously_configured_analog_channel() {
        use crate::analog_in::{AnalogInConfig, analog_channel};

        let _guard = TEST_LOCK.lock().unwrap();
        let _shared_guard =
            crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
        let channel = analog_channel(0).unwrap();
        channel.configure_oid(5);
        channel.configure_sampling(
            &AnalogInConfig {
                sample_ticks: 10,
                sample_count: 4,
                rest_ticks: 1000,
                min_value: 100,
                max_value: 10_000,
                range_check_count: 2,
            },
            50,
        );
        assert_eq!(channel.oid(), Some(5), "sanity: channel is configured");
        assert!(channel.is_due(50), "sanity: sampling is scheduled");

        let mut ctx = MockCtx::new();
        ctx.state.mcu_state = McuState::Shutdown { reason: 0 };
        let mut sender = ();
        config_reset(&mut ctx, &mut sender);

        assert_eq!(
            channel.oid(),
            None,
            "config_reset must clear a stale analog OID"
        );
        assert!(
            !channel.is_due(50),
            "config_reset must not leave a stale sampling schedule active"
        );
    }
}
