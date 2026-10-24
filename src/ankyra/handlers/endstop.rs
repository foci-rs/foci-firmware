//! Endstop command handlers.

use ankyra::prelude::klipper_command;

use crate::context::{EndstopHost, ShutdownRecovery, TrsyncHost};
use crate::shutdown::ShutdownReason;
use crate::state::trsync::{ENDSTOP_PIN_IDS, GPIO_MAX_PIN_VALUE};

/// Register an endstop object on a GPIO input pin.
/// Virtual pins (>= `GPIO_MAX_PIN_VALUE`) skip GPIO setup and may map to a TMC stall channel.
#[klipper_command]
pub fn config_endstop(ctx: &mut dyn EndstopConfigHost, oid: u8, pin: u8, pull_up: u8) {
    let idx = match ctx.register_endstop(oid) {
        Ok(idx) => idx,
        Err(()) => {
            defmt::error!("config_endstop: oid={}", oid);
            if ctx.enter_shutdown() {
                let clock = ctx.now();
                ctx.send_shutdown(ShutdownReason::ConfigEndstopFailed, clock);
            }
            return;
        }
    };

    let pin = ctx.physical_pin_for_alias(pin).unwrap_or(pin);
    if pin < GPIO_MAX_PIN_VALUE {
        ctx.configure_input(pin, pull_up != 0);
        defmt::info!(
            "config_endstop: oid={} idx={} pin=P{}{}{}",
            oid,
            idx,
            ['A', 'B', 'C', 'D', 'E'][(pin / 16) as usize],
            pin % 16,
            if pull_up != 0 { " (pull-up)" } else { "" }
        );
    } else {
        if let Some(channel) = ctx.stall_channel_for_pin(pin)
            && let Some(state) = crate::state::tmc::tmc_channel(channel)
        {
            state
                .stall_endstop_configured
                .store(true, core::sync::atomic::Ordering::Relaxed);
        }
        defmt::info!(
            "config_endstop: oid={} idx={} virtual pin={}",
            oid,
            idx,
            pin
        );
    }
    ENDSTOP_PIN_IDS[idx].store(pin, core::sync::atomic::Ordering::Relaxed);
}

/// Arm an endstop for homing and link it to a TrSync.
/// A `sample_count` of zero disarms; `trsync_oid` is not validated then.
#[klipper_command]
#[allow(clippy::too_many_arguments)]
pub fn endstop_home(
    ctx: &mut dyn EndstopHomeHost,
    oid: u8,
    clock: u32,
    sample_ticks: u32,
    sample_count: u8,
    rest_ticks: u32,
    pin_value: u8,
    trsync_oid: u8,
    trigger_reason: u8,
) {
    let result = ctx.endstop_start_homing(
        oid,
        crate::context::trsync::HomingRequest {
            clock,
            sample_ticks,
            sample_count,
            rest_ticks,
            pin_value_expected: pin_value != 0,
            trsync_oid,
            trigger_reason,
        },
    );
    if result.is_err() {
        defmt::error!("endstop_home: oid={}", oid);
        if ctx.enter_shutdown() {
            let shutdown_clock = ctx.now();
            ctx.send_shutdown(ShutdownReason::EndstopHomeFailed, shutdown_clock);
        }
    }
}

/// Query homing status, next sample clock, and pin value.
#[klipper_command]
pub fn endstop_query_state(ctx: &mut dyn EndstopConfigHost, oid: u8) {
    match ctx.endstop_query(oid) {
        Ok((homing, next_clock, pin_id)) => {
            let pin_state = if pin_id != 0xFF {
                ctx.read_pin(pin_id)
            } else {
                false
            };
            ::ankyra::klipper_reply!(
                crate::ankyra::replies::EndstopState,
                oid: u8 = oid,
                homing: u8 = u8::from(homing),
                next_clock: u32 = next_clock,
                pin_value: u8 = u8::from(pin_state)
            );
        }
        Err(()) => {
            defmt::error!("endstop_query_state: unknown oid={}", oid);
            if ctx.enter_shutdown() {
                let clock = ctx.now();
                ctx.send_shutdown(ShutdownReason::EndstopQueryStateFailed, clock);
            }
        }
    }
}

/// Composite bound for `config_endstop` and `endstop_query_state`:
/// shutdown recovery, the trigger-domain bridge and the endstop host.
pub trait EndstopConfigHost: ShutdownRecovery + TrsyncHost + EndstopHost {}
impl<T: ?Sized + ShutdownRecovery + TrsyncHost + EndstopHost> EndstopConfigHost for T {}

/// Composite bound for `endstop_home`: shutdown recovery plus the
/// trigger-domain bridge.
pub trait EndstopHomeHost: ShutdownRecovery + TrsyncHost {}
impl<T: ?Sized + ShutdownRecovery + TrsyncHost> EndstopHomeHost for T {}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::context::{ClockSource, StateAccess};
    use core::sync::atomic::Ordering::Relaxed;
    use foci_klipper::state::State;

    struct MockCtx {
        state: State,
        last_shutdown: Option<ShutdownReason>,
        configured_inputs: std::vec::Vec<(u8, bool)>,
    }

    impl MockCtx {
        fn new() -> Self {
            Self {
                state: State::new(),
                last_shutdown: None,
                configured_inputs: std::vec::Vec::new(),
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

    impl TrsyncHost for MockCtx {
        fn reset_all(&mut self) {
            panic!("reset_all not used in this test")
        }
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
            Ok(0)
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

    impl EndstopHost for MockCtx {
        fn configure_input(&mut self, pin: u8, pull_up: bool) {
            self.configured_inputs.push((pin, pull_up));
        }
        fn read_pin(&self, _pin: u8) -> bool {
            false
        }
        fn stall_channel_for_pin(&self, pin: u8) -> Option<u8> {
            if pin == 200 { Some(1) } else { None }
        }
        fn physical_pin_for_alias(&self, pin: u8) -> Option<u8> {
            if pin == 201 { Some(55) } else { None }
        }
    }

    #[test]
    fn config_endstop_stores_virtual_pin_and_marks_stall_channel() {
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
        let mut ctx = MockCtx::new();
        let mut sender = ();
        let tmc_state = crate::state::tmc::tmc_channel(1).unwrap();
        tmc_state.stall_endstop_configured.store(false, Relaxed);

        config_endstop(&mut ctx, &mut sender, 5, 200, 0);

        assert_eq!(ENDSTOP_PIN_IDS[0].load(Relaxed), 200);
        assert!(tmc_state.stall_endstop_configured.load(Relaxed));
        assert!(
            ctx.configured_inputs.is_empty(),
            "virtual pin must not touch GPIO"
        );
    }

    #[test]
    fn config_endstop_configures_gpio_pin_unchanged() {
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
        let mut ctx = MockCtx::new();
        let mut sender = ();

        config_endstop(&mut ctx, &mut sender, 6, 10, 1);

        assert_eq!(ENDSTOP_PIN_IDS[0].load(Relaxed), 10);
        assert_eq!(ctx.configured_inputs, std::vec![(10, true)]);
    }

    #[test]
    fn config_endstop_resolves_alias_to_physical_gpio() {
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
        let mut ctx = MockCtx::new();
        let mut sender = ();

        config_endstop(&mut ctx, &mut sender, 7, 201, 1);

        assert_eq!(ENDSTOP_PIN_IDS[0].load(Relaxed), 55);
        assert_eq!(ctx.configured_inputs, std::vec![(55, true)]);
    }
}
