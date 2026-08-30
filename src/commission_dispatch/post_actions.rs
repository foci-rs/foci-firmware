//! Terminal stepper post-action state helpers for commissioning dispatch.

use crate::commission_dispatch::{DispatchState, StepperPostAction};

/// Board task to run after a stepper post-action state transition.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum StepperTaskDirective {
    /// No board task should be spawned.
    None,
    /// Spawn the board-local arm-check task.
    ArmCheck,
    /// Spawn the board-local stop-check task.
    StopCheck,
}

/// Apply P1-visible stepper state for a terminal commissioning action.
///
/// Returns the board task directive only after all shared atomics and step
/// epoch state are updated, so the board cannot spawn `arm_check` or
/// `stop_check` before those tasks can observe the intended state.
pub fn apply_stepper_post_action_state(
    action: StepperPostAction,
    state: &mut DispatchState<'_>,
) -> StepperTaskDirective {
    match action {
        StepperPostAction::None => StepperTaskDirective::None,
        StepperPostAction::Stop => {
            state
                .tmc
                .motor_armed
                .store(false, core::sync::atomic::Ordering::Relaxed);
            StepperTaskDirective::StopCheck
        }
        StepperPostAction::ArmZeroPosition => {
            state
                .stepper
                .position
                .store(0, core::sync::atomic::Ordering::Relaxed);
            crate::tmc_control::lifecycle::arm_motor_for_steps(state);
            StepperTaskDirective::ArmCheck
        }
        StepperPostAction::ArmPreservePosition => {
            crate::tmc_control::lifecycle::arm_motor_for_steps(state);
            StepperTaskDirective::ArmCheck
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::commission_dispatch::test_support::fresh_state_locals;
    use foci_firmware_macros::fixture_test;

    fn dispatch_state<'a>(
        calibrated: &'a mut bool,
        motor_enabled: &'a mut bool,
        stored_gains: &'a mut Option<crate::pid::InnerLoopGains>,
    ) -> DispatchState<'a> {
        DispatchState::new(
            0,
            crate::state::tmc::tmc_channel0(),
            crate::state::stepper::stepper_channel0(),
            calibrated,
            motor_enabled,
            stored_gains,
        )
    }

    fn tmc_state() -> &'static crate::state::tmc::TmcChannelState {
        crate::state::tmc::tmc_channel0()
    }

    fn stepper_state() -> &'static crate::state::stepper::StepperChannelState {
        crate::state::stepper::stepper_channel0()
    }

    #[fixture_test]
    fn stepper_post_action_zeroes_before_arm_check_directive() {
        use core::sync::atomic::Ordering;
        let (mut cal, _, mut gains) = fresh_state_locals();
        let mut en = true;
        stepper_state().position.store(123, Ordering::Relaxed);
        tmc_state().motor_armed.store(false, Ordering::Relaxed);
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);

        let directive =
            apply_stepper_post_action_state(StepperPostAction::ArmZeroPosition, &mut state);

        assert_eq!(directive, StepperTaskDirective::ArmCheck);
        assert_eq!(stepper_state().position.load(Ordering::Relaxed), 0);
        assert!(tmc_state().motor_armed.load(Ordering::Relaxed));
        tmc_state().motor_armed.store(false, Ordering::Relaxed);
    }

    #[fixture_test]
    fn stepper_post_action_stop_clears_before_stop_check_directive() {
        use core::sync::atomic::Ordering;
        let (mut cal, mut en, mut gains) = fresh_state_locals();
        tmc_state().motor_armed.store(true, Ordering::Relaxed);
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);

        let directive = apply_stepper_post_action_state(StepperPostAction::Stop, &mut state);

        assert_eq!(directive, StepperTaskDirective::StopCheck);
        assert!(!tmc_state().motor_armed.load(Ordering::Relaxed));
    }
}
