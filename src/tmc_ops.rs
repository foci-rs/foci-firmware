//! Board-agnostic TMC4671 operations extracted from the RTIC task body.
//!
//! Functions here are monomorphized over `B: SpiDevice<u8>` rather than
//! dyn-dispatched: the hot-path SPI access does not benefit from vtable
//! indirection, and `async fn` in traits is not viable in `no_std`. The RTIC
//! task shell (`#[task]` declaration, shared-resource lock, `spawn()`, EXTI
//! setup) stays board-local; the body calls into this module.
//!
//! Populated progressively by Phase A-7 Tasks 26–27 of the internal-
//! restructure plan (`docs/plans/archive/2026-04-20-phase-a-internal-
//! restructure.md`). See the extraction blueprint in this session's
//! logbook entry for commit-by-commit scope.

use crate::commission_dispatch::{DispatchState, StepperPostAction};
use crate::commissioning::types::CommissioningBackend;
use crate::state::tmc::TmcChannelState;

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

/// Pack TMC4671 outer-loop gain registers from raw Q8.8 gain values.
///
/// The TMC4671 stores the P term in the upper 16 bits and the I term in the
/// lower 16 bits for both velocity and position outer-loop registers.
pub fn pack_outer_gain_registers(
    position_p: u16,
    position_i: u16,
    velocity_p: u16,
    velocity_i: u16,
) -> (u32, u32) {
    let velocity_reg = ((velocity_p as u32) << 16) | velocity_i as u32;
    let position_reg = ((position_p as u32) << 16) | position_i as u32;
    (velocity_reg, position_reg)
}

/// Pack TMC4671 inner current-loop gain registers from raw gain values.
///
/// The TMC4671 stores the P term in the upper 16 bits and the I term in the
/// lower 16 bits for both flux and torque PI registers.
pub fn pack_inner_gain_registers(gains: crate::pid::InnerLoopGains) -> (u32, u32) {
    let flux_reg = ((gains.flux_p as u32) << 16) | gains.flux_i as u32;
    let torque_reg = ((gains.torque_p as u32) << 16) | gains.torque_i as u32;
    (flux_reg, torque_reg)
}

/// SPI restore sequence for the OuterDone success path.
///
/// Writes tuned velocity and position gains, the operational velocity limit,
/// current limit from channel state, the
/// safety-net `PIDOUT_UQ_UD_LIMITS` cap (1024 counts), clears the
/// position-actual integrator, snaps the position target to the current
/// encoder count scaled by the channel step width, and
/// enters position mode via `MODE_RAMP_MODE_MOTION = 0x8000_0003`.
///
/// The write order matches the OuterDone restore block in `main.rs` exactly.
/// Motor safety depends on setting limits (`PID_TORQUE_FLUX_LIMITS`,
/// `PIDOUT_UQ_UD_LIMITS`) before entering motion mode.
///
/// # Parameters
///
/// - `backend` — mutable reference to a `CommissioningBackend` implementation.
/// - `outer` — outer commissioning result containing tuned gain values.
///
/// # Errors
///
/// Returns the backend's SPI error on the first failed write.
pub async fn restore_position_mode_after_outer<B: CommissioningBackend>(
    backend: &mut B,
    tmc_state: &TmcChannelState,
    outer: &crate::commissioning::types::OuterResult,
) -> Result<(), B::Error> {
    use core::sync::atomic::Ordering::Relaxed;
    use foci_tmc4671::registers::*;

    // Tuned velocity + position gains.
    let (velocity_reg, position_reg) = pack_outer_gain_registers(
        outer.position_p,
        outer.position_i,
        outer.velocity_p,
        outer.velocity_i,
    );
    backend
        .write_register(PID_VELOCITY_P_VELOCITY_I, velocity_reg)
        .await?;
    backend
        .write_register(PID_POSITION_P_POSITION_I, position_reg)
        .await?;

    // Operational limits (not safety caps).
    backend
        .write_register(PID_VELOCITY_LIMIT, outer.velocity_limit)
        .await?;

    let current_limit = tmc_state.current_limit_reg.load(Relaxed);
    backend
        .write_register(PID_TORQUE_FLUX_LIMITS, current_limit)
        .await?;

    // Safety-net voltage cap — must be written before entering motion mode.
    backend.write_register(PIDOUT_UQ_UD_LIMITS, 1024).await?;

    // Clear PID error sums before mode switch.
    backend.write_register(PID_POSITION_ACTUAL, 0).await?;

    let target = backend.read_register(ABN_DECODER_COUNT).await?;
    let target_scaled = target.wrapping_mul(tmc_state.step_width_value.load(Relaxed) as u32);
    backend
        .write_register(PID_POSITION_TARGET, target_scaled)
        .await?;

    // Enter position mode (advanced PID cascade).
    backend
        .write_register(MODE_RAMP_MODE_MOTION, 0x8000_0003)
        .await?;

    Ok(())
}

/// SPI restore sequence for the CalibrateDone path.
///
/// Reapplies host-stored inner gains (if any) to `PID_FLUX_P_FLUX_I` and
/// `PID_TORQUE_P_TORQUE_I`, then reads outer gain values from the
/// [`crate::state::tmc`] atomics and writes velocity gains, position gains,
/// velocity limit, and current limit back to the TMC4671.
///
/// The write order matches the CalibrateDone gain-restore block in `main.rs`
/// exactly. SPI failures abort immediately; reporting success with wrong
/// gains active is worse than failing.
///
/// # Parameters
///
/// - `backend` — mutable reference to a `CommissioningBackend` implementation.
/// - `stored` — optional host-stored inner loop gains; skipped when `None`.
///
/// # Errors
///
/// Returns the backend's SPI error on the first failed write.
pub async fn restore_calibrate_gains<B: CommissioningBackend>(
    backend: &mut B,
    tmc_state: &TmcChannelState,
    stored: Option<crate::pid::InnerLoopGains>,
) -> Result<(), B::Error> {
    use core::sync::atomic::Ordering::Relaxed;
    use foci_tmc4671::registers::*;

    // Current-loop gains from prior commissioning.
    if let Some(gains) = stored {
        let (flux_reg, torque_reg) = pack_inner_gain_registers(gains);
        backend.write_register(PID_FLUX_P_FLUX_I, flux_reg).await?;
        backend
            .write_register(PID_TORQUE_P_TORQUE_I, torque_reg)
            .await?;
    }

    let velocity_p = tmc_state.velocity_p_value.load(Relaxed);
    let velocity_i = tmc_state.velocity_i_value.load(Relaxed);
    let position_p = tmc_state.position_p_value.load(Relaxed);
    let position_i = tmc_state.position_i_value.load(Relaxed);
    let (velocity_reg, position_reg) =
        pack_outer_gain_registers(position_p, position_i, velocity_p, velocity_i);

    // Velocity gains.
    backend
        .write_register(PID_VELOCITY_P_VELOCITY_I, velocity_reg)
        .await?;

    // Position gains.
    backend
        .write_register(PID_POSITION_P_POSITION_I, position_reg)
        .await?;

    // Velocity limit.
    let velocity_limit = tmc_state.velocity_limit_cfg.load(Relaxed);
    backend
        .write_register(PID_VELOCITY_LIMIT, velocity_limit)
        .await?;

    // Current limit (TMC4671 resets on mode change).
    let current_limit = tmc_state.current_limit_reg.load(Relaxed);
    backend
        .write_register(PID_TORQUE_FLUX_LIMITS, current_limit)
        .await?;

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::commission_dispatch::DispatchState;
    use crate::commission_dispatch::test_support::{MockBackend, fresh_state_locals};

    fn dispatch_state<'a>(
        calibrated: &'a mut bool,
        motor_enabled: &'a mut bool,
        stored_gains: &'a mut Option<crate::pid::InnerLoopGains>,
    ) -> DispatchState<'a> {
        DispatchState::new(
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

    #[test]
    fn stepper_post_action_zeroes_before_arm_check_directive() {
        use core::sync::atomic::Ordering;
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
        let (mut cal, _, mut gains) = fresh_state_locals();
        let mut en = true;
        stepper_state().position.store(123, Ordering::Relaxed);
        tmc_state().motor_armed.store(false, Ordering::Relaxed);
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);

        let directive = apply_stepper_post_action_state(
            crate::commission_dispatch::StepperPostAction::ArmZeroPosition,
            &mut state,
        );

        assert_eq!(directive, StepperTaskDirective::ArmCheck);
        assert_eq!(stepper_state().position.load(Ordering::Relaxed), 0);
        assert!(tmc_state().motor_armed.load(Ordering::Relaxed));
        tmc_state().motor_armed.store(false, Ordering::Relaxed);
    }

    #[test]
    fn stepper_post_action_stop_clears_before_stop_check_directive() {
        use core::sync::atomic::Ordering;
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
        let (mut cal, mut en, mut gains) = fresh_state_locals();
        tmc_state().motor_armed.store(true, Ordering::Relaxed);
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);

        let directive = apply_stepper_post_action_state(
            crate::commission_dispatch::StepperPostAction::Stop,
            &mut state,
        );

        assert_eq!(directive, StepperTaskDirective::StopCheck);
        assert!(!tmc_state().motor_armed.load(Ordering::Relaxed));
    }

    #[test]
    fn restore_position_mode_after_outer_writes_six_registers() {
        pollster::block_on(async {
            use crate::commission_dispatch::test_support::SpiOp;
            use foci_tmc4671::registers::*;
            let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
                .lock()
                .await;
            let mut backend = MockBackend::new();
            tmc_state()
                .current_limit_reg
                .store(0x0010_0010, core::sync::atomic::Ordering::Relaxed);
            tmc_state()
                .step_width_value
                .store(50, core::sync::atomic::Ordering::Relaxed);
            // Pre-seed ABN_DECODER_COUNT for the position-target compute.
            let _ = backend.write_register(ABN_DECODER_COUNT, 1234).await;
            backend.spi_log.clear();

            let outer = sample_outer_result();
            let result = restore_position_mode_after_outer(&mut backend, tmc_state(), &outer).await;
            assert!(result.is_ok());

            let writes: heapless::Vec<u8, 32> = backend
                .spi_log
                .iter()
                .filter_map(|op| {
                    if let SpiOp::Write(a, _) = op {
                        Some(*a)
                    } else {
                        None
                    }
                })
                .collect();
            assert!(writes.contains(&PID_VELOCITY_P_VELOCITY_I.addr()));
            assert!(writes.contains(&PID_POSITION_P_POSITION_I.addr()));
            assert!(writes.contains(&PID_VELOCITY_LIMIT.addr()));
            assert!(writes.contains(&PID_TORQUE_FLUX_LIMITS.addr()));
            assert!(writes.contains(&PIDOUT_UQ_UD_LIMITS.addr()));
            assert!(writes.contains(&MODE_RAMP_MODE_MOTION.addr()));

            // Cleanup atomics.
            tmc_state()
                .current_limit_reg
                .store(0, core::sync::atomic::Ordering::Relaxed);
            tmc_state()
                .step_width_value
                .store(0, core::sync::atomic::Ordering::Relaxed);
        });
    }

    #[test]
    fn restore_position_mode_after_outer_propagates_spi_failure() {
        pollster::block_on(async {
            use foci_tmc4671::registers::*;
            let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
                .lock()
                .await;
            let mut backend = MockBackend::new();
            backend.fail_on_write = Some(MODE_RAMP_MODE_MOTION.addr());
            let outer = sample_outer_result();
            let result = restore_position_mode_after_outer(&mut backend, tmc_state(), &outer).await;
            assert!(result.is_err());
        });
    }

    fn sample_outer_result() -> crate::commissioning::types::OuterResult {
        crate::commissioning::types::OuterResult {
            status: crate::autotune::CommissionStatus::Accepted,
            velocity_p: 1000,
            velocity_i: 100,
            velocity_limit: 500_000,
            position_p: 640,
            position_i: 0,
            velocity_filter_hz: 0,
            position_filter_hz: 0,
            flux_filter_hz: 0,
            torque_filter_hz: 0,
            j_eff: 0,
            b_eff: 0,
            warning_code: 0,
            mech_torque_step: 0,
            mech_fwd_accel: 0,
            mech_rev_accel: 0,
            mech_fwd_peak_velocity: 0,
            mech_rev_peak_velocity: 0,
            mech_fwd_travel: 0,
            mech_rev_travel: 0,
            synth_velocity_p: 0,
            synth_velocity_i: 0,
            synth_position_p: 0,
            synth_position_i: 0,
        }
    }

    #[test]
    fn restore_calibrate_gains_writes_all_gains_when_stored_present() {
        pollster::block_on(async {
            use crate::commission_dispatch::test_support::SpiOp;
            use foci_tmc4671::registers::*;
            let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
                .lock()
                .await;
            let mut backend = MockBackend::new();
            let stored = Some(crate::pid::InnerLoopGains {
                flux_p: 100,
                flux_i: 10,
                torque_p: 200,
                torque_i: 20,
            });
            let result = restore_calibrate_gains(&mut backend, tmc_state(), stored).await;
            assert!(result.is_ok());
            let writes: heapless::Vec<u8, 32> = backend
                .spi_log
                .iter()
                .filter_map(|op| {
                    if let SpiOp::Write(a, _) = op {
                        Some(*a)
                    } else {
                        None
                    }
                })
                .collect();
            assert!(writes.contains(&PID_FLUX_P_FLUX_I.addr()));
            assert!(writes.contains(&PID_TORQUE_P_TORQUE_I.addr()));
            assert!(writes.contains(&PID_VELOCITY_P_VELOCITY_I.addr()));
            assert!(writes.contains(&PID_POSITION_P_POSITION_I.addr()));
            assert!(writes.contains(&PID_VELOCITY_LIMIT.addr()));
            assert!(writes.contains(&PID_TORQUE_FLUX_LIMITS.addr()));
        });
    }

    #[test]
    fn restore_calibrate_gains_skips_inner_when_stored_none() {
        pollster::block_on(async {
            use crate::commission_dispatch::test_support::SpiOp;
            use foci_tmc4671::registers::*;
            let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
                .lock()
                .await;
            let mut backend = MockBackend::new();
            let result = restore_calibrate_gains(&mut backend, tmc_state(), None).await;
            assert!(result.is_ok());
            let writes: heapless::Vec<u8, 32> = backend
                .spi_log
                .iter()
                .filter_map(|op| {
                    if let SpiOp::Write(a, _) = op {
                        Some(*a)
                    } else {
                        None
                    }
                })
                .collect();
            assert!(!writes.contains(&PID_FLUX_P_FLUX_I.addr()));
            assert!(writes.contains(&PID_VELOCITY_P_VELOCITY_I.addr()));
        });
    }

    #[test]
    fn pack_outer_gain_registers_places_p_in_upper_half() {
        let (velocity_reg, position_reg) = pack_outer_gain_registers(256, 3, 6656, 4);

        assert_eq!(velocity_reg, 0x1a00_0004);
        assert_eq!(position_reg, 0x0100_0003);
    }

    #[test]
    fn pack_inner_gain_registers_places_p_in_upper_half() {
        let gains = crate::pid::InnerLoopGains {
            flux_p: 707,
            flux_i: 162,
            torque_p: 707,
            torque_i: 162,
        };

        let (flux_reg, torque_reg) = pack_inner_gain_registers(gains);

        assert_eq!(flux_reg, 0x02c3_00a2);
        assert_eq!(torque_reg, 0x02c3_00a2);
    }
}
