//! Commissioning restore sequences used by terminal dispatch outcomes.

use crate::commissioning::types::CommissioningBackend;
use crate::state::tmc::TmcChannelState;

/// SPI restore sequence for the OuterDone success path.
///
/// Writes tuned velocity and position gains, the operational velocity limit,
/// current limit from channel state, the safety-net `PIDOUT_UQ_UD_LIMITS` cap
/// (1024 counts), clears the position-actual integrator, snaps the position
/// target to the current encoder count scaled by the encoder resolution, and
/// enters position mode via `MODE_RAMP_MODE_MOTION = 0x8000_0003`.
///
/// The write order matches the OuterDone restore block in `main.rs` exactly.
/// Motor safety depends on setting limits (`PID_TORQUE_FLUX_LIMITS`,
/// `PIDOUT_UQ_UD_LIMITS`) before entering motion mode.
///
/// # Parameters
///
/// - `backend` - mutable reference to a `CommissioningBackend` implementation.
/// - `outer` - outer commissioning result containing tuned gain values.
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
    let gains = crate::pid::OuterLoopGains {
        position_p: outer.position_p,
        position_i: outer.position_i,
        velocity_p: outer.velocity_p,
        velocity_i: outer.velocity_i,
    };
    let (velocity_reg, position_reg) = gains.to_registers();
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

    // Safety-net voltage cap - must be written before entering motion mode.
    backend.write_register(PIDOUT_UQ_UD_LIMITS, 1024).await?;

    // Clear PID error sums before mode switch.
    backend.write_register(PID_POSITION_ACTUAL, 0).await?;

    let encoder_count = backend.read_register(ABN_DECODER_COUNT).await?;
    let target_scaled =
        encoder_count_to_position_target(encoder_count, backend.encoder_counts_per_rev());
    backend
        .write_register(PID_POSITION_TARGET, target_scaled)
        .await?;

    // Enter position mode (advanced PID cascade).
    backend
        .write_register(MODE_RAMP_MODE_MOTION, 0x8000_0003)
        .await?;

    Ok(())
}

fn encoder_count_to_position_target(encoder_count: u32, encoder_counts_per_rev: u32) -> u32 {
    assert!(
        encoder_counts_per_rev != 0,
        "encoder scale must be configured"
    );

    let scaled = i64::from(encoder_count as i32) * 65_536;
    let denominator = i64::from(encoder_counts_per_rev);
    let rounding = denominator / 2;
    let rounded = if scaled < 0 {
        (scaled - rounding) / denominator
    } else {
        (scaled + rounding) / denominator
    };
    rounded as u32
}

/// SPI restore sequence for the CalibrateDone path.
///
/// Reapplies host-stored inner gains (if any) to `PID_FLUX_P_FLUX_I` and
/// `PID_TORQUE_P_TORQUE_I`, then reads outer gain values from the
/// [`crate::state::tmc`] atomics and writes velocity gains, position gains,
/// velocity limit, and current limit back to the TMC4671.
///
/// The write order matches the CalibrateDone gain-restore block in `main.rs`
/// exactly. SPI failures abort immediately; reporting success with wrong gains
/// active is worse than failing.
///
/// # Parameters
///
/// - `backend` - mutable reference to a `CommissioningBackend` implementation.
/// - `stored` - optional host-stored inner loop gains; skipped when `None`.
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
        let (flux_reg, torque_reg) = gains.to_registers();
        backend.write_register(PID_FLUX_P_FLUX_I, flux_reg).await?;
        backend
            .write_register(PID_TORQUE_P_TORQUE_I, torque_reg)
            .await?;
    }

    let velocity_p = tmc_state.velocity_p_value.load(Relaxed);
    let velocity_i = tmc_state.velocity_i_value.load(Relaxed);
    let position_p = tmc_state.position_p_value.load(Relaxed);
    let position_i = tmc_state.position_i_value.load(Relaxed);
    let gains = crate::pid::OuterLoopGains {
        position_p,
        position_i,
        velocity_p,
        velocity_i,
    };
    let (velocity_reg, position_reg) = gains.to_registers();

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
    use crate::commission_dispatch::test_support::{MockBackend, SpiOp};

    fn tmc_state() -> &'static crate::state::tmc::TmcChannelState {
        crate::state::tmc::tmc_channel0()
    }

    fn write_log(backend: &MockBackend) -> heapless::Vec<(u8, u32), 128> {
        let write_count = backend
            .spi_log
            .iter()
            .filter(|op| matches!(op, SpiOp::Write(_, _)))
            .count();
        let writes = backend
            .spi_log
            .iter()
            .filter_map(|op| {
                if let SpiOp::Write(addr, val) = op {
                    Some((*addr, *val))
                } else {
                    None
                }
            })
            .collect::<heapless::Vec<_, 128>>();
        assert_eq!(
            writes.len(),
            write_count,
            "write_log helper capacity truncated the SPI log"
        );
        writes
    }

    #[test]
    fn restore_position_mode_after_outer_writes_exact_sequence() {
        pollster::block_on(async {
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

            let (velocity_reg, position_reg) = crate::pid::OuterLoopGains {
                position_p: outer.position_p,
                position_i: outer.position_i,
                velocity_p: outer.velocity_p,
                velocity_i: outer.velocity_i,
            }
            .to_registers();
            assert_eq!(
                write_log(&backend).as_slice(),
                &[
                    (PID_VELOCITY_P_VELOCITY_I.addr(), velocity_reg),
                    (PID_POSITION_P_POSITION_I.addr(), position_reg),
                    (PID_VELOCITY_LIMIT.addr(), outer.velocity_limit),
                    (PID_TORQUE_FLUX_LIMITS.addr(), 0x0010_0010),
                    (PIDOUT_UQ_UD_LIMITS.addr(), 1024),
                    (PID_POSITION_ACTUAL.addr(), 0),
                    (PID_POSITION_TARGET.addr(), 20_218),
                    (MODE_RAMP_MODE_MOTION.addr(), 0x8000_0003),
                ]
            );

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
            outer_evidence_flags: 0,
            stiffness_timebase_ms: 0,
            velocity_search_stop_reason: 0,
            motion_budget_mrev: 0,
        }
    }

    #[test]
    fn restore_calibrate_gains_writes_all_gains_when_stored_present() {
        pollster::block_on(async {
            use core::sync::atomic::Ordering::Relaxed;
            use foci_tmc4671::registers::*;
            let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
                .lock()
                .await;
            let mut backend = MockBackend::new();
            tmc_state().velocity_p_value.store(111, Relaxed);
            tmc_state().velocity_i_value.store(22, Relaxed);
            tmc_state().position_p_value.store(333, Relaxed);
            tmc_state().position_i_value.store(44, Relaxed);
            tmc_state().velocity_limit_cfg.store(555_000, Relaxed);
            tmc_state().current_limit_reg.store(0x0020_0030, Relaxed);
            let stored_gains = crate::pid::InnerLoopGains {
                flux_p: 100,
                flux_i: 10,
                torque_p: 200,
                torque_i: 20,
            };
            let (flux_reg, torque_reg) = stored_gains.to_registers();
            let result =
                restore_calibrate_gains(&mut backend, tmc_state(), Some(stored_gains)).await;
            assert!(result.is_ok());
            let (velocity_reg, position_reg) = crate::pid::OuterLoopGains {
                position_p: 333,
                position_i: 44,
                velocity_p: 111,
                velocity_i: 22,
            }
            .to_registers();
            assert_eq!(
                write_log(&backend).as_slice(),
                &[
                    (PID_FLUX_P_FLUX_I.addr(), flux_reg),
                    (PID_TORQUE_P_TORQUE_I.addr(), torque_reg),
                    (PID_VELOCITY_P_VELOCITY_I.addr(), velocity_reg),
                    (PID_POSITION_P_POSITION_I.addr(), position_reg),
                    (PID_VELOCITY_LIMIT.addr(), 555_000),
                    (PID_TORQUE_FLUX_LIMITS.addr(), 0x0020_0030),
                ]
            );
            tmc_state().velocity_p_value.store(0, Relaxed);
            tmc_state().velocity_i_value.store(0, Relaxed);
            tmc_state().position_p_value.store(0, Relaxed);
            tmc_state().position_i_value.store(0, Relaxed);
            tmc_state().velocity_limit_cfg.store(0, Relaxed);
            tmc_state().current_limit_reg.store(0, Relaxed);
        });
    }

    #[test]
    fn restore_calibrate_gains_skips_inner_when_stored_none() {
        pollster::block_on(async {
            use core::sync::atomic::Ordering::Relaxed;
            use foci_tmc4671::registers::*;
            let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
                .lock()
                .await;
            let mut backend = MockBackend::new();
            tmc_state().velocity_p_value.store(211, Relaxed);
            tmc_state().velocity_i_value.store(33, Relaxed);
            tmc_state().position_p_value.store(377, Relaxed);
            tmc_state().position_i_value.store(55, Relaxed);
            tmc_state().velocity_limit_cfg.store(666_000, Relaxed);
            tmc_state().current_limit_reg.store(0x0040_0050, Relaxed);
            let result = restore_calibrate_gains(&mut backend, tmc_state(), None).await;
            assert!(result.is_ok());
            let (velocity_reg, position_reg) = crate::pid::OuterLoopGains {
                position_p: 377,
                position_i: 55,
                velocity_p: 211,
                velocity_i: 33,
            }
            .to_registers();
            assert_eq!(
                write_log(&backend).as_slice(),
                &[
                    (PID_VELOCITY_P_VELOCITY_I.addr(), velocity_reg),
                    (PID_POSITION_P_POSITION_I.addr(), position_reg),
                    (PID_VELOCITY_LIMIT.addr(), 666_000),
                    (PID_TORQUE_FLUX_LIMITS.addr(), 0x0040_0050),
                ]
            );
            tmc_state().velocity_p_value.store(0, Relaxed);
            tmc_state().velocity_i_value.store(0, Relaxed);
            tmc_state().position_p_value.store(0, Relaxed);
            tmc_state().position_i_value.store(0, Relaxed);
            tmc_state().velocity_limit_cfg.store(0, Relaxed);
            tmc_state().current_limit_reg.store(0, Relaxed);
        });
    }
}
