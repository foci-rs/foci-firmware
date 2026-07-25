//! Commissioning restore sequences used by terminal dispatch outcomes.

use crate::commissioning::types::CommissioningBackend;
use crate::state::tmc::TmcChannelState;
use foci_tmc4671::pid::{PidIGain, PidPGain};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RestoreError<E> {
    Spi(E),
    InvalidGains,
    PiRepresentation,
    #[cfg_attr(not(feature = "trace-velocity-sweep"), allow(dead_code))]
    VelocityClamp,
}

/// Leave a trace velocity workflow disarmed with its operational current limit restored.
#[cfg(feature = "trace-velocity-sweep")]
pub async fn disarm_and_restore_velocity_limit<B: CommissioningBackend>(
    backend: &mut B,
    state: &mut super::DispatchState<'_>,
    operational_limit: u16,
) -> Result<(), RestoreError<B::Error>> {
    use foci_tmc4671::registers::{PID_ERROR_PID_VELOCITY_ERROR_SUM, PID_VELOCITY_TARGET};

    backend
        .write_register(PID_VELOCITY_TARGET, 0)
        .await
        .map_err(RestoreError::Spi)?;
    backend
        .write_register(
            PID_ERROR_PID_VELOCITY_ERROR_SUM.addr,
            u32::from(PID_ERROR_PID_VELOCITY_ERROR_SUM.sub_addr),
        )
        .await
        .map_err(RestoreError::Spi)?;
    backend
        .write_register(PID_ERROR_PID_VELOCITY_ERROR_SUM.data, 0)
        .await
        .map_err(RestoreError::Spi)?;

    crate::tmc_control::lifecycle::disarm_motor(backend, state).await;

    let mut clamp =
        crate::commissioning::outer::velocity::current_guard::VelocityCurrentClampSession::try_new(
            u32::from(operational_limit),
        )
        .map_err(|_| RestoreError::VelocityClamp)?;
    clamp
        .restore_operational_limit(backend)
        .await
        .map_err(|_| RestoreError::VelocityClamp)
}

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
) -> Result<crate::pid::OuterLoopGains, RestoreError<B::Error>> {
    use core::sync::atomic::Ordering::Relaxed;
    use foci_tmc4671::registers::*;

    crate::tmc_control::pi_representation::verify_pi_representation(backend, tmc_state)
        .await
        .map_err(|_| RestoreError::PiRepresentation)?;

    // Tuned velocity + position gains.
    let gains = crate::pid::OuterLoopGains {
        position_p: PidPGain::new(outer.position_p).map_err(|_| RestoreError::InvalidGains)?,
        position_i: PidIGain::new(outer.position_i).map_err(|_| RestoreError::InvalidGains)?,
        velocity_p: PidPGain::new(outer.velocity_p).map_err(|_| RestoreError::InvalidGains)?,
        velocity_i: PidIGain::new(outer.velocity_i).map_err(|_| RestoreError::InvalidGains)?,
    };
    let (velocity_reg, position_reg) = gains.to_registers();
    backend
        .write_register(PID_VELOCITY_P_VELOCITY_I, velocity_reg)
        .await
        .map_err(RestoreError::Spi)?;
    backend
        .write_register(PID_POSITION_P_POSITION_I, position_reg)
        .await
        .map_err(RestoreError::Spi)?;

    // Operational limits (not safety caps).
    backend
        .write_register(PID_VELOCITY_LIMIT, outer.velocity_limit)
        .await
        .map_err(RestoreError::Spi)?;

    let current_limit = tmc_state.load_current_limit(Relaxed);
    backend
        .write_register(PID_TORQUE_FLUX_LIMITS, current_limit.register_value())
        .await
        .map_err(RestoreError::Spi)?;

    // Safety-net voltage cap - must be written before entering motion mode.
    backend
        .write_register(PIDOUT_UQ_UD_LIMITS, 1024)
        .await
        .map_err(RestoreError::Spi)?;

    // Clear PID error sums before mode switch.
    backend
        .write_register(PID_POSITION_ACTUAL, 0)
        .await
        .map_err(RestoreError::Spi)?;

    let encoder_count = backend
        .read_register(ABN_DECODER_COUNT)
        .await
        .map_err(RestoreError::Spi)?;
    let target_scaled = encoder_count_to_position_target(
        encoder_count,
        backend.mechanical_encoder_quadrature_counts_per_rev(),
    );
    backend
        .write_register(PID_POSITION_TARGET, target_scaled)
        .await
        .map_err(RestoreError::Spi)?;

    // Enter position mode (advanced PID cascade).
    backend
        .write_register(MODE_RAMP_MODE_MOTION, 0x8000_0003)
        .await
        .map_err(RestoreError::Spi)?;

    Ok(gains)
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
) -> Result<(), RestoreError<B::Error>> {
    use core::sync::atomic::Ordering::Relaxed;
    use foci_tmc4671::registers::*;

    crate::tmc_control::pi_representation::verify_pi_representation(backend, tmc_state)
        .await
        .map_err(|_| RestoreError::PiRepresentation)?;

    // Current-loop gains from prior commissioning.
    if let Some(gains) = stored {
        let (flux_reg, torque_reg) = gains.to_registers();
        backend
            .write_register(PID_FLUX_P_FLUX_I, flux_reg)
            .await
            .map_err(RestoreError::Spi)?;
        backend
            .write_register(PID_TORQUE_P_TORQUE_I, torque_reg)
            .await
            .map_err(RestoreError::Spi)?;
    }

    let gains = tmc_state
        .load_outer_gains(Relaxed)
        .map_err(|_| RestoreError::InvalidGains)?;
    let (velocity_reg, position_reg) = gains.to_registers();

    // Velocity gains.
    backend
        .write_register(PID_VELOCITY_P_VELOCITY_I, velocity_reg)
        .await
        .map_err(RestoreError::Spi)?;

    // Position gains.
    backend
        .write_register(PID_POSITION_P_POSITION_I, position_reg)
        .await
        .map_err(RestoreError::Spi)?;

    // Velocity limit.
    let velocity_limit = tmc_state.velocity_limit_cfg.load(Relaxed);
    backend
        .write_register(PID_VELOCITY_LIMIT, velocity_limit)
        .await
        .map_err(RestoreError::Spi)?;

    // Current limit (TMC4671 resets on mode change).
    let current_limit = tmc_state.load_current_limit(Relaxed);
    backend
        .write_register(PID_TORQUE_FLUX_LIMITS, current_limit.register_value())
        .await
        .map_err(RestoreError::Spi)?;

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::commission_dispatch::test_support::{MockBackend, SpiOp};

    fn tmc_state() -> &'static crate::state::tmc::TmcChannelState {
        let state = crate::state::tmc::tmc_channel0();
        state.mark_pi_representation_established();
        state
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
            tmc_state().store_current_limit(
                crate::commission_dispatch::test_support::current_limit(0x10),
                core::sync::atomic::Ordering::Relaxed,
            );
            // Pre-seed ABN_DECODER_COUNT for the position-target compute.
            let _ = backend.write_register(ABN_DECODER_COUNT, 1234).await;
            backend.spi_log.clear();

            let outer = sample_outer_result();
            let restored = restore_position_mode_after_outer(&mut backend, tmc_state(), &outer)
                .await
                .unwrap();

            let (velocity_reg, position_reg) = restored.to_registers();
            assert_eq!(
                write_log(&backend).as_slice(),
                &[
                    (
                        CONFIG_ADVANCED_PI_REPRESENT.addr.addr(),
                        u32::from(CONFIG_ADVANCED_PI_REPRESENT.sub_addr),
                    ),
                    (PID_VELOCITY_P_VELOCITY_I.addr(), velocity_reg),
                    (PID_POSITION_P_POSITION_I.addr(), position_reg),
                    (PID_VELOCITY_LIMIT.addr(), outer.velocity_limit),
                    (PID_TORQUE_FLUX_LIMITS.addr(), 0x10),
                    (PIDOUT_UQ_UD_LIMITS.addr(), 1024),
                    (PID_POSITION_ACTUAL.addr(), 0),
                    (PID_POSITION_TARGET.addr(), 20_218),
                    (MODE_RAMP_MODE_MOTION.addr(), 0x8000_0003),
                ]
            );

            // Cleanup atomics.
            tmc_state().store_current_limit(
                foci_tmc4671::current_limit::PidTorqueFluxLimit::ZERO,
                core::sync::atomic::Ordering::Relaxed,
            );
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
            let outer_gains = crate::pid::test_outer_gains(333, 44, 111, 22);
            tmc_state().store_outer_gains(outer_gains, Relaxed);
            tmc_state().velocity_limit_cfg.store(555_000, Relaxed);
            tmc_state().store_current_limit(
                crate::commission_dispatch::test_support::current_limit(0x30),
                Relaxed,
            );
            let stored_gains = crate::pid::InnerLoopGains {
                flux_p: crate::pid::p_gain(100),
                flux_i: crate::pid::i_gain(10),
                torque_p: crate::pid::p_gain(200),
                torque_i: crate::pid::i_gain(20),
            };
            let (flux_reg, torque_reg) = stored_gains.to_registers();
            let result =
                restore_calibrate_gains(&mut backend, tmc_state(), Some(stored_gains)).await;
            assert!(result.is_ok());
            let (velocity_reg, position_reg) = outer_gains.to_registers();
            assert_eq!(
                write_log(&backend).as_slice(),
                &[
                    (
                        CONFIG_ADVANCED_PI_REPRESENT.addr.addr(),
                        u32::from(CONFIG_ADVANCED_PI_REPRESENT.sub_addr),
                    ),
                    (PID_FLUX_P_FLUX_I.addr(), flux_reg),
                    (PID_TORQUE_P_TORQUE_I.addr(), torque_reg),
                    (PID_VELOCITY_P_VELOCITY_I.addr(), velocity_reg),
                    (PID_POSITION_P_POSITION_I.addr(), position_reg),
                    (PID_VELOCITY_LIMIT.addr(), 555_000),
                    (PID_TORQUE_FLUX_LIMITS.addr(), 0x30),
                ]
            );
            tmc_state().store_outer_gains(crate::pid::DEFAULT_OUTER_GAINS, Relaxed);
            tmc_state().velocity_limit_cfg.store(0, Relaxed);
            tmc_state().store_current_limit(
                foci_tmc4671::current_limit::PidTorqueFluxLimit::ZERO,
                Relaxed,
            );
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
            let outer_gains = crate::pid::test_outer_gains(377, 55, 211, 33);
            tmc_state().store_outer_gains(outer_gains, Relaxed);
            tmc_state().velocity_limit_cfg.store(666_000, Relaxed);
            tmc_state().store_current_limit(
                crate::commission_dispatch::test_support::current_limit(0x50),
                Relaxed,
            );
            let result = restore_calibrate_gains(&mut backend, tmc_state(), None).await;
            assert!(result.is_ok());
            let (velocity_reg, position_reg) = outer_gains.to_registers();
            assert_eq!(
                write_log(&backend).as_slice(),
                &[
                    (
                        CONFIG_ADVANCED_PI_REPRESENT.addr.addr(),
                        u32::from(CONFIG_ADVANCED_PI_REPRESENT.sub_addr),
                    ),
                    (PID_VELOCITY_P_VELOCITY_I.addr(), velocity_reg),
                    (PID_POSITION_P_POSITION_I.addr(), position_reg),
                    (PID_VELOCITY_LIMIT.addr(), 666_000),
                    (PID_TORQUE_FLUX_LIMITS.addr(), 0x50),
                ]
            );
            tmc_state().store_outer_gains(crate::pid::DEFAULT_OUTER_GAINS, Relaxed);
            tmc_state().velocity_limit_cfg.store(0, Relaxed);
            tmc_state().store_current_limit(
                foci_tmc4671::current_limit::PidTorqueFluxLimit::ZERO,
                Relaxed,
            );
        });
    }

    #[test]
    fn restore_calibrate_gains_rejects_corrupt_outer_storage_before_pid_write() {
        pollster::block_on(async {
            let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
                .lock()
                .await;
            let mut backend = MockBackend::new();
            tmc_state().corrupt_outer_gain_for_test(0x8000);

            let result = restore_calibrate_gains(&mut backend, tmc_state(), None).await;

            assert_eq!(result, Err(RestoreError::InvalidGains));
            assert!(!backend.spi_log.iter().any(|op| {
                matches!(
                    op,
                    SpiOp::Write(addr, _)
                        if *addr
                            == foci_tmc4671::registers::PID_VELOCITY_P_VELOCITY_I.addr()
                            || *addr
                                == foci_tmc4671::registers::PID_POSITION_P_POSITION_I.addr()
                )
            }));
            tmc_state().store_outer_gains(
                crate::pid::DEFAULT_OUTER_GAINS,
                core::sync::atomic::Ordering::Relaxed,
            );
        });
    }

    #[test]
    fn restore_calibrate_gains_rejects_pi_representation_mismatch_before_pid_write() {
        pollster::block_on(async {
            let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
                .lock()
                .await;
            let mut backend = MockBackend::new();
            tmc_state().mark_pi_representation_established();
            backend.read_override = Some((
                foci_tmc4671::registers::CONFIG_ADVANCED_PI_REPRESENT
                    .data
                    .addr(),
                0,
            ));

            let result = restore_calibrate_gains(&mut backend, tmc_state(), None).await;

            assert_eq!(result, Err(RestoreError::PiRepresentation));
            assert!(!backend.spi_log.iter().any(|op| {
                matches!(
                    op,
                    SpiOp::Write(addr, _)
                        if *addr
                            == foci_tmc4671::registers::PID_VELOCITY_P_VELOCITY_I.addr()
                            || *addr
                                == foci_tmc4671::registers::PID_POSITION_P_POSITION_I.addr()
                )
            }));
            tmc_state().clear_pi_representation_established();
        });
    }
}
