//! Small board-agnostic helpers for TMC control request handling.

use core::sync::atomic::Ordering::Relaxed;

use embedded_hal_async::spi::SpiDevice;
use foci_tmc4671::registers::{
    PID_FLUX_P_FLUX_I, PID_POSITION_P_POSITION_I, PID_TORQUE_P_TORQUE_I, PID_VELOCITY_P_VELOCITY_I,
};
use foci_tmc4671::spi::Tmc4671Spi;

use crate::pid::{InnerLoopGains, OuterLoopGains};
use crate::state::tmc::TmcChannelState;
use crate::tmc::compute_velocity_scale_factor;
use crate::tmc_control::TmcControlRuntime;
use crate::tmc_control::request_writes::write_motion_scale;

/// Result of applying two live gain writes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct LiveGainWriteStatus {
    /// Whether the first register write completed successfully.
    pub first_write_ok: bool,
    /// Whether the second register write completed successfully.
    pub second_write_ok: bool,
}

impl LiveGainWriteStatus {
    /// Return true when both live writes completed successfully.
    pub const fn all_ok(self) -> bool {
        self.first_write_ok && self.second_write_ok
    }
}

/// Recompute runtime velocity scale from the channel's STEP_WIDTH state.
pub fn refresh_velocity_scale_from_state(
    runtime: &mut TmcControlRuntime,
    tmc_state: &TmcChannelState,
    clock_freq: u32,
    pwm_freq_hz: u32,
) {
    runtime.set_velocity_scale(compute_velocity_scale_factor(
        tmc_state.step_width_value.load(Relaxed).unsigned_abs(),
        clock_freq,
        pwm_freq_hz,
    ));
    let rational_scale = crate::state::stepper::stepper_channel(runtime.channel())
        .and_then(|stepper| stepper.planner_steps_per_rev(tmc_state))
        .and_then(|planner_steps_per_rev| {
            tmc_state.encoder_ppr().and_then(|encoder_ppr| {
                crate::motion_scale::MotionScale::new(encoder_ppr, planner_steps_per_rev).ok()
            })
        })
        .and_then(|scale| scale.planner_velocity_scale(clock_freq, pwm_freq_hz));
    runtime.set_planner_velocity_scale(rational_scale);
}

/// Write and publish one validated motion scale as an idle P1 transaction.
///
/// # Errors
///
/// Returns `Err(())` after clearing configured scale state when either SPI write or the
/// board-owned mapper installation fails.
pub async fn apply_motion_scale<Spi, InstallMapper, InstallError>(
    tmc: &mut Tmc4671Spi<Spi>,
    tmc_state: &TmcChannelState,
    stepper_state: &crate::state::stepper::StepperChannelState,
    scale: crate::motion_scale::MotionScale,
    install_mapper: InstallMapper,
) -> Result<(), ()>
where
    Spi: SpiDevice<u8>,
    InstallMapper:
        FnOnce(foci_klipper::pulse_mapper::PulseMapperConfig) -> Result<(), InstallError>,
{
    if write_motion_scale(tmc, scale).await.is_err() {
        tmc_state.clear_motion_scale(stepper_state);
        return Err(());
    }
    let mapper = foci_klipper::pulse_mapper::PulseMapperConfig::new(scale.planner_steps_per_rev())
        .expect("validated motion scale must contain a valid mapper");
    if install_mapper(mapper).is_err() {
        tmc_state.clear_motion_scale(stepper_state);
        return Err(());
    }
    let configured = tmc_state.configure_motion_scale(
        stepper_state,
        scale.encoder_ppr(),
        scale.planner_steps_per_rev(),
    );
    assert!(configured.is_ok(), "validated motion scale must publish");
    Ok(())
}

/// Store inner-loop gains and apply the live flux/torque register writes.
pub async fn apply_live_inner_gains<Spi>(
    runtime: &mut TmcControlRuntime,
    tmc: &mut Tmc4671Spi<Spi>,
    gains: InnerLoopGains,
) -> LiveGainWriteStatus
where
    Spi: SpiDevice<u8>,
{
    runtime.set_stored_gains(gains);
    let (flux_reg, torque_reg) = gains.to_registers();
    let first_write_ok = tmc
        .write_register(PID_FLUX_P_FLUX_I, flux_reg)
        .await
        .is_ok();
    let second_write_ok = tmc
        .write_register(PID_TORQUE_P_TORQUE_I, torque_reg)
        .await
        .is_ok();

    LiveGainWriteStatus {
        first_write_ok,
        second_write_ok,
    }
}

/// Store outer-loop gains and apply the live velocity/position register writes.
pub async fn apply_live_outer_gains<Spi>(
    tmc_state: &TmcChannelState,
    tmc: &mut Tmc4671Spi<Spi>,
    gains: OuterLoopGains,
) -> LiveGainWriteStatus
where
    Spi: SpiDevice<u8>,
{
    tmc_state.position_p_value.store(gains.position_p, Relaxed);
    tmc_state.position_i_value.store(gains.position_i, Relaxed);
    tmc_state.velocity_p_value.store(gains.velocity_p, Relaxed);
    tmc_state.velocity_i_value.store(gains.velocity_i, Relaxed);

    let (velocity_reg, position_reg) = gains.to_registers();
    let first_write_ok = tmc
        .write_register(PID_VELOCITY_P_VELOCITY_I, velocity_reg)
        .await
        .is_ok();
    let second_write_ok = tmc
        .write_register(PID_POSITION_P_POSITION_I, position_reg)
        .await
        .is_ok();

    LiveGainWriteStatus {
        first_write_ok,
        second_write_ok,
    }
}
