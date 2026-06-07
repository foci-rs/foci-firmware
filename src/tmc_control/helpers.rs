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
    let (flux_reg, torque_reg) = crate::tmc_ops::pack_inner_gain_registers(gains);
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

    let (velocity_reg, position_reg) = crate::tmc_ops::pack_outer_gain_registers(
        gains.position_p,
        gains.position_i,
        gains.velocity_p,
        gains.velocity_i,
    );
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
