//! Shared board-agnostic TMC request dispatch helpers.

use embedded_hal_async::spi::SpiDevice;
use foci_tmc4671::spi::Tmc4671Spi;

use crate::state::tmc::TmcChannelState;
use crate::tmc::TmcRequestKind;
use crate::tmc_control::filters::{BiquadFilterTarget, apply_live_filter};
use crate::tmc_control::request_writes::{
    apply_set_current, apply_set_encoder, apply_set_encoder_dir, apply_set_voltage_limit,
};

/// Result of attempting board-agnostic TMC request dispatch.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum SharedTmcDispatch {
    /// The request was fully handled by shared dispatch.
    Handled,
    /// The request needs board-local dispatch.
    Unhandled,
}

/// Apply the board-agnostic subset of TMC requests.
///
/// This helper intentionally does not log, emit replies, validate channels,
/// delay, enter shutdown, select mux devices, or touch enable pins. Board
/// wrappers own that policy and call this only after selecting the correct TMC
/// device and channel state.
pub async fn apply_shared_tmc_request<Spi>(
    kind: &TmcRequestKind,
    tmc: &mut Tmc4671Spi<Spi>,
    tmc_state: &TmcChannelState,
    current_scale_ma_per_lsb: f32,
    pwm_maxcnt: u16,
) -> Result<SharedTmcDispatch, ()>
where
    Spi: SpiDevice<u8>,
{
    match kind {
        TmcRequestKind::SetCurrent { run_ma } => {
            apply_set_current(tmc, tmc_state, *run_ma, current_scale_ma_per_lsb).await?;
            Ok(SharedTmcDispatch::Handled)
        }
        TmcRequestKind::SetEncoder { ppr } => {
            apply_set_encoder(tmc, *ppr).await?;
            Ok(SharedTmcDispatch::Handled)
        }
        TmcRequestKind::SetEncoderDir { invert } => {
            apply_set_encoder_dir(tmc, tmc_state, *invert).await?;
            Ok(SharedTmcDispatch::Handled)
        }
        TmcRequestKind::SetVoltageLimit { voltage_limit } => {
            apply_set_voltage_limit(tmc, tmc_state, *voltage_limit).await?;
            Ok(SharedTmcDispatch::Handled)
        }
        TmcRequestKind::SetVelocityFilter { filter_hz } => {
            apply_live_filter(
                tmc,
                tmc_state,
                BiquadFilterTarget::Velocity,
                *filter_hz,
                pwm_maxcnt,
            )
            .await?;
            Ok(SharedTmcDispatch::Handled)
        }
        TmcRequestKind::SetTorqueFilter { filter_hz } => {
            apply_live_filter(
                tmc,
                tmc_state,
                BiquadFilterTarget::Torque,
                *filter_hz,
                pwm_maxcnt,
            )
            .await?;
            Ok(SharedTmcDispatch::Handled)
        }
        TmcRequestKind::SetPositionFilter { filter_hz } => {
            apply_live_filter(
                tmc,
                tmc_state,
                BiquadFilterTarget::Position,
                *filter_hz,
                pwm_maxcnt,
            )
            .await?;
            Ok(SharedTmcDispatch::Handled)
        }
        TmcRequestKind::SetFluxFilter { filter_hz } => {
            apply_live_filter(
                tmc,
                tmc_state,
                BiquadFilterTarget::Flux,
                *filter_hz,
                pwm_maxcnt,
            )
            .await?;
            Ok(SharedTmcDispatch::Handled)
        }
        TmcRequestKind::SetVelocityFeedforward { .. }
        | TmcRequestKind::SetVelocityTransientFeedforward { .. }
        | TmcRequestKind::SetAccelFeedforward { .. }
        | TmcRequestKind::SetDecouplingFeedforward { .. }
        | TmcRequestKind::SetPositionLead { .. }
        | TmcRequestKind::SetPhaseAdvance { .. } => Ok(SharedTmcDispatch::Handled),
        _ => Ok(SharedTmcDispatch::Unhandled),
    }
}
