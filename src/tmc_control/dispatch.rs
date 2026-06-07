//! Shared board-agnostic TMC request dispatch helpers.

use embedded_hal_async::spi::SpiDevice;
use foci_tmc4671::spi::Tmc4671Spi;

use crate::state::tmc::TmcChannelState;
use crate::tmc::TmcRequestKind;

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
) -> Result<SharedTmcDispatch, ()>
where
    Spi: SpiDevice<u8>,
{
    match kind {
        TmcRequestKind::SetCurrent { run_ma } => {
            crate::tmc_ops::apply_set_current(tmc, tmc_state, *run_ma, current_scale_ma_per_lsb)
                .await?;
            Ok(SharedTmcDispatch::Handled)
        }
        TmcRequestKind::SetEncoder { ppr } => {
            crate::tmc_ops::apply_set_encoder(tmc, *ppr).await?;
            Ok(SharedTmcDispatch::Handled)
        }
        TmcRequestKind::SetEncoderDir { invert } => {
            crate::tmc_ops::apply_set_encoder_dir(tmc, tmc_state, *invert).await?;
            Ok(SharedTmcDispatch::Handled)
        }
        TmcRequestKind::SetVoltageLimit { voltage_limit } => {
            crate::tmc_ops::apply_set_voltage_limit(tmc, tmc_state, *voltage_limit).await?;
            Ok(SharedTmcDispatch::Handled)
        }
        TmcRequestKind::SetVelocityFilter { .. }
        | TmcRequestKind::SetTorqueFilter { .. }
        | TmcRequestKind::SetPositionFilter { .. }
        | TmcRequestKind::SetFluxFilter { .. }
        | TmcRequestKind::SetVelocityFeedforward { .. }
        | TmcRequestKind::SetVelocityTransientFeedforward { .. }
        | TmcRequestKind::SetAccelFeedforward { .. }
        | TmcRequestKind::SetDecouplingFeedforward { .. }
        | TmcRequestKind::SetPositionLead { .. }
        | TmcRequestKind::SetPhaseAdvance { .. } => Ok(SharedTmcDispatch::Handled),
        _ => Ok(SharedTmcDispatch::Unhandled),
    }
}
