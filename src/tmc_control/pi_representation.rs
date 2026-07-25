//! Fixed TMC4671 proportional/integral representation ownership.

use crate::commissioning::backend::CommissionRegisterIo;
use crate::state::tmc::TmcChannelState;
use foci_tmc4671::registers::{ADVANCED_PI_REPRESENT_P_Q8_8_I_Q4_12, CONFIG_ADVANCED_PI_REPRESENT};

/// Failure to establish or verify the fixed PI representation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum PiRepresentationError {
    /// Writing the selector or representation value failed.
    Write,
    /// Selecting or reading the representation value failed.
    Read,
    /// Hardware returned a representation other than the required fixed value.
    Mismatch {
        /// Raw representation value returned by the TMC4671.
        actual: u32,
    },
}

/// Program and verify the fixed PI representation while the motor is disarmed.
///
/// The channel invariant is cleared before any SPI operation and published
/// only after exact readback.
///
/// # Errors
///
/// Returns [`PiRepresentationError`] on any SPI failure or mismatching
/// readback. Failure always leaves the channel invariant cleared.
pub async fn bootstrap_pi_representation<B: CommissionRegisterIo>(
    backend: &mut B,
    state: &TmcChannelState,
) -> Result<(), PiRepresentationError> {
    state.clear_pi_representation_established();
    backend
        .write_register(
            CONFIG_ADVANCED_PI_REPRESENT.addr,
            u32::from(CONFIG_ADVANCED_PI_REPRESENT.sub_addr),
        )
        .await
        .map_err(|_| PiRepresentationError::Write)?;
    backend
        .write_register(
            CONFIG_ADVANCED_PI_REPRESENT.data,
            ADVANCED_PI_REPRESENT_P_Q8_8_I_Q4_12,
        )
        .await
        .map_err(|_| PiRepresentationError::Write)?;
    backend
        .write_register(
            CONFIG_ADVANCED_PI_REPRESENT.addr,
            u32::from(CONFIG_ADVANCED_PI_REPRESENT.sub_addr),
        )
        .await
        .map_err(|_| PiRepresentationError::Read)?;
    let actual = backend
        .read_register(CONFIG_ADVANCED_PI_REPRESENT.data)
        .await
        .map_err(|_| PiRepresentationError::Read)?;
    if actual != ADVANCED_PI_REPRESENT_P_Q8_8_I_Q4_12 {
        return Err(PiRepresentationError::Mismatch { actual });
    }
    state.mark_pi_representation_established();
    Ok(())
}
