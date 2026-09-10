//! Fixed TMC4671 proportional/integral representation ownership.

use crate::commissioning::backend::CommissionRegisterIo;
use crate::state::tmc::TmcChannelState;
use embedded_hal_async::spi::SpiDevice;
use foci_tmc4671::registers::{ADVANCED_PI_REPRESENT_P_Q8_8_I_Q4_12, CONFIG_ADVANCED_PI_REPRESENT};
use foci_tmc4671::spi::Tmc4671Spi;

/// Failure to establish or verify the fixed PI representation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum PiRepresentationError {
    /// Startup has not established the required representation.
    NotEstablished,
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

fn require_established(state: &TmcChannelState) -> Result<(), PiRepresentationError> {
    if state.pi_representation_established() {
        Ok(())
    } else {
        Err(PiRepresentationError::NotEstablished)
    }
}

fn require_expected(actual: u32) -> Result<(), PiRepresentationError> {
    if actual == ADVANCED_PI_REPRESENT_P_Q8_8_I_Q4_12 {
        Ok(())
    } else {
        Err(PiRepresentationError::Mismatch { actual })
    }
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
    require_expected(actual)?;
    state.mark_pi_representation_established();
    Ok(())
}

/// One step of `verify_pi_representation`'s two-call sequence, advanced by
/// exactly one raw backend operation per `advance()` call. Unlike
/// `write_register_verified`, this sequence has no retry loop and no
/// driver-level opacity forcing its write and read together, so it splits
/// cleanly into two steps rather than needing an exception.
#[derive(Clone, Copy)]
pub(crate) enum PiRepresentationVerifyStep {
    WriteSelector,
    ReadBack,
}

impl PiRepresentationVerifyStep {
    pub(crate) const fn start() -> Self {
        Self::WriteSelector
    }

    /// Perform the next raw backend operation. Returns `None` while a step
    /// remains; `Some(result)` once verification has succeeded or failed,
    /// mirroring `verify_pi_representation`'s original behavior exactly,
    /// including clearing `pi_representation_established` on any failure.
    pub(crate) async fn advance<B: CommissionRegisterIo>(
        &mut self,
        backend: &mut B,
        state: &TmcChannelState,
    ) -> Option<Result<(), PiRepresentationError>> {
        match self {
            Self::WriteSelector => {
                if let Err(error) = require_established(state) {
                    return Some(Err(error));
                }
                if backend
                    .write_register(
                        CONFIG_ADVANCED_PI_REPRESENT.addr,
                        u32::from(CONFIG_ADVANCED_PI_REPRESENT.sub_addr),
                    )
                    .await
                    .is_err()
                {
                    state.clear_pi_representation_established();
                    return Some(Err(PiRepresentationError::Read));
                }
                *self = Self::ReadBack;
                None
            }
            Self::ReadBack => {
                let result = match backend
                    .read_register(CONFIG_ADVANCED_PI_REPRESENT.data)
                    .await
                {
                    Ok(actual) => require_expected(actual),
                    Err(_) => Err(PiRepresentationError::Read),
                };
                if result.is_err() {
                    state.clear_pi_representation_established();
                }
                Some(result)
            }
        }
    }
}

/// Verify the fixed PI representation through a commissioning backend.
///
/// This verifier never repairs missing or mismatching state.
pub(crate) async fn verify_pi_representation<B: CommissionRegisterIo>(
    backend: &mut B,
    state: &TmcChannelState,
) -> Result<(), PiRepresentationError> {
    let mut step = PiRepresentationVerifyStep::start();
    loop {
        if let Some(result) = step.advance(backend, state).await {
            return result;
        }
    }
}

/// Verify the fixed PI representation through the raw request-path SPI owner.
///
/// This has the same fail-closed semantics as [`verify_pi_representation`].
pub(crate) async fn verify_pi_representation_spi<Spi: SpiDevice<u8>>(
    tmc: &mut Tmc4671Spi<Spi>,
    state: &TmcChannelState,
) -> Result<(), PiRepresentationError> {
    require_established(state)?;
    let result = async {
        tmc.write_register(
            CONFIG_ADVANCED_PI_REPRESENT.addr,
            u32::from(CONFIG_ADVANCED_PI_REPRESENT.sub_addr),
        )
        .await
        .map_err(|_| PiRepresentationError::Read)?;
        let actual = tmc
            .read_register(CONFIG_ADVANCED_PI_REPRESENT.data)
            .await
            .map_err(|_| PiRepresentationError::Read)?;
        require_expected(actual)
    }
    .await;
    if result.is_err() {
        state.clear_pi_representation_established();
    }
    result
}
