//! Fault interlock consulted before any `STATUS_FLAGS` clear and before an
//! enable pin is raised.

use core::sync::atomic::Ordering::Relaxed;

use embedded_hal_async::spi::SpiDevice;
use foci_tmc4671::registers::STATUS_FLAGS;
use foci_tmc4671::spi::Tmc4671Spi;

use crate::autotune::CommissionError;
use crate::commissioning::backend::{CommissionMotorControl, CommissionRegisterIo};
use crate::resistance_identification::{ResistanceBackend, ResistanceError};
use crate::state::lifecycle::FAULT_ACTIVE;
use crate::state::tmc::{TMC_DISABLE_SIGNAL, TmcChannelState};

/// True when a fault is latched in firmware, a disable is pending, or the
/// channel's STATUS pin is asserted.
pub fn fault_interlock_tripped(tmc: &TmcChannelState) -> bool {
    FAULT_ACTIVE.load(Relaxed)
        || TMC_DISABLE_SIGNAL.signaled()
        || tmc.status_pin_asserted.load(Relaxed)
}

/// Clear `STATUS_FLAGS` through a raw SPI driver, refusing when the interlock
/// is tripped. The check and the write have no await between them.
///
/// # Errors
///
/// `Err(())` when the interlock is tripped (nothing is written) or the write
/// fails.
pub async fn clear_status_flags<S>(
    tmc: &mut Tmc4671Spi<S>,
    tmc_state: &TmcChannelState,
) -> Result<(), ()>
where
    S: SpiDevice<u8>,
{
    if fault_interlock_tripped(tmc_state) {
        return Err(());
    }
    tmc.write_register(STATUS_FLAGS, 0).await.map_err(|_| ())
}

/// Clear `STATUS_FLAGS` through a commissioning backend, refusing when the
/// backend reports a tripped interlock.
///
/// # Errors
///
/// [`CommissionError::FaultInterlock`] when tripped (nothing is written);
/// [`CommissionError::Spi`] when the write fails.
pub async fn clear_status_flags_via_backend<B>(backend: &mut B) -> Result<(), CommissionError>
where
    B: CommissionRegisterIo + CommissionMotorControl,
{
    if backend.interlock_tripped() {
        return Err(CommissionError::FaultInterlock);
    }
    backend
        .write_register(STATUS_FLAGS, 0)
        .await
        .map_err(|_| CommissionError::Spi)
}

/// Clear `STATUS_FLAGS` through a resistance-identification backend, refusing
/// when the backend reports a tripped interlock.
///
/// # Errors
///
/// [`ResistanceError::CriticalStatus`] when tripped (nothing is written);
/// [`ResistanceError::BackendIo`] when the write fails.
pub async fn clear_status_flags_via_resistance_backend<B>(
    backend: &mut B,
) -> Result<(), ResistanceError>
where
    B: ResistanceBackend,
{
    if backend.interlock_tripped() {
        return Err(ResistanceError::CriticalStatus);
    }
    backend
        .write_register(STATUS_FLAGS.addr(), 0)
        .await
        .map_err(|_| ResistanceError::BackendIo)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::commission_dispatch::test_support::{MockBackend, SpiOp};
    use foci_firmware_macros::fixture_test;

    fn tmc_state() -> &'static TmcChannelState {
        crate::state::tmc::tmc_channel0()
    }

    fn wrote_status_flags(backend: &MockBackend) -> bool {
        backend
            .spi_log
            .iter()
            .any(|op| matches!(op, SpiOp::Write(addr, 0) if *addr == STATUS_FLAGS.addr()))
    }

    #[fixture_test]
    async fn backend_clear_writes_when_the_interlock_is_clear() {
        let mut backend = MockBackend::new();
        assert_eq!(clear_status_flags_via_backend(&mut backend).await, Ok(()));
        assert!(wrote_status_flags(&backend));
    }

    #[fixture_test]
    async fn backend_clear_refuses_when_the_interlock_is_tripped() {
        let mut backend = MockBackend::new();
        backend.interlock_tripped = true;
        assert_eq!(
            clear_status_flags_via_backend(&mut backend).await,
            Err(CommissionError::FaultInterlock)
        );
        assert!(!wrote_status_flags(&backend));
    }

    #[fixture_test]
    fn interlock_is_clear_when_nothing_is_latched() {
        FAULT_ACTIVE.store(false, Relaxed);
        let _ = TMC_DISABLE_SIGNAL.try_take();
        assert!(!fault_interlock_tripped(tmc_state()));
    }

    #[fixture_test]
    fn interlock_trips_on_fault_active() {
        let _ = TMC_DISABLE_SIGNAL.try_take();
        FAULT_ACTIVE.store(true, Relaxed);
        assert!(fault_interlock_tripped(tmc_state()));
        FAULT_ACTIVE.store(false, Relaxed);
    }

    #[fixture_test]
    fn interlock_trips_on_pending_disable_signal() {
        FAULT_ACTIVE.store(false, Relaxed);
        TMC_DISABLE_SIGNAL.signal(());
        assert!(fault_interlock_tripped(tmc_state()));
        let _ = TMC_DISABLE_SIGNAL.try_take();
    }

    #[fixture_test]
    fn interlock_trips_on_asserted_status_pin() {
        FAULT_ACTIVE.store(false, Relaxed);
        let _ = TMC_DISABLE_SIGNAL.try_take();
        tmc_state().status_pin_asserted.store(true, Relaxed);
        assert!(fault_interlock_tripped(tmc_state()));
    }
}
