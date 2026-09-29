//! PID hold-settle helpers for the TMC control runtime.

use embedded_hal_async::spi::SpiDevice;
use foci_tmc4671::spi::Tmc4671Spi;

use crate::state::tmc::TmcChannelState;

/// Acceptable residual following error after a completed move, in physical
/// emitted-pulse counts.
pub const HOLD_SETTLE_DEADBAND_COUNTS: u32 = 5;

/// Clear TMC4671 PID error accumulators.
///
/// Configured gains and the active position target are unchanged.
pub async fn clear_pid_error_sums<B>(tmc: &mut Tmc4671Spi<B>) -> Result<(), ()>
where
    B: SpiDevice<u8>,
{
    use foci_tmc4671::registers::*;

    tmc.write_sub_register(PID_ERROR_PID_TORQUE_ERROR_SUM, 0)
        .await
        .map_err(|_| ())?;
    tmc.write_sub_register(PID_ERROR_PID_FLUX_ERROR_SUM, 0)
        .await
        .map_err(|_| ())?;
    tmc.write_sub_register(PID_ERROR_PID_VELOCITY_ERROR_SUM, 0)
        .await
        .map_err(|_| ())?;
    tmc.write_sub_register(PID_ERROR_PID_POSITION_ERROR_SUM, 0)
        .await
        .map_err(|_| ())?;
    Ok(())
}

/// Snap the TMC4671 hold target to the measured actual position when within
/// the settle deadband, then clear all PID error accumulators and STATUS_FLAGS.
///
/// Firmware and Klipper logical step counts are unchanged. Per the TMC4671
/// datasheet, writing `PID_POSITION_ACTUAL` also writes `PID_POSITION_TARGET`.
///
/// # Errors
///
/// `Err(())` when the motion scale is unconfigured, a transaction fails, or
/// the fault interlock is tripped at the `STATUS_FLAGS` clear.
pub async fn settle_hold_and_clear_pid_errors<B>(
    tmc: &mut Tmc4671Spi<B>,
    tmc_state: &TmcChannelState,
) -> Result<(), ()>
where
    B: SpiDevice<u8>,
{
    use foci_tmc4671::registers::*;

    let physical_step_width = tmc_state.physical_step_width().ok_or(())?;
    let target = tmc
        .read_register(PID_POSITION_TARGET)
        .await
        .map_err(|_| ())?;
    let actual = tmc
        .read_register(PID_POSITION_ACTUAL)
        .await
        .map_err(|_| ())?;
    let deadband = physical_step_width.saturating_mul(HOLD_SETTLE_DEADBAND_COUNTS);
    let error = (actual as i32).wrapping_sub(target as i32);
    if error.unsigned_abs() <= deadband {
        tmc.write_register(PID_POSITION_ACTUAL, actual)
            .await
            .map_err(|_| ())?;
    }

    clear_pid_error_sums(tmc).await?;
    crate::tmc_control::interlock::clear_status_flags(tmc, tmc_state).await
}

#[cfg(test)]
mod tests {
    use super::*;
    use foci_firmware_macros::fixture_test;
    use foci_tmc4671::spi::Tmc4671Spi;

    fn tmc_state() -> &'static crate::state::tmc::TmcChannelState {
        crate::state::tmc::tmc_channel0()
    }

    #[derive(Default)]
    struct MockRawSpi {
        writes: std::vec::Vec<std::vec::Vec<u8>>,
        reads: std::collections::VecDeque<u32>,
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    struct MockRawSpiError;

    impl embedded_hal_async::spi::Error for MockRawSpiError {
        fn kind(&self) -> embedded_hal_async::spi::ErrorKind {
            embedded_hal_async::spi::ErrorKind::Other
        }
    }

    impl embedded_hal_async::spi::ErrorType for MockRawSpi {
        type Error = MockRawSpiError;
    }

    impl SpiDevice<u8> for MockRawSpi {
        async fn transaction(
            &mut self,
            operations: &mut [embedded_hal_async::spi::Operation<'_, u8>],
        ) -> Result<(), Self::Error> {
            for op in operations {
                match op {
                    embedded_hal_async::spi::Operation::Read(buf) => {
                        let value = self.reads.pop_front().unwrap_or(0);
                        buf.copy_from_slice(&value.to_be_bytes());
                    }
                    embedded_hal_async::spi::Operation::Write(buf) => {
                        self.writes.push(buf.to_vec())
                    }
                    embedded_hal_async::spi::Operation::Transfer(read, write) => {
                        read.copy_from_slice(write);
                    }
                    embedded_hal_async::spi::Operation::TransferInPlace(_) => {}
                    embedded_hal_async::spi::Operation::DelayNs(_) => {}
                }
            }
            Ok(())
        }
    }

    #[fixture_test]
    async fn settle_hold_uses_configured_physical_step_width() {
        use foci_tmc4671::registers::PID_POSITION_ACTUAL;

        tmc_state()
            .configure_motion_scale(crate::state::stepper::stepper_channel0(), 1_000, 3_200)
            .expect("motion scale should configure");
        let mut raw_spi = MockRawSpi::default();
        raw_spi.reads.push_back(0x0000_bb80);
        raw_spi.reads.push_back(0x0000_bbc0);
        let mut tmc = Tmc4671Spi::new(raw_spi);

        settle_hold_and_clear_pid_errors(&mut tmc, tmc_state())
            .await
            .expect("settle should succeed");

        let raw_spi = tmc.into_inner();
        assert!(
            raw_spi.writes.iter().any(|write| {
                write == &[PID_POSITION_ACTUAL.addr() | 0x80, 0x00, 0x00, 0xbb, 0xc0]
            }),
            "expected PID_POSITION_ACTUAL to be rewritten with the observed actual"
        );

        tmc_state().clear_motion_scale(crate::state::stepper::stepper_channel0());
    }

    #[fixture_test]
    async fn settle_hold_rejects_missing_motion_scale() {
        tmc_state().clear_motion_scale(crate::state::stepper::stepper_channel0());
        let mut tmc = Tmc4671Spi::new(MockRawSpi::default());

        let result = settle_hold_and_clear_pid_errors(&mut tmc, tmc_state()).await;

        assert_eq!(result, Err(()));
        assert!(
            tmc.into_inner().writes.is_empty(),
            "missing scale must not substitute or mutate controller state"
        );
    }

    #[fixture_test]
    async fn settle_clear_skips_status_flags_when_the_interlock_is_tripped() {
        use core::sync::atomic::Ordering::Relaxed;
        use foci_tmc4671::registers::STATUS_FLAGS;

        tmc_state()
            .configure_motion_scale(crate::state::stepper::stepper_channel0(), 1_000, 3_200)
            .expect("motion scale should configure");
        let mut raw_spi = MockRawSpi::default();
        raw_spi.reads.push_back(0x0000_bb80);
        raw_spi.reads.push_back(0x0000_bbc0);
        let mut tmc = Tmc4671Spi::new(raw_spi);
        crate::state::lifecycle::FAULT_ACTIVE.store(true, Relaxed);

        let result = settle_hold_and_clear_pid_errors(&mut tmc, tmc_state()).await;

        crate::state::lifecycle::FAULT_ACTIVE.store(false, Relaxed);
        assert_eq!(result, Err(()));
        let raw_spi = tmc.into_inner();
        assert!(
            !raw_spi
                .writes
                .iter()
                .any(|write| write.first() == Some(&(STATUS_FLAGS.addr() | 0x80))),
            "STATUS_FLAGS must not be written while the interlock is tripped"
        );
        tmc_state().clear_motion_scale(crate::state::stepper::stepper_channel0());
    }
}
