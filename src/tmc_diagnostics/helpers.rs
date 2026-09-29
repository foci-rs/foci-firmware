use foci_tmc4671::spi::Tmc4671Spi;

use super::DiagnosticDelay;

pub(crate) const HOST_LIVENESS_POLL_SLICE_MS: u64 = 100;

/// Delay `ms` milliseconds in sliced sub-waits, checking `host_liveness_expired` between
/// slices. Returns `Err(())` as soon as the check reports expired.
pub(crate) async fn checked_delay_ms<D: DiagnosticDelay>(
    delay: &mut D,
    mut host_liveness_expired: Option<&mut dyn FnMut() -> bool>,
    mut ms: u64,
) -> Result<(), ()> {
    fn expired(check: &mut Option<&mut dyn FnMut() -> bool>) -> bool {
        match check {
            Some(check) => check(),
            None => false,
        }
    }

    if expired(&mut host_liveness_expired) {
        return Err(());
    }
    while ms > 0 {
        let slice = ms.min(HOST_LIVENESS_POLL_SLICE_MS);
        delay.delay_ms(slice).await;
        ms -= slice;
        if expired(&mut host_liveness_expired) {
            return Err(());
        }
    }
    Ok(())
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum RunnerFailure {
    Other,
    HostLiveness,
}

/// Split a packed signed upper/lower-half register value into `(upper, lower)`.
pub(crate) fn split_signed_halves(value: u32) -> (i16, i16) {
    (
        crate::tmc::decode_signed_upper_half(value),
        crate::tmc::decode_signed_lower_half(value),
    )
}

/// Pack UQ and UD external voltage values into one `UQ_UD_EXT` register word.
pub fn pack_voltage_ext(uq_ext: i16, ud_ext: i16) -> u32 {
    (u32::from(uq_ext as u16) << 16) | u32::from(ud_ext as u16)
}

pub(crate) async fn write_current_step_sub_register<B>(
    tmc: &mut Tmc4671Spi<B>,
    sub: foci_tmc4671::registers::SubRegister,
    value: u32,
) -> Result<(), ()>
where
    B: embedded_hal_async::spi::SpiDevice<u8>,
{
    tmc.write_register(sub.addr, sub.sub_addr as u32)
        .await
        .map_err(|_| ())?;
    tmc.write_register(sub.data, value).await.map_err(|_| ())
}

pub(crate) async fn clear_current_step_integrators<B>(tmc: &mut Tmc4671Spi<B>) -> Result<(), ()>
where
    B: embedded_hal_async::spi::SpiDevice<u8>,
{
    use foci_tmc4671::registers::{PID_ERROR_PID_FLUX_ERROR_SUM, PID_ERROR_PID_TORQUE_ERROR_SUM};

    write_current_step_sub_register(tmc, PID_ERROR_PID_TORQUE_ERROR_SUM, 0).await?;
    write_current_step_sub_register(tmc, PID_ERROR_PID_FLUX_ERROR_SUM, 0).await
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Default)]
    struct RecordingDelay {
        delays_ms: std::vec::Vec<u64>,
    }

    impl crate::tmc_diagnostics::DiagnosticDelay for RecordingDelay {
        async fn delay_ms(&mut self, ms: u64) {
            self.delays_ms.push(ms);
        }

        fn now_us(&self) -> u64 {
            0
        }
    }

    #[test]
    fn checked_delay_ms_with_no_check_runs_the_full_delay_sliced() {
        pollster::block_on(async {
            let mut delay = RecordingDelay::default();
            let result = checked_delay_ms(&mut delay, None, 250).await;
            assert_eq!(result, Ok(()));
            assert_eq!(delay.delays_ms, std::vec![100, 100, 50]);
        });
    }

    #[test]
    fn checked_delay_ms_expired_before_first_slice_delays_nothing() {
        pollster::block_on(async {
            let mut delay = RecordingDelay::default();
            let mut always_expired = || true;
            let result = checked_delay_ms(&mut delay, Some(&mut always_expired), 250).await;
            assert_eq!(result, Err(()));
            assert!(delay.delays_ms.is_empty());
        });
    }

    #[test]
    fn checked_delay_ms_expired_during_final_slice_returns_err_within_that_slice() {
        pollster::block_on(async {
            let mut delay = RecordingDelay::default();
            let mut calls = 0u32;
            let mut expire_after_final_slice = || {
                calls += 1;
                calls >= 3
            };
            let result =
                checked_delay_ms(&mut delay, Some(&mut expire_after_final_slice), 150).await;
            assert_eq!(result, Err(()));
            assert_eq!(delay.delays_ms, std::vec![100, 50]);
        });
    }
}
