//! Stored biquad-filter restore helpers for TMC control state.

use embedded_hal_async::spi::SpiDevice;
use foci_tmc4671::registers::SubRegister;
use foci_tmc4671::spi::{SpiError, Tmc4671Spi};

use crate::commissioning::types::CommissioningBackend;
use crate::state::tmc::TmcChannelState;

/// TMC4671 biquad feedback filter selected by a live filter request.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BiquadFilterTarget {
    /// Velocity feedback filter.
    Velocity,
    /// Torque-current feedback filter.
    Torque,
    /// Position feedback filter.
    Position,
    /// Flux-current feedback filter.
    Flux,
}

#[derive(Clone, Copy)]
struct BiquadFilterSubRegs {
    a_1: SubRegister,
    a_2: SubRegister,
    b_0: SubRegister,
    b_1: SubRegister,
    b_2: SubRegister,
    enable: SubRegister,
}

fn biquad_filter_regs(target: BiquadFilterTarget) -> BiquadFilterSubRegs {
    use foci_tmc4671::registers::*;

    match target {
        BiquadFilterTarget::Velocity => BiquadFilterSubRegs {
            a_1: CONFIG_BIQUAD_V_A_1,
            a_2: CONFIG_BIQUAD_V_A_2,
            b_0: CONFIG_BIQUAD_V_B_0,
            b_1: CONFIG_BIQUAD_V_B_1,
            b_2: CONFIG_BIQUAD_V_B_2,
            enable: CONFIG_BIQUAD_V_ENABLE,
        },
        BiquadFilterTarget::Torque => BiquadFilterSubRegs {
            a_1: CONFIG_BIQUAD_T_A_1,
            a_2: CONFIG_BIQUAD_T_A_2,
            b_0: CONFIG_BIQUAD_T_B_0,
            b_1: CONFIG_BIQUAD_T_B_1,
            b_2: CONFIG_BIQUAD_T_B_2,
            enable: CONFIG_BIQUAD_T_ENABLE,
        },
        BiquadFilterTarget::Position => BiquadFilterSubRegs {
            a_1: CONFIG_BIQUAD_X_A_1,
            a_2: CONFIG_BIQUAD_X_A_2,
            b_0: CONFIG_BIQUAD_X_B_0,
            b_1: CONFIG_BIQUAD_X_B_1,
            b_2: CONFIG_BIQUAD_X_B_2,
            enable: CONFIG_BIQUAD_X_ENABLE,
        },
        BiquadFilterTarget::Flux => BiquadFilterSubRegs {
            a_1: CONFIG_BIQUAD_F_A_1,
            a_2: CONFIG_BIQUAD_F_A_2,
            b_0: CONFIG_BIQUAD_F_B_0,
            b_1: CONFIG_BIQUAD_F_B_1,
            b_2: CONFIG_BIQUAD_F_B_2,
            enable: CONFIG_BIQUAD_F_ENABLE,
        },
    }
}

fn store_filter_hz(tmc_state: &TmcChannelState, target: BiquadFilterTarget, filter_hz: u16) {
    use core::sync::atomic::Ordering::Relaxed;

    match target {
        BiquadFilterTarget::Velocity => {
            tmc_state.velocity_filter_hz.store(Some(filter_hz), Relaxed)
        }
        BiquadFilterTarget::Torque => tmc_state.torque_filter_hz.store(Some(filter_hz), Relaxed),
        BiquadFilterTarget::Position => {
            tmc_state.position_filter_hz.store(Some(filter_hz), Relaxed)
        }
        BiquadFilterTarget::Flux => tmc_state.flux_filter_hz.store(Some(filter_hz), Relaxed),
    }
}

/// Apply one live biquad low-pass filter request.
///
/// A cutoff of `0` disables the selected filter. Non-zero cutoffs are
/// validated by [`crate::biquad::butterworth_lowpass`] and written before
/// the cached filter state is updated.
///
/// # Errors
///
/// Returns `Err(())` if the cutoff is invalid or if any SPI write fails.
pub async fn apply_live_filter<B>(
    tmc: &mut Tmc4671Spi<B>,
    tmc_state: &TmcChannelState,
    target: BiquadFilterTarget,
    filter_hz: u16,
    pwm_maxcnt: u16,
) -> Result<(), ()>
where
    B: SpiDevice<u8>,
{
    let regs = biquad_filter_regs(target);
    if filter_hz == 0 {
        tmc.write_sub_register(regs.enable, 0)
            .await
            .map_err(|_| ())?;
        store_filter_hz(tmc_state, target, 0);
        return Ok(());
    }

    let Some(coeffs) = crate::biquad::butterworth_lowpass(filter_hz, pwm_maxcnt) else {
        defmt::error!("Invalid biquad filter cutoff: {} Hz", filter_hz);
        return Err(());
    };

    tmc.write_sub_register(regs.a_1, coeffs.a_1 as u32)
        .await
        .map_err(|_| ())?;
    tmc.write_sub_register(regs.a_2, coeffs.a_2 as u32)
        .await
        .map_err(|_| ())?;
    tmc.write_sub_register(regs.b_0, coeffs.b_0 as u32)
        .await
        .map_err(|_| ())?;
    tmc.write_sub_register(regs.b_1, coeffs.b_1 as u32)
        .await
        .map_err(|_| ())?;
    tmc.write_sub_register(regs.b_2, coeffs.b_2 as u32)
        .await
        .map_err(|_| ())?;
    tmc.write_sub_register(regs.enable, 1)
        .await
        .map_err(|_| ())?;
    store_filter_hz(tmc_state, target, filter_hz);
    Ok(())
}

/// Re-apply biquad filters from stored atomics.
///
/// Called after `CalibrateDone` to restore filters that the commissioning
/// engine's `ClosedLoopEntry` phase disabled (it has no access to the filter
/// Hz atomics).
///
/// `pwm_maxcnt` is board-specific and must be supplied by the caller
/// (e.g. `TMC_CONFIG.pwm_maxcnt`).
///
/// # Errors
///
/// Returns the first SPI write error encountered while restoring a filter.
pub async fn apply_stored_filters<B>(
    tmc: &mut Tmc4671Spi<B>,
    tmc_state: &TmcChannelState,
    pwm_maxcnt: u16,
) -> Result<(), SpiError<B::Error>>
where
    B: SpiDevice<u8>,
{
    use core::sync::atomic::Ordering::Relaxed;
    use foci_tmc4671::registers::{SubRegister, *};

    struct BiquadFilterSubRegs {
        a_1: SubRegister,
        a_2: SubRegister,
        b_0: SubRegister,
        b_1: SubRegister,
        b_2: SubRegister,
        enable: SubRegister,
    }

    async fn apply_one<B>(
        tmc: &mut Tmc4671Spi<B>,
        filter_hz: u16,
        pwm_maxcnt: u16,
        regs: BiquadFilterSubRegs,
    ) -> Result<(), SpiError<B::Error>>
    where
        B: SpiDevice<u8>,
    {
        if filter_hz == 0 {
            return Ok(());
        }
        let Some(coeffs) = crate::biquad::butterworth_lowpass(filter_hz, pwm_maxcnt) else {
            return Ok(());
        };

        tmc.write_sub_register(regs.a_1, coeffs.a_1 as u32).await?;
        tmc.write_sub_register(regs.a_2, coeffs.a_2 as u32).await?;
        tmc.write_sub_register(regs.b_0, coeffs.b_0 as u32).await?;
        tmc.write_sub_register(regs.b_1, coeffs.b_1 as u32).await?;
        tmc.write_sub_register(regs.b_2, coeffs.b_2 as u32).await?;
        tmc.write_sub_register(regs.enable, 1).await?;
        Ok(())
    }

    apply_one(
        tmc,
        tmc_state.velocity_filter_hz.load_or_zero(Relaxed),
        pwm_maxcnt,
        BiquadFilterSubRegs {
            a_1: CONFIG_BIQUAD_V_A_1,
            a_2: CONFIG_BIQUAD_V_A_2,
            b_0: CONFIG_BIQUAD_V_B_0,
            b_1: CONFIG_BIQUAD_V_B_1,
            b_2: CONFIG_BIQUAD_V_B_2,
            enable: CONFIG_BIQUAD_V_ENABLE,
        },
    )
    .await?;
    apply_one(
        tmc,
        tmc_state.torque_filter_hz.load_or_zero(Relaxed),
        pwm_maxcnt,
        BiquadFilterSubRegs {
            a_1: CONFIG_BIQUAD_T_A_1,
            a_2: CONFIG_BIQUAD_T_A_2,
            b_0: CONFIG_BIQUAD_T_B_0,
            b_1: CONFIG_BIQUAD_T_B_1,
            b_2: CONFIG_BIQUAD_T_B_2,
            enable: CONFIG_BIQUAD_T_ENABLE,
        },
    )
    .await?;
    apply_one(
        tmc,
        tmc_state.position_filter_hz.load_or_zero(Relaxed),
        pwm_maxcnt,
        BiquadFilterSubRegs {
            a_1: CONFIG_BIQUAD_X_A_1,
            a_2: CONFIG_BIQUAD_X_A_2,
            b_0: CONFIG_BIQUAD_X_B_0,
            b_1: CONFIG_BIQUAD_X_B_1,
            b_2: CONFIG_BIQUAD_X_B_2,
            enable: CONFIG_BIQUAD_X_ENABLE,
        },
    )
    .await?;
    apply_one(
        tmc,
        tmc_state.flux_filter_hz.load_or_zero(Relaxed),
        pwm_maxcnt,
        BiquadFilterSubRegs {
            a_1: CONFIG_BIQUAD_F_A_1,
            a_2: CONFIG_BIQUAD_F_A_2,
            b_0: CONFIG_BIQUAD_F_B_0,
            b_1: CONFIG_BIQUAD_F_B_1,
            b_2: CONFIG_BIQUAD_F_B_2,
            enable: CONFIG_BIQUAD_F_ENABLE,
        },
    )
    .await?;

    Ok(())
}

/// Apply biquad filters via `CommissioningBackend`.
///
/// Mirror of [`apply_stored_filters`] (which uses raw `Tmc4671Spi<B>`), but
/// routes all writes through the backend trait so dispatch can call it without
/// holding a raw SPI device.
///
/// Each `SubRegister` write reproduces the addr-then-data sequence that
/// `Tmc4671Spi::write_sub_register` does internally: write `sub_addr` to the
/// `addr` register, then write the value to the `data` register.
///
/// # Parameters
///
/// - `backend` — mutable reference to a `CommissioningBackend` implementation.
/// - `pwm_maxcnt` — board-specific PWM counter maximum, used to derive
///   the biquad sampling frequency for coefficient calculation.
pub async fn apply_stored_filters_via_backend<B: CommissioningBackend>(
    backend: &mut B,
    tmc_state: &TmcChannelState,
    pwm_maxcnt: u16,
) -> Result<(), B::Error> {
    use core::sync::atomic::Ordering::Relaxed;
    use foci_tmc4671::registers::{SubRegister, *};

    /// The six SubRegister addresses that make up one biquad channel.
    struct BiquadFilterSubRegs {
        a_1: SubRegister,
        a_2: SubRegister,
        b_0: SubRegister,
        b_1: SubRegister,
        b_2: SubRegister,
        enable: SubRegister,
    }

    async fn write_sub<B: CommissioningBackend>(
        backend: &mut B,
        sub: SubRegister,
        val: u32,
    ) -> Result<(), B::Error> {
        backend
            .write_register(sub.addr, sub.sub_addr as u32)
            .await?;
        backend.write_register(sub.data, val).await?;
        Ok(())
    }

    async fn apply_one<B: CommissioningBackend>(
        backend: &mut B,
        filter_hz: u16,
        pwm_maxcnt: u16,
        regs: BiquadFilterSubRegs,
    ) -> Result<(), B::Error> {
        if filter_hz == 0 {
            return Ok(());
        }
        let Some(coeffs) = crate::biquad::butterworth_lowpass(filter_hz, pwm_maxcnt) else {
            return Ok(());
        };
        write_sub(backend, regs.a_1, coeffs.a_1 as u32).await?;
        write_sub(backend, regs.a_2, coeffs.a_2 as u32).await?;
        write_sub(backend, regs.b_0, coeffs.b_0 as u32).await?;
        write_sub(backend, regs.b_1, coeffs.b_1 as u32).await?;
        write_sub(backend, regs.b_2, coeffs.b_2 as u32).await?;
        write_sub(backend, regs.enable, 1).await?;
        Ok(())
    }

    apply_one(
        backend,
        tmc_state.velocity_filter_hz.load_or_zero(Relaxed),
        pwm_maxcnt,
        BiquadFilterSubRegs {
            a_1: CONFIG_BIQUAD_V_A_1,
            a_2: CONFIG_BIQUAD_V_A_2,
            b_0: CONFIG_BIQUAD_V_B_0,
            b_1: CONFIG_BIQUAD_V_B_1,
            b_2: CONFIG_BIQUAD_V_B_2,
            enable: CONFIG_BIQUAD_V_ENABLE,
        },
    )
    .await?;
    apply_one(
        backend,
        tmc_state.torque_filter_hz.load_or_zero(Relaxed),
        pwm_maxcnt,
        BiquadFilterSubRegs {
            a_1: CONFIG_BIQUAD_T_A_1,
            a_2: CONFIG_BIQUAD_T_A_2,
            b_0: CONFIG_BIQUAD_T_B_0,
            b_1: CONFIG_BIQUAD_T_B_1,
            b_2: CONFIG_BIQUAD_T_B_2,
            enable: CONFIG_BIQUAD_T_ENABLE,
        },
    )
    .await?;
    apply_one(
        backend,
        tmc_state.position_filter_hz.load_or_zero(Relaxed),
        pwm_maxcnt,
        BiquadFilterSubRegs {
            a_1: CONFIG_BIQUAD_X_A_1,
            a_2: CONFIG_BIQUAD_X_A_2,
            b_0: CONFIG_BIQUAD_X_B_0,
            b_1: CONFIG_BIQUAD_X_B_1,
            b_2: CONFIG_BIQUAD_X_B_2,
            enable: CONFIG_BIQUAD_X_ENABLE,
        },
    )
    .await?;
    apply_one(
        backend,
        tmc_state.flux_filter_hz.load_or_zero(Relaxed),
        pwm_maxcnt,
        BiquadFilterSubRegs {
            a_1: CONFIG_BIQUAD_F_A_1,
            a_2: CONFIG_BIQUAD_F_A_2,
            b_0: CONFIG_BIQUAD_F_B_0,
            b_1: CONFIG_BIQUAD_F_B_1,
            b_2: CONFIG_BIQUAD_F_B_2,
            enable: CONFIG_BIQUAD_F_ENABLE,
        },
    )
    .await?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use foci_tmc4671::spi::Tmc4671Spi;

    fn tmc_state() -> &'static crate::state::tmc::TmcChannelState {
        crate::state::tmc::tmc_channel0()
    }

    #[derive(Default)]
    struct MockRawSpi {
        writes: std::vec::Vec<std::vec::Vec<u8>>,
        fail_write_after: Option<usize>,
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
                        buf.copy_from_slice(&0u32.to_be_bytes());
                    }
                    embedded_hal_async::spi::Operation::Write(buf) => {
                        if self
                            .fail_write_after
                            .is_some_and(|limit| self.writes.len() >= limit)
                        {
                            return Err(MockRawSpiError);
                        }
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

    #[test]
    fn apply_stored_filters_via_backend_writes_enabled_filters() {
        pollster::block_on(async {
            use core::sync::atomic::Ordering;
            let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
                .lock()
                .await;
            tmc_state()
                .velocity_filter_hz
                .store(Some(100), Ordering::Relaxed);
            tmc_state()
                .torque_filter_hz
                .store(Some(0), Ordering::Relaxed);
            tmc_state()
                .position_filter_hz
                .store(Some(0), Ordering::Relaxed);
            tmc_state().flux_filter_hz.store(Some(0), Ordering::Relaxed);

            let mut backend = crate::commission_dispatch::test_support::MockBackend::new();
            apply_stored_filters_via_backend(&mut backend, tmc_state(), 4095)
                .await
                .expect("filter writes should succeed");

            use crate::commission_dispatch::test_support::SpiOp;
            use foci_tmc4671::registers::*;
            let writes: std::vec::Vec<(u8, u32)> = backend
                .spi_log
                .iter()
                .filter_map(|op| {
                    if let SpiOp::Write(addr, val) = op {
                        Some((*addr, *val))
                    } else {
                        None
                    }
                })
                .collect();

            let has_sub_write = |sub: foci_tmc4671::registers::SubRegister, data_val: u32| {
                writes.windows(2).any(|pair| {
                    pair[0].0 == sub.addr.addr()
                        && pair[0].1 == sub.sub_addr as u32
                        && pair[1].0 == sub.data.addr()
                        && pair[1].1 == data_val
                })
            };

            assert!(
                has_sub_write(CONFIG_BIQUAD_V_ENABLE, 1),
                "velocity filter enable=1 write pair not found"
            );
            assert!(
                !has_sub_write(CONFIG_BIQUAD_T_ENABLE, 1),
                "torque filter should not have been enabled"
            );
            assert!(
                !has_sub_write(CONFIG_BIQUAD_X_ENABLE, 1),
                "position filter should not have been enabled"
            );
            assert!(
                !has_sub_write(CONFIG_BIQUAD_F_ENABLE, 1),
                "flux filter should not have been enabled"
            );

            tmc_state()
                .velocity_filter_hz
                .store(None, Ordering::Relaxed);
        });
    }

    #[test]
    fn apply_stored_filters_reports_raw_spi_write_failure() {
        pollster::block_on(async {
            use core::sync::atomic::Ordering;
            let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
                .lock()
                .await;
            tmc_state()
                .velocity_filter_hz
                .store(Some(100), Ordering::Relaxed);
            tmc_state()
                .torque_filter_hz
                .store(Some(0), Ordering::Relaxed);
            tmc_state()
                .position_filter_hz
                .store(Some(0), Ordering::Relaxed);
            tmc_state().flux_filter_hz.store(Some(0), Ordering::Relaxed);

            let raw_spi = MockRawSpi {
                fail_write_after: Some(1),
                ..MockRawSpi::default()
            };
            let mut tmc = Tmc4671Spi::new(raw_spi);

            let result = apply_stored_filters(&mut tmc, tmc_state(), 4095).await;

            assert!(
                result.is_err(),
                "raw stored-filter restore should report SPI write failures"
            );

            tmc_state()
                .velocity_filter_hz
                .store(None, Ordering::Relaxed);
        });
    }
}
