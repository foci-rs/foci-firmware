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

/// Force the velocity and position biquad filters disabled.
///
/// Writes `enable=0` directly, bypassing the zero-cutoff no-op in
/// [`apply_one_backend_filter`] (which leaves an already-enabled filter
/// active on `filter_hz == 0`). Used at Stage 2 dispatch so a commissioning
/// measurement's filter state is deterministic and independent of whatever
/// `printer.cfg` has configured, rather than inheriting it.
///
/// # Errors
///
/// Returns the first SPI write error encountered.
pub async fn force_velocity_position_filters_disabled<B>(
    tmc: &mut Tmc4671Spi<B>,
) -> Result<(), SpiError<B::Error>>
where
    B: SpiDevice<u8>,
{
    use foci_tmc4671::registers::{CONFIG_BIQUAD_V_ENABLE, CONFIG_BIQUAD_X_ENABLE};

    tmc.write_sub_register(CONFIG_BIQUAD_V_ENABLE, 0).await?;
    tmc.write_sub_register(CONFIG_BIQUAD_X_ENABLE, 0).await
}

/// Re-apply biquad filters from stored atomics.
///
/// Called after `CalibrateDone` to restore filters that the commissioning
/// engine's `ClosedLoopActivation` phase disabled (it has no access to the filter
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

    apply_backend_filter_cutoffs(
        backend,
        BackendFilterCutoffs {
            velocity_hz: tmc_state.velocity_filter_hz.load_or_zero(Relaxed),
            torque_hz: tmc_state.torque_filter_hz.load_or_zero(Relaxed),
            position_hz: tmc_state.position_filter_hz.load_or_zero(Relaxed),
            flux_hz: tmc_state.flux_filter_hz.load_or_zero(Relaxed),
        },
        pwm_maxcnt,
    )
    .await
}

/// Apply Stage 2 filter cutoffs directly from an unpublished outer result.
///
/// This uses the same backend write path as [`apply_stored_filters_via_backend`]
/// without reading or changing the published filter cache.
///
/// # Errors
///
/// Returns the first backend register-write error.
pub async fn apply_outer_result_filters_via_backend<B: CommissioningBackend>(
    backend: &mut B,
    outer: &crate::commissioning::types::OuterResult,
    pwm_maxcnt: u16,
) -> Result<(), B::Error> {
    apply_backend_filter_cutoffs(
        backend,
        BackendFilterCutoffs {
            velocity_hz: outer.velocity_filter_hz,
            torque_hz: outer.torque_filter_hz,
            position_hz: outer.position_filter_hz,
            flux_hz: outer.flux_filter_hz,
        },
        pwm_maxcnt,
    )
    .await
}

/// Apply Stage 2 entry-snapshot filter cutoffs, the inverse of
/// [`force_velocity_position_filters_disabled`].
///
/// Used to restore the operator's baseline filters on a commissioning
/// terminal that returns registers to their pre-Stage-2 state without
/// publishing a new tuned result.
///
/// # Errors
///
/// Returns the first backend register-write error.
pub async fn apply_entry_snapshot_filters_via_backend<B: CommissioningBackend>(
    backend: &mut B,
    snapshot: &crate::commissioning::OuterGainRollbackSnapshot,
    pwm_maxcnt: u16,
) -> Result<(), B::Error> {
    apply_backend_filter_cutoffs(
        backend,
        BackendFilterCutoffs {
            velocity_hz: snapshot.velocity_filter_hz,
            torque_hz: snapshot.torque_filter_hz,
            position_hz: snapshot.position_filter_hz,
            flux_hz: snapshot.flux_filter_hz,
        },
        pwm_maxcnt,
    )
    .await
}

#[derive(Clone, Copy)]
struct BackendFilterCutoffs {
    velocity_hz: u16,
    torque_hz: u16,
    position_hz: u16,
    flux_hz: u16,
}

async fn write_backend_sub_register<B: CommissioningBackend>(
    backend: &mut B,
    sub: SubRegister,
    value: u32,
) -> Result<(), B::Error> {
    backend
        .write_register(sub.addr, sub.sub_addr as u32)
        .await?;
    backend.write_register(sub.data, value).await
}

async fn apply_one_backend_filter<B: CommissioningBackend>(
    backend: &mut B,
    target: BiquadFilterTarget,
    filter_hz: u16,
    pwm_maxcnt: u16,
) -> Result<(), B::Error> {
    if filter_hz == 0 {
        return Ok(());
    }
    let Some(coeffs) = crate::biquad::butterworth_lowpass(filter_hz, pwm_maxcnt) else {
        return Ok(());
    };
    let regs = biquad_filter_regs(target);
    write_backend_sub_register(backend, regs.a_1, coeffs.a_1 as u32).await?;
    write_backend_sub_register(backend, regs.a_2, coeffs.a_2 as u32).await?;
    write_backend_sub_register(backend, regs.b_0, coeffs.b_0 as u32).await?;
    write_backend_sub_register(backend, regs.b_1, coeffs.b_1 as u32).await?;
    write_backend_sub_register(backend, regs.b_2, coeffs.b_2 as u32).await?;
    write_backend_sub_register(backend, regs.enable, 1).await
}

async fn apply_backend_filter_cutoffs<B: CommissioningBackend>(
    backend: &mut B,
    cutoffs: BackendFilterCutoffs,
    pwm_maxcnt: u16,
) -> Result<(), B::Error> {
    apply_one_backend_filter(
        backend,
        BiquadFilterTarget::Velocity,
        cutoffs.velocity_hz,
        pwm_maxcnt,
    )
    .await?;
    apply_one_backend_filter(
        backend,
        BiquadFilterTarget::Torque,
        cutoffs.torque_hz,
        pwm_maxcnt,
    )
    .await?;
    apply_one_backend_filter(
        backend,
        BiquadFilterTarget::Position,
        cutoffs.position_hz,
        pwm_maxcnt,
    )
    .await?;
    apply_one_backend_filter(
        backend,
        BiquadFilterTarget::Flux,
        cutoffs.flux_hz,
        pwm_maxcnt,
    )
    .await
}

#[derive(Clone, Copy)]
pub(crate) struct FilterCutoffsStep {
    ops: [(foci_tmc4671::registers::Register, u32); 48],
    len: u8,
    next: u8,
}

impl FilterCutoffsStep {
    fn start(cutoffs: BackendFilterCutoffs, pwm_maxcnt: u16) -> Self {
        let placeholder = foci_tmc4671::registers::Register::new(0);
        let mut ops = [(placeholder, 0u32); 48];
        let mut len = 0usize;
        for (target, filter_hz) in [
            (BiquadFilterTarget::Velocity, cutoffs.velocity_hz),
            (BiquadFilterTarget::Torque, cutoffs.torque_hz),
            (BiquadFilterTarget::Position, cutoffs.position_hz),
            (BiquadFilterTarget::Flux, cutoffs.flux_hz),
        ] {
            if filter_hz == 0 {
                continue;
            }
            let Some(coeffs) = crate::biquad::butterworth_lowpass(filter_hz, pwm_maxcnt) else {
                continue;
            };
            let regs = biquad_filter_regs(target);
            for (sub, value) in [
                (regs.a_1, coeffs.a_1 as u32),
                (regs.a_2, coeffs.a_2 as u32),
                (regs.b_0, coeffs.b_0 as u32),
                (regs.b_1, coeffs.b_1 as u32),
                (regs.b_2, coeffs.b_2 as u32),
                (regs.enable, 1u32),
            ] {
                ops[len] = (sub.addr, u32::from(sub.sub_addr));
                len += 1;
                ops[len] = (sub.data, value);
                len += 1;
            }
        }
        Self {
            ops,
            len: len as u8,
            next: 0,
        }
    }

    pub(crate) fn from_stored(tmc_state: &TmcChannelState, pwm_maxcnt: u16) -> Self {
        use core::sync::atomic::Ordering::Relaxed;

        Self::start(
            BackendFilterCutoffs {
                velocity_hz: tmc_state.velocity_filter_hz.load_or_zero(Relaxed),
                torque_hz: tmc_state.torque_filter_hz.load_or_zero(Relaxed),
                position_hz: tmc_state.position_filter_hz.load_or_zero(Relaxed),
                flux_hz: tmc_state.flux_filter_hz.load_or_zero(Relaxed),
            },
            pwm_maxcnt,
        )
    }

    pub(crate) fn from_outer_result(
        outer: &crate::commissioning::types::OuterResult,
        pwm_maxcnt: u16,
    ) -> Self {
        Self::start(
            BackendFilterCutoffs {
                velocity_hz: outer.velocity_filter_hz,
                torque_hz: outer.torque_filter_hz,
                position_hz: outer.position_filter_hz,
                flux_hz: outer.flux_filter_hz,
            },
            pwm_maxcnt,
        )
    }

    pub(crate) fn from_entry_snapshot(
        snapshot: &crate::commissioning::OuterGainRollbackSnapshot,
        pwm_maxcnt: u16,
    ) -> Self {
        Self::start(
            BackendFilterCutoffs {
                velocity_hz: snapshot.velocity_filter_hz,
                torque_hz: snapshot.torque_filter_hz,
                position_hz: snapshot.position_filter_hz,
                flux_hz: snapshot.flux_filter_hz,
            },
            pwm_maxcnt,
        )
    }

    pub(crate) async fn advance<B: CommissioningBackend>(
        &mut self,
        backend: &mut B,
    ) -> Option<Result<(), ()>> {
        if self.next >= self.len {
            return Some(Ok(()));
        }
        let (reg, value) = self.ops[self.next as usize];
        if backend.write_register(reg, value).await.is_err() {
            return Some(Err(()));
        }
        self.next += 1;
        if self.next >= self.len {
            Some(Ok(()))
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tmc_control::test_spi::MockRawSpi;
    use foci_firmware_macros::fixture_test;
    use foci_tmc4671::spi::Tmc4671Spi;

    fn tmc_state() -> &'static crate::state::tmc::TmcChannelState {
        crate::state::tmc::tmc_channel0()
    }

    #[fixture_test]
    async fn apply_stored_filters_via_backend_writes_enabled_filters() {
        use core::sync::atomic::Ordering;
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
    }

    #[fixture_test]
    async fn apply_stored_filters_reports_raw_spi_write_failure() {
        use core::sync::atomic::Ordering;
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
    }

    #[test]
    fn backend_filter_cutoffs_apply_explicit_values() {
        pollster::block_on(async {
            use crate::commission_dispatch::test_support::{MockBackend, SpiOp};
            use foci_tmc4671::registers::CONFIG_BIQUAD_V_ENABLE;

            let mut backend = MockBackend::new();
            apply_backend_filter_cutoffs(
                &mut backend,
                BackendFilterCutoffs {
                    velocity_hz: 100,
                    torque_hz: 0,
                    position_hz: 0,
                    flux_hz: 0,
                },
                4095,
            )
            .await
            .expect("explicit cutoff apply should succeed");

            let writes: std::vec::Vec<(u8, u32)> = backend
                .spi_log
                .iter()
                .filter_map(|op| match op {
                    SpiOp::Write(addr, value) => Some((*addr, *value)),
                    SpiOp::Read
                    | SpiOp::EnablePin(_)
                    | SpiOp::Lifecycle { .. }
                    | SpiOp::DrainPendingConfig => None,
                })
                .collect();
            let has_sub_write = |sub: foci_tmc4671::registers::SubRegister, value: u32| {
                writes.windows(2).any(|pair| {
                    pair[0] == (sub.addr.addr(), sub.sub_addr as u32)
                        && pair[1] == (sub.data.addr(), value)
                })
            };
            let coeffs = crate::biquad::butterworth_lowpass(100, 4095)
                .expect("100 Hz is a valid test cutoff");
            use foci_tmc4671::registers::{
                CONFIG_BIQUAD_V_A_1, CONFIG_BIQUAD_V_A_2, CONFIG_BIQUAD_V_B_0, CONFIG_BIQUAD_V_B_1,
                CONFIG_BIQUAD_V_B_2,
            };
            assert!(has_sub_write(CONFIG_BIQUAD_V_A_1, coeffs.a_1 as u32));
            assert!(has_sub_write(CONFIG_BIQUAD_V_A_2, coeffs.a_2 as u32));
            assert!(has_sub_write(CONFIG_BIQUAD_V_B_0, coeffs.b_0 as u32));
            assert!(has_sub_write(CONFIG_BIQUAD_V_B_1, coeffs.b_1 as u32));
            assert!(has_sub_write(CONFIG_BIQUAD_V_B_2, coeffs.b_2 as u32));
            assert!(has_sub_write(CONFIG_BIQUAD_V_ENABLE, 1));
        });
    }

    #[fixture_test]
    async fn force_velocity_position_filters_disabled_writes_enable_zero_for_both() {
        use foci_tmc4671::registers::{
            CONFIG_BIQUAD_V_ENABLE, CONFIG_BIQUAD_X_ENABLE, SubRegister,
        };

        let raw_spi = MockRawSpi::default();
        let mut tmc = Tmc4671Spi::new(raw_spi);

        force_velocity_position_filters_disabled(&mut tmc)
            .await
            .expect("forcing filters off should succeed");

        let raw_spi = tmc.into_inner();
        assert_eq!(
            raw_spi.writes.len(),
            4,
            "expected two sub-register writes (selector + data each)"
        );

        let frame = |addr: u8, value: u32| -> std::vec::Vec<u8> {
            let bytes = value.to_be_bytes();
            std::vec![addr | 0x80, bytes[0], bytes[1], bytes[2], bytes[3]]
        };
        let expect_sub_write_zero = |writes: &[std::vec::Vec<u8>], sub: SubRegister| {
            assert_eq!(writes[0], frame(sub.addr.addr(), sub.sub_addr as u32));
            assert_eq!(writes[1], frame(sub.data.addr(), 0));
        };
        expect_sub_write_zero(&raw_spi.writes[0..2], CONFIG_BIQUAD_V_ENABLE);
        expect_sub_write_zero(&raw_spi.writes[2..4], CONFIG_BIQUAD_X_ENABLE);
    }

    #[test]
    fn backend_filter_cutoffs_propagate_write_failure() {
        pollster::block_on(async {
            let mut backend = crate::commission_dispatch::test_support::MockBackend::new();
            backend.fail_on_write = Some(foci_tmc4671::registers::CONFIG_BIQUAD_V_A_1.data.addr());

            let result = apply_backend_filter_cutoffs(
                &mut backend,
                BackendFilterCutoffs {
                    velocity_hz: 100,
                    torque_hz: 0,
                    position_hz: 0,
                    flux_hz: 0,
                },
                4095,
            )
            .await;

            assert!(result.is_err());
        });
    }

    #[fixture_test]
    async fn filter_cutoffs_step_matches_flat_apply_stored_filters() {
        use core::sync::atomic::Ordering;
        tmc_state()
            .velocity_filter_hz
            .store(Some(100), Ordering::Relaxed);
        tmc_state()
            .torque_filter_hz
            .store(Some(0), Ordering::Relaxed);
        tmc_state()
            .position_filter_hz
            .store(Some(250), Ordering::Relaxed);
        tmc_state().flux_filter_hz.store(Some(0), Ordering::Relaxed);

        let mut flat_backend = crate::commission_dispatch::test_support::MockBackend::new();
        apply_stored_filters_via_backend(&mut flat_backend, tmc_state(), 4095)
            .await
            .expect("flat apply must succeed");
        let flat_log = flat_backend.spi_log.clone();

        let mut stepped_backend = crate::commission_dispatch::test_support::MockBackend::new();
        let mut step = FilterCutoffsStep::from_stored(tmc_state(), 4095);
        loop {
            match step.advance(&mut stepped_backend).await {
                None => continue,
                Some(Ok(())) => break,
                Some(Err(())) => panic!("stepped apply failed"),
            }
        }
        let stepped_log = stepped_backend.spi_log.clone();

        assert_eq!(
            flat_log, stepped_log,
            "stepped FilterCutoffsStep must write the identical register sequence \
             as apply_stored_filters_via_backend"
        );
    }

    #[fixture_test]
    async fn filter_cutoffs_step_advances_one_write_per_call() {
        use core::sync::atomic::Ordering;
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
        let mut step = FilterCutoffsStep::from_stored(tmc_state(), 4095);
        let mut ticks = 0usize;
        loop {
            let before = backend.spi_log.len();
            let result = step.advance(&mut backend).await;
            let after = backend.spi_log.len();
            assert!(
                after - before <= 1,
                "tick {}: advance() issued {} writes, expected at most 1",
                ticks,
                after - before
            );
            ticks += 1;
            if !matches!(result, None) {
                break;
            }
        }
        assert_eq!(
            ticks, 12,
            "expected exactly 12 poll ticks for one enabled filter"
        );
    }

    #[fixture_test]
    async fn filter_cutoffs_step_skips_filter_with_no_valid_coefficients() {
        use core::sync::atomic::Ordering;
        // pwm_maxcnt = 0 makes butterworth_lowpass return None for every
        // nonzero cutoff -- the second no-op path, distinct from filter_hz == 0.
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
        let mut step = FilterCutoffsStep::from_stored(tmc_state(), 0);
        loop {
            match step.advance(&mut backend).await {
                None => continue,
                Some(Ok(())) => break,
                Some(Err(())) => panic!("stepped apply failed"),
            }
        }
        assert!(
            backend.spi_log.is_empty(),
            "pwm_maxcnt == 0 must produce zero writes, matching \
             apply_one_backend_filter's butterworth_lowpass(..) == None no-op path"
        );
    }
}
