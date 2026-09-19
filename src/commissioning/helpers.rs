//! Shared low-level helpers used by multiple commissioning stages.
//!
//! These are thin wrappers over `CommissioningBackend` register I/O that
//! several phases reach for: sub-register packing, integrator resets,
//! and biquad-filter coefficient writes. Lives outside the per-phase
//! submodules so the inner and outer state machines can both call them
//! without circular dependencies.

use crate::commissioning::types::CommissioningBackend;
use foci_tmc4671::registers::SubRegister;

/// Write a sub-register via the backend's regular register writes.
///
/// TMC4671 sub-registers use selector/address register 0x4E and data register 0x4D.
pub(crate) async fn write_sub_register<B: CommissioningBackend>(
    backend: &mut B,
    sub: SubRegister,
    value: u32,
) {
    let _ = backend.write_register(sub.addr, sub.sub_addr as u32).await;
    let _ = backend.write_register(sub.data, value).await;
}

/// Write a sub-register, returning `Err(())` if either SPI transfer fails.
pub(crate) async fn write_sub_register_checked<B: CommissioningBackend>(
    backend: &mut B,
    sub: SubRegister,
    value: u32,
) -> Result<(), ()> {
    backend
        .write_register(sub.addr, sub.sub_addr as u32)
        .await
        .map_err(|_| ())?;
    backend
        .write_register(sub.data, value)
        .await
        .map_err(|_| ())
}

/// Clear torque/flux PI error sums for current-loop captures.
pub(crate) async fn clear_current_integrators<B: CommissioningBackend>(
    backend: &mut B,
) -> Result<(), ()> {
    use foci_tmc4671::registers::{PID_ERROR_PID_FLUX_ERROR_SUM, PID_ERROR_PID_TORQUE_ERROR_SUM};

    write_sub_register_checked(backend, PID_ERROR_PID_TORQUE_ERROR_SUM, 0).await?;
    write_sub_register_checked(backend, PID_ERROR_PID_FLUX_ERROR_SUM, 0).await
}

/// Write velocity biquad filter coefficients.
pub(crate) async fn write_biquad_velocity<B: CommissioningBackend>(
    backend: &mut B,
    cutoff_hz: u16,
    pwm_maxcnt: u16,
) {
    let _ = write_biquad_velocity_checked(backend, cutoff_hz, pwm_maxcnt).await;
}

/// Write velocity biquad filter coefficients and report SPI failures.
pub(crate) async fn write_biquad_velocity_checked<B: CommissioningBackend>(
    backend: &mut B,
    cutoff_hz: u16,
    pwm_maxcnt: u16,
) -> Result<(), ()> {
    use foci_tmc4671::registers::*;
    if let Some(coeffs) = crate::biquad::butterworth_lowpass(
        cutoff_hz,
        pwm_maxcnt,
        crate::biquad::MOTION_CUTOFF_MAX_HZ,
    ) {
        write_sub_register_checked(backend, CONFIG_BIQUAD_V_A_1, coeffs.a_1 as u32).await?;
        write_sub_register_checked(backend, CONFIG_BIQUAD_V_A_2, coeffs.a_2 as u32).await?;
        write_sub_register_checked(backend, CONFIG_BIQUAD_V_B_0, coeffs.b_0 as u32).await?;
        write_sub_register_checked(backend, CONFIG_BIQUAD_V_B_1, coeffs.b_1 as u32).await?;
        write_sub_register_checked(backend, CONFIG_BIQUAD_V_B_2, coeffs.b_2 as u32).await?;
        write_sub_register_checked(backend, CONFIG_BIQUAD_V_ENABLE, 1).await?;
        defmt::info!("biquad velocity: {}Hz enabled", cutoff_hz);
    }
    Ok(())
}

/// Write position biquad filter coefficients.
pub(crate) async fn write_biquad_position<B: CommissioningBackend>(
    backend: &mut B,
    cutoff_hz: u16,
    pwm_maxcnt: u16,
) {
    let _ = write_biquad_position_checked(backend, cutoff_hz, pwm_maxcnt).await;
}

/// Write position biquad filter coefficients and report SPI failures.
pub(crate) async fn write_biquad_position_checked<B: CommissioningBackend>(
    backend: &mut B,
    cutoff_hz: u16,
    pwm_maxcnt: u16,
) -> Result<(), ()> {
    use foci_tmc4671::registers::*;
    if let Some(coeffs) = crate::biquad::butterworth_lowpass(
        cutoff_hz,
        pwm_maxcnt,
        crate::biquad::MOTION_CUTOFF_MAX_HZ,
    ) {
        write_sub_register_checked(backend, CONFIG_BIQUAD_X_A_1, coeffs.a_1 as u32).await?;
        write_sub_register_checked(backend, CONFIG_BIQUAD_X_A_2, coeffs.a_2 as u32).await?;
        write_sub_register_checked(backend, CONFIG_BIQUAD_X_B_0, coeffs.b_0 as u32).await?;
        write_sub_register_checked(backend, CONFIG_BIQUAD_X_B_1, coeffs.b_1 as u32).await?;
        write_sub_register_checked(backend, CONFIG_BIQUAD_X_B_2, coeffs.b_2 as u32).await?;
        write_sub_register_checked(backend, CONFIG_BIQUAD_X_ENABLE, 1).await?;
        defmt::info!("biquad position: {}Hz enabled", cutoff_hz);
    }
    Ok(())
}

/// Write flux biquad filter coefficients.
pub(crate) async fn write_biquad_flux<B: CommissioningBackend>(
    backend: &mut B,
    cutoff_hz: u16,
    pwm_maxcnt: u16,
) {
    let _ = write_biquad_flux_checked(backend, cutoff_hz, pwm_maxcnt).await;
}

/// Write flux biquad filter coefficients and report SPI failures.
pub(crate) async fn write_biquad_flux_checked<B: CommissioningBackend>(
    backend: &mut B,
    cutoff_hz: u16,
    pwm_maxcnt: u16,
) -> Result<(), ()> {
    use foci_tmc4671::registers::*;
    if let Some(coeffs) = crate::biquad::butterworth_lowpass(
        cutoff_hz,
        pwm_maxcnt,
        crate::biquad::CURRENT_CUTOFF_MAX_HZ,
    ) {
        write_sub_register_checked(backend, CONFIG_BIQUAD_F_A_1, coeffs.a_1 as u32).await?;
        write_sub_register_checked(backend, CONFIG_BIQUAD_F_A_2, coeffs.a_2 as u32).await?;
        write_sub_register_checked(backend, CONFIG_BIQUAD_F_B_0, coeffs.b_0 as u32).await?;
        write_sub_register_checked(backend, CONFIG_BIQUAD_F_B_1, coeffs.b_1 as u32).await?;
        write_sub_register_checked(backend, CONFIG_BIQUAD_F_B_2, coeffs.b_2 as u32).await?;
        write_sub_register_checked(backend, CONFIG_BIQUAD_F_ENABLE, 1).await?;
        defmt::info!("biquad flux: {}Hz enabled", cutoff_hz);
    }
    Ok(())
}

/// Write torque biquad filter coefficients.
pub(crate) async fn write_biquad_torque<B: CommissioningBackend>(
    backend: &mut B,
    cutoff_hz: u16,
    pwm_maxcnt: u16,
) {
    let _ = write_biquad_torque_checked(backend, cutoff_hz, pwm_maxcnt).await;
}

/// Write torque biquad filter coefficients and report SPI failures.
pub(crate) async fn write_biquad_torque_checked<B: CommissioningBackend>(
    backend: &mut B,
    cutoff_hz: u16,
    pwm_maxcnt: u16,
) -> Result<(), ()> {
    use foci_tmc4671::registers::*;
    if let Some(coeffs) = crate::biquad::butterworth_lowpass(
        cutoff_hz,
        pwm_maxcnt,
        crate::biquad::CURRENT_CUTOFF_MAX_HZ,
    ) {
        write_sub_register_checked(backend, CONFIG_BIQUAD_T_A_1, coeffs.a_1 as u32).await?;
        write_sub_register_checked(backend, CONFIG_BIQUAD_T_A_2, coeffs.a_2 as u32).await?;
        write_sub_register_checked(backend, CONFIG_BIQUAD_T_B_0, coeffs.b_0 as u32).await?;
        write_sub_register_checked(backend, CONFIG_BIQUAD_T_B_1, coeffs.b_1 as u32).await?;
        write_sub_register_checked(backend, CONFIG_BIQUAD_T_B_2, coeffs.b_2 as u32).await?;
        write_sub_register_checked(backend, CONFIG_BIQUAD_T_ENABLE, 1).await?;
        defmt::info!("biquad torque: {}Hz enabled", cutoff_hz);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn write_biquad_velocity_checked_rejects_above_motion_ceiling() {
        pollster::block_on(async {
            let mut backend = crate::commission_dispatch::test_support::MockBackend::new();

            write_biquad_velocity_checked(&mut backend, 2000, 3999)
                .await
                .expect("an invalid cutoff is a silent no-op, not an SPI error");

            assert!(
                backend.spi_log.is_empty(),
                "a 2000 Hz velocity cutoff must produce zero SPI writes -- it exceeds \
                 the motion-filter ceiling even though it is a valid current-filter value"
            );
        });
    }

    #[test]
    fn write_biquad_position_checked_rejects_above_motion_ceiling() {
        pollster::block_on(async {
            let mut backend = crate::commission_dispatch::test_support::MockBackend::new();

            write_biquad_position_checked(&mut backend, 2000, 3999)
                .await
                .expect("an invalid cutoff is a silent no-op, not an SPI error");

            assert!(
                backend.spi_log.is_empty(),
                "a 2000 Hz position cutoff must produce zero SPI writes -- it exceeds \
                 the motion-filter ceiling even though it is a valid current-filter value"
            );
        });
    }

    #[test]
    fn write_biquad_torque_checked_accepts_ten_khz() {
        pollster::block_on(async {
            let mut backend = crate::commission_dispatch::test_support::MockBackend::new();

            write_biquad_torque_checked(&mut backend, 10_000, 3999)
                .await
                .expect("10 kHz is within the current-filter ceiling");

            assert!(
                !backend.spi_log.is_empty(),
                "a 10 kHz torque cutoff must produce SPI writes -- it is within the \
                 current-filter ceiling"
            );
        });
    }

    #[test]
    fn write_biquad_flux_checked_accepts_ten_khz() {
        pollster::block_on(async {
            let mut backend = crate::commission_dispatch::test_support::MockBackend::new();

            write_biquad_flux_checked(&mut backend, 10_000, 3999)
                .await
                .expect("10 kHz is within the current-filter ceiling");

            assert!(
                !backend.spi_log.is_empty(),
                "a 10 kHz flux cutoff must produce SPI writes -- it is within the \
                 current-filter ceiling"
            );
        });
    }
}
