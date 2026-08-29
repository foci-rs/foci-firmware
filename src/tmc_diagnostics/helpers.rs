use foci_tmc4671::spi::Tmc4671Spi;

/// Split a packed signed upper/lower-half register value into `(upper, lower)`.
#[allow(dead_code)]
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

#[allow(dead_code)]
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

#[allow(dead_code)]
pub(crate) async fn clear_current_step_integrators<B>(tmc: &mut Tmc4671Spi<B>) -> Result<(), ()>
where
    B: embedded_hal_async::spi::SpiDevice<u8>,
{
    use foci_tmc4671::registers::{PID_ERROR_PID_FLUX_ERROR_SUM, PID_ERROR_PID_TORQUE_ERROR_SUM};

    write_current_step_sub_register(tmc, PID_ERROR_PID_TORQUE_ERROR_SUM, 0).await?;
    write_current_step_sub_register(tmc, PID_ERROR_PID_FLUX_ERROR_SUM, 0).await
}
