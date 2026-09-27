//! TMC request write helpers for board-agnostic request dispatch.

use embedded_hal_async::spi::SpiDevice;
use foci_tmc4671::spi::Tmc4671Spi;

use crate::state::tmc::TmcChannelState;
use crate::tmc_control::current_limit::{CurrentLimitConversionError, current_limit_from_run_ma};

/// Failure while applying a validated RMS current request.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum SetCurrentError {
    /// The RMS current or board scale cannot form a valid chip limit.
    InvalidLimit(CurrentLimitConversionError),
    /// The validated register write failed.
    Spi,
}

/// Apply a new RMS current limit to the TMC4671.
///
/// Converts the Klipper-convention RMS milliamp value to the TMC4671 peak
/// amplitude count (datasheet eq. 19/21: `I_phase = I_D * sin(phi_e)`),
/// writes `PID_TORQUE_FLUX_LIMITS`, and stores the register value in
/// channel current-limit state for use by the commissioning engine.
///
/// While a homing clamp is active on this channel (`tmc_state.homing_clamp_active`),
/// the register write is deferred: the requested run current is still stored
/// so it takes effect once homing's own restore step reads it back, but
/// writing it live here would silently lift the homing clamp mid-move.
///
/// # Parameters
///
/// - `tmc` — mutable reference to the TMC4671 SPI driver.
/// - `run_ma` — target RMS current in milliamps.
/// - `current_scale_ma_per_lsb` — board-specific milliamps-per-LSB scaling
///   factor (e.g. `1.155` for OpenFFBoard v1.2.x).
///
/// # Errors
///
/// Returns a cause-specific error if conversion or the SPI write fails.
pub async fn apply_set_current<B>(
    tmc: &mut Tmc4671Spi<B>,
    tmc_state: &TmcChannelState,
    run_ma: u32,
    current_scale_ma_per_lsb: f32,
) -> Result<(), SetCurrentError>
where
    B: SpiDevice<u8>,
{
    use core::sync::atomic::Ordering::Relaxed;
    use foci_tmc4671::registers::*;

    let limit = current_limit_from_run_ma(run_ma, current_scale_ma_per_lsb).map_err(|error| {
        defmt::error!(
            "Invalid TMC4671 current limit: run_ma={} scale={} cause={}",
            run_ma,
            current_scale_ma_per_lsb,
            error,
        );
        SetCurrentError::InvalidLimit(error)
    })?;
    let register_value = limit.register_value();
    if tmc_state.homing_clamp_active.load(Relaxed) {
        tmc_state.store_current_limit(limit, Relaxed);
        defmt::info!(
            "TMC4671 current: {}mA RMS -> {} counts (peak), deferred: homing clamp active",
            run_ma,
            register_value
        );
        return Ok(());
    }
    if tmc
        .write_register(PID_TORQUE_FLUX_LIMITS, register_value)
        .await
        .is_err()
    {
        defmt::error!("TMC4671 SPI write failed: PID_TORQUE_FLUX_LIMITS");
        return Err(SetCurrentError::Spi);
    }
    tmc_state.store_current_limit(limit, Relaxed);
    defmt::info!(
        "TMC4671 current: {}mA RMS -> {} counts (peak)",
        run_ma,
        register_value
    );
    Ok(())
}

/// Apply a new operational PIDOUT_UQ_UD_LIMITS value.
///
/// This is a live bringup diagnostic for current-loop voltage authority. It
/// does not energize the motor and does not change current limits; it only
/// adjusts the q/d voltage-output clamp used by the TMC4671 PID cascade.
///
/// # Errors
///
/// Returns `Err(())` if `voltage_limit` is outside the accepted diagnostic
/// window or if the SPI write fails.
pub async fn apply_set_voltage_limit<B>(
    tmc: &mut Tmc4671Spi<B>,
    tmc_state: &TmcChannelState,
    voltage_limit: u32,
) -> Result<(), ()>
where
    B: SpiDevice<u8>,
{
    use core::sync::atomic::Ordering::Relaxed;
    use foci_tmc4671::registers::*;

    if !(crate::state::tmc::MIN_OPERATIONAL_PIDOUT_UQ_UD_LIMIT
        ..=crate::state::tmc::MAX_OPERATIONAL_PIDOUT_UQ_UD_LIMIT)
        .contains(&voltage_limit)
    {
        defmt::error!("Invalid PIDOUT_UQ_UD_LIMITS: {}", voltage_limit);
        return Err(());
    }
    if tmc
        .write_register(PIDOUT_UQ_UD_LIMITS, voltage_limit)
        .await
        .is_err()
    {
        defmt::error!("TMC4671 SPI write failed: PIDOUT_UQ_UD_LIMITS");
        return Err(());
    }
    tmc_state.voltage_limit_cfg.store(voltage_limit, Relaxed);
    defmt::info!("TMC4671 voltage limit: {}", voltage_limit);
    Ok(())
}

/// Write the hardware registers for a validated motion scale.
///
/// # Errors
///
/// Returns a cause-specific rejection on either SPI write failure.
pub async fn write_motion_scale<B>(
    tmc: &mut Tmc4671Spi<B>,
    scale: crate::motion_scale::MotionScale,
) -> Result<(), crate::motion_scale::MotionScaleRejection>
where
    B: SpiDevice<u8>,
{
    use foci_tmc4671::registers::*;

    if tmc
        .write_register(ABN_DECODER_PPR, scale.encoder_counts_per_rev())
        .await
        .is_err()
    {
        defmt::error!("TMC4671 SPI write failed: ABN_DECODER_PPR");
        return Err(crate::motion_scale::MotionScaleRejection::EncoderRegisterWriteFailed);
    }
    // STEP_WIDTH consumes position units per emitted physical TMC pulse.
    if tmc
        .write_register(STEP_WIDTH, scale.physical_step_width())
        .await
        .is_err()
    {
        defmt::error!("TMC4671 SPI write failed: STEP_WIDTH");
        return Err(crate::motion_scale::MotionScaleRejection::StepWidthRegisterWriteFailed);
    }
    Ok(())
}

/// Apply an encoder direction setting to the TMC4671.
///
/// Writes `ABN_DECODER_MODE` (bit 12 selects reversed/default direction),
/// stores the inversion flag, and sets the channel feedforward reset flag to
/// prevent a one-tick velocity impulse from the sign-flipped delta in the
/// feedforward path on the next tick.
///
/// # Parameters
///
/// - `tmc` — mutable reference to the TMC4671 SPI driver.
/// - `invert` — `true` to reverse the encoder direction, `false` for default.
///
/// # Errors
///
/// Returns `Err(())` if the SPI write fails.
pub async fn apply_set_encoder_dir<B>(
    tmc: &mut Tmc4671Spi<B>,
    tmc_state: &TmcChannelState,
    invert: bool,
) -> Result<(), ()>
where
    B: SpiDevice<u8>,
{
    use foci_tmc4671::registers::*;

    let mode = AbnDecoderMode::new().with_abn_direction(invert).into_bits();
    if tmc.write_register(ABN_DECODER_MODE, mode).await.is_err() {
        defmt::error!("TMC4671 SPI write failed: ABN_DECODER_MODE");
        return Err(());
    }
    tmc_state
        .encoder_reversed
        .store(invert, core::sync::atomic::Ordering::Relaxed);
    // Signal the tick loop to reset feedforward state: prev_position_target
    // has the wrong sign after a direction flip, which would produce a
    // one-tick velocity impulse in the feedforward delta computation.
    tmc_state
        .feedforward_reset
        .store(true, core::sync::atomic::Ordering::Relaxed);
    defmt::info!(
        "TMC4671 encoder direction: {}",
        if invert { "reversed" } else { "default" }
    );
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::sync::atomic::Ordering;
    use foci_firmware_macros::fixture_test;
    use foci_tmc4671::spi::Tmc4671Spi;

    fn tmc_state() -> &'static crate::state::tmc::TmcChannelState {
        crate::state::tmc::tmc_channel0()
    }

    #[derive(Default)]
    struct MockRawSpi {
        writes: std::vec::Vec<std::vec::Vec<u8>>,
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
    async fn apply_set_current_writes_single_current_limit_register() {
        use foci_tmc4671::registers::PID_TORQUE_FLUX_LIMITS;

        tmc_state().store_current_limit(
            foci_tmc4671::current_limit::PidTorqueFluxLimit::ZERO,
            Ordering::Relaxed,
        );

        let raw_spi = MockRawSpi::default();
        let mut tmc = Tmc4671Spi::new(raw_spi);
        apply_set_current(&mut tmc, tmc_state(), 1500, 1.155)
            .await
            .expect("current write should succeed");

        assert_eq!(
            tmc_state().load_current_limit(Ordering::Relaxed).raw(),
            0x072c
        );
        let raw_spi = tmc.into_inner();
        assert_eq!(raw_spi.writes.len(), 1);
        assert_eq!(
            raw_spi.writes[0],
            [PID_TORQUE_FLUX_LIMITS.addr() | 0x80, 0x00, 0x00, 0x07, 0x2c]
        );

        tmc_state().store_current_limit(
            foci_tmc4671::current_limit::PidTorqueFluxLimit::ZERO,
            Ordering::Relaxed,
        );
    }

    #[fixture_test]
    async fn apply_set_current_defers_register_write_while_homing_clamp_active() {
        tmc_state().store_current_limit(
            foci_tmc4671::current_limit::PidTorqueFluxLimit::ZERO,
            Ordering::Relaxed,
        );
        tmc_state()
            .homing_clamp_active
            .store(true, Ordering::Relaxed);

        let raw_spi = MockRawSpi::default();
        let mut tmc = Tmc4671Spi::new(raw_spi);
        apply_set_current(&mut tmc, tmc_state(), 1500, 1.155)
            .await
            .expect("current write should succeed even while deferred");

        assert_eq!(
            tmc_state().load_current_limit(Ordering::Relaxed).raw(),
            0x072c,
            "the run current must still be stored for homing's restore step to pick up"
        );
        assert!(
            tmc.into_inner().writes.is_empty(),
            "PID_TORQUE_FLUX_LIMITS must not be written while a homing clamp is active"
        );

        tmc_state()
            .homing_clamp_active
            .store(false, Ordering::Relaxed);
        tmc_state().store_current_limit(
            foci_tmc4671::current_limit::PidTorqueFluxLimit::ZERO,
            Ordering::Relaxed,
        );
    }

    #[fixture_test]
    async fn apply_set_current_rejects_before_write_or_publication() {
        tmc_state().store_current_limit(
            crate::commission_dispatch::test_support::current_limit(123),
            Ordering::Relaxed,
        );

        let raw_spi = MockRawSpi::default();
        let mut tmc = Tmc4671Spi::new(raw_spi);
        let result = apply_set_current(&mut tmc, tmc_state(), u32::MAX, 1.155).await;

        assert_eq!(
            result,
            Err(SetCurrentError::InvalidLimit(
                crate::tmc_control::current_limit::CurrentLimitConversionError::OutOfRange,
            )),
        );
        assert_eq!(tmc_state().load_current_limit(Ordering::Relaxed).raw(), 123);
        assert!(tmc.into_inner().writes.is_empty());

        tmc_state().store_current_limit(
            foci_tmc4671::current_limit::PidTorqueFluxLimit::ZERO,
            Ordering::Relaxed,
        );
    }

    #[fixture_test]
    async fn apply_set_voltage_limit_writes_pidout_limit_register() {
        use foci_tmc4671::registers::PIDOUT_UQ_UD_LIMITS;

        tmc_state()
            .voltage_limit_cfg
            .store(16_000, Ordering::Relaxed);

        let raw_spi = MockRawSpi::default();
        let mut tmc = Tmc4671Spi::new(raw_spi);
        apply_set_voltage_limit(&mut tmc, tmc_state(), 32_767)
            .await
            .expect("voltage-limit write should succeed");

        assert_eq!(
            tmc_state().voltage_limit_cfg.load(Ordering::Relaxed),
            32_767
        );
        let raw_spi = tmc.into_inner();
        assert_eq!(raw_spi.writes.len(), 1);
        assert_eq!(
            raw_spi.writes[0],
            [PIDOUT_UQ_UD_LIMITS.addr() | 0x80, 0x00, 0x00, 0x7f, 0xff]
        );

        tmc_state()
            .voltage_limit_cfg
            .store(16_000, Ordering::Relaxed);
    }

    #[fixture_test]
    async fn apply_set_voltage_limit_accepts_raw_chip_minimum() {
        use foci_tmc4671::registers::PIDOUT_UQ_UD_LIMITS;

        tmc_state()
            .voltage_limit_cfg
            .store(16_000, Ordering::Relaxed);

        let raw_spi = MockRawSpi::default();
        let mut tmc = Tmc4671Spi::new(raw_spi);
        apply_set_voltage_limit(&mut tmc, tmc_state(), 0)
            .await
            .expect("raw minimum voltage-limit write should succeed");

        assert_eq!(tmc_state().voltage_limit_cfg.load(Ordering::Relaxed), 0);
        let raw_spi = tmc.into_inner();
        assert_eq!(raw_spi.writes.len(), 1);
        assert_eq!(
            raw_spi.writes[0],
            [PIDOUT_UQ_UD_LIMITS.addr() | 0x80, 0x00, 0x00, 0x00, 0x00]
        );

        tmc_state()
            .voltage_limit_cfg
            .store(16_000, Ordering::Relaxed);
    }
}
