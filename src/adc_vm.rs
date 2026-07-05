//! ADC_VM startup offset capture helpers.

use embedded_hal_async::spi::SpiDevice;
use foci_tmc4671::registers::{ADC_AGPI_A_RAW_ADC_VM_RAW, ADC_RAW_ADDR, DS_ANALOG_INPUT_STAGE_CFG};
use foci_tmc4671::spi::Tmc4671Spi;

/// ADC_VM input-stage mode that routes the internal VDD/2 reference.
///
/// TMC4671-LA Rev 1, 02/2024, `DS_ANALOG_INPUT_STAGE_CFG.ADC_VM`
/// bits 11:8: mode 5 is `VDD/2`.
pub const ADC_VM_STAGE_MODE_VDD_HALF: u8 = 5;
/// Number of mode-5 ADC_VM samples folded into the cached offset.
pub const ADC_VM_OFFSET_SAMPLE_COUNT: u8 = 8;
/// Number of initial mode-5 reads discarded after switching the mux.
pub const ADC_VM_OFFSET_SETTLE_READS: u8 = 2;

const ADC_VM_STAGE_SHIFT: u32 = 8;
const ADC_VM_STAGE_MASK: u32 = 0x0f << ADC_VM_STAGE_SHIFT;

/// Captured ADC_VM mode-5 offset evidence.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct AdcVmOffsetCapture {
    /// Averaged raw ADC_VM count.
    pub offset_raw: u16,
    /// Number of samples folded into `offset_raw`.
    pub sample_count: u8,
}

/// Return `stage_cfg` with the ADC_VM input-stage field set to `mode`.
pub const fn adc_vm_stage_cfg_with_mode(stage_cfg: u32, mode: u8) -> u32 {
    (stage_cfg & !ADC_VM_STAGE_MASK) | (((mode as u32) << ADC_VM_STAGE_SHIFT) & ADC_VM_STAGE_MASK)
}

/// Capture the ADC_VM mode-5 startup offset and restore the ADC mux registers.
pub async fn capture_adc_vm_offset<Spi>(tmc: &mut Tmc4671Spi<Spi>) -> Result<AdcVmOffsetCapture, ()>
where
    Spi: SpiDevice<u8>,
{
    let saved_stage = tmc
        .read_register(DS_ANALOG_INPUT_STAGE_CFG)
        .await
        .map_err(|_| ())?;
    let saved_addr = tmc.read_register(ADC_RAW_ADDR).await.map_err(|_| ())?;
    let result = capture_adc_vm_offset_inner(tmc, saved_stage).await;
    let restore_stage = tmc
        .write_register(DS_ANALOG_INPUT_STAGE_CFG, saved_stage)
        .await;
    let restore_addr = tmc.write_register(ADC_RAW_ADDR, saved_addr).await;
    if restore_stage.is_err() || restore_addr.is_err() {
        return Err(());
    }
    result
}

async fn capture_adc_vm_offset_inner<Spi>(
    tmc: &mut Tmc4671Spi<Spi>,
    saved_stage: u32,
) -> Result<AdcVmOffsetCapture, ()>
where
    Spi: SpiDevice<u8>,
{
    tmc.write_register(
        DS_ANALOG_INPUT_STAGE_CFG,
        adc_vm_stage_cfg_with_mode(saved_stage, ADC_VM_STAGE_MODE_VDD_HALF),
    )
    .await
    .map_err(|_| ())?;
    for _ in 0..ADC_VM_OFFSET_SETTLE_READS {
        read_adc_vm_raw(tmc).await?;
    }
    let mut sum = 0u32;
    for _ in 0..ADC_VM_OFFSET_SAMPLE_COUNT {
        sum = sum.saturating_add(u32::from(read_adc_vm_raw(tmc).await?));
    }
    let count = u32::from(ADC_VM_OFFSET_SAMPLE_COUNT);
    let offset_raw = ((sum + (count / 2)) / count) as u16;
    Ok(AdcVmOffsetCapture {
        offset_raw,
        sample_count: ADC_VM_OFFSET_SAMPLE_COUNT,
    })
}

async fn read_adc_vm_raw<Spi>(tmc: &mut Tmc4671Spi<Spi>) -> Result<u16, ()>
where
    Spi: SpiDevice<u8>,
{
    let raw = tmc
        .read_sub_register(ADC_AGPI_A_RAW_ADC_VM_RAW)
        .await
        .map_err(|_| ())?;
    Ok((raw & 0xffff) as u16)
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal_async::spi::Operation;
    use foci_tmc4671::registers::{ADC_RAW_ADDR, ADC_RAW_DATA, DS_ANALOG_INPUT_STAGE_CFG};
    use foci_tmc4671::spi::Tmc4671Spi;

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    struct MockSpiError;

    impl embedded_hal_async::spi::Error for MockSpiError {
        fn kind(&self) -> embedded_hal_async::spi::ErrorKind {
            embedded_hal_async::spi::ErrorKind::Other
        }
    }

    #[derive(Default)]
    struct MockSpi {
        writes: std::vec::Vec<(u8, u32)>,
        read_requests: std::vec::Vec<u8>,
        reads: std::collections::VecDeque<u32>,
        fail_reads_from: Option<u8>,
    }

    impl MockSpi {
        fn with_read(mut self, value: u32) -> Self {
            self.reads.push_back(value);
            self
        }

        fn fail_reads_from(mut self, addr: u8) -> Self {
            self.fail_reads_from = Some(addr);
            self
        }
    }

    impl embedded_hal_async::spi::ErrorType for MockSpi {
        type Error = MockSpiError;
    }

    impl embedded_hal_async::spi::SpiDevice<u8> for MockSpi {
        async fn transaction(
            &mut self,
            operations: &mut [Operation<'_, u8>],
        ) -> Result<(), Self::Error> {
            for op in operations {
                match op {
                    Operation::Write(buf) if buf.len() == 5 => {
                        let addr = buf[0] & 0x7f;
                        let value = u32::from_be_bytes([buf[1], buf[2], buf[3], buf[4]]);
                        self.writes.push((addr, value));
                    }
                    Operation::Write(buf) if buf.len() == 1 => {
                        let addr = buf[0];
                        self.read_requests.push(addr);
                        if self.fail_reads_from == Some(addr) {
                            return Err(MockSpiError);
                        }
                    }
                    Operation::Read(buf) => {
                        let value = self.reads.pop_front().unwrap_or(0);
                        buf.copy_from_slice(&value.to_be_bytes());
                    }
                    Operation::Write(_)
                    | Operation::DelayNs(_)
                    | Operation::Transfer(_, _)
                    | Operation::TransferInPlace(_) => {}
                }
            }
            Ok(())
        }
    }

    #[test]
    fn adc_vm_stage_cfg_with_mode_updates_only_adc_vm_bits() {
        let original = 0x1234_a678;

        let updated = adc_vm_stage_cfg_with_mode(original, ADC_VM_STAGE_MODE_VDD_HALF);

        assert_eq!(updated, 0x1234_a578);
    }

    #[test]
    fn capture_adc_vm_offset_averages_mode5_samples_and_restores_mux() {
        pollster::block_on(async {
            let saved_stage = 0x1234_a678;
            let saved_addr = 3;
            let mut mock = MockSpi::default()
                .with_read(saved_stage)
                .with_read(saved_addr)
                .with_read(0xaaaa_8100)
                .with_read(0xbbbb_8101);
            for raw in 33_000u32..33_008 {
                mock = mock.with_read(0xcccc_0000 | raw);
            }
            let mut tmc = Tmc4671Spi::new(mock);

            let capture = capture_adc_vm_offset(&mut tmc).await.unwrap();

            assert_eq!(
                capture,
                AdcVmOffsetCapture {
                    offset_raw: 33_004,
                    sample_count: ADC_VM_OFFSET_SAMPLE_COUNT,
                }
            );
            let mock = tmc.into_inner();
            assert_eq!(mock.read_requests[0], DS_ANALOG_INPUT_STAGE_CFG.addr());
            assert_eq!(mock.read_requests[1], ADC_RAW_ADDR.addr());
            assert_eq!(
                mock.writes[0],
                (DS_ANALOG_INPUT_STAGE_CFG.addr(), 0x1234_a578)
            );
            assert_eq!(
                mock.writes[mock.writes.len() - 2],
                (DS_ANALOG_INPUT_STAGE_CFG.addr(), saved_stage)
            );
            assert_eq!(
                mock.writes[mock.writes.len() - 1],
                (ADC_RAW_ADDR.addr(), saved_addr)
            );
            assert_eq!(
                mock.writes
                    .iter()
                    .filter(|(addr, value)| *addr == ADC_RAW_ADDR.addr() && *value == 1)
                    .count(),
                usize::from(ADC_VM_OFFSET_SETTLE_READS + ADC_VM_OFFSET_SAMPLE_COUNT)
            );
        });
    }

    #[test]
    fn capture_adc_vm_offset_restores_mux_after_sample_failure() {
        pollster::block_on(async {
            let saved_stage = 0x1234_a678;
            let saved_addr = 3;
            let mock = MockSpi::default()
                .with_read(saved_stage)
                .with_read(saved_addr)
                .fail_reads_from(ADC_RAW_DATA.addr());
            let mut tmc = Tmc4671Spi::new(mock);

            let result = capture_adc_vm_offset(&mut tmc).await;

            assert_eq!(result, Err(()));
            let mock = tmc.into_inner();
            assert_eq!(
                mock.writes[mock.writes.len() - 2],
                (DS_ANALOG_INPUT_STAGE_CFG.addr(), saved_stage)
            );
            assert_eq!(
                mock.writes[mock.writes.len() - 1],
                (ADC_RAW_ADDR.addr(), saved_addr)
            );
        });
    }
}
