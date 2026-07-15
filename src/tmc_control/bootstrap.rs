//! Checked TMC4671 safety-register bootstrap shared by board firmware.

use crate::commissioning::backend::CommissionRegisterIo;
use crate::shutdown::ShutdownReason;
use foci_tmc4671::registers::{ADC_VM_LIMITS, Register, STATUS_FLAGS, STATUS_MASK};
use foci_tmc4671::spi::RetryPolicy;

/// Safety-bootstrap operation that exhausted its retry budget.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum TmcSafetyBootstrapStage {
    /// Writing and verifying `ADC_VM_LIMITS` failed.
    AdcVmLimitsWrite,
    /// Writing and verifying `STATUS_MASK` failed.
    StatusMaskWrite,
    /// Reading pending `STATUS_FLAGS` failed.
    StatusFlagsRead,
    /// Clearing pending `STATUS_FLAGS` failed.
    StatusFlagsClear,
}

/// Retained failure from one TMC4671 safety bootstrap.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct TmcSafetyBootstrapFailure {
    /// First bootstrap stage that exhausted its retry budget.
    pub stage: TmcSafetyBootstrapStage,
}

impl TmcSafetyBootstrapFailure {
    /// Map this failure to the channel-specific static shutdown reason.
    pub const fn shutdown_reason(self, channel: u8) -> ShutdownReason {
        match (channel, self.stage) {
            (0, TmcSafetyBootstrapStage::AdcVmLimitsWrite) => {
                ShutdownReason::TmcBootstrapCh0AdcVmLimitsWriteFailed
            }
            (0, TmcSafetyBootstrapStage::StatusMaskWrite) => {
                ShutdownReason::TmcBootstrapCh0StatusMaskWriteFailed
            }
            (0, TmcSafetyBootstrapStage::StatusFlagsRead) => {
                ShutdownReason::TmcBootstrapCh0StatusFlagsReadFailed
            }
            (0, TmcSafetyBootstrapStage::StatusFlagsClear) => {
                ShutdownReason::TmcBootstrapCh0StatusFlagsClearFailed
            }
            (1, TmcSafetyBootstrapStage::AdcVmLimitsWrite) => {
                ShutdownReason::TmcBootstrapCh1AdcVmLimitsWriteFailed
            }
            (1, TmcSafetyBootstrapStage::StatusMaskWrite) => {
                ShutdownReason::TmcBootstrapCh1StatusMaskWriteFailed
            }
            (1, TmcSafetyBootstrapStage::StatusFlagsRead) => {
                ShutdownReason::TmcBootstrapCh1StatusFlagsReadFailed
            }
            (1, TmcSafetyBootstrapStage::StatusFlagsClear) => {
                ShutdownReason::TmcBootstrapCh1StatusFlagsClearFailed
            }
            _ => ShutdownReason::InvalidTmcChannel,
        }
    }
}

async fn retry_plain_read<B: CommissionRegisterIo>(
    backend: &mut B,
    register: Register,
) -> Result<u32, ()> {
    let max_attempts = RetryPolicy::production().max_attempts.max(1);
    for attempt in 0..max_attempts {
        match backend.read_register(register).await {
            Ok(value) => return Ok(value),
            Err(_) if attempt + 1 == max_attempts => return Err(()),
            Err(_) => {}
        }
    }
    Err(())
}

async fn retry_plain_write<B: CommissionRegisterIo>(
    backend: &mut B,
    register: Register,
    value: u32,
) -> Result<(), ()> {
    let max_attempts = RetryPolicy::production().max_attempts.max(1);
    for attempt in 0..max_attempts {
        match backend.write_register(register, value).await {
            Ok(()) => return Ok(()),
            Err(_) if attempt + 1 == max_attempts => return Err(()),
            Err(_) => {}
        }
    }
    Err(())
}

/// Configure the safety registers required before normal TMC operation.
pub async fn bootstrap_tmc_safety<B: CommissionRegisterIo>(
    backend: &mut B,
    status_mask: u32,
) -> Result<(), TmcSafetyBootstrapFailure> {
    backend
        .write_register_verified(ADC_VM_LIMITS, u32::MAX)
        .await
        .map_err(|_| TmcSafetyBootstrapFailure {
            stage: TmcSafetyBootstrapStage::AdcVmLimitsWrite,
        })?;
    backend
        .write_register_verified(STATUS_MASK, status_mask)
        .await
        .map_err(|_| TmcSafetyBootstrapFailure {
            stage: TmcSafetyBootstrapStage::StatusMaskWrite,
        })?;
    retry_plain_read(backend, STATUS_FLAGS)
        .await
        .map_err(|_| TmcSafetyBootstrapFailure {
            stage: TmcSafetyBootstrapStage::StatusFlagsRead,
        })?;
    retry_plain_write(backend, STATUS_FLAGS, 0)
        .await
        .map_err(|_| TmcSafetyBootstrapFailure {
            stage: TmcSafetyBootstrapStage::StatusFlagsClear,
        })
}

/// Return the lowest channel whose bootstrap failed without consuming results.
pub fn first_tmc_safety_bootstrap_failure(
    results: &[Result<(), TmcSafetyBootstrapFailure>],
) -> Option<(u8, TmcSafetyBootstrapFailure)> {
    results.iter().enumerate().find_map(|(index, result)| {
        let channel = u8::try_from(index).ok()?;
        result
            .as_ref()
            .err()
            .copied()
            .map(|failure| (channel, failure))
    })
}

#[cfg(test)]
mod tests {
    use super::{
        TmcSafetyBootstrapFailure, TmcSafetyBootstrapStage, bootstrap_tmc_safety,
        first_tmc_safety_bootstrap_failure,
    };
    use crate::commission_dispatch::test_support::{MockBackend, SpiOp};
    use crate::shutdown::ShutdownReason;
    use foci_tmc4671::registers::{ADC_VM_LIMITS, STATUS_FLAGS, STATUS_MASK};

    const MASKS: [u32; 2] = [1 << 26, (1 << 26) | (1 << 27)];

    fn failure(stage: TmcSafetyBootstrapStage) -> TmcSafetyBootstrapFailure {
        TmcSafetyBootstrapFailure { stage }
    }

    #[test]
    fn tmc_safety_bootstrap_writes_exact_sequence_for_each_board_mask() {
        pollster::block_on(async {
            for mask in MASKS {
                let mut backend = MockBackend::new();
                let result = bootstrap_tmc_safety(&mut backend, mask).await;

                assert_eq!(result, Ok(()));
                assert_eq!(backend.spi_log.len(), 6);
                assert!(
                    matches!(backend.spi_log[0], SpiOp::Write(addr, u32::MAX) if addr == ADC_VM_LIMITS.addr())
                );
                assert!(matches!(backend.spi_log[1], SpiOp::Read));
                assert!(
                    matches!(backend.spi_log[2], SpiOp::Write(addr, value) if addr == STATUS_MASK.addr() && value == mask)
                );
                assert!(matches!(backend.spi_log[3], SpiOp::Read));
                assert!(matches!(backend.spi_log[4], SpiOp::Read));
                assert!(
                    matches!(backend.spi_log[5], SpiOp::Write(addr, 0) if addr == STATUS_FLAGS.addr())
                );
            }
        });
    }

    #[test]
    fn tmc_safety_bootstrap_recovers_one_transient_failure_at_every_stage() {
        pollster::block_on(async {
            let cases = [
                (ADC_VM_LIMITS.addr(), true),
                (STATUS_MASK.addr(), true),
                (STATUS_FLAGS.addr(), false),
                (STATUS_FLAGS.addr(), true),
            ];

            for (index, (address, is_write)) in cases.into_iter().enumerate() {
                let mut backend = MockBackend::new();
                if is_write {
                    backend.fail_writes_remaining = Some((address, 1));
                } else {
                    backend.fail_reads_remaining = Some((address, 1));
                }

                let result = bootstrap_tmc_safety(&mut backend, MASKS[0]).await;

                assert_eq!(result, Ok(()), "transient stage {index}");
                assert_eq!(backend.spi_log.len(), 7, "transient stage {index}");
            }
        });
    }

    #[test]
    fn tmc_safety_bootstrap_stops_after_first_exhausted_stage() {
        pollster::block_on(async {
            let cases = [
                (
                    TmcSafetyBootstrapStage::AdcVmLimitsWrite,
                    ADC_VM_LIMITS.addr(),
                    true,
                    3,
                ),
                (
                    TmcSafetyBootstrapStage::StatusMaskWrite,
                    STATUS_MASK.addr(),
                    true,
                    5,
                ),
                (
                    TmcSafetyBootstrapStage::StatusFlagsRead,
                    STATUS_FLAGS.addr(),
                    false,
                    7,
                ),
                (
                    TmcSafetyBootstrapStage::StatusFlagsClear,
                    STATUS_FLAGS.addr(),
                    true,
                    8,
                ),
            ];

            for (stage, address, is_write, expected_ops) in cases {
                let mut backend = MockBackend::new();
                if is_write {
                    backend.fail_on_write = Some(address);
                } else {
                    backend.fail_on_read = Some(address);
                }

                let result = bootstrap_tmc_safety(&mut backend, MASKS[0]).await;

                assert_eq!(result, Err(failure(stage)));
                assert_eq!(backend.spi_log.len(), expected_ops, "stage {stage:?}");
            }
        });
    }

    #[test]
    fn tmc_safety_bootstrap_selects_lowest_failed_channel_without_consuming_results() {
        let channel_zero = failure(TmcSafetyBootstrapStage::StatusMaskWrite);
        let channel_one = failure(TmcSafetyBootstrapStage::StatusFlagsClear);
        let results = [Err(channel_zero), Err(channel_one)];

        assert_eq!(
            first_tmc_safety_bootstrap_failure(&results),
            Some((0, channel_zero))
        );
        assert_eq!(results, [Err(channel_zero), Err(channel_one)]);
    }

    #[test]
    fn tmc_safety_bootstrap_maps_all_channel_stage_shutdown_reasons() {
        let cases = [
            (
                0,
                TmcSafetyBootstrapStage::AdcVmLimitsWrite,
                ShutdownReason::TmcBootstrapCh0AdcVmLimitsWriteFailed,
                "TMC4671 ch0 bootstrap: ADC_VM_LIMITS write failed",
            ),
            (
                0,
                TmcSafetyBootstrapStage::StatusMaskWrite,
                ShutdownReason::TmcBootstrapCh0StatusMaskWriteFailed,
                "TMC4671 ch0 bootstrap: STATUS_MASK write failed",
            ),
            (
                0,
                TmcSafetyBootstrapStage::StatusFlagsRead,
                ShutdownReason::TmcBootstrapCh0StatusFlagsReadFailed,
                "TMC4671 ch0 bootstrap: STATUS_FLAGS read failed",
            ),
            (
                0,
                TmcSafetyBootstrapStage::StatusFlagsClear,
                ShutdownReason::TmcBootstrapCh0StatusFlagsClearFailed,
                "TMC4671 ch0 bootstrap: STATUS_FLAGS clear failed",
            ),
            (
                1,
                TmcSafetyBootstrapStage::AdcVmLimitsWrite,
                ShutdownReason::TmcBootstrapCh1AdcVmLimitsWriteFailed,
                "TMC4671 ch1 bootstrap: ADC_VM_LIMITS write failed",
            ),
            (
                1,
                TmcSafetyBootstrapStage::StatusMaskWrite,
                ShutdownReason::TmcBootstrapCh1StatusMaskWriteFailed,
                "TMC4671 ch1 bootstrap: STATUS_MASK write failed",
            ),
            (
                1,
                TmcSafetyBootstrapStage::StatusFlagsRead,
                ShutdownReason::TmcBootstrapCh1StatusFlagsReadFailed,
                "TMC4671 ch1 bootstrap: STATUS_FLAGS read failed",
            ),
            (
                1,
                TmcSafetyBootstrapStage::StatusFlagsClear,
                ShutdownReason::TmcBootstrapCh1StatusFlagsClearFailed,
                "TMC4671 ch1 bootstrap: STATUS_FLAGS clear failed",
            ),
        ];

        for (channel, stage, expected_reason, expected_message) in cases {
            let reason = failure(stage).shutdown_reason(channel);
            assert_eq!(reason, expected_reason);
            assert_eq!(reason.message(), expected_message);
        }
        assert_eq!(
            failure(TmcSafetyBootstrapStage::StatusMaskWrite).shutdown_reason(2),
            ShutdownReason::InvalidTmcChannel
        );
    }
}
