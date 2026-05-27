//! Canonical TMC4671 safe-state restore primitive.
//!
//! State-neutral: writes the well-known safe-state register sequence and
//! lowers the enable pin via [`crate::commissioning::types::CommissioningBackend`].
//! Higher-level lifecycle helpers like [`crate::tmc_ops::disarm_motor`] still
//! own `DispatchState` updates; this helper only touches the chip.
//!
//! Best-effort: every write is attempted even if an earlier write fails.
//! Returns `()` so commissioning failure paths cannot accidentally surface
//! safe-state SPI errors as the user-visible result.

use crate::commissioning::types::CommissioningBackend;
use foci_tmc4671::registers::Register;

/// Diagnostic summary for a safe-state restore attempt.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub(crate) struct SafeStateRestoreReport {
    /// Number of verified safe-state writes that exhausted retries.
    pub(crate) failed_writes: u8,
}

/// Restore the TMC4671 to a safe non-energized state via `CommissioningBackend`.
///
/// Sequence (each step best-effort):
/// 1. `set_enable_pin(false)`
/// 2. `MODE_RAMP_MODE_MOTION = 0`
/// 3. `PID_TORQUE_FLUX_TARGET = 0`
/// 4. `PID_TORQUE_FLUX_OFFSET = 0`
/// 5. `UQ_UD_EXT = 0`
/// 6. `PHI_E_SELECTION = 0`
/// 7. `PWM_SV_CHOP = 0`
/// 8. `PID_VELOCITY_OFFSET = 0`
/// 9. `PIDOUT_UQ_UD_LIMITS = 1024`
/// 10. Clear PID error sums
pub async fn restore_safe_state_via_backend<B>(backend: &mut B)
where
    B: CommissioningBackend,
{
    let report = restore_safe_state_via_backend_report(backend).await;
    if report.failed_writes > 0 {
        defmt::error!(
            "safe-state restore had {=u8} verified-write failures after retries",
            report.failed_writes
        );
    }
}

pub(crate) async fn restore_safe_state_via_backend_report<B>(
    backend: &mut B,
) -> SafeStateRestoreReport
where
    B: CommissioningBackend,
{
    use foci_tmc4671::registers::*;

    let mut report = SafeStateRestoreReport::default();
    backend.set_enable_pin(false);
    safe_state_write(backend, MODE_RAMP_MODE_MOTION, 0, &mut report).await;
    safe_state_write(backend, PID_TORQUE_FLUX_TARGET, 0, &mut report).await;
    safe_state_write(backend, PID_TORQUE_FLUX_OFFSET, 0, &mut report).await;
    safe_state_write(backend, UQ_UD_EXT, 0, &mut report).await;
    safe_state_write(backend, PHI_E_SELECTION, 0, &mut report).await;
    safe_state_write(backend, PWM_SV_CHOP, 0, &mut report).await;
    safe_state_write(backend, PID_VELOCITY_OFFSET, 0, &mut report).await;
    safe_state_write(backend, PIDOUT_UQ_UD_LIMITS, 1024, &mut report).await;
    let _ = crate::tmc_ops::clear_pid_error_sums_via_backend(backend).await;
    report
}

async fn safe_state_write<B>(
    backend: &mut B,
    reg: Register,
    val: u32,
    report: &mut SafeStateRestoreReport,
) where
    B: CommissioningBackend,
{
    if backend.write_register_verified(reg, val).await.is_err() {
        report.failed_writes = report.failed_writes.saturating_add(1);
        defmt::warn!(
            "safe-state restore write failed after retries: reg={=u8}",
            reg.addr()
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::commission_dispatch::test_support::{MockBackend, SpiOp};
    use foci_tmc4671::registers::*;

    #[test]
    fn restore_writes_canonical_sequence_in_order() {
        pollster::block_on(async {
            let mut backend = MockBackend::new();
            backend.enable_pin = true;
            restore_safe_state_via_backend(&mut backend).await;

            assert!(!backend.enable_pin, "enable pin must be lowered");
            let mut writes: heapless::Vec<(u8, u32), 16> = heapless::Vec::new();
            for op in backend.spi_log.iter() {
                if let SpiOp::Write(addr, val) = op {
                    writes.push((*addr, *val)).ok();
                }
            }
            assert_eq!(
                writes.as_slice(),
                &[
                    (MODE_RAMP_MODE_MOTION.addr(), 0),
                    (PID_TORQUE_FLUX_TARGET.addr(), 0),
                    (PID_TORQUE_FLUX_OFFSET.addr(), 0),
                    (UQ_UD_EXT.addr(), 0),
                    (PHI_E_SELECTION.addr(), 0),
                    (PWM_SV_CHOP.addr(), 0),
                    (PID_VELOCITY_OFFSET.addr(), 0),
                    (PIDOUT_UQ_UD_LIMITS.addr(), 1024),
                    (
                        PID_ERROR_PID_TORQUE_ERROR_SUM.addr.addr(),
                        PID_ERROR_PID_TORQUE_ERROR_SUM.sub_addr as u32,
                    ),
                    (PID_ERROR_PID_TORQUE_ERROR_SUM.data.addr(), 0),
                    (
                        PID_ERROR_PID_FLUX_ERROR_SUM.addr.addr(),
                        PID_ERROR_PID_FLUX_ERROR_SUM.sub_addr as u32,
                    ),
                    (PID_ERROR_PID_FLUX_ERROR_SUM.data.addr(), 0),
                    (
                        PID_ERROR_PID_VELOCITY_ERROR_SUM.addr.addr(),
                        PID_ERROR_PID_VELOCITY_ERROR_SUM.sub_addr as u32,
                    ),
                    (PID_ERROR_PID_VELOCITY_ERROR_SUM.data.addr(), 0),
                    (
                        PID_ERROR_PID_POSITION_ERROR_SUM.addr.addr(),
                        PID_ERROR_PID_POSITION_ERROR_SUM.sub_addr as u32,
                    ),
                    (PID_ERROR_PID_POSITION_ERROR_SUM.data.addr(), 0),
                ]
            );
        });
    }

    #[test]
    fn restore_keeps_writing_after_a_failure() {
        pollster::block_on(async {
            let mut backend = MockBackend::new();
            backend.fail_on_write = Some(UQ_UD_EXT.addr());
            restore_safe_state_via_backend(&mut backend).await;

            let mut attempted: heapless::Vec<u8, 16> = heapless::Vec::new();
            for op in backend.spi_log.iter() {
                if let SpiOp::Write(addr, _) = op {
                    attempted.push(*addr).ok();
                }
            }
            assert!(attempted.contains(&PID_TORQUE_FLUX_OFFSET.addr()));
            assert!(attempted.contains(&PWM_SV_CHOP.addr()));
            assert!(attempted.contains(&PID_VELOCITY_OFFSET.addr()));
            assert!(attempted.contains(&PIDOUT_UQ_UD_LIMITS.addr()));
        });
    }

    #[test]
    fn restore_reports_write_failures_for_diagnostics() {
        pollster::block_on(async {
            let mut backend = MockBackend::new();
            backend.fail_on_write = Some(UQ_UD_EXT.addr());

            let report = restore_safe_state_via_backend_report(&mut backend).await;

            assert_eq!(report.failed_writes, 1);
        });
    }

    #[test]
    fn safe_state_writes_phi_e_selection_zero() {
        pollster::block_on(async {
            let mut backend = MockBackend::new();
            backend.enable_pin = true;

            restore_safe_state_via_backend(&mut backend).await;

            let writes: heapless::Vec<(u8, u32), 16> = backend
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
            assert!(
                writes
                    .iter()
                    .any(|(addr, val)| *addr == PHI_E_SELECTION.addr() && *val == 0),
                "expected PHI_E_SELECTION=0 in safe-state sequence, log={writes:?}",
            );
        });
    }

    #[test]
    fn safe_state_clears_pid_error_sums() {
        pollster::block_on(async {
            let mut backend = MockBackend::new();
            backend.enable_pin = true;

            restore_safe_state_via_backend(&mut backend).await;

            let writes: heapless::Vec<(u8, u32), 32> = backend
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
            let position_err_sum_addr = PID_ERROR_PID_POSITION_ERROR_SUM.addr.addr();
            assert!(
                writes
                    .iter()
                    .any(|(addr, _)| *addr == position_err_sum_addr),
                "expected PID_ERROR sub-register access for clear; log={writes:?}",
            );
        });
    }

    #[test]
    fn safe_state_uses_verified_writes() {
        pollster::block_on(async {
            let mut backend = MockBackend::new();
            backend.read_override = Some((MODE_RAMP_MODE_MOTION.addr(), 0xFFFF_FFFF));

            let report = restore_safe_state_via_backend_report(&mut backend).await;

            assert!(
                report.failed_writes >= 1,
                "expected at least one failed verified write"
            );
        });
    }
}
