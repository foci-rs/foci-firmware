//! Canonical TMC4671 safe-state restore: writes the safe-state register sequence and lowers the
//! enable pin. Best-effort (every write is attempted), returns `()`, and touches only the chip,
//! not `DispatchState`.

use crate::commissioning::types::CommissioningBackend;
use crate::tmc_register_values::{MOTION_STOPPED, PWM_CHOP_OFF};
use foci_tmc4671::registers::Register;

/// Diagnostic summary for a safe-state restore attempt.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct SafeStateRestoreReport {
    /// Number of verified safe-state writes that exhausted retries.
    pub failed_writes: u8,
    /// Address of the first register or sub-register write that failed.
    pub first_failed_register: Option<u8>,
    /// Logical sub-register selector (4=torque, 5=flux, 6=velocity, 7=position) when the first
    /// failure was a PID error-sum clear; the four sums share physical registers.
    pub first_failed_sub_addr: Option<u8>,
}

/// One step of the safe-state restore sequence, advanced by exactly one backend register
/// operation per call. Steps 0-7 are the eight [`safe_state_write`] calls; steps 8-15 are the four
/// PID error-sum clears (addr-select then data per pair: torque, flux, velocity, position). The
/// enable pin is lowered synchronously with step 0.
#[derive(Clone, Copy, Default)]
pub(crate) struct SafeStateRestoreStep {
    index: u8,
    report: SafeStateRestoreReport,
}

const SAFE_STATE_STEP_COUNT: u8 = 16;

impl SafeStateRestoreStep {
    /// True before any step has run.
    pub(crate) const fn is_start(&self) -> bool {
        self.index == 0
    }

    /// Diagnostic report accumulated so far; final once `advance` has returned `false`.
    pub(crate) const fn report(&self) -> SafeStateRestoreReport {
        self.report
    }

    /// Performs the next backend register operation; `true` while steps remain. On `false` the
    /// report has been recorded via `backend.record_safe_state_report`.
    pub(crate) async fn advance<B: CommissioningBackend>(&mut self, backend: &mut B) -> bool {
        use foci_tmc4671::registers::*;

        if self.index == 0 {
            backend.set_enable_pin(false);
        }
        match self.index {
            0 => {
                safe_state_write(
                    backend,
                    MODE_RAMP_MODE_MOTION,
                    MOTION_STOPPED,
                    &mut self.report,
                )
                .await
            }
            1 => safe_state_write(backend, PID_TORQUE_FLUX_TARGET, 0, &mut self.report).await,
            2 => safe_state_write(backend, PID_TORQUE_FLUX_OFFSET, 0, &mut self.report).await,
            3 => safe_state_write(backend, UQ_UD_EXT, 0, &mut self.report).await,
            4 => safe_state_write(backend, PHI_E_SELECTION, 0, &mut self.report).await,
            5 => safe_state_write(backend, PWM_SV_CHOP, PWM_CHOP_OFF, &mut self.report).await,
            6 => safe_state_write(backend, PID_VELOCITY_OFFSET, 0, &mut self.report).await,
            7 => safe_state_write(backend, PIDOUT_UQ_UD_LIMITS, 1024, &mut self.report).await,
            8 => {
                let sub = PID_ERROR_PID_TORQUE_ERROR_SUM;
                record_sub_register_write(
                    backend,
                    sub.addr,
                    u32::from(sub.sub_addr),
                    sub.sub_addr,
                    &mut self.report,
                )
                .await;
            }
            9 => {
                let sub = PID_ERROR_PID_TORQUE_ERROR_SUM;
                record_sub_register_write(backend, sub.data, 0, sub.sub_addr, &mut self.report)
                    .await;
            }
            10 => {
                let sub = PID_ERROR_PID_FLUX_ERROR_SUM;
                record_sub_register_write(
                    backend,
                    sub.addr,
                    u32::from(sub.sub_addr),
                    sub.sub_addr,
                    &mut self.report,
                )
                .await;
            }
            11 => {
                let sub = PID_ERROR_PID_FLUX_ERROR_SUM;
                record_sub_register_write(backend, sub.data, 0, sub.sub_addr, &mut self.report)
                    .await;
            }
            12 => {
                let sub = PID_ERROR_PID_VELOCITY_ERROR_SUM;
                record_sub_register_write(
                    backend,
                    sub.addr,
                    u32::from(sub.sub_addr),
                    sub.sub_addr,
                    &mut self.report,
                )
                .await;
            }
            13 => {
                let sub = PID_ERROR_PID_VELOCITY_ERROR_SUM;
                record_sub_register_write(backend, sub.data, 0, sub.sub_addr, &mut self.report)
                    .await;
            }
            14 => {
                let sub = PID_ERROR_PID_POSITION_ERROR_SUM;
                record_sub_register_write(
                    backend,
                    sub.addr,
                    u32::from(sub.sub_addr),
                    sub.sub_addr,
                    &mut self.report,
                )
                .await;
            }
            15 => {
                let sub = PID_ERROR_PID_POSITION_ERROR_SUM;
                record_sub_register_write(backend, sub.data, 0, sub.sub_addr, &mut self.report)
                    .await;
            }
            _ => unreachable!("SafeStateRestoreStep polled after completion"),
        }
        self.index += 1;
        if self.index < SAFE_STATE_STEP_COUNT {
            return true;
        }
        let timestamp_ms = (backend.elapsed_us() / 1000) as u32;
        backend.record_safe_state_report(self.report, timestamp_ms);
        false
    }
}

/// Restore the TMC4671 to a safe non-energized state via `CommissioningBackend`; every step is
/// best-effort.
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
    let mut step = SafeStateRestoreStep::default();
    while step.advance(backend).await {}
    step.report
}

async fn record_sub_register_write<B>(
    backend: &mut B,
    reg: Register,
    val: u32,
    sub_addr: u8,
    report: &mut SafeStateRestoreReport,
) where
    B: CommissioningBackend,
{
    if backend.write_register(reg, val).await.is_err() {
        report.failed_writes = report.failed_writes.saturating_add(1);
        if report.first_failed_register.is_none() {
            report.first_failed_register = Some(reg.addr());
            report.first_failed_sub_addr = Some(sub_addr);
        }
        defmt::warn!(
            "safe-state restore PID error-sum clear write failed: reg={=u8}",
            reg.addr()
        );
    }
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
        if report.first_failed_register.is_none() {
            report.first_failed_register = Some(reg.addr());
        }
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
    use foci_firmware_macros::fixture_test;
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
    fn restore_persists_last_pass_failed_via_backend() {
        pollster::block_on(async {
            let mut backend = MockBackend::new();
            backend.fail_on_write = Some(UQ_UD_EXT.addr());

            restore_safe_state_via_backend(&mut backend).await;

            assert!(backend.safe_state_last_pass_failed);
            let recorded = backend
                .safe_state_report
                .expect("record_safe_state_report should have been called");
            assert_eq!(recorded.failed_writes, 1);
            assert_eq!(recorded.first_failed_register, Some(UQ_UD_EXT.addr()));
        });
    }

    #[test]
    fn restore_tracks_total_pass_count_regardless_of_outcome() {
        pollster::block_on(async {
            let mut backend = MockBackend::new();
            backend.fail_on_write = Some(UQ_UD_EXT.addr());

            restore_safe_state_via_backend(&mut backend).await;
            assert_eq!(backend.safe_state_pass_count, 1);

            restore_safe_state_via_backend(&mut backend).await;
            assert_eq!(backend.safe_state_pass_count, 2);

            backend.fail_on_write = None;
            restore_safe_state_via_backend(&mut backend).await;
            assert_eq!(
                backend.safe_state_pass_count, 3,
                "a clean pass still counts as a completed restore attempt"
            );
            assert!(
                !backend.safe_state_last_pass_failed,
                "the clean pass should still update the last-pass-failed flag"
            );
        });
    }

    #[test]
    fn restore_clears_last_pass_failed_on_clean_pass() {
        pollster::block_on(async {
            let mut backend = MockBackend::new();

            restore_safe_state_via_backend(&mut backend).await;

            assert!(!backend.safe_state_last_pass_failed);
        });
    }

    #[test]
    fn restore_alone_never_clears_the_compound_incomplete_latch() {
        pollster::block_on(async {
            let mut backend = MockBackend::new();

            restore_safe_state_via_backend(&mut backend).await;

            assert!(
                backend.safe_state_incomplete,
                "a restore pass alone does not confirm a fresh sentinel pass; \
                 only arm_motor_electrical may clear this latch"
            );
        });
    }

    #[test]
    fn restore_counts_pid_error_sum_clear_failures() {
        pollster::block_on(async {
            let mut backend = MockBackend::new();
            backend.fail_on_write = Some(PID_ERROR_PID_TORQUE_ERROR_SUM.addr.addr());

            let report = restore_safe_state_via_backend_report(&mut backend).await;

            assert_eq!(
                report.failed_writes, 4,
                "all four sum-clear selector writes share this address and should all fail"
            );
            assert_eq!(
                report.first_failed_register,
                Some(PID_ERROR_PID_TORQUE_ERROR_SUM.addr.addr())
            );
            assert_eq!(
                report.first_failed_sub_addr,
                Some(PID_ERROR_PID_TORQUE_ERROR_SUM.sub_addr),
                "torque is the first sum in iteration order"
            );
        });
    }

    #[test]
    fn restore_identifies_which_pid_sum_failed_when_not_the_first() {
        pollster::block_on(async {
            let mut backend = MockBackend::new();
            backend.fail_on_write_occurrence =
                Some((PID_ERROR_PID_TORQUE_ERROR_SUM.addr.addr(), 2));

            let report = restore_safe_state_via_backend_report(&mut backend).await;

            assert_eq!(
                report.first_failed_register,
                Some(PID_ERROR_PID_TORQUE_ERROR_SUM.addr.addr()),
                "torque, flux, and velocity selector writes share this physical address"
            );
            assert_eq!(
                report.first_failed_sub_addr,
                Some(PID_ERROR_PID_VELOCITY_ERROR_SUM.sub_addr),
                "the shared physical address alone cannot tell torque/flux/velocity/position apart"
            );
        });
    }

    #[test]
    fn restore_reports_first_failed_register_on_write_failure() {
        pollster::block_on(async {
            let mut backend = MockBackend::new();
            backend.fail_on_write = Some(UQ_UD_EXT.addr());

            let report = restore_safe_state_via_backend_report(&mut backend).await;

            assert_eq!(report.first_failed_register, Some(UQ_UD_EXT.addr()));
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

    #[fixture_test]
    async fn safe_state_restore_step_advances_one_logical_op_per_call() {
        let mut backend = MockBackend::new();
        backend.enable_pin = true;
        let mut step = SafeStateRestoreStep::default();
        let mut calls = 0u32;
        loop {
            assert!(calls < 32, "step sequence did not terminate");
            let verified_before = backend.verified_write_calls;
            let attributed_before = backend.raw_ops_attributed_to_verified_calls;
            let ops_before = backend.spi_log.len();
            let more = step.advance(&mut backend).await;
            let verified_delta = backend.verified_write_calls - verified_before;
            let attributed_delta = backend.raw_ops_attributed_to_verified_calls - attributed_before;
            let total_raw = backend.spi_log[ops_before..]
                .iter()
                .filter(|op| matches!(op, SpiOp::Write(_, _) | SpiOp::Read))
                .count();
            let outside_delta = total_raw.saturating_sub(attributed_delta);
            let ok = match verified_delta {
                0 => outside_delta <= 1,
                1 => outside_delta == 0,
                _ => false,
            };
            assert!(
                ok,
                "call {calls}: {verified_delta} verified-write call(s), {outside_delta} raw \
                 register op(s) unattributed to any verified call (of {total_raw} total) -- \
                 expected at most one logical register operation"
            );
            calls += 1;
            if !more {
                break;
            }
        }
        assert_eq!(calls, 16, "expected exactly 16 steps");
        assert!(!backend.enable_pin, "enable pin must be lowered");
    }
}
