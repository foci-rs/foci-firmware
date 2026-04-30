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

/// Restore the TMC4671 to a safe non-energized state via `CommissioningBackend`.
///
/// Sequence (each step best-effort):
/// 1. `set_enable_pin(false)`
/// 2. `MODE_RAMP_MODE_MOTION = 0`
/// 3. `PID_TORQUE_FLUX_TARGET = 0`
/// 4. `UQ_UD_EXT = 0`
/// 5. `PWM_SV_CHOP = 0`
/// 6. `PID_VELOCITY_OFFSET = 0`
/// 7. `PIDOUT_UQ_UD_LIMITS = 1024`
pub async fn restore_safe_state_via_backend<B>(backend: &mut B)
where
    B: CommissioningBackend,
{
    use foci_tmc4671::registers::*;

    backend.set_enable_pin(false);
    let _ = backend
        .write_register(MODE_RAMP_MODE_MOTION, 0x0000_0000)
        .await;
    let _ = backend.write_register(PID_TORQUE_FLUX_TARGET, 0).await;
    let _ = backend.write_register(UQ_UD_EXT, 0).await;
    let _ = backend.write_register(PWM_SV_CHOP, 0).await;
    let _ = backend.write_register(PID_VELOCITY_OFFSET, 0).await;
    let _ = backend.write_register(PIDOUT_UQ_UD_LIMITS, 1024).await;
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
                    (UQ_UD_EXT.addr(), 0),
                    (PWM_SV_CHOP.addr(), 0),
                    (PID_VELOCITY_OFFSET.addr(), 0),
                    (PIDOUT_UQ_UD_LIMITS.addr(), 1024),
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
            assert!(attempted.contains(&PWM_SV_CHOP.addr()));
            assert!(attempted.contains(&PID_VELOCITY_OFFSET.addr()));
            assert!(attempted.contains(&PIDOUT_UQ_UD_LIMITS.addr()));
        });
    }
}
