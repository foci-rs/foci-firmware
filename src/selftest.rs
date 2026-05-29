//! TMC4671 safety primitives and selftest stage-result types.
//!
//! Contains [`restore_safe_state`] (raw `Tmc4671Spi` + GPIO variant of the
//! emergency motor shutdown sequence) and the [`SelftestStageValue`] /
//! [`SelftestResults`] containers streamed to the host during a selftest run.
//!
//! All logic is generic over `embedded-hal-async` traits for cross-board
//! reuse and host-side unit testing.

use embedded_hal::digital::OutputPin;
use embedded_hal_async::spi::SpiDevice;
use foci_tmc4671::spi::Tmc4671Spi;

/// Put the TMC4671 into a safe state: enable pin LOW, all drive inputs and
/// integrator state zeroed, voltage authority capped.
///
/// Raw `Tmc4671Spi` + GPIO variant of the canonical
/// [`crate::tmc_safe_state::restore_safe_state_via_backend`]. It writes the
/// same register sequence in the same order so a caller holding only a raw
/// SPI device and the enable pin (e.g. a board fault path with no
/// `CommissioningBackend`) lands in an identical safe state. Keep the two
/// sequences in lockstep: a divergence is a latent safety gap.
///
/// Best-effort: SPI errors are silently ignored. The enable pin is always
/// driven low first, before any register write, so the half-bridges are
/// gated regardless of prior TMC4671 state.
///
/// Zeroing `PID_TORQUE_FLUX_OFFSET` matters as much as zeroing the target:
/// a stale non-zero offset is added to the current-loop PID output and would
/// inject torque/flux current the instant a later path re-asserts a motion
/// mode. `PHI_E_SELECTION` is forced to a deterministic source and the PID
/// error accumulators are cleared so integrator windup cannot drive current
/// on re-enable. `PIDOUT_UQ_UD_LIMITS` is reset to the conservative default
/// (1024) matching AdcCalibrate so high voltage authority cannot leak across
/// a safe-state transition into a subsequent partial re-enable.
///
/// # Parameters
/// - `tmc`: mutable reference to the TMC4671 SPI driver
/// - `enable_pin`: mutable reference to the motor enable output pin
pub async fn restore_safe_state<SPI, PIN, E>(tmc: &mut Tmc4671Spi<SPI>, enable_pin: &mut PIN)
where
    SPI: SpiDevice<u8, Error = E>,
    PIN: OutputPin,
{
    use foci_tmc4671::registers::*;

    let _ = enable_pin.set_low();
    let _ = tmc.write_register(MODE_RAMP_MODE_MOTION, 0x0000_0000).await;
    let _ = tmc.write_register(PID_TORQUE_FLUX_TARGET, 0).await;
    let _ = tmc.write_register(PID_TORQUE_FLUX_OFFSET, 0).await;
    let _ = tmc.write_register(UQ_UD_EXT, 0).await;
    let _ = tmc.write_register(PHI_E_SELECTION, 0).await;
    let _ = tmc.write_register(PWM_SV_CHOP, 0).await;
    let _ = tmc.write_register(PID_VELOCITY_OFFSET, 0).await;
    // Conservative voltage ceiling matching AdcCalibrate's safety default.
    // Prevents high-voltage drive authority from leaking across a safe-state
    // transition into a subsequent partial re-enable.
    let _ = tmc.write_register(PIDOUT_UQ_UD_LIMITS, 1024).await;
    // Clear PID error accumulators so integrator windup from the last active
    // mode cannot drive corrective current when outputs are re-enabled.
    let _ = tmc
        .write_sub_register(PID_ERROR_PID_TORQUE_ERROR_SUM, 0)
        .await;
    let _ = tmc
        .write_sub_register(PID_ERROR_PID_FLUX_ERROR_SUM, 0)
        .await;
    let _ = tmc
        .write_sub_register(PID_ERROR_PID_VELOCITY_ERROR_SUM, 0)
        .await;
    let _ = tmc
        .write_sub_register(PID_ERROR_PID_POSITION_ERROR_SUM, 0)
        .await;
}

/// Result of one selftest stage, streamed to the host.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, defmt::Format)]
pub struct SelftestStageValue {
    /// 0 = PASS, 1 = FAIL, 2 = SKIP.
    pub status: u8,
    /// Stage-specific encoded value.
    pub value: u32,
}

/// Maximum number of commissioning diagnostic events retained per run.
pub const COMMISSION_DIAGNOSTIC_CAPACITY: usize = 16;

/// Structured commissioning diagnostic event streamed to the host.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, defmt::Format)]
pub struct CommissionDiagnostic {
    /// Commissioning phase identifier.
    pub phase: u8,
    /// Phase-local diagnostic code.
    pub code: u8,
    /// Diagnostic status (0 = measurement, 1 = rejection/failure, 2 = warning).
    pub status: u8,
    /// First code-specific value.
    pub value0: u32,
    /// Second code-specific value.
    pub value1: u32,
    /// Third code-specific value.
    pub value2: u32,
}

/// Bounded commissioning diagnostic event buffer.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct CommissionDiagnostics {
    events: heapless::Vec<CommissionDiagnostic, COMMISSION_DIAGNOSTIC_CAPACITY>,
}

impl CommissionDiagnostics {
    /// Append one event. Events beyond capacity are dropped.
    pub fn push(&mut self, event: CommissionDiagnostic) {
        let _ = self.events.push(event);
    }

    /// Iterate retained events in insertion order.
    pub fn iter(&self) -> impl Iterator<Item = CommissionDiagnostic> + '_ {
        self.events.iter().copied()
    }
}

impl SelftestStageValue {
    /// Pack two ADC channel offsets into a single u32 for stage 1.
    pub fn adc_offsets(offset_i0: u16, offset_i1: u16) -> Self {
        Self {
            status: 0,
            value: ((offset_i1 as u32) << 16) | offset_i0 as u32,
        }
    }

    /// Coil current measurement (stages 2, 3).
    ///
    /// The i16 is bit-reinterpreted as u16 (two's-complement bit pattern
    /// preserved), then zero-extended to u32. The host decoder must treat
    /// bit 15 of the lower 16 bits as the sign bit to recover negative
    /// values (values >= 0x8000 in the low 16 bits represent negative i16).
    pub fn coil_current(current: i16) -> Self {
        Self {
            status: 0,
            value: current as u16 as u32,
        }
    }

    /// Constructor for a failed stage. `value` may carry an error code or
    /// zero if no detail is available.
    pub fn fail(value: u32) -> Self {
        Self { status: 1, value }
    }

    /// Constructor for a skipped stage.
    pub fn skip() -> Self {
        Self {
            status: 2,
            value: 0,
        }
    }
}

/// Per-stage results collected during a selftest run.
///
/// Stages are populated in phase order as they complete; `None` means
/// the stage was skipped or aborted before running.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, defmt::Format)]
pub struct SelftestResults {
    stages: [Option<SelftestStageValue>; 8],
}

impl SelftestResults {
    /// Write a stage result. `stage` is 1-indexed (1..=8). Out-of-range
    /// stages are silently ignored.
    pub fn set(&mut self, stage: u8, value: SelftestStageValue) {
        if (1..=8).contains(&stage) {
            self.stages[(stage - 1) as usize] = Some(value);
        }
    }

    /// Read a stage result. Returns `None` if unpopulated or out of range.
    pub fn stage(&self, stage: u8) -> Option<SelftestStageValue> {
        if (1..=8).contains(&stage) {
            self.stages[(stage - 1) as usize]
        } else {
            None
        }
    }

    /// Iterate populated stages in order, yielding `(stage_id, value)` pairs.
    pub fn iter(&self) -> impl Iterator<Item = (u8, SelftestStageValue)> + '_ {
        self.stages
            .iter()
            .enumerate()
            .filter_map(|(i, o)| o.map(|v| (i as u8 + 1, v)))
    }
}

#[cfg(test)]
mod selftest_results_tests {
    use super::*;

    #[test]
    fn results_default_has_all_stages_unpopulated() {
        let r = SelftestResults::default();
        for stage in 1..=8u8 {
            assert!(r.stage(stage).is_none(), "stage {} should be None", stage);
        }
    }

    #[test]
    fn set_and_get_round_trips_value() {
        let mut r = SelftestResults::default();
        r.set(
            3,
            SelftestStageValue {
                status: 0,
                value: 0x1234,
            },
        );
        let got = r.stage(3).expect("stage 3 populated");
        assert_eq!(got.status, 0);
        assert_eq!(got.value, 0x1234);
    }

    #[test]
    fn adc_offsets_pack_into_value() {
        let v = SelftestStageValue::adc_offsets(0x1111, 0x2222);
        assert_eq!(v.value, (0x2222_u32 << 16) | 0x1111_u32);
        assert_eq!(v.status, 0);
    }

    #[test]
    fn fail_constructor_sets_status_one() {
        let v = SelftestStageValue::fail(42);
        assert_eq!(v.status, 1);
        assert_eq!(v.value, 42);
    }

    #[test]
    fn skip_constructor_sets_status_two_and_zero_value() {
        let v = SelftestStageValue::skip();
        assert_eq!(v.status, 2);
        assert_eq!(v.value, 0);
    }

    #[test]
    fn commission_diagnostics_iterate_in_push_order() {
        let mut diagnostics = CommissionDiagnostics::default();
        let first = CommissionDiagnostic {
            phase: 5,
            code: 1,
            status: 0,
            value0: 10,
            value1: 20,
            value2: 30,
        };
        let second = CommissionDiagnostic {
            phase: 5,
            code: 29,
            status: 1,
            value0: 40,
            value1: 50,
            value2: 60,
        };

        diagnostics.push(first);
        diagnostics.push(second);

        let items: heapless::Vec<_, 4> = diagnostics.iter().collect();
        assert_eq!(items.as_slice(), &[first, second]);
    }

    #[test]
    fn commission_diagnostics_drop_events_after_capacity() {
        let mut diagnostics = CommissionDiagnostics::default();
        for code in 0..(COMMISSION_DIAGNOSTIC_CAPACITY as u8 + 2) {
            diagnostics.push(CommissionDiagnostic {
                phase: 5,
                code,
                status: 0,
                value0: code as u32,
                value1: 0,
                value2: 0,
            });
        }

        let items: heapless::Vec<_, COMMISSION_DIAGNOSTIC_CAPACITY> = diagnostics.iter().collect();
        assert_eq!(items.len(), COMMISSION_DIAGNOSTIC_CAPACITY);
        assert_eq!(items[0].code, 0);
        assert_eq!(
            items[COMMISSION_DIAGNOSTIC_CAPACITY - 1].code,
            COMMISSION_DIAGNOSTIC_CAPACITY as u8 - 1
        );
    }
}

#[cfg(test)]
mod restore_safe_state_tests {
    use super::*;
    use embedded_hal_async::spi::{ErrorType, Operation, SpiDevice};
    use foci_tmc4671::registers::*;

    /// Records every 5-byte register write as `(addr, value)` in order.
    #[derive(Default)]
    struct RecordingSpi {
        writes: std::vec::Vec<(u8, u32)>,
    }

    impl ErrorType for RecordingSpi {
        type Error = core::convert::Infallible;
    }

    impl SpiDevice<u8> for RecordingSpi {
        async fn transaction(
            &mut self,
            operations: &mut [Operation<'_, u8>],
        ) -> Result<(), Self::Error> {
            for op in operations {
                if let Operation::Write(buf) = op
                    && buf.len() == 5
                {
                    let addr = buf[0] & 0x7F;
                    let value = u32::from_be_bytes([buf[1], buf[2], buf[3], buf[4]]);
                    self.writes.push((addr, value));
                }
            }
            Ok(())
        }
    }

    struct RecordingPin {
        is_high: bool,
    }

    impl embedded_hal::digital::ErrorType for RecordingPin {
        type Error = core::convert::Infallible;
    }

    impl embedded_hal::digital::OutputPin for RecordingPin {
        fn set_low(&mut self) -> Result<(), Self::Error> {
            self.is_high = false;
            Ok(())
        }

        fn set_high(&mut self) -> Result<(), Self::Error> {
            self.is_high = true;
            Ok(())
        }
    }

    /// The raw safe-state must write the same register sequence, in the same
    /// order, as the canonical `restore_safe_state_via_backend`. A divergence
    /// is a latent safety gap: a board fault path wired to this primitive would
    /// otherwise leave a non-zero torque/flux offset or stale integrator that
    /// re-energizes the motor on the next partial re-enable.
    #[test]
    fn restore_safe_state_matches_canonical_register_sequence() {
        pollster::block_on(async {
            let mut tmc = Tmc4671Spi::new(RecordingSpi::default());
            let mut enable_pin = RecordingPin { is_high: true };

            restore_safe_state(&mut tmc, &mut enable_pin).await;

            assert!(!enable_pin.is_high, "enable pin must be driven low");

            let writes = tmc.into_inner().writes;
            assert_eq!(
                writes,
                std::vec![
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
                ],
            );
        });
    }
}
