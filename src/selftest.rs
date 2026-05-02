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

/// Put the TMC4671 into a safe state: MODE=0, targets zeroed, enable pin LOW.
///
/// Best-effort: SPI errors are silently ignored. The enable pin is always
/// driven low regardless of prior TMC4671 state.
///
/// Also resets `PIDOUT_UQ_UD_LIMITS` to the conservative default (1024)
/// so that any subsequent partial re-enable path inherits a low voltage
/// ceiling instead of whatever closed-loop entry last raised it to. This
/// mirrors the value AdcCalibrate writes as its safety default, and
/// prevents runaway current in fault recovery paths that re-assert PWM
/// without running full closed-loop initialization.
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
    let _ = tmc.write_register(UQ_UD_EXT, 0).await;
    let _ = tmc.write_register(PWM_SV_CHOP, 0).await;
    let _ = tmc.write_register(PID_VELOCITY_OFFSET, 0).await;
    // Conservative voltage ceiling matching AdcCalibrate's safety default.
    // Prevents high-voltage drive authority from leaking across a safe-state
    // transition into a subsequent partial re-enable.
    let _ = tmc.write_register(PIDOUT_UQ_UD_LIMITS, 1024).await;
}

/// Result of one selftest stage, streamed to the host.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, defmt::Format)]
pub struct SelftestStageValue {
    /// 0 = PASS, 1 = FAIL, 2 = SKIP.
    pub status: u8,
    /// Stage-specific encoded value.
    pub value: u32,
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
}
