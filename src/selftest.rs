//! TMC4671 self-test diagnostic.
//!
//! Runs an 8-stage hardware diagnostic: ADC calibration, motor presence
//! (per coil), phase wiring, encoder presence, encoder direction,
//! resistance, and inductance measurement.
//!
//! All logic is generic over `embedded-hal-async` traits for cross-board
//! reuse and host-side unit testing.

use embedded_hal::digital::OutputPin;
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::spi::SpiDevice;
use tmc4671::spi::Tmc4671Spi;

/// Stage numbers for `foci_selftest_result`.
pub const STAGE_ADC_CAL: u8 = 1;
/// Stage number for motor coil A presence check.
pub const STAGE_MOTOR_COIL_A: u8 = 2;
/// Stage number for motor coil B presence check.
pub const STAGE_MOTOR_COIL_B: u8 = 3;
/// Stage number for phase wiring check.
pub const STAGE_PHASE_WIRING: u8 = 4;
/// Stage number for encoder presence check.
pub const STAGE_ENCODER_PRESENCE: u8 = 5;
/// Stage number for encoder direction check.
pub const STAGE_ENCODER_DIRECTION: u8 = 6;
/// Stage number for winding resistance measurement.
pub const STAGE_RESISTANCE: u8 = 7;
/// Stage number for winding inductance measurement.
pub const STAGE_INDUCTANCE: u8 = 8;

/// Per-stage status: stage passed all checks.
pub const SELFTEST_PASS: u8 = 0;
/// Per-stage status: stage detected a fault.
pub const SELFTEST_FAIL: u8 = 1;
/// Per-stage status: stage was skipped (prerequisite failed).
pub const SELFTEST_SKIP: u8 = 2;

/// Overall selftest completion status: all stages passed.
pub const SELFTEST_DONE_PASS: u8 = 0;
/// Overall selftest completion status: one or more stages failed.
pub const SELFTEST_DONE_FAIL: u8 = 1;
/// Overall selftest completion status: sequence aborted before completion.
pub const SELFTEST_DONE_ABORTED: u8 = 2;

/// Callback for streaming selftest results to the host.
///
/// Firmware implements this with `klipper_reply!` calls.
/// Test code implements this with a vector/array collector.
pub trait SelftestReporter {
    /// Report the result of a single selftest stage.
    ///
    /// # Parameters
    /// - `stage`: 1-8, matching `STAGE_*` constants
    /// - `status`: `SELFTEST_PASS`, `SELFTEST_FAIL`, or `SELFTEST_SKIP`
    /// - `value`: stage-specific measurement (0 if not applicable)
    fn report_stage(&mut self, stage: u8, status: u8, value: u32);
}

/// Run the full 8-stage selftest sequence.
///
/// The caller must ensure the motor is disabled before calling.
/// Results are streamed via `reporter` as each stage completes.
/// On completion (pass or fail), the TMC4671 is left in safe state
/// (MODE=0, targets zeroed, enable pin LOW).
///
/// # Parameters
/// - `_tmc`: mutable reference to the TMC4671 SPI driver
/// - `_enable_pin`: mutable reference to the motor enable output pin
/// - `_delay`: mutable reference to an async delay source
/// - `_current_limit`: maximum current in milliamps for test excitation
/// - `_reporter`: mutable reference to the result callback
///
/// # Returns
/// Overall status: `SELFTEST_DONE_PASS`, `SELFTEST_DONE_FAIL`,
/// or `SELFTEST_DONE_ABORTED`.
pub async fn run_selftest<SPI, PIN, D, E>(
    _tmc: &mut Tmc4671Spi<SPI>,
    _enable_pin: &mut PIN,
    _delay: &mut D,
    _current_limit: u32,
    _reporter: &mut impl SelftestReporter,
) -> u8
where
    SPI: SpiDevice<u8, Error = E>,
    PIN: OutputPin,
    D: DelayNs,
{
    // Stub — stages will be implemented in subsequent tasks
    SELFTEST_DONE_PASS
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A mock reporter that collects results for assertions.
    pub(crate) struct MockReporter {
        pub results: [(u8, u8, u32); 8],
        pub count: usize,
    }

    impl MockReporter {
        pub fn new() -> Self {
            Self {
                results: [(0, 0, 0); 8],
                count: 0,
            }
        }
    }

    impl SelftestReporter for MockReporter {
        fn report_stage(&mut self, stage: u8, status: u8, value: u32) {
            if self.count < 8 {
                self.results[self.count] = (stage, status, value);
                self.count += 1;
            }
        }
    }

    #[test]
    fn mock_reporter_collects_results() {
        let mut r = MockReporter::new();
        r.report_stage(1, SELFTEST_PASS, 33012);
        r.report_stage(2, SELFTEST_FAIL, 0);
        assert_eq!(r.count, 2);
        assert_eq!(r.results[0], (1, SELFTEST_PASS, 33012));
        assert_eq!(r.results[1], (2, SELFTEST_FAIL, 0));
    }

    #[test]
    fn status_constants_are_distinct() {
        assert_ne!(SELFTEST_PASS, SELFTEST_FAIL);
        assert_ne!(SELFTEST_FAIL, SELFTEST_SKIP);
        assert_ne!(SELFTEST_PASS, SELFTEST_SKIP);
    }

    #[test]
    fn done_constants_are_distinct() {
        assert_ne!(SELFTEST_DONE_PASS, SELFTEST_DONE_FAIL);
        assert_ne!(SELFTEST_DONE_FAIL, SELFTEST_DONE_ABORTED);
    }
}
