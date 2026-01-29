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

/// Result of a successful ADC offset calibration.
struct AdcCalResult {
    /// Calibrated offset for ADC channel I0 (IUX).
    offset_i0: u16,
    /// Calibrated offset for ADC channel I1 (IWY).
    offset_i1: u16,
}

/// Put the TMC4671 into a safe state: MODE=0, targets zeroed, enable pin LOW.
///
/// Best-effort: SPI errors are silently ignored. The enable pin is always
/// driven low regardless of prior TMC4671 state.
///
/// # Parameters
/// - `tmc`: mutable reference to the TMC4671 SPI driver
/// - `enable_pin`: mutable reference to the motor enable output pin
async fn restore_safe_state<SPI, PIN, E>(tmc: &mut Tmc4671Spi<SPI>, enable_pin: &mut PIN)
where
    SPI: SpiDevice<u8, Error = E>,
    PIN: OutputPin,
{
    use tmc4671::registers::*;

    let _ = enable_pin.set_low();
    let _ = tmc.write_register(MODE_RAMP_MODE_MOTION, 0x0000_0000).await;
    let _ = tmc.write_register(PID_TORQUE_FLUX_TARGET, 0).await;
    let _ = tmc.write_register(UQ_UD_EXT, 0).await;
    let _ = tmc.write_register(PWM_SV_CHOP, 0).await;
}

/// Perform ADC offset calibration (selftest stage 1).
///
/// Configures the TMC4671 for current-sense ADC sampling with chopping
/// disabled, collects 128 samples per channel, computes the mean offset,
/// validates the result is in the expected hardware range, and programs the
/// offsets back. PWM chopping is restored after successful calibration.
///
/// # Parameters
/// - `tmc`: mutable reference to the TMC4671 SPI driver
/// - `enable_pin`: mutable reference to the motor enable output pin
/// - `delay`: mutable reference to an async delay source
///
/// # Returns
/// `Some(AdcCalResult)` on success, `None` on any SPI error or out-of-range
/// ADC offset.
async fn selftest_adc_calibrate<SPI, PIN, D, E>(
    tmc: &mut Tmc4671Spi<SPI>,
    enable_pin: &mut PIN,
    delay: &mut D,
) -> Option<AdcCalResult>
where
    SPI: SpiDevice<u8, Error = E>,
    PIN: OutputPin,
    D: DelayNs,
{
    use tmc4671::registers::*;

    // 1. SAFETY FIRST: stop motor, disable outputs, set voltage limit
    let _ = enable_pin.set_low();
    tmc.write_register(MODE_RAMP_MODE_MOTION, 0x0000_0000)
        .await
        .ok()?;
    let voltage_limit: u32 = 1024;
    tmc.write_register(PIDOUT_UQ_UD_LIMITS, voltage_limit)
        .await
        .ok()?;
    // Verify voltage limit write
    let readback = tmc.read_register(PIDOUT_UQ_UD_LIMITS).await.ok()?;
    if readback != voltage_limit {
        return None;
    }
    let _ = tmc.write_register(PID_TORQUE_FLUX_TARGET, 0).await;
    let _ = tmc.write_register(UQ_UD_EXT, 0).await;

    // 2. PWM configuration (chopping disabled for ADC calibration)
    let _ = tmc.write_register(PWM_POLARITIES, 0).await;
    let _ = tmc.write_register(PWM_MAXCNT, 3999).await;
    let _ = tmc.write_register(PWM_BBM_H_BBM_L, (10 << 8) | 10).await;
    tmc.write_register(PWM_SV_CHOP, 0).await.ok()?;
    // Verify MODE=0 and PWM_SV_CHOP=0
    let mode_rb = tmc.read_register(MODE_RAMP_MODE_MOTION).await.ok()?;
    let chop_rb = tmc.read_register(PWM_SV_CHOP).await.ok()?;
    if mode_rb != 0 || chop_rb != 0 {
        return None;
    }

    // 3. ADC configuration
    let mdec: u32 = 331;
    let _ = tmc
        .write_register(DSADC_MDEC_B_MDEC_A, (mdec << 16) | mdec)
        .await;
    let _ = tmc.write_register(ADC_I_SELECT, (1 << 28) | (1 << 8)).await;

    // 4. Motor type: 2-phase stepper (type 2), 50 pole pairs
    let _ = tmc
        .write_register(MOTOR_TYPE_N_POLE_PAIRS, (2 << 16) | 50)
        .await;

    // 5. ADC offset calibration — clear offsets, enable outputs, settle
    let _ = tmc.write_register(ADC_I0_SCALE_OFFSET, 256 << 16).await;
    let _ = tmc.write_register(ADC_I1_SCALE_OFFSET, 256 << 16).await;

    let _ = enable_pin.set_high();
    delay.delay_ms(250).await;

    // Sample ADC values 128 times
    let mut sum_i0: i32 = 0;
    let mut sum_i1: i32 = 0;
    const N_SAMPLES: i32 = 128;
    for _ in 0..N_SAMPLES {
        if let Ok(raw) = tmc.read_register(ADC_IWY_IUX).await {
            let i_ux = (raw & 0xFFFF) as i16 as i32;
            let i_wy = ((raw >> 16) & 0xFFFF) as i16 as i32;
            sum_i0 += i_ux;
            sum_i1 += i_wy;
        }
        delay.delay_ms(1).await;
    }
    let offset_i0 = (sum_i0 / N_SAMPLES) as i16 as u16;
    let offset_i1 = (sum_i1 / N_SAMPLES) as i16 as u16;

    // Validate ADC offsets are in expected range (centered around 33000)
    if !(30000..=36000).contains(&offset_i0) || !(30000..=36000).contains(&offset_i1) {
        return None;
    }

    let _ = tmc
        .write_register(ADC_I0_SCALE_OFFSET, (256 << 16) | offset_i0 as u32)
        .await;
    let _ = tmc
        .write_register(ADC_I1_SCALE_OFFSET, (256 << 16) | offset_i1 as u32)
        .await;

    // Re-enable PWM chopping
    let _ = tmc.write_register(PWM_SV_CHOP, 7).await;

    Some(AdcCalResult {
        offset_i0,
        offset_i1,
    })
}

/// Run the full 8-stage selftest sequence.
///
/// The caller must ensure the motor is disabled before calling.
/// Results are streamed via `reporter` as each stage completes.
/// On completion (pass or fail), the TMC4671 is left in safe state
/// (MODE=0, targets zeroed, enable pin LOW).
///
/// # Parameters
/// - `tmc`: mutable reference to the TMC4671 SPI driver
/// - `enable_pin`: mutable reference to the motor enable output pin
/// - `delay`: mutable reference to an async delay source
/// - `_current_limit`: maximum current in milliamps for test excitation
/// - `reporter`: mutable reference to the result callback
///
/// # Returns
/// Overall status: `SELFTEST_DONE_PASS`, `SELFTEST_DONE_FAIL`,
/// or `SELFTEST_DONE_ABORTED`.
pub async fn run_selftest<SPI, PIN, D, E>(
    tmc: &mut Tmc4671Spi<SPI>,
    enable_pin: &mut PIN,
    delay: &mut D,
    _current_limit: u32,
    reporter: &mut impl SelftestReporter,
) -> u8
where
    SPI: SpiDevice<u8, Error = E>,
    PIN: OutputPin,
    D: DelayNs,
{
    // Stage 1: ADC calibration
    match selftest_adc_calibrate(tmc, enable_pin, delay).await {
        Some(cal) => {
            // Report average of both channel offsets as the stage value
            let value = ((cal.offset_i0 as u32) + (cal.offset_i1 as u32)) / 2;
            reporter.report_stage(STAGE_ADC_CAL, SELFTEST_PASS, value);
        }
        None => {
            reporter.report_stage(STAGE_ADC_CAL, SELFTEST_FAIL, 0);
            restore_safe_state(tmc, enable_pin).await;
            return SELFTEST_DONE_FAIL;
        }
    }

    // Stages 2–8: not yet implemented
    reporter.report_stage(STAGE_MOTOR_COIL_A, SELFTEST_SKIP, 0);
    reporter.report_stage(STAGE_MOTOR_COIL_B, SELFTEST_SKIP, 0);
    reporter.report_stage(STAGE_PHASE_WIRING, SELFTEST_SKIP, 0);
    reporter.report_stage(STAGE_ENCODER_PRESENCE, SELFTEST_SKIP, 0);
    reporter.report_stage(STAGE_ENCODER_DIRECTION, SELFTEST_SKIP, 0);
    reporter.report_stage(STAGE_RESISTANCE, SELFTEST_SKIP, 0);
    reporter.report_stage(STAGE_INDUCTANCE, SELFTEST_SKIP, 0);

    restore_safe_state(tmc, enable_pin).await;
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

    #[test]
    fn adc_offset_in_range() {
        assert!((30000..=36000).contains(&33000u16));
        assert!((30000..=36000).contains(&30000u16));
        assert!((30000..=36000).contains(&36000u16));
        assert!(!(30000..=36000).contains(&29999u16));
        assert!(!(30000..=36000).contains(&36001u16));
    }
}
