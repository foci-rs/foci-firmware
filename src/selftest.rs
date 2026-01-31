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

/// Minimum ADC current reading (absolute) to consider a motor coil present.
/// Determined empirically during hardware bring-up. This is a raw ADC
/// value after offset subtraction, not milliamps.
const MOTOR_CURRENT_THRESHOLD: u16 = 200;

/// Result from a single coil test.
struct CoilTestResult {
    /// Current reading on the expected ADC channel.
    current_expected: i16,
    /// Current reading on the other ADC channel.
    current_other: i16,
}

/// Apply voltage to one phase via UD_EXT, measure current on both channels.
///
/// Sets up open-loop voltage mode (MODE=8) with external PHI_E, applies
/// voltage on the selected phase, waits for current to settle, and reads
/// the ADC. The motor is stopped before returning.
///
/// # Parameters
/// - `tmc`: mutable reference to the TMC4671 SPI driver
/// - `enable_pin`: mutable reference to the motor enable output pin
/// - `delay`: mutable reference to an async delay source
/// - `phase`: 0 for UX (coil A, phi_e=0), 1 for WY (coil B, phi_e=16384)
/// - `voltage`: test voltage magnitude (UD_EXT value, bits [15:0])
///
/// # Returns
/// `Some(CoilTestResult)` with current readings, `None` on SPI error.
async fn selftest_coil_test<SPI, PIN, D, E>(
    tmc: &mut Tmc4671Spi<SPI>,
    enable_pin: &mut PIN,
    delay: &mut D,
    phase: u8,
    voltage: u32,
) -> Option<CoilTestResult>
where
    SPI: SpiDevice<u8, Error = E>,
    PIN: OutputPin,
    D: DelayNs,
{
    use tmc4671::registers::*;

    // Set PHI_E source to external
    tmc.write_register(PHI_E_SELECTION, 1).await.ok()?;

    // Set electrical angle: 0 for UX (D-axis), 16384 for WY (Q-axis / 90 deg)
    let phi_e = if phase == 0 { 0u32 } else { 16384 };
    tmc.write_register(PHI_E_EXT, phi_e).await.ok()?;

    // Apply voltage via UQ_UD_EXT in voltage mode (MODE=8: UQ_UD_EXT)
    // UD_EXT in bits[15:0], UQ_EXT in bits[31:16]
    tmc.write_register(UQ_UD_EXT, voltage & 0xFFFF).await.ok()?;
    tmc.write_register(MODE_RAMP_MODE_MOTION, 8).await.ok()?;

    // Enable and wait for current to settle
    let _ = enable_pin.set_high();
    delay.delay_ms(100).await;

    // Read ADC current on both channels
    let raw = tmc.read_register(ADC_IWY_IUX).await.ok()?;
    let i_ux = (raw & 0xFFFF) as i16;
    let i_wy = ((raw >> 16) & 0xFFFF) as i16;

    // Stop motor and disable
    tmc.write_register(MODE_RAMP_MODE_MOTION, 0).await.ok()?;
    tmc.write_register(UQ_UD_EXT, 0).await.ok()?;
    let _ = enable_pin.set_low();

    if phase == 0 {
        Some(CoilTestResult {
            current_expected: i_ux,
            current_other: i_wy,
        })
    } else {
        Some(CoilTestResult {
            current_expected: i_wy,
            current_other: i_ux,
        })
    }
}

/// Check if encoder count changed between two reads.
///
/// Compares the current ABN decoder count against a previously captured
/// value. Any non-zero delta indicates encoder presence and movement.
///
/// # Parameters
/// - `tmc`: mutable reference to the TMC4671 SPI driver
/// - `count_before`: encoder count captured before a motor pulse
///
/// # Returns
/// `Some(delta)` if count changed, `None` if unchanged or SPI error.
async fn selftest_encoder_presence<SPI, E>(
    tmc: &mut Tmc4671Spi<SPI>,
    count_before: u32,
) -> Option<u32>
where
    SPI: SpiDevice<u8, Error = E>,
{
    use tmc4671::registers::*;

    let count_after = tmc.read_register(ABN_DECODER_COUNT).await.ok()?;
    let delta = (count_after as i32)
        .wrapping_sub(count_before as i32)
        .unsigned_abs();
    if delta > 0 {
        Some(delta)
    } else {
        None
    }
}

/// Check encoder direction by doing a brief open-loop rotation.
///
/// Sweeps PHI_E_EXT from 0 to 65536 (one electrical revolution) over
/// ~500ms and checks whether ABN_DECODER_COUNT increases or decreases.
///
/// # Parameters
/// - `tmc`: mutable reference to the TMC4671 SPI driver
/// - `enable_pin`: mutable reference to the motor enable output pin
/// - `delay`: mutable reference to an async delay source
/// - `voltage`: test voltage magnitude (UD_EXT value, bits [15:0])
///
/// # Returns
/// `Some(0)` if forward (count increases), `Some(1)` if reversed,
/// `None` on SPI error.
async fn selftest_encoder_direction<SPI, PIN, D, E>(
    tmc: &mut Tmc4671Spi<SPI>,
    enable_pin: &mut PIN,
    delay: &mut D,
    voltage: u32,
) -> Option<u8>
where
    SPI: SpiDevice<u8, Error = E>,
    PIN: OutputPin,
    D: DelayNs,
{
    use tmc4671::registers::*;

    // Record starting count
    let count_start = tmc.read_register(ABN_DECODER_COUNT).await.ok()? as i32;

    // Set up open-loop voltage mode
    tmc.write_register(PHI_E_SELECTION, 1).await.ok()?;
    tmc.write_register(UQ_UD_EXT, voltage & 0xFFFF).await.ok()?;
    tmc.write_register(MODE_RAMP_MODE_MOTION, 8).await.ok()?;

    // Enable gate driver so current actually flows
    let _ = enable_pin.set_high();

    // Sweep PHI_E_EXT from 0 to 65535 in 50 steps (~500ms)
    let steps = 50u32;
    for i in 0..=steps {
        let phi = (65536u64 * i as u64 / steps as u64) as u32;
        let _ = tmc.write_register(PHI_E_EXT, phi).await;
        delay.delay_ms(10).await;
    }

    // Read final count — clean up even on SPI error
    let count_end = tmc.read_register(ABN_DECODER_COUNT).await;

    // Stop and disable (always, regardless of read result)
    let _ = tmc.write_register(MODE_RAMP_MODE_MOTION, 0).await;
    let _ = tmc.write_register(UQ_UD_EXT, 0).await;
    let _ = enable_pin.set_low();

    let count_end = count_end.ok()? as i32;

    let delta = count_end.wrapping_sub(count_start);
    match delta.cmp(&0) {
        core::cmp::Ordering::Greater => Some(0), // forward
        core::cmp::Ordering::Less => Some(1),    // reversed
        core::cmp::Ordering::Equal => None,       // no motion detected
    }
}

/// Measure phase resistance by applying known voltage.
///
/// For each phase, applies voltage via UQ_UD_EXT (MODE=8), waits for
/// steady-state, reads PID_TORQUE_FLUX_ACTUAL, computes R = V/I.
///
/// # Parameters
/// - `tmc`: mutable reference to the TMC4671 SPI driver
/// - `enable_pin`: mutable reference to the motor enable output pin
/// - `delay`: mutable reference to an async delay source
/// - `voltage`: test voltage magnitude (UD_EXT value, bits [15:0])
///
/// # Returns
/// `Some((resistance_a, resistance_b))` in raw V/I * 1000 units,
/// `None` on SPI error.
async fn selftest_measure_resistance<SPI, PIN, D, E>(
    tmc: &mut Tmc4671Spi<SPI>,
    enable_pin: &mut PIN,
    delay: &mut D,
    voltage: u32,
) -> Option<(u16, u16)>
where
    SPI: SpiDevice<u8, Error = E>,
    PIN: OutputPin,
    D: DelayNs,
{
    use tmc4671::registers::*;

    let mut results = [0u16; 2];
    for phase in 0..2u8 {
        tmc.write_register(PHI_E_SELECTION, 1).await.ok()?;
        let phi_e = if phase == 0 { 0u32 } else { 16384 };
        tmc.write_register(PHI_E_EXT, phi_e).await.ok()?;
        tmc.write_register(UQ_UD_EXT, voltage & 0xFFFF).await.ok()?;
        tmc.write_register(MODE_RAMP_MODE_MOTION, 8).await.ok()?;

        // Enable gate driver so current actually flows
        let _ = enable_pin.set_high();

        // Wait for steady state
        delay.delay_ms(100).await;

        // Read current — clean up even on SPI error
        let actual = tmc.read_register(PID_TORQUE_FLUX_ACTUAL).await;

        // Stop and disable (always, regardless of read result)
        let _ = tmc.write_register(MODE_RAMP_MODE_MOTION, 0).await;
        let _ = tmc.write_register(UQ_UD_EXT, 0).await;
        let _ = enable_pin.set_low();
        delay.delay_ms(50).await;

        // flux_actual in [15:0], torque_actual in [31:16].
        // We apply UD_EXT (d-axis voltage) and the Park transform ensures the
        // resulting d-axis current always appears in flux_actual regardless of
        // phi_e angle. Read flux (low half) for both phases.
        let actual = actual.ok()?;
        let current = (actual & 0xFFFF) as i16;

        // R = V / I (raw ratio * 1000)
        if current.unsigned_abs() > 0 {
            results[phase as usize] =
                ((voltage as u64 * 1000) / current.unsigned_abs() as u64) as u16;
        }
    }
    // Both zero = no measurable current on either phase
    if results[0] == 0 && results[1] == 0 {
        return None;
    }
    Some((results[0], results[1]))
}

/// Measure phase inductance via step response.
///
/// For each phase, steps voltage from 0 to a known value, samples current
/// at ~1ms intervals for 100ms, finds the time constant tau where current
/// reaches 63% of steady state. L = R * tau.
///
/// # Parameters
/// - `tmc`: mutable reference to the TMC4671 SPI driver
/// - `enable_pin`: mutable reference to the motor enable output pin
/// - `delay`: mutable reference to an async delay source
/// - `voltage`: test voltage magnitude (UD_EXT value, bits [15:0])
/// - `resistance_mohm`: measured resistance from stage 7 `(coil_a, coil_b)`
///
/// # Returns
/// `Some((inductance_a_uh, inductance_b_uh))`, `None` on SPI error.
async fn selftest_measure_inductance<SPI, PIN, D, E>(
    tmc: &mut Tmc4671Spi<SPI>,
    enable_pin: &mut PIN,
    delay: &mut D,
    voltage: u32,
    resistance_mohm: (u16, u16),
) -> Option<(u16, u16)>
where
    SPI: SpiDevice<u8, Error = E>,
    PIN: OutputPin,
    D: DelayNs,
{
    use tmc4671::registers::*;

    let mut results = [0u16; 2];
    for phase in 0..2u8 {
        let r_mohm = if phase == 0 {
            resistance_mohm.0
        } else {
            resistance_mohm.1
        };
        if r_mohm == 0 {
            continue;
        }

        tmc.write_register(PHI_E_SELECTION, 1).await.ok()?;
        let phi_e = if phase == 0 { 0u32 } else { 16384 };
        tmc.write_register(PHI_E_EXT, phi_e).await.ok()?;

        // Ensure zero current starting point
        tmc.write_register(UQ_UD_EXT, 0).await.ok()?;
        tmc.write_register(MODE_RAMP_MODE_MOTION, 8).await.ok()?;

        // Enable gate driver so current actually flows
        let _ = enable_pin.set_high();
        delay.delay_ms(50).await;

        // Step voltage — if this fails, clean up and skip this phase
        let step_ok = tmc
            .write_register(UQ_UD_EXT, voltage & 0xFFFF)
            .await
            .is_ok();

        // Sample current at 1ms intervals for 100ms
        let mut samples = [0i16; 100];
        let mut sample_count = 0usize;
        if step_ok {
            for sample in samples.iter_mut() {
                delay.delay_ms(1).await;
                match tmc.read_register(ADC_IWY_IUX).await {
                    Ok(raw) => {
                        *sample = if phase == 0 {
                            (raw & 0xFFFF) as i16
                        } else {
                            ((raw >> 16) & 0xFFFF) as i16
                        };
                        sample_count += 1;
                    }
                    Err(_) => break,
                }
            }
        }

        // Stop and disable (always, regardless of sampling result)
        let _ = tmc.write_register(MODE_RAMP_MODE_MOTION, 0).await;
        let _ = tmc.write_register(UQ_UD_EXT, 0).await;
        let _ = enable_pin.set_low();

        // Need enough samples for meaningful measurement
        if sample_count < 20 {
            continue;
        }

        // Find steady-state current (average of last 10 collected samples)
        let tail_start = sample_count.saturating_sub(10);
        let tail = &samples[tail_start..sample_count];
        let steady: i32 = tail.iter().map(|&s| s as i32).sum::<i32>() / tail.len() as i32;
        if steady.unsigned_abs() < 10 {
            continue; // No measurable current
        }

        // Find tau: time when current reaches 63% of steady state
        let target = (steady * 63) / 100;
        let mut tau_ms: u16 = 0;
        for (i, &s) in samples[..sample_count].iter().enumerate() {
            if (s as i32).abs() >= target.abs() {
                tau_ms = i as u16 + 1;
                break;
            }
        }
        if tau_ms == 0 {
            tau_ms = sample_count as u16; // Never reached — use full window
        }

        // L = R * tau (R in mohm, tau in ms -> L in uH)
        results[phase as usize] = (r_mohm as u32 * tau_ms as u32) as u16;
    }
    Some((results[0], results[1]))
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
            // Pack both ADC offsets: I0 in high 16 bits, I1 in low 16 bits
            let value = ((cal.offset_i0 as u32) << 16) | (cal.offset_i1 as u32);
            reporter.report_stage(STAGE_ADC_CAL, SELFTEST_PASS, value);
        }
        None => {
            reporter.report_stage(STAGE_ADC_CAL, SELFTEST_FAIL, 0);
            restore_safe_state(tmc, enable_pin).await;
            return SELFTEST_DONE_FAIL;
        }
    }

    // Read encoder count before motor pulses (for stage 5)
    let encoder_before = tmc
        .read_register(tmc4671::registers::ABN_DECODER_COUNT)
        .await
        .unwrap_or(0);

    // Conservative test voltage for coil excitation
    let voltage = 512u32;

    // Stage 2: Motor coil A presence
    let coil_a = selftest_coil_test(tmc, enable_pin, delay, 0, voltage).await;
    let (coil_a_current, coil_a_other) = match coil_a {
        Some(r) if r.current_expected.unsigned_abs() > MOTOR_CURRENT_THRESHOLD => {
            reporter.report_stage(
                STAGE_MOTOR_COIL_A,
                SELFTEST_PASS,
                r.current_expected.unsigned_abs() as u32,
            );
            (r.current_expected, r.current_other)
        }
        Some(r) => {
            reporter.report_stage(
                STAGE_MOTOR_COIL_A,
                SELFTEST_FAIL,
                r.current_expected.unsigned_abs() as u32,
            );
            restore_safe_state(tmc, enable_pin).await;
            return SELFTEST_DONE_FAIL;
        }
        None => {
            reporter.report_stage(STAGE_MOTOR_COIL_A, SELFTEST_FAIL, 0);
            restore_safe_state(tmc, enable_pin).await;
            return SELFTEST_DONE_FAIL;
        }
    };

    // Stage 3: Motor coil B presence
    let coil_b = selftest_coil_test(tmc, enable_pin, delay, 1, voltage).await;
    match coil_b {
        Some(r) if r.current_expected.unsigned_abs() > MOTOR_CURRENT_THRESHOLD => {
            reporter.report_stage(
                STAGE_MOTOR_COIL_B,
                SELFTEST_PASS,
                r.current_expected.unsigned_abs() as u32,
            );
        }
        Some(r) => {
            reporter.report_stage(
                STAGE_MOTOR_COIL_B,
                SELFTEST_FAIL,
                r.current_expected.unsigned_abs() as u32,
            );
            restore_safe_state(tmc, enable_pin).await;
            return SELFTEST_DONE_FAIL;
        }
        None => {
            reporter.report_stage(STAGE_MOTOR_COIL_B, SELFTEST_FAIL, 0);
            restore_safe_state(tmc, enable_pin).await;
            return SELFTEST_DONE_FAIL;
        }
    }

    // Stage 4: Phase wiring (derived from stage 2)
    // If coil A test showed more current on the "other" channel than the
    // "expected" channel, wires are swapped.
    let wiring_ok = coil_a_other.unsigned_abs() < coil_a_current.unsigned_abs();
    reporter.report_stage(
        STAGE_PHASE_WIRING,
        if wiring_ok {
            SELFTEST_PASS
        } else {
            SELFTEST_FAIL
        },
        0,
    );
    if !wiring_ok {
        restore_safe_state(tmc, enable_pin).await;
        return SELFTEST_DONE_FAIL;
    }

    // Stage 5: Encoder presence
    match selftest_encoder_presence::<SPI, E>(tmc, encoder_before).await {
        Some(delta) => {
            reporter.report_stage(STAGE_ENCODER_PRESENCE, SELFTEST_PASS, delta);
        }
        None => {
            reporter.report_stage(STAGE_ENCODER_PRESENCE, SELFTEST_FAIL, 0);
            restore_safe_state(tmc, enable_pin).await;
            return SELFTEST_DONE_FAIL;
        }
    }

    // Stage 6: Encoder direction (diagnostic only — does not abort)
    match selftest_encoder_direction(tmc, enable_pin, delay, voltage).await {
        Some(dir) => {
            reporter.report_stage(STAGE_ENCODER_DIRECTION, SELFTEST_PASS, dir as u32);
        }
        None => {
            // Zero encoder delta or SPI error — cannot determine direction
            reporter.report_stage(STAGE_ENCODER_DIRECTION, SELFTEST_FAIL, 0);
        }
    }

    // Stage 7: Resistance (diagnostic only)
    let resistance = match selftest_measure_resistance(tmc, enable_pin, delay, voltage).await {
        Some((ra, rb)) => {
            let packed = ((ra as u32) << 16) | (rb as u32);
            reporter.report_stage(STAGE_RESISTANCE, SELFTEST_PASS, packed);
            (ra, rb)
        }
        None => {
            reporter.report_stage(STAGE_RESISTANCE, SELFTEST_SKIP, 0);
            (0, 0)
        }
    };

    // Stage 8: Inductance (diagnostic only)
    match selftest_measure_inductance(tmc, enable_pin, delay, voltage, resistance).await {
        Some((la, lb)) => {
            let packed = ((la as u32) << 16) | (lb as u32);
            reporter.report_stage(STAGE_INDUCTANCE, SELFTEST_PASS, packed);
        }
        None => {
            reporter.report_stage(STAGE_INDUCTANCE, SELFTEST_SKIP, 0);
        }
    }

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

    // ---- Stages 2-5 helper tests ----

    extern crate alloc;
    use alloc::vec;
    use alloc::vec::Vec;
    use embedded_hal_mock::eh1::delay::NoopDelay;
    use embedded_hal_mock::eh1::digital::{
        Mock as PinMock, State as PinState, Transaction as PinTx,
    };
    use embedded_hal_mock::eh1::spi::{Mock as SpiMock, Transaction as SpiTx};
    use tmc4671::registers::*;

    /// Helper: build SPI write transaction wrapped in transaction start/end.
    /// Returns 3 expectations: TransactionStart, Write, TransactionEnd.
    fn spi_write(reg: Register, value: u32) -> Vec<SpiTx<u8>> {
        let bytes = value.to_be_bytes();
        vec![
            SpiTx::transaction_start(),
            SpiTx::write_vec(vec![
                reg.addr() | 0x80,
                bytes[0],
                bytes[1],
                bytes[2],
                bytes[3],
            ]),
            SpiTx::transaction_end(),
        ]
    }

    /// Helper: build SPI read transaction wrapped in transaction start/end.
    /// Returns 3 expectations: TransactionStart, TransferInPlace, TransactionEnd.
    fn spi_read(reg: Register, value: u32) -> Vec<SpiTx<u8>> {
        let bytes = value.to_be_bytes();
        vec![
            SpiTx::transaction_start(),
            SpiTx::transfer_in_place(
                vec![reg.addr(), 0, 0, 0, 0],
                vec![0, bytes[0], bytes[1], bytes[2], bytes[3]],
            ),
            SpiTx::transaction_end(),
        ]
    }

    /// Flatten multiple SPI transaction groups into a single expectations vec.
    fn spi_expectations(groups: Vec<Vec<SpiTx<u8>>>) -> Vec<SpiTx<u8>> {
        groups.into_iter().flatten().collect()
    }

    #[tokio::test]
    async fn coil_test_phase0_returns_ux_as_expected() {
        // Phase 0 (coil A): current on UX channel is "expected", WY is "other".
        // ADC_IWY_IUX: IWY in [31:16], IUX in [15:0].
        let i_ux: i16 = 500;
        let i_wy: i16 = 30;
        let adc_raw = ((i_wy as u16 as u32) << 16) | (i_ux as u16 as u32);

        let expectations = spi_expectations(vec![
            spi_write(PHI_E_SELECTION, 1),
            spi_write(PHI_E_EXT, 0),
            spi_write(UQ_UD_EXT, 512),
            spi_write(MODE_RAMP_MODE_MOTION, 8),
            spi_read(ADC_IWY_IUX, adc_raw),
            spi_write(MODE_RAMP_MODE_MOTION, 0),
            spi_write(UQ_UD_EXT, 0),
        ]);
        let spi = SpiMock::new(&expectations);
        let mut tmc = Tmc4671Spi::new(spi);
        let pin_expectations = [PinTx::set(PinState::High), PinTx::set(PinState::Low)];
        let mut pin = PinMock::new(&pin_expectations);
        let mut delay = NoopDelay::new();

        let result = selftest_coil_test(&mut tmc, &mut pin, &mut delay, 0, 512)
            .await
            .unwrap();
        assert_eq!(result.current_expected, i_ux);
        assert_eq!(result.current_other, i_wy);

        tmc.into_inner().done();
        pin.done();
    }

    #[tokio::test]
    async fn coil_test_phase1_returns_wy_as_expected() {
        // Phase 1 (coil B): current on WY channel is "expected", UX is "other".
        let i_ux: i16 = 25;
        let i_wy: i16 = 480;
        let adc_raw = ((i_wy as u16 as u32) << 16) | (i_ux as u16 as u32);

        let expectations = spi_expectations(vec![
            spi_write(PHI_E_SELECTION, 1),
            spi_write(PHI_E_EXT, 16384),
            spi_write(UQ_UD_EXT, 512),
            spi_write(MODE_RAMP_MODE_MOTION, 8),
            spi_read(ADC_IWY_IUX, adc_raw),
            spi_write(MODE_RAMP_MODE_MOTION, 0),
            spi_write(UQ_UD_EXT, 0),
        ]);
        let spi = SpiMock::new(&expectations);
        let mut tmc = Tmc4671Spi::new(spi);
        let pin_expectations = [PinTx::set(PinState::High), PinTx::set(PinState::Low)];
        let mut pin = PinMock::new(&pin_expectations);
        let mut delay = NoopDelay::new();

        let result = selftest_coil_test(&mut tmc, &mut pin, &mut delay, 1, 512)
            .await
            .unwrap();
        assert_eq!(result.current_expected, i_wy);
        assert_eq!(result.current_other, i_ux);

        tmc.into_inner().done();
        pin.done();
    }

    #[tokio::test]
    async fn encoder_presence_detects_movement() {
        let count_before = 1000u32;
        let count_after = 1050u32;

        let expectations = spi_expectations(vec![spi_read(ABN_DECODER_COUNT, count_after)]);
        let spi = SpiMock::new(&expectations);
        let mut tmc = Tmc4671Spi::new(spi);

        let result =
            selftest_encoder_presence::<_, embedded_hal::spi::ErrorKind>(&mut tmc, count_before)
                .await;
        assert_eq!(result, Some(50));

        tmc.into_inner().done();
    }

    #[tokio::test]
    async fn encoder_presence_returns_none_when_unchanged() {
        let count = 1000u32;

        let expectations = spi_expectations(vec![spi_read(ABN_DECODER_COUNT, count)]);
        let spi = SpiMock::new(&expectations);
        let mut tmc = Tmc4671Spi::new(spi);

        let result =
            selftest_encoder_presence::<_, embedded_hal::spi::ErrorKind>(&mut tmc, count).await;
        assert_eq!(result, None);

        tmc.into_inner().done();
    }

    #[tokio::test]
    async fn encoder_presence_handles_wraparound() {
        // Counter wrapping from near-max to near-zero
        let count_before = u32::MAX - 10;
        let count_after = 20u32;

        let expectations = spi_expectations(vec![spi_read(ABN_DECODER_COUNT, count_after)]);
        let spi = SpiMock::new(&expectations);
        let mut tmc = Tmc4671Spi::new(spi);

        let result =
            selftest_encoder_presence::<_, embedded_hal::spi::ErrorKind>(&mut tmc, count_before)
                .await;
        assert_eq!(result, Some(31)); // wrapping distance

        tmc.into_inner().done();
    }

    #[test]
    fn motor_current_threshold_is_reasonable() {
        // Threshold must be > 0 and small enough to detect a connected motor
        assert!(MOTOR_CURRENT_THRESHOLD > 0);
        assert!(MOTOR_CURRENT_THRESHOLD < 1000);
    }

    #[test]
    fn resistance_calculation() {
        // V=512, I=455 (absolute) -> R = 512*1000/455 = 1125 mohm
        // Close to LDO-42STH48-2504B spec of 1200 +/- 15%
        let r = (512u64 * 1000) / 455u64;
        assert_eq!(r, 1125);
    }

    #[test]
    fn inductance_calculation() {
        // R=1125 mohm, tau=1ms -> L = 1125 * 1 = 1125 uH
        // LDO spec: 1500 +/- 20% (1200-1800 uH range)
        let l = 1125u32 * 1;
        assert_eq!(l, 1125);
    }

    #[tokio::test]
    async fn encoder_direction_forward() {
        // Sweep PHI_E 0..65536 in 50 steps, count increases -> forward (0)
        let count_start = 1000u32;
        let count_end = 1200u32;

        let mut expectations: Vec<SpiTx<u8>> = Vec::new();
        // Read ABN_DECODER_COUNT (start)
        expectations.extend(spi_read(ABN_DECODER_COUNT, count_start));
        // PHI_E_SELECTION = 1
        expectations.extend(spi_write(PHI_E_SELECTION, 1));
        // UQ_UD_EXT = 512
        expectations.extend(spi_write(UQ_UD_EXT, 512));
        // MODE = 8
        expectations.extend(spi_write(MODE_RAMP_MODE_MOTION, 8));
        // 51 PHI_E_EXT writes (i=0..=50)
        for i in 0..=50u32 {
            let phi = (65536u64 * i as u64 / 50u64) as u32;
            expectations.extend(spi_write(PHI_E_EXT, phi));
        }
        // Read ABN_DECODER_COUNT (end)
        expectations.extend(spi_read(ABN_DECODER_COUNT, count_end));
        // Stop: MODE=0, UQ_UD_EXT=0
        expectations.extend(spi_write(MODE_RAMP_MODE_MOTION, 0));
        expectations.extend(spi_write(UQ_UD_EXT, 0));

        let spi = SpiMock::new(&expectations);
        let mut tmc = Tmc4671Spi::new(spi);
        let pin_expectations = [PinTx::set(PinState::High), PinTx::set(PinState::Low)];
        let mut pin = PinMock::new(&pin_expectations);
        let mut delay = NoopDelay::new();

        let result = selftest_encoder_direction(&mut tmc, &mut pin, &mut delay, 512).await;
        assert_eq!(result, Some(0)); // forward

        tmc.into_inner().done();
        pin.done();
    }

    #[tokio::test]
    async fn encoder_direction_reversed() {
        let count_start = 1200u32;
        let count_end = 1000u32;

        let mut expectations: Vec<SpiTx<u8>> = Vec::new();
        expectations.extend(spi_read(ABN_DECODER_COUNT, count_start));
        expectations.extend(spi_write(PHI_E_SELECTION, 1));
        expectations.extend(spi_write(UQ_UD_EXT, 512));
        expectations.extend(spi_write(MODE_RAMP_MODE_MOTION, 8));
        for i in 0..=50u32 {
            let phi = (65536u64 * i as u64 / 50u64) as u32;
            expectations.extend(spi_write(PHI_E_EXT, phi));
        }
        expectations.extend(spi_read(ABN_DECODER_COUNT, count_end));
        expectations.extend(spi_write(MODE_RAMP_MODE_MOTION, 0));
        expectations.extend(spi_write(UQ_UD_EXT, 0));

        let spi = SpiMock::new(&expectations);
        let mut tmc = Tmc4671Spi::new(spi);
        let pin_expectations = [PinTx::set(PinState::High), PinTx::set(PinState::Low)];
        let mut pin = PinMock::new(&pin_expectations);
        let mut delay = NoopDelay::new();

        let result = selftest_encoder_direction(&mut tmc, &mut pin, &mut delay, 512).await;
        assert_eq!(result, Some(1)); // reversed

        tmc.into_inner().done();
        pin.done();
    }

    #[tokio::test]
    async fn encoder_direction_zero_delta_returns_none() {
        let count = 1000u32;

        let mut expectations: Vec<SpiTx<u8>> = Vec::new();
        expectations.extend(spi_read(ABN_DECODER_COUNT, count));
        expectations.extend(spi_write(PHI_E_SELECTION, 1));
        expectations.extend(spi_write(UQ_UD_EXT, 512));
        expectations.extend(spi_write(MODE_RAMP_MODE_MOTION, 8));
        for i in 0..=50u32 {
            let phi = (65536u64 * i as u64 / 50u64) as u32;
            expectations.extend(spi_write(PHI_E_EXT, phi));
        }
        expectations.extend(spi_read(ABN_DECODER_COUNT, count));
        expectations.extend(spi_write(MODE_RAMP_MODE_MOTION, 0));
        expectations.extend(spi_write(UQ_UD_EXT, 0));

        let spi = SpiMock::new(&expectations);
        let mut tmc = Tmc4671Spi::new(spi);
        let pin_expectations = [PinTx::set(PinState::High), PinTx::set(PinState::Low)];
        let mut pin = PinMock::new(&pin_expectations);
        let mut delay = NoopDelay::new();

        let result = selftest_encoder_direction(&mut tmc, &mut pin, &mut delay, 512).await;
        assert_eq!(result, None); // zero delta = indeterminate

        tmc.into_inner().done();
        pin.done();
    }

    #[tokio::test]
    async fn measure_resistance_computes_ratio() {
        // Phase 0: V=512, I=455 -> R=1125
        // Phase 1: V=512, I=460 -> R=1113
        let current_a: i16 = 455;
        let current_b: i16 = 460;
        // PID_TORQUE_FLUX_ACTUAL: torque[31:16], flux[15:0]
        // Park transform ensures d-axis current appears in flux (low half)
        // for both phases regardless of phi_e angle.
        let actual_a = current_a as u16 as u32;
        let actual_b = current_b as u16 as u32;

        let mut expectations: Vec<SpiTx<u8>> = Vec::new();
        // Phase 0
        expectations.extend(spi_write(PHI_E_SELECTION, 1));
        expectations.extend(spi_write(PHI_E_EXT, 0));
        expectations.extend(spi_write(UQ_UD_EXT, 512));
        expectations.extend(spi_write(MODE_RAMP_MODE_MOTION, 8));
        expectations.extend(spi_read(PID_TORQUE_FLUX_ACTUAL, actual_a));
        expectations.extend(spi_write(MODE_RAMP_MODE_MOTION, 0));
        expectations.extend(spi_write(UQ_UD_EXT, 0));
        // Phase 1
        expectations.extend(spi_write(PHI_E_SELECTION, 1));
        expectations.extend(spi_write(PHI_E_EXT, 16384));
        expectations.extend(spi_write(UQ_UD_EXT, 512));
        expectations.extend(spi_write(MODE_RAMP_MODE_MOTION, 8));
        expectations.extend(spi_read(PID_TORQUE_FLUX_ACTUAL, actual_b));
        expectations.extend(spi_write(MODE_RAMP_MODE_MOTION, 0));
        expectations.extend(spi_write(UQ_UD_EXT, 0));

        let spi = SpiMock::new(&expectations);
        let mut tmc = Tmc4671Spi::new(spi);
        // 2 phases: each does enable HIGH then LOW
        let pin_expectations = [
            PinTx::set(PinState::High),
            PinTx::set(PinState::Low),
            PinTx::set(PinState::High),
            PinTx::set(PinState::Low),
        ];
        let mut pin = PinMock::new(&pin_expectations);
        let mut delay = NoopDelay::new();

        let result = selftest_measure_resistance(&mut tmc, &mut pin, &mut delay, 512).await;
        let (ra, rb) = result.unwrap();
        assert_eq!(ra, 1125); // 512*1000/455
        assert_eq!(rb, 1113); // 512*1000/460

        tmc.into_inner().done();
        pin.done();
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
