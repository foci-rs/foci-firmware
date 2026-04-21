//! TMC4671 safety and measurement utilities.
//!
//! Contains `restore_safe_state` for emergency motor shutdown, and
//! autotune measurement functions for resistance and inductance.
//!
//! All logic is generic over `embedded-hal-async` traits for cross-board
//! reuse and host-side unit testing.

use embedded_hal::digital::OutputPin;
use embedded_hal_async::delay::DelayNs;
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

/// Measure per-coil resistance with PWM-rate current averaging for autotune.
///
/// Same setup as the old selftest resistance measurement (MODE=8, UQ_UD_EXT,
/// per-coil phi_e), but samples current 1000 times at PWM rate instead of a
/// single reading. Returns averaged current per coil for internal-unit gain
/// computation.
///
/// # Parameters
/// - `tmc`: TMC4671 SPI driver
/// - `enable_pin`: motor enable output
/// - `delay`: async delay source
/// - `config`: board-specific TMC4671 hardware configuration
///
/// # Returns
/// `Some((current_a, current_b))` with averaged unsigned absolute current
/// per coil, or `None` on SPI error.
pub async fn autotune_measure_resistance<SPI, PIN, D, E>(
    tmc: &mut Tmc4671Spi<SPI>,
    enable_pin: &mut PIN,
    delay: &mut D,
    config: &crate::config::TmcConfig,
) -> Option<(u16, u16)>
where
    SPI: SpiDevice<u8, Error = E>,
    PIN: OutputPin,
    D: DelayNs,
{
    use foci_tmc4671::registers::*;

    const SAMPLE_COUNT: u32 = 1000;
    const SAMPLE_PERIOD_US: u32 = 160;

    let voltage = config.excitation_voltage as u32;
    let mut currents = [0u16; 2];
    for phase in 0..2u8 {
        tmc.write_register(PHI_E_SELECTION, 1).await.ok()?;
        let phi_e = if phase == 0 { 0u32 } else { 16384 };
        tmc.write_register(PHI_E_EXT, phi_e).await.ok()?;
        tmc.write_register(UQ_UD_EXT, voltage & 0xFFFF).await.ok()?;
        tmc.write_register(MODE_RAMP_MODE_MOTION, 8).await.ok()?;

        let _ = enable_pin.set_high();

        // Wait for steady state (100ms = ~625 PWM cycles)
        delay.delay_ms(100).await;

        // Sample current at PWM rate and accumulate
        let mut sum: i64 = 0;
        let mut count: u32 = 0;
        for _ in 0..SAMPLE_COUNT {
            delay.delay_us(SAMPLE_PERIOD_US).await;
            if let Ok(actual) = tmc.read_register(PID_TORQUE_FLUX_ACTUAL).await {
                let current = (actual & 0xFFFF) as i16;
                sum += current.unsigned_abs() as i64;
                count += 1;
            }
        }

        // Stop and disable
        let _ = tmc.write_register(MODE_RAMP_MODE_MOTION, 0).await;
        let _ = tmc.write_register(UQ_UD_EXT, 0).await;
        let _ = enable_pin.set_low();
        delay.delay_ms(50).await;

        if count > 0 {
            currents[phase as usize] = (sum / count as i64) as u16;
        }
    }

    if currents[0] == 0 && currents[1] == 0 {
        return None;
    }
    Some((currents[0], currents[1]))
}

/// Result of square-wave di/dt inductance measurement.
pub struct DiDtResult {
    /// Per-coil sum of |delta_I| over all cycles (ADC units).
    pub sum_delta: (u64, u64),
    /// Per-coil number of valid delta_I samples.
    pub count: (u32, u32),
}

/// Total settle/cooldown time inside `autotune_measure_inductance` in
/// milliseconds. The caller must subtract this from any wall-clock
/// timing that wraps the function call.
///
/// Per coil: 10ms pre-toggle settle + 50ms post-toggle cooldown = 60ms.
/// Two coils: 120ms total.
pub const DIDT_SETTLE_MS: u32 = 120;

/// Measure per-coil inductance via square-wave di/dt method for autotune.
///
/// Toggles UQ_UD_EXT between +V and -V as fast as SPI allows (no
/// additional delay — the SPI write+read paces each cycle at ~50us).
/// Reads current via PID_TORQUE_FLUX_ACTUAL and accumulates |delta_I|
/// between consecutive samples over `config.didt_cycles` iterations per coil.
///
/// Returns raw accumulated sum_delta and count. The **caller** must
/// measure wall-clock time (e.g., via DWT cycle counter) to compute
/// the actual dt per cycle, since SPI transfer time varies and
/// dominates the cycle period.
///
/// L_int = V * total_time_s / sum_delta (for each coil)
///
/// All values are in TMC4671 internal units (s16 ADC/DAC space).
///
/// # Parameters
/// - `tmc`: TMC4671 SPI driver
/// - `enable_pin`: motor enable output
/// - `delay`: async delay source (used only for settle periods)
/// - `config`: board-specific TMC4671 hardware configuration
///
/// # Returns
/// `Some(DiDtResult)` with per-coil raw accumulations, or `None` on
/// SPI error. Caller computes L_int using measured wall-clock time.
pub async fn autotune_measure_inductance<SPI, PIN, D, E>(
    tmc: &mut Tmc4671Spi<SPI>,
    enable_pin: &mut PIN,
    delay: &mut D,
    config: &crate::config::TmcConfig,
) -> Option<DiDtResult>
where
    SPI: SpiDevice<u8, Error = E>,
    PIN: OutputPin,
    D: DelayNs,
{
    use foci_tmc4671::registers::*;

    let voltage = config.excitation_voltage as u32;
    let cycles = config.didt_cycles;
    let mut sum_deltas = [0u64; 2];
    let mut counts = [0u32; 2];
    for phase in 0..2u8 {
        tmc.write_register(PHI_E_SELECTION, 1).await.ok()?;
        let phi_e = if phase == 0 { 0u32 } else { 16384 };
        tmc.write_register(PHI_E_EXT, phi_e).await.ok()?;

        // Start with zero voltage in UQ_UD_EXT mode
        tmc.write_register(UQ_UD_EXT, 0).await.ok()?;
        tmc.write_register(MODE_RAMP_MODE_MOTION, 8).await.ok()?;
        let _ = enable_pin.set_high();

        // Brief settle before toggling
        delay.delay_ms(10).await;

        // Toggle voltage and accumulate |delta_I|.
        // SPI transfer time paces each iteration naturally. At 2 MHz SPI
        // this is ~100us; at 8 MHz it is ~11us. Faster cycles produce
        // smaller per-cycle dI but more samples per unit time. The DWT
        // wall-clock timing in the caller measures actual elapsed time,
        // so the gain formula (L = V * dt / dI) compensates correctly
        // regardless of cycle speed. Use more cycles (didt_cycles in
        // TmcConfig) to compensate for noisier per-cycle dI at higher
        // SPI speeds.
        let v_pos = voltage & 0xFFFF;
        let v_neg = (-(voltage as i32) as u32) & 0xFFFF;
        let mut sum_delta: u64 = 0;
        let mut count: u32 = 0;
        let mut prev_current: i16 = 0;
        let mut first = true;

        for cycle in 0..cycles {
            let v = if cycle % 2 == 0 { v_pos } else { v_neg };
            let _ = tmc.write_register(UQ_UD_EXT, v).await;

            // No explicit delay — SPI transfer provides pacing.
            if let Ok(actual) = tmc.read_register(PID_TORQUE_FLUX_ACTUAL).await {
                let current = (actual & 0xFFFF) as i16;
                if !first {
                    let delta = (current as i32 - prev_current as i32).unsigned_abs();
                    sum_delta += delta as u64;
                    count += 1;
                }
                prev_current = current;
                first = false;
            }
        }

        // Stop and disable
        let _ = tmc.write_register(UQ_UD_EXT, 0).await;
        let _ = tmc.write_register(MODE_RAMP_MODE_MOTION, 0).await;
        let _ = enable_pin.set_low();
        delay.delay_ms(50).await;

        sum_deltas[phase as usize] = sum_delta;
        counts[phase as usize] = count;
    }

    if counts[0] == 0 && counts[1] == 0 {
        return None;
    }
    Some(DiDtResult {
        sum_delta: (sum_deltas[0], sum_deltas[1]),
        count: (counts[0], counts[1]),
    })
}

/// Result of one selftest stage, streamed to the host.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
