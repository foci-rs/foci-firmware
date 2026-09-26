//! Generic-3950 NTC curve math, used to synthesize a raw ADC reading for
//! the chip-internal temperature sensor that decodes correctly under
//! Klipper's stock `sensor_type: Generic 3950` (a three-point
//! Steinhart-Hart fit, not a beta equation -- verified against
//! `klippy/extras/thermistor.py` and `temperature_sensors.cfg`).

/// Steinhart-Hart coefficients: `1/T = c1 + c2*ln(R) + c3*ln(R)^3`.
#[derive(Debug, Clone, Copy)]
pub struct SteinhartHartCoefficients {
    /// First Steinhart-Hart coefficient.
    pub c1: f32,
    /// Second Steinhart-Hart coefficient.
    pub c2: f32,
    /// Third Steinhart-Hart coefficient.
    pub c3: f32,
}

const KELVIN_TO_CELSIUS: f32 = -273.15;

/// Coefficients for Klipper's `[thermistor Generic 3950]` preset
/// (t1=25C/r1=100000, t2=150C/r2=1770, t3=250C/r3=230), computed by
/// directly importing and calling `klippy/extras/thermistor.py` (see this
/// task's header for the exact command) -- not re-derived by hand.
pub const GENERIC_3950: SteinhartHartCoefficients = SteinhartHartCoefficients {
    c1: 0.000_793_472_15,
    c2: 0.000_200_763_25,
    c3: 0.000_000_163_283_41,
};

/// Assumed pullup for the synthesized MCU_TEMP channel -- must equal the
/// host config's `pullup_resistor:` for that sensor entry exactly.
pub const ASSUMED_PULLUP_OHMS: f32 = 4700.0;

/// Synthesize the 0..1 ADC fraction a Generic-3950 divider would produce
/// at `temp_c`, replicating `thermistor.py`'s `calc_adc` (Cardano's-formula
/// cubic inverse).
///
/// # Arguments
///
/// * `coeffs` - Steinhart-Hart coefficients defining the curve
/// * `pullup_ohms` - Pullup resistor value in ohms
/// * `temp_c` - Temperature in degrees Celsius
///
/// # Returns
///
/// ADC fraction in the range (0.0, 1.0) representing the analog reading
pub fn temperature_to_adc_fraction(
    coeffs: &SteinhartHartCoefficients,
    pullup_ohms: f32,
    temp_c: f32,
) -> f32 {
    let inv_t = 1.0 / (temp_c - KELVIN_TO_CELSIUS);
    let ln_r = if coeffs.c3 != 0.0 {
        let y = (coeffs.c1 - inv_t) / (2.0 * coeffs.c3);
        let c2_term = coeffs.c2 / (3.0 * coeffs.c3);
        let x = libm::sqrtf(c2_term * c2_term * c2_term + y * y);
        libm::cbrtf(x - y) - libm::cbrtf(x + y)
    } else {
        (inv_t - coeffs.c1) / coeffs.c2
    };
    let r = libm::expf(ln_r);
    r / (pullup_ohms + r)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx_eq(a: f32, b: f32, tol: f32) -> bool {
        (a - b).abs() < tol
    }

    /// (temp_c, expected_adc_fraction) computed by directly importing and
    /// calling the real `klippy/extras/thermistor.py` module -- an
    /// independent oracle, not a Rust
    /// reimplementation of the same formula being tested.
    const KLIPPER_FIXTURES: [(f32, f32); 7] = [
        (-20.0, 0.994_504_45),
        (0.0, 0.985_007_9),
        (25.0, 0.955_109_83),
        (60.0, 0.844_677_03),
        (100.0, 0.593_863),
        (150.0, 0.273_570_33),
        (200.0, 0.110_067_67),
    ];

    #[test]
    fn matches_the_real_thermistor_py_fixture_table() {
        for (temp_c, expected_fraction) in KLIPPER_FIXTURES {
            let fraction = temperature_to_adc_fraction(&GENERIC_3950, ASSUMED_PULLUP_OHMS, temp_c);
            assert!(
                approx_eq(fraction, expected_fraction, 0.0005),
                "temp={temp_c} expected={expected_fraction} got={fraction}"
            );
        }
    }

    #[test]
    fn adc_fraction_is_bounded_to_the_open_unit_interval() {
        let fraction = temperature_to_adc_fraction(&GENERIC_3950, ASSUMED_PULLUP_OHMS, 300.0);
        assert!(fraction > 0.0 && fraction < 1.0);
    }
}

/// Two-point factory calibration for a chip's internal temperature
/// sensor: `(temperature_c, raw_adc_count)` at each of two known
/// temperatures, read from the chip's documented system-memory
/// calibration addresses.
#[derive(Debug, Clone, Copy)]
pub struct FactoryCalibration {
    /// Lower calibration point: `(temperature_c, raw_adc_count)`.
    pub cal_low: (f32, u16),
    /// Higher calibration point: `(temperature_c, raw_adc_count)`.
    pub cal_high: (f32, u16),
}

/// Convert an internal-sensor raw ADC count to °C via linear interpolation
/// between two factory calibration points, matching
/// `temperature_mcu.py`'s `config_stm32f4`/`config_stm32h723` formula.
///
/// # Arguments
///
/// * `cal` - Factory calibration points for this chip
/// * `raw` - Raw ADC count sampled from the internal temperature sensor
/// * `adc_max` - Full-scale ADC count (e.g. 4095.0 for a 12-bit ADC)
///
/// # Returns
///
/// Interpolated temperature in degrees Celsius.
pub fn internal_sensor_raw_to_celsius(cal: &FactoryCalibration, raw: u16, adc_max: f32) -> f32 {
    let (t_low, raw_low) = cal.cal_low;
    let (t_high, raw_high) = cal.cal_high;
    let adc_low = f32::from(raw_low) / adc_max;
    let adc_high = f32::from(raw_high) / adc_max;
    let adc = f32::from(raw) / adc_max;
    let slope = (t_high - t_low) / (adc_high - adc_low);
    t_low + (adc - adc_low) * slope
}

/// Convert one internal-sensor raw reading into the synthesized raw ADC
/// count `sample_count` reports of a Generic-3950 divider at that
/// temperature would sum to (see `analog_in`'s sampling state -- the wire
/// value is a bounded sum, not an average).
///
/// # Arguments
///
/// * `cal` - Factory calibration points for this chip
/// * `raw_internal_sensor` - Raw ADC count from the internal temperature sensor
/// * `adc_max` - Full-scale ADC count (e.g. 4095.0 for a 12-bit ADC)
/// * `sample_count` - Number of samples the wire protocol sums together
///
/// # Returns
///
/// Synthesized summed raw ADC count for the `sample_count` samples.
pub fn synthesize_mcu_temp_reading(
    cal: &FactoryCalibration,
    raw_internal_sensor: u16,
    adc_max: f32,
    sample_count: u8,
) -> u16 {
    let temp_c = internal_sensor_raw_to_celsius(cal, raw_internal_sensor, adc_max);
    let fraction = temperature_to_adc_fraction(&GENERIC_3950, ASSUMED_PULLUP_OHMS, temp_c);
    let per_sample = libm::roundf(fraction * adc_max).clamp(0.0, adc_max) as u32;
    let sum = per_sample.saturating_mul(u32::from(sample_count));
    sum.min(u32::from(u16::MAX)) as u16
}

#[cfg(test)]
mod calibration_tests {
    use super::*;

    fn stm32f4_style_calibration() -> FactoryCalibration {
        // TS_CAL1 at 30C, TS_CAL2 at 110C, 12-bit raw counts -- representative
        // values, not the real per-chip bytes (those come from silicon at
        // runtime; this test only checks the interpolation arithmetic).
        FactoryCalibration {
            cal_low: (30.0, 1500),
            cal_high: (110.0, 1900),
        }
    }

    #[test]
    fn interpolates_between_calibration_points() {
        let cal = stm32f4_style_calibration();
        let temp_at_low = internal_sensor_raw_to_celsius(&cal, 1500, 4095.0);
        let temp_at_high = internal_sensor_raw_to_celsius(&cal, 1900, 4095.0);
        assert!((temp_at_low - 30.0).abs() < 0.01);
        assert!((temp_at_high - 110.0).abs() < 0.01);
        let temp_mid = internal_sensor_raw_to_celsius(&cal, 1700, 4095.0);
        assert!(temp_mid > 30.0 && temp_mid < 110.0);
    }

    #[test]
    fn synthesized_reading_scales_with_sample_count_for_the_summed_wire_protocol() {
        let cal = stm32f4_style_calibration();
        let single = synthesize_mcu_temp_reading(&cal, 1700, 4095.0, 1);
        let summed_of_eight = synthesize_mcu_temp_reading(&cal, 1700, 4095.0, 8);
        assert_eq!(u32::from(summed_of_eight), u32::from(single) * 8);
    }

    /// End-to-end fixture: `(raw_internal_sensor, expected_quantized_count,
    /// klipper_decoded_temp_c)` for `stm32f4_style_calibration()`'s
    /// (30C/1500, 110C/1900) calibration points, `ADC_MAX = 4095`, spanning
    /// the spec's full -20..200C operating range (`raw_internal` values
    /// chosen so this calibration maps them exactly onto
    /// `KLIPPER_FIXTURES`' -20, 0, 25, 60, 100, 150, 200C points). Computed independently by
    /// importing and calling the real `klippy/extras/thermistor.py` module
    /// (`calc_adc`/`calc_temp`) with `internal_sensor_raw_to_celsius`'s own
    /// linear-interpolation formula evaluated in Python at each
    /// `raw_internal` -- not by calling the Rust code under test in either
    /// direction. `expected_quantized_count` is
    /// `round(thermistor.calc_adc(interpolated_temp_c) * 4095)`; this is
    /// the exact value `synthesize_mcu_temp_reading(..., sample_count=1)`
    /// must produce. `klipper_decoded_temp_c` is
    /// `thermistor.calc_temp(expected_quantized_count / 4095)` -- decoding
    /// the *quantized* count, so it also captures the rounding-to-integer-
    /// count error, which grows toward the range's extremes (see the
    /// tolerance test below).
    const END_TO_END_FIXTURES: [(u16, u16, f32); 7] = [
        (1250, 4072, -19.589_466),
        (1350, 4034, -0.135_576_87),
        (1475, 3911, 25.023_502),
        (1650, 3459, 59.997_528),
        (1850, 2432, 99.995_58),
        (2100, 1120, 150.013_58),
        (2350, 451, 199.966_42),
    ];

    #[test]
    fn synthesized_count_matches_independent_klipper_fixtures() {
        let cal = stm32f4_style_calibration();
        for (raw_internal, expected_count, _decoded_temp) in END_TO_END_FIXTURES {
            let synthesized = synthesize_mcu_temp_reading(&cal, raw_internal, 4095.0, 1);
            assert_eq!(
                synthesized, expected_count,
                "raw_internal={raw_internal} expected={expected_count} got={synthesized}"
            );
        }
    }

    #[test]
    fn synthesized_count_decodes_within_tolerance_of_the_interpolated_temperature() {
        let cal = stm32f4_style_calibration();
        for (raw_internal, _expected_count, decoded_temp) in END_TO_END_FIXTURES {
            let interpolated_temp = internal_sensor_raw_to_celsius(&cal, raw_internal, 4095.0);
            assert!(
                (decoded_temp - interpolated_temp).abs() < 0.5,
                "raw_internal={raw_internal} interpolated={interpolated_temp} klipper_decoded={decoded_temp}"
            );
        }
    }
}
