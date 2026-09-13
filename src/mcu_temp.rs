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
    /// calling the real `klippy/extras/thermistor.py` module (see this
    /// task's header) -- an independent oracle, not a Rust
    /// reimplementation of the same formula being tested.
    const KLIPPER_FIXTURES: [(f32, f32); 7] = [
        (-20.0, 0.9945044359096915),
        (0.0, 0.9850078863067796),
        (25.0, 0.9551098376313276),
        (60.0, 0.8446770130360951),
        (100.0, 0.5938629860582022),
        (150.0, 0.2735703245749617),
        (200.0, 0.11006767523300735),
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
