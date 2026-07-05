//! Biquad filter coefficient computation for TMC4671 feedback filters.
//!
//! The TMC4671 provides on-chip second-order IIR (biquad) filters for velocity
//! feedback. Coefficients are stored in Q3.29 fixed-point format.
//!
//! This module computes coefficients for a second-order Butterworth low-pass
//! filter using the standard bilinear (Tustin) Z-transform with frequency
//! pre-warping. The pre-warping ensures the cutoff frequency is preserved
//! exactly in the discrete domain.

/// Biquad filter coefficients for TMC4671 feedback filter registers.
///
/// All fields are Q3.29 fixed-point values as defined by the TMC4671
/// datasheet (rev 2.08, section 4.8.1). The a-coefficients are negated
/// per datasheet equations 43 and 44 (the hardware adds them, so the
/// sign convention is inverted relative to the standard difference equation).
///
/// Register mapping:
/// - `b_0`, `b_1`, `b_2` → BIQUAD_V_B (bits 29:0 of each sub-field)
/// - `a_1`, `a_2`        → BIQUAD_V_A (bits 29:0 of each sub-field)
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct BiquadCoeffs {
    /// Feedback coefficient a_1, negated for TMC4671 register format (Q3.29).
    pub a_1: i32,
    /// Feedback coefficient a_2, negated for TMC4671 register format (Q3.29).
    pub a_2: i32,
    /// Feed-forward coefficient b_0 (Q3.29).
    pub b_0: i32,
    /// Feed-forward coefficient b_1 (Q3.29).
    pub b_1: i32,
    /// Feed-forward coefficient b_2 (Q3.29).
    pub b_2: i32,
}

/// Scale factor for Q3.29 fixed-point: 2^29.
const Q3_29_SCALE: f32 = 536_870_912.0; // 2^29

/// Minimum valid cutoff frequency in Hz.
const CUTOFF_MIN_HZ: u16 = 10;

/// Maximum valid cutoff frequency in Hz.
const CUTOFF_MAX_HZ: u16 = 6000;

/// Compute Q3.29 biquad coefficients for a second-order Butterworth low-pass filter.
///
/// Uses the standard bilinear (Tustin) Z-transform with frequency pre-warping
/// to place the cutoff frequency exactly in the discrete domain. The Butterworth
/// damping factor `D = 1/sqrt(2)` gives a maximally flat passband.
///
/// The sample period is derived from `pwm_maxcnt`:
/// ```text
/// T = (pwm_maxcnt + 1) * 10e-9  seconds
/// ```
/// (TMC4671 internal clock is 100 MHz, so each count = 10 ns.)
///
/// The discrete-domain transfer function is:
/// ```text
/// H(z) = b_0*(1 + 2*z^-1 + z^-2) / (denom + a_1*z^-1 + a_2*z^-2)
/// ```
/// where coefficients are derived via bilinear substitution
/// `s = 2/T * (1 - z^-1) / (1 + z^-1)` applied to the continuous prototype.
///
/// Coefficient convention follows TMC4671 datasheet rev 2.08, section 4.8.1,
/// equations 43–44: the a-coefficients stored in registers are negated.
///
/// # Parameters
///
/// - `cutoff_hz`: desired filter cutoff frequency in Hz, must be in `10..=6000`
/// - `pwm_maxcnt`: TMC4671 PWM_MAXCNT register value, must be `> 0`
///
/// # Returns
///
/// `Some(BiquadCoeffs)` with Q3.29 register values, or `None` if either
/// argument is out of range.
pub fn butterworth_lowpass(cutoff_hz: u16, pwm_maxcnt: u16) -> Option<BiquadCoeffs> {
    if !(CUTOFF_MIN_HZ..=CUTOFF_MAX_HZ).contains(&cutoff_hz) {
        return None;
    }
    if pwm_maxcnt == 0 {
        return None;
    }

    // Sample period T in seconds.
    // TMC4671 clock = 100 MHz → 10 ns per count.
    let t = (pwm_maxcnt as f32 + 1.0) * 10.0e-9_f32;

    // Continuous-time angular cutoff frequency.
    let omega_c = 2.0 * core::f32::consts::PI * cutoff_hz as f32;

    // Pre-warp omega_c to correct for bilinear frequency warping:
    //   omega_w = (2/T) * tan(omega_c * T / 2)
    // This ensures the discrete cutoff equals the desired analog cutoff exactly.
    let omega_w = (2.0 / t) * libm::tanf(omega_c * t / 2.0);

    // Bilinear transform parameter: k = omega_w * T / 2
    let k = omega_w * t / 2.0;
    let k2 = k * k;

    // Butterworth denominator: D = 1/sqrt(2), so sqrt(2)*k = k*D*2 = k/D  (same for D=1/√2)
    // Standard form: denom = 1 + sqrt(2)*k + k^2
    let sqrt2_k = libm::sqrtf(2.0) * k;
    let denom = 1.0 + sqrt2_k + k2;

    // Z-domain coefficients (un-negated a, unity gain b):
    //   b_0 = b_2 = k^2 / denom
    //   b_1       = 2 * k^2 / denom
    //   a_1       = 2 * (k^2 - 1) / denom
    //   a_2       = (1 - sqrt(2)*k + k^2) / denom
    let b0_z = k2 / denom;
    let b1_z = 2.0 * k2 / denom;
    let a1_z = 2.0 * (k2 - 1.0) / denom;
    let a2_z = (1.0 - sqrt2_k + k2) / denom;

    // Convert to Q3.29. a-coefficients are negated per TMC4671 datasheet eqs 43-44.
    let b_0 = libm::roundf(b0_z * Q3_29_SCALE) as i32;
    let b_1 = libm::roundf(b1_z * Q3_29_SCALE) as i32;
    let b_2 = b_0; // symmetric: b_2 == b_0 by construction
    let a_1 = libm::roundf(-a1_z * Q3_29_SCALE) as i32;
    let a_2 = libm::roundf(-a2_z * Q3_29_SCALE) as i32;

    Some(BiquadCoeffs {
        a_1,
        a_2,
        b_0,
        b_1,
        b_2,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 200 Hz cutoff at 25 kHz PWM (pwm_maxcnt = 3999 → T = 4000 * 10 ns = 40 µs).
    #[test]
    fn butterworth_200hz_at_25khz_pwm() {
        let coeffs = butterworth_lowpass(200, 3999).expect("valid params");
        assert_ne!(coeffs.b_0, 0, "b_0 must be non-zero");
        assert_ne!(coeffs.b_1, 0, "b_1 must be non-zero");
        assert_ne!(coeffs.b_2, 0, "b_2 must be non-zero");
        assert_eq!(
            coeffs.b_0, coeffs.b_2,
            "Butterworth LP is symmetric: b_0 == b_2"
        );
        assert_eq!(coeffs.b_1, 2 * coeffs.b_0, "Butterworth LP: b_1 == 2 * b_0");
    }

    #[test]
    fn rejects_zero_cutoff() {
        assert!(butterworth_lowpass(0, 3999).is_none());
    }

    #[test]
    fn rejects_cutoff_below_minimum() {
        assert!(butterworth_lowpass(9, 3999).is_none());
    }

    #[test]
    fn rejects_cutoff_above_maximum() {
        assert!(butterworth_lowpass(6001, 3999).is_none());
    }

    #[test]
    fn allows_current_filter_cutoff_at_six_khz() {
        assert!(butterworth_lowpass(6000, 3999).is_some());
    }

    #[test]
    fn rejects_zero_pwm_maxcnt() {
        assert!(butterworth_lowpass(200, 0).is_none());
    }

    #[test]
    fn higher_cutoff_has_larger_b_coefficients() {
        let low = butterworth_lowpass(50, 3999).expect("50 Hz valid");
        let high = butterworth_lowpass(500, 3999).expect("500 Hz valid");
        assert!(
            high.b_0 > low.b_0,
            "higher cutoff should have larger b_0: 50 Hz={} 500 Hz={}",
            low.b_0,
            high.b_0,
        );
    }

    #[test]
    fn coefficients_are_within_q3_29_range() {
        let coeffs = butterworth_lowpass(200, 3999).expect("valid params");
        // a_2 is negated (-a2_z stored). a2_z is positive for a stable filter,
        // so the stored value should be negative.
        assert!(
            coeffs.a_2 < 0,
            "a_2 (negated) should be negative for stable filter, got {}",
            coeffs.a_2
        );
        // b coefficients represent energy and must be positive.
        assert!(coeffs.b_0 > 0, "b_0 must be positive, got {}", coeffs.b_0);
        assert!(coeffs.b_1 > 0, "b_1 must be positive, got {}", coeffs.b_1);
        assert!(coeffs.b_2 > 0, "b_2 must be positive, got {}", coeffs.b_2);
    }
}
