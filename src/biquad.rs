//! Q3.29 second-order Butterworth low-pass coefficients for the TMC4671 feedback biquads.

/// Q3.29 coefficients for the TMC4671 biquad registers. The a-coefficients are negated
/// (hardware adds them, datasheet eqs. 43-44).
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct BiquadCoeffs {
    pub a_1: i32,
    pub a_2: i32,
    pub b_0: i32,
    pub b_1: i32,
    pub b_2: i32,
}

const Q3_29_SCALE: f32 = 536_870_912.0;

const CUTOFF_MIN_HZ: u16 = 10;

pub(crate) const MOTION_CUTOFF_MAX_HZ: u16 = 1000;

pub(crate) const CURRENT_CUTOFF_MAX_HZ: u16 = 10_000;

/// Butterworth low-pass via bilinear transform with pre-warping; sample period is
/// `(pwm_maxcnt + 1) * 10 ns`.
///
/// Returns `None` unless `CUTOFF_MIN_HZ..=max_hz` contains `cutoff_hz` and `pwm_maxcnt > 0`.
/// `max_hz` is `MOTION_CUTOFF_MAX_HZ` (velocity/position) or `CURRENT_CUTOFF_MAX_HZ` (torque/flux).
pub fn butterworth_lowpass(cutoff_hz: u16, pwm_maxcnt: u16, max_hz: u16) -> Option<BiquadCoeffs> {
    if !(CUTOFF_MIN_HZ..=max_hz).contains(&cutoff_hz) {
        return None;
    }
    if pwm_maxcnt == 0 {
        return None;
    }

    let t = (pwm_maxcnt as f32 + 1.0) * 10.0e-9_f32;

    let omega_c = 2.0 * core::f32::consts::PI * cutoff_hz as f32;

    let omega_w = (2.0 / t) * libm::tanf(omega_c * t / 2.0);

    let k = omega_w * t / 2.0;
    let k2 = k * k;

    let sqrt2_k = libm::sqrtf(2.0) * k;
    let denom = 1.0 + sqrt2_k + k2;

    let b0_z = k2 / denom;
    let b1_z = 2.0 * k2 / denom;
    let a1_z = 2.0 * (k2 - 1.0) / denom;
    let a2_z = (1.0 - sqrt2_k + k2) / denom;

    let b_0 = libm::roundf(b0_z * Q3_29_SCALE) as i32;
    let b_1 = libm::roundf(b1_z * Q3_29_SCALE) as i32;
    let b_2 = b_0;
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

    #[test]
    fn butterworth_200hz_at_25khz_pwm() {
        let coeffs = butterworth_lowpass(200, 3999, CURRENT_CUTOFF_MAX_HZ).expect("valid params");
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
        assert!(butterworth_lowpass(0, 3999, CURRENT_CUTOFF_MAX_HZ).is_none());
    }

    #[test]
    fn rejects_cutoff_below_minimum() {
        assert!(butterworth_lowpass(9, 3999, CURRENT_CUTOFF_MAX_HZ).is_none());
    }

    #[test]
    fn rejects_cutoff_above_current_maximum() {
        assert!(butterworth_lowpass(10_001, 3999, CURRENT_CUTOFF_MAX_HZ).is_none());
    }

    #[test]
    fn allows_current_filter_cutoff_at_ten_khz() {
        assert!(butterworth_lowpass(10_000, 3999, CURRENT_CUTOFF_MAX_HZ).is_some());
    }

    #[test]
    fn allows_motion_filter_cutoff_at_one_khz() {
        assert!(butterworth_lowpass(1000, 3999, MOTION_CUTOFF_MAX_HZ).is_some());
    }

    #[test]
    fn motion_ceiling_rejects_cutoff_valid_for_current_ceiling() {
        assert!(butterworth_lowpass(2000, 3999, CURRENT_CUTOFF_MAX_HZ).is_some());
        assert!(butterworth_lowpass(2000, 3999, MOTION_CUTOFF_MAX_HZ).is_none());
    }

    #[test]
    fn rejects_zero_pwm_maxcnt() {
        assert!(butterworth_lowpass(200, 0, CURRENT_CUTOFF_MAX_HZ).is_none());
    }

    #[test]
    fn higher_cutoff_has_larger_b_coefficients() {
        let low = butterworth_lowpass(50, 3999, CURRENT_CUTOFF_MAX_HZ).expect("50 Hz valid");
        let high = butterworth_lowpass(500, 3999, CURRENT_CUTOFF_MAX_HZ).expect("500 Hz valid");
        assert!(
            high.b_0 > low.b_0,
            "higher cutoff should have larger b_0: 50 Hz={} 500 Hz={}",
            low.b_0,
            high.b_0,
        );
    }

    #[test]
    fn coefficients_are_within_q3_29_range() {
        let coeffs = butterworth_lowpass(200, 3999, CURRENT_CUTOFF_MAX_HZ).expect("valid params");
        assert!(
            coeffs.a_2 < 0,
            "a_2 (negated) should be negative for stable filter, got {}",
            coeffs.a_2
        );
        assert!(coeffs.b_0 > 0, "b_0 must be positive, got {}", coeffs.b_0);
        assert!(coeffs.b_1 > 0, "b_1 must be positive, got {}", coeffs.b_1);
        assert!(coeffs.b_2 > 0, "b_2 must be positive, got {}", coeffs.b_2);
    }
}
