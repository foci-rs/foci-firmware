//! PID gain computation for TMC4671 inner current loops.
//!
//! Computes flux and torque PI gains from measured motor resistance
//! and inductance. All math is integer-only (no_std compatible).

/// Validated motor parameters from R/L measurement.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MotorParams {
    /// Winding resistance in milliohms (average of both coils).
    pub resistance_mohm: u32,
    /// Winding inductance in microhenries (average of both coils).
    pub inductance_uh: u32,
}

/// Validation error for motor parameters.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum MotorParamError {
    /// Resistance is zero (open circuit or measurement failure).
    ZeroResistance,
    /// Resistance exceeds 50 ohms (50_000 milliohms).
    ResistanceTooHigh,
    /// Inductance is zero (no inductance or measurement failure).
    ZeroInductance,
    /// Inductance exceeds 50 mH (50_000 microhenries).
    InductanceTooHigh,
    /// Coil A and B resistance differ by more than 50%.
    ResistanceAsymmetry,
    /// Coil A and B inductance differ by more than 50%.
    InductanceAsymmetry,
}

/// Validate and average per-coil R/L measurements.
///
/// # Parameters
/// - `r_a`, `r_b`: per-coil resistance in milliohms
/// - `l_a`, `l_b`: per-coil inductance in microhenries
///
/// # Returns
/// Averaged `MotorParams` or validation error.
pub fn validate_motor_params(
    r_a: u16,
    r_b: u16,
    l_a: u16,
    l_b: u16,
) -> Result<MotorParams, MotorParamError> {
    if r_a == 0 || r_b == 0 {
        return Err(MotorParamError::ZeroResistance);
    }
    if r_a > 50_000 || r_b > 50_000 {
        return Err(MotorParamError::ResistanceTooHigh);
    }
    if l_a == 0 || l_b == 0 {
        return Err(MotorParamError::ZeroInductance);
    }
    if l_a > 50_000 || l_b > 50_000 {
        return Err(MotorParamError::InductanceTooHigh);
    }

    // Check asymmetry: |a - b| > max(a, b) / 2
    let r_max = r_a.max(r_b) as u32;
    let r_diff = (r_a as i32 - r_b as i32).unsigned_abs();
    if r_diff > r_max / 2 {
        return Err(MotorParamError::ResistanceAsymmetry);
    }

    let l_max = l_a.max(l_b) as u32;
    let l_diff = (l_a as i32 - l_b as i32).unsigned_abs();
    if l_diff > l_max / 2 {
        return Err(MotorParamError::InductanceAsymmetry);
    }

    Ok(MotorParams {
        resistance_mohm: (r_a as u32 + r_b as u32) / 2,
        inductance_uh: (l_a as u32 + l_b as u32) / 2,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn validate_typical_stepper() {
        // 17HS19-2004S1: ~1.2 ohm, ~1.5 mH per coil
        let result = validate_motor_params(1200, 1210, 1500, 1480);
        assert!(result.is_ok());
        let params = result.unwrap();
        assert_eq!(params.resistance_mohm, 1205);
        assert_eq!(params.inductance_uh, 1490);
    }

    #[test]
    fn reject_zero_resistance() {
        let result = validate_motor_params(0, 1200, 1500, 1500);
        assert_eq!(result, Err(MotorParamError::ZeroResistance));
    }

    #[test]
    fn reject_zero_inductance() {
        let result = validate_motor_params(1200, 1200, 0, 1500);
        assert_eq!(result, Err(MotorParamError::ZeroInductance));
    }

    #[test]
    fn reject_zero_resistance_coil_b() {
        let result = validate_motor_params(1200, 0, 1500, 1500);
        assert_eq!(result, Err(MotorParamError::ZeroResistance));
    }

    #[test]
    fn reject_zero_inductance_coil_b() {
        let result = validate_motor_params(1200, 1200, 1500, 0);
        assert_eq!(result, Err(MotorParamError::ZeroInductance));
    }

    #[test]
    fn reject_high_resistance() {
        let result = validate_motor_params(65535, 65535, 1500, 1500);
        assert_eq!(result, Err(MotorParamError::ResistanceTooHigh));
    }

    #[test]
    fn reject_high_inductance() {
        let result = validate_motor_params(1200, 1200, 65535, 65535);
        assert_eq!(result, Err(MotorParamError::InductanceTooHigh));
    }

    #[test]
    fn reject_resistance_asymmetry() {
        // 1200 vs 500 — differ by >50% of max
        let result = validate_motor_params(1200, 500, 1500, 1500);
        assert_eq!(result, Err(MotorParamError::ResistanceAsymmetry));
    }

    #[test]
    fn reject_inductance_asymmetry() {
        let result = validate_motor_params(1200, 1200, 1500, 600);
        assert_eq!(result, Err(MotorParamError::InductanceAsymmetry));
    }

    #[test]
    fn accept_moderate_asymmetry() {
        // 1200 vs 900 — differ by 25%, within 50% threshold
        let result = validate_motor_params(1200, 900, 1500, 1200);
        assert!(result.is_ok());
    }
}
