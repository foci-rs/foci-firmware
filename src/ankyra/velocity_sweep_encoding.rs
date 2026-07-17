//! Canonical bounded encodings shared by velocity-sweep replies and integrity.

/// Encode an unsigned value as an outward-rounded mantissa and right shift.
pub(crate) fn upper_bound_u128(value: u128) -> (u32, u8) {
    for shift in 0..=127u8 {
        let divisor = 1u128 << shift;
        let rounded = value.div_ceil(divisor);
        if let Ok(mantissa) = u32::try_from(rounded) {
            return (mantissa, shift);
        }
    }
    (u32::MAX, 127)
}

/// Encode a signed value as an outward-rounded mantissa and right shift.
pub(crate) fn upper_bound_i64(value: i64) -> (i32, u8) {
    let magnitude = i128::from(value).unsigned_abs();
    for shift in 0..=63u8 {
        let rounded = magnitude.div_ceil(1u128 << shift);
        if rounded <= i32::MAX as u128 {
            let mantissa = rounded as i32;
            return (if value < 0 { -mantissa } else { mantissa }, shift);
        }
    }
    (if value < 0 { -i32::MAX } else { i32::MAX }, 63)
}

/// Encode settled-rate evidence with one common outward-rounded right shift.
pub(crate) fn encode_rate_values(values: [i64; 5]) -> ([i32; 5], u8) {
    for shift in 0..=63u8 {
        let divisor = 1i128 << shift;
        let mut encoded = [0i32; 5];
        let mut fits = true;
        for (index, value) in values.into_iter().enumerate() {
            let value = i128::from(value);
            let quotient = value / divisor;
            let remainder = value % divisor;
            let is_lower_edge = matches!(index, 0 | 3);
            let is_upper_edge = matches!(index, 2 | 4);
            let outward = if is_lower_edge && remainder != 0 && value < 0 {
                quotient - 1
            } else if is_upper_edge && remainder != 0 && value > 0 {
                quotient + 1
            } else if index == 1 && remainder != 0 {
                quotient + value.signum()
            } else {
                quotient
            };
            let Ok(narrow) = i32::try_from(outward) else {
                fits = false;
                break;
            };
            encoded[index] = narrow;
        }
        if fits {
            return (encoded, shift);
        }
    }
    ([i32::MIN, 0, i32::MAX, i32::MIN, i32::MAX], 63)
}
