//! Canonical bounded encodings shared by velocity-sweep replies and integrity.

/// Maximum bytes in one canonical velocity-sweep evidence record.
pub const VELOCITY_SWEEP_CANONICAL_RECORD_MAX: usize = 128;

const FNV1A64_PRIME: u64 = 0x0000_0100_0000_01b3;

/// One fixed-width canonical velocity-sweep evidence record.
///
/// The same bytes feed the firmware integrity digest and the trace-only offline
/// audit stream, preventing those two representations from drifting apart.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct VelocitySweepCanonicalRecord {
    bytes: [u8; VELOCITY_SWEEP_CANONICAL_RECORD_MAX],
    len: u8,
}

impl VelocitySweepCanonicalRecord {
    #[cfg(feature = "trace-velocity-sweep")]
    pub(crate) const fn new() -> Self {
        Self {
            bytes: [0; VELOCITY_SWEEP_CANONICAL_RECORD_MAX],
            len: 0,
        }
    }

    /// Return the exact canonical bytes represented by this record.
    pub fn as_bytes(&self) -> &[u8] {
        &self.bytes[..usize::from(self.len)]
    }

    #[cfg(feature = "trace-velocity-sweep")]
    pub(crate) fn push_u8(&mut self, value: u8) {
        self.push_bytes(&value.to_le_bytes());
    }

    #[cfg(feature = "trace-velocity-sweep")]
    pub(crate) fn push_u16(&mut self, value: u16) {
        self.push_bytes(&value.to_le_bytes());
    }

    #[cfg(feature = "trace-velocity-sweep")]
    pub(crate) fn push_u32(&mut self, value: u32) {
        self.push_bytes(&value.to_le_bytes());
    }

    #[cfg(feature = "trace-velocity-sweep")]
    pub(crate) fn push_i32(&mut self, value: i32) {
        self.push_bytes(&value.to_le_bytes());
    }

    #[cfg(feature = "trace-velocity-sweep")]
    fn push_bytes(&mut self, value: &[u8]) {
        let start = usize::from(self.len);
        let end = start
            .checked_add(value.len())
            .expect("canonical velocity-sweep record length fits usize");
        assert!(
            end <= self.bytes.len(),
            "canonical velocity-sweep record exceeds audited capacity"
        );
        self.bytes[start..end].copy_from_slice(value);
        self.len = u8::try_from(end).expect("canonical velocity-sweep record length fits u8");
    }
}

/// Extend an existing FNV-1a sweep digest with one exact canonical record.
pub fn extend_velocity_sweep_digest(mut digest: u64, record: &VelocitySweepCanonicalRecord) -> u64 {
    for byte in record.as_bytes() {
        digest ^= u64::from(*byte);
        digest = digest.wrapping_mul(FNV1A64_PRIME);
    }
    digest
}

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
