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

    pub(crate) fn push_u8(&mut self, value: u8) {
        self.push_bytes(&value.to_le_bytes());
    }

    pub(crate) fn push_i8(&mut self, value: i8) {
        self.push_bytes(&value.to_le_bytes());
    }

    pub(crate) fn push_u16(&mut self, value: u16) {
        self.push_bytes(&value.to_le_bytes());
    }

    pub(crate) fn push_u32(&mut self, value: u32) {
        self.push_bytes(&value.to_le_bytes());
    }

    pub(crate) fn push_i32(&mut self, value: i32) {
        self.push_bytes(&value.to_le_bytes());
    }

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

/// Encode command-lifetime outer-safety timing outside stage evidence digests.
///
/// Raw-only: its sole caller is the raw-emission helper
/// `emit_outer_safety_timing_trace`, which pushes the encoded record to
/// `record_velocity_sweep_trace_record`. Kept on the raw-emission `trace`
/// feature rather than un-gated with the rest of the digest-path encoders.
#[cfg(feature = "trace")]
pub(crate) fn encode_outer_safety_timing_record(
    engine_sequence: u32,
    run_sequence: u32,
    summary: crate::commissioning::outer::OuterSafetyTimingSummary,
) -> VelocitySweepCanonicalRecord {
    let mut record = VelocitySweepCanonicalRecord::new();
    record.push_u8(30);
    record.push_u32(engine_sequence);
    record.push_u32(run_sequence);
    record.push_u32(summary.sample_count);
    for count in summary.bin_counts {
        record.push_u32(count);
    }
    record.push_u32(summary.max_gap_us);
    if let Some(context) = summary.max_context {
        record.push_u8(1);
        record.push_u8(context.stage as u8);
        record.push_u8(context.phase as u8);
        record.push_u8(context.rung);
        record.push_u8(context.slot);
        record.push_u8(u8::from(context.target_zero));
    } else {
        for _ in 0..6 {
            record.push_u8(0);
        }
    }
    record.push_u8(u8::try_from(summary.zero_count).expect("zero gap faults on first sample"));
    record.push_u8(
        u8::try_from(summary.over_bound_count).expect("over-bound gap faults on first sample"),
    );
    record
}

/// Encode one stage-neutral primitive current-evidence record.
pub(crate) fn encode_velocity_primitive_current_evidence(
    evidence: crate::commissioning::types::VelocityPrimitiveCurrentEvidence,
) -> VelocitySweepCanonicalRecord {
    let mut record = VelocitySweepCanonicalRecord::new();
    record.push_u8(31);
    record.push_u32(evidence.run_sequence);
    record.push_u16(evidence.evidence_sequence);
    record.push_u8(evidence.stage);
    record.push_u8(evidence.rung_index);
    record.push_u8(evidence.slot);
    record.push_u16(evidence.clamp_limit);
    record.push_u16(evidence.clamp_readback);
    record.push_u16(evidence.moving_pid_output_peak);
    record.push_u16(evidence.zero_pid_output_peak);
    record.push_u32(evidence.moving_status_flags);
    record.push_u32(evidence.zero_status_flags);
    record.push_u8(evidence.capability);
    record.push_u8(evidence.contact);
    record
}

#[cfg(all(test, feature = "trace-velocity-sweep"))]
mod outer_safety_timing_record_tests {
    use super::encode_outer_safety_timing_record;
    use crate::commissioning::outer::{
        OuterSafetyTimingContext, OuterSafetyTimingPhase, OuterSafetyTimingStage,
        OuterSafetyTimingSummary,
    };

    #[test]
    fn outer_safety_timing_record_is_bounded_and_exact() {
        let summary = OuterSafetyTimingSummary {
            sample_count: 89,
            bin_counts: [80, 7, 2, 0],
            max_gap_us: 14_329,
            max_context: Some(
                OuterSafetyTimingContext::new(
                    OuterSafetyTimingStage::Proportional,
                    OuterSafetyTimingPhase::EvidenceEmission,
                    21,
                    7,
                    true,
                )
                .unwrap(),
            ),
            zero_count: 0,
            over_bound_count: 0,
        };

        let record = encode_outer_safety_timing_record(11, 17, summary);
        let bytes = record.as_bytes();

        assert_eq!(bytes[0], 30);
        assert!(bytes.len() <= super::VELOCITY_SWEEP_CANONICAL_RECORD_MAX);
        assert_eq!(u32::from_le_bytes(bytes[1..5].try_into().unwrap()), 11);
        assert_eq!(u32::from_le_bytes(bytes[5..9].try_into().unwrap()), 17);
        assert_eq!(u32::from_le_bytes(bytes[9..13].try_into().unwrap()), 89);
        assert_eq!(
            u32::from_le_bytes(bytes[29..33].try_into().unwrap()),
            14_329
        );
        assert_eq!(&bytes[33..41], &[1, 0, 4, 21, 7, 1, 0, 0]);
    }
}

#[cfg(all(test, feature = "trace-velocity-sweep"))]
mod velocity_primitive_current_record_tests {
    use super::encode_velocity_primitive_current_evidence;
    use crate::commissioning::types::VelocityPrimitiveCurrentEvidence;

    #[test]
    fn velocity_primitive_current_record_is_bounded_and_exact() {
        let record = encode_velocity_primitive_current_evidence(VelocityPrimitiveCurrentEvidence {
            run_sequence: 0x0102_0304,
            evidence_sequence: 0x0506,
            stage: 1,
            rung_index: 7,
            slot: 3,
            clamp_limit: 2_534,
            clamp_readback: 2_534,
            moving_pid_output_peak: 2_111,
            zero_pid_output_peak: 2_534,
            moving_status_flags: 0x1122_3344,
            zero_status_flags: 0x5566_7788,
            capability: 0,
            contact: 2,
        });

        assert_eq!(
            record.as_bytes(),
            &[
                31, 4, 3, 2, 1, 6, 5, 1, 7, 3, 0xe6, 0x09, 0xe6, 0x09, 0x3f, 0x08, 0xe6, 0x09,
                0x44, 0x33, 0x22, 0x11, 0x88, 0x77, 0x66, 0x55, 0, 2,
            ]
        );
        assert!(record.as_bytes().len() <= super::VELOCITY_SWEEP_CANONICAL_RECORD_MAX);
    }
}
