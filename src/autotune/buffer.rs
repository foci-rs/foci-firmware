/// Number of samples in the step-response transient capture buffer.
///
/// Sized to fit one complete transient capture (256 × 4 bytes = 1,024 bytes).
/// Only one capture is active at a time; the buffer is reused across phases.
pub const TRANSIENT_LEN: usize = 256;

/// Actual observation offsets for one electrical-delay transient capture.
///
/// Offsets are measured in microseconds from the successful voltage-step
/// write. The fixed array is parallel to [`TransientBuffer`] samples.
#[derive(Debug, Clone, defmt::Format)]
pub struct DelayTimestampOffsets {
    offsets_us: [u16; TRANSIENT_LEN],
    len: u16,
}

impl Default for DelayTimestampOffsets {
    fn default() -> Self {
        Self::new()
    }
}

impl DelayTimestampOffsets {
    /// Create an empty offset container.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            offsets_us: [0; TRANSIENT_LEN],
            len: 0,
        }
    }

    /// Append one strictly increasing nonzero offset.
    ///
    /// # Errors
    ///
    /// Returns `TimestampRange` when `offset_us` cannot fit in `u16`,
    /// `NonMonotonicTime` when it does not advance, or `TelemetryOverflow`
    /// when the fixed container is full.
    pub fn push(
        &mut self,
        offset_us: u32,
    ) -> Result<(), crate::commissioning::capture_timing::TimingReject> {
        use crate::commissioning::capture_timing::TimingReject;

        let offset_us = u16::try_from(offset_us).map_err(|_| TimingReject::TimestampRange)?;
        if offset_us == 0
            || (self.len != 0 && offset_us <= self.offsets_us[usize::from(self.len - 1)])
        {
            return Err(TimingReject::NonMonotonicTime);
        }
        let index = usize::from(self.len);
        if index >= TRANSIENT_LEN {
            return Err(TimingReject::TelemetryOverflow);
        }
        self.offsets_us[index] = offset_us;
        self.len += 1;
        Ok(())
    }

    /// Number of stored offsets.
    #[must_use]
    pub const fn len(&self) -> u16 {
        self.len
    }

    /// Whether no offsets have been stored.
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Stored offsets in capture order.
    #[must_use]
    pub fn offsets_us(&self) -> &[u16] {
        &self.offsets_us[..usize::from(self.len)]
    }
}

/// Fixed-size ring buffer for raw step-response capture.
///
/// Statically allocated and reused across commissioning phases. After each
/// capture phase completes, scalar metrics are extracted and the buffer is
/// reset for the next phase. Raw transient streaming is deferred to Phase 6.
///
/// Memory: 1,024 bytes for samples, plus 4 bytes of metadata.
#[derive(Clone, defmt::Format)]
pub struct TransientBuffer {
    samples: [u32; TRANSIENT_LEN],
    /// Number of samples captured so far (always <= `TRANSIENT_LEN`).
    len: u16,
    /// Microseconds between consecutive samples.
    sample_period_us: u16,
}

impl Default for TransientBuffer {
    fn default() -> Self {
        Self::new()
    }
}

impl TransientBuffer {
    /// Create an empty buffer.
    ///
    /// Sample period defaults to 0. Call [`reset`](Self::reset) before
    /// each capture to set the period for that measurement.
    pub const fn new() -> Self {
        Self {
            samples: [0u32; TRANSIENT_LEN],
            len: 0,
            sample_period_us: 0,
        }
    }

    /// Reset the buffer for a new capture, optionally changing the sample period.
    ///
    /// # Parameters
    /// - `sample_period_us`: microseconds between consecutive samples for the
    ///   new capture
    pub fn reset(&mut self, sample_period_us: u16) {
        self.len = 0;
        self.sample_period_us = sample_period_us;
    }

    /// Append one sample to the buffer.
    ///
    /// Returns `true` if the sample was stored. Returns `false` if the buffer
    /// is already full; the sample is discarded.
    ///
    /// # Parameters
    /// - `value`: raw ADC or encoder count to store
    pub fn push(&mut self, value: u32) -> bool {
        let idx = self.len as usize;
        if idx >= TRANSIENT_LEN {
            return false;
        }
        self.samples[idx] = value;
        self.len += 1;
        true
    }

    /// Number of samples currently stored.
    pub fn len(&self) -> u16 {
        self.len
    }

    /// Returns `true` if no samples have been captured yet.
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Returns `true` if the buffer holds `TRANSIENT_LEN` samples.
    pub fn is_full(&self) -> bool {
        self.len as usize >= TRANSIENT_LEN
    }

    /// Microseconds between consecutive samples.
    pub fn sample_period_us(&self) -> u16 {
        self.sample_period_us
    }

    /// Slice of the captured samples (length may be less than `TRANSIENT_LEN`).
    pub fn samples(&self) -> &[u32] {
        &self.samples[..self.len as usize]
    }

    /// Write one engine-owned workspace word outside a transient capture.
    ///
    /// Commissioning phases that do not use the transient samples may reuse
    /// this fixed allocation instead of growing the engine's largest enum
    /// variant. The logical capture length is intentionally unchanged.
    pub(crate) fn write_workspace_word(&mut self, index: usize, value: u32) -> bool {
        let Some(destination) = self.samples.get_mut(index) else {
            return false;
        };
        *destination = value;
        true
    }

    /// Read one engine-owned workspace word outside a transient capture.
    pub(crate) fn workspace_word(&self, index: usize) -> Option<u32> {
        self.samples.get(index).copied()
    }
}

impl core::fmt::Debug for TransientBuffer {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("TransientBuffer")
            .field("len", &self.len)
            .field("sample_period_us", &self.sample_period_us)
            .finish()
    }
}

/// Capture direction for bidirectional current-loop validation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum CaptureDirection {
    /// Positive torque step.
    Forward,
    /// Negative torque step.
    Reverse,
}

/// Normalize a raw torque-actual reading by capture direction.
///
/// Forward captures keep the sign as-is; reverse captures negate so that a
/// correctly responding motor produces a positive value in both directions.
/// Negative normalized values (wrong-sign current) are clamped to zero — the
/// tail-level metric will be near-zero and the target-attainment check will
/// flag the failure.
pub fn normalize_capture_sample(torque_actual: i16, direction: CaptureDirection) -> u32 {
    let normalized = match direction {
        CaptureDirection::Forward => torque_actual as i32,
        CaptureDirection::Reverse => -(torque_actual as i32),
    };
    normalized.max(0) as u32
}

/// Capacity of the high-rate electrical-delay capture buffer and its
/// parallel offset container. Sized for FOCI-205: large enough to cover
/// `5 * tau_e_us` at both boards' default periods (see
/// `high_rate_sample_count` in `autotune/current.rs`). This is the same
/// constant `electrical_capture::HIGH_RATE_CAPTURE_SAMPLE_CAPACITY` uses
/// (re-exported, not redeclared, so the two capacities cannot drift).
pub const HIGH_RATE_DELAY_LEN: usize = crate::electrical_capture::HIGH_RATE_CAPTURE_SAMPLE_CAPACITY;

/// Fixed-capacity raw-sample buffer for the high-rate electrical-delay
/// capture. Parallel to [`HighRateDelayOffsets`]; unlike [`TransientBuffer`]
/// this buffer has no notion of a single fixed sample period, because each
/// sample's actual offset is recorded separately.
#[derive(Clone, defmt::Format)]
pub struct HighRateSampleBuffer {
    samples: [u32; HIGH_RATE_DELAY_LEN],
    len: u16,
}

impl Default for HighRateSampleBuffer {
    fn default() -> Self {
        Self::new()
    }
}

impl HighRateSampleBuffer {
    /// Create an empty buffer.
    pub const fn new() -> Self {
        Self {
            samples: [0u32; HIGH_RATE_DELAY_LEN],
            len: 0,
        }
    }

    /// Reset the buffer for a new capture.
    pub fn reset(&mut self) {
        self.len = 0;
    }

    /// Append one sample. Returns `true` if stored, `false` if full.
    pub fn push(&mut self, value: u32) -> bool {
        let idx = self.len as usize;
        if idx >= HIGH_RATE_DELAY_LEN {
            return false;
        }
        self.samples[idx] = value;
        self.len += 1;
        true
    }

    /// Number of samples currently stored.
    pub fn len(&self) -> u16 {
        self.len
    }

    /// Returns `true` if no samples have been captured yet.
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Returns `true` if the buffer holds `HIGH_RATE_DELAY_LEN` samples.
    pub fn is_full(&self) -> bool {
        self.len as usize >= HIGH_RATE_DELAY_LEN
    }

    /// Slice of the captured samples (length may be less than `HIGH_RATE_DELAY_LEN`).
    pub fn samples(&self) -> &[u32] {
        &self.samples[..self.len as usize]
    }
}

impl core::fmt::Debug for HighRateSampleBuffer {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("HighRateSampleBuffer")
            .field("len", &self.len)
            .finish()
    }
}

/// Actual per-sample observation offsets (microseconds from capture origin)
/// for one high-rate electrical-delay capture. Parallel to
/// [`HighRateSampleBuffer`]. Unlike [`DelayTimestampOffsets`] the values
/// pushed here are true elapsed time since capture origin, not derived from
/// a fixed nominal period.
#[derive(Debug, Clone, defmt::Format)]
pub struct HighRateDelayOffsets {
    offsets_us: [u16; HIGH_RATE_DELAY_LEN],
    len: u16,
}

impl Default for HighRateDelayOffsets {
    fn default() -> Self {
        Self::new()
    }
}

impl HighRateDelayOffsets {
    /// Create an empty offset container.
    pub const fn new() -> Self {
        Self {
            offsets_us: [0; HIGH_RATE_DELAY_LEN],
            len: 0,
        }
    }

    /// Append one strictly increasing nonzero offset.
    ///
    /// # Errors
    ///
    /// Returns `TimestampRange` when `offset_us` cannot fit in `u16`,
    /// `NonMonotonicTime` when it does not advance, or `TelemetryOverflow`
    /// when the fixed container is full.
    pub fn push(
        &mut self,
        offset_us: u32,
    ) -> Result<(), crate::commissioning::capture_timing::TimingReject> {
        use crate::commissioning::capture_timing::TimingReject;

        let offset_us = u16::try_from(offset_us).map_err(|_| TimingReject::TimestampRange)?;
        if offset_us == 0
            || (self.len != 0 && offset_us <= self.offsets_us[usize::from(self.len - 1)])
        {
            return Err(TimingReject::NonMonotonicTime);
        }
        let index = usize::from(self.len);
        if index >= HIGH_RATE_DELAY_LEN {
            return Err(TimingReject::TelemetryOverflow);
        }
        self.offsets_us[index] = offset_us;
        self.len += 1;
        Ok(())
    }

    /// Number of stored offsets.
    pub const fn len(&self) -> u16 {
        self.len
    }

    /// Whether no offsets have been stored.
    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Stored offsets in capture order.
    pub fn offsets_us(&self) -> &[u16] {
        &self.offsets_us[..usize::from(self.len)]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn transient_buffer_push_and_read() {
        let mut buf = TransientBuffer::new();
        buf.reset(160);
        assert!(buf.is_empty());
        assert_eq!(buf.len(), 0);
        assert_eq!(buf.sample_period_us(), 160);

        assert!(buf.push(10));
        assert!(buf.push(20));
        assert!(buf.push(30));

        assert!(!buf.is_empty());
        assert_eq!(buf.len(), 3);
        let s = buf.samples();
        assert_eq!(s, &[10u32, 20, 30]);
    }

    #[test]
    fn transient_buffer_full() {
        let mut buf = TransientBuffer::new();
        buf.reset(100);
        for i in 0u32..TRANSIENT_LEN as u32 {
            assert!(buf.push(i), "push {i} should succeed");
        }
        assert!(buf.is_full());
        assert_eq!(buf.len() as usize, TRANSIENT_LEN);

        // One more push must be rejected.
        assert!(!buf.push(9999), "push to full buffer must return false");
        // Length must not change.
        assert_eq!(buf.len() as usize, TRANSIENT_LEN);
    }

    #[test]
    fn transient_buffer_reset_clears() {
        let mut buf = TransientBuffer::new();
        buf.reset(160);
        for i in 0u32..10 {
            buf.push(i);
        }
        assert_eq!(buf.len(), 10);

        buf.reset(200);
        assert!(buf.is_empty());
        assert_eq!(buf.len(), 0);
        assert_eq!(buf.sample_period_us(), 200);
        assert_eq!(buf.samples(), &[] as &[u32]);
    }

    #[test]
    fn workspace_words_reuse_storage_without_forging_capture_samples() {
        let mut buf = TransientBuffer::new();

        assert!(buf.write_workspace_word(127, 0x89ab_cdef));
        assert_eq!(buf.workspace_word(127), Some(0x89ab_cdef));
        assert!(buf.is_empty());
        assert_eq!(buf.samples(), &[] as &[u32]);
        assert!(!buf.write_workspace_word(TRANSIENT_LEN, 1));
        assert_eq!(buf.workspace_word(TRANSIENT_LEN), None);
    }

    #[test]
    fn delay_timestamp_offsets_are_compact_and_range_checked() {
        assert_eq!(core::mem::size_of::<DelayTimestampOffsets>(), 514);
        let mut offsets = DelayTimestampOffsets::new();

        assert_eq!(offsets.push(160), Ok(()));
        assert_eq!(offsets.push(40_960), Ok(()));
        assert_eq!(offsets.offsets_us(), &[160, 40_960]);
        assert_eq!(
            offsets.push(65_536),
            Err(crate::commissioning::capture_timing::TimingReject::TimestampRange)
        );
    }

    #[test]
    fn delay_timestamp_offsets_reject_nonmonotonic_values() {
        let mut offsets = DelayTimestampOffsets::new();
        assert_eq!(offsets.push(160), Ok(()));
        assert_eq!(
            offsets.push(160),
            Err(crate::commissioning::capture_timing::TimingReject::NonMonotonicTime)
        );
        assert_eq!(
            offsets.push(159),
            Err(crate::commissioning::capture_timing::TimingReject::NonMonotonicTime)
        );
    }

    #[test]
    fn normalize_forward_positive_current() {
        // Forward capture, positive torque_actual → stored as-is.
        assert_eq!(
            normalize_capture_sample(367, CaptureDirection::Forward),
            367
        );
    }

    #[test]
    fn normalize_reverse_negative_current() {
        // Reverse capture, negative torque_actual → negated to positive.
        // A correctly responding motor produces negative current for a
        // negative torque step.
        assert_eq!(
            normalize_capture_sample(-367, CaptureDirection::Reverse),
            367
        );
    }

    #[test]
    fn normalize_reverse_wrong_sign_clamps_to_zero() {
        // Reverse capture but torque_actual is positive (wrong sign).
        // Normalized = -(+367) = -367 → clamped to 0.
        assert_eq!(normalize_capture_sample(367, CaptureDirection::Reverse), 0);
    }

    #[test]
    fn normalize_forward_wrong_sign_clamps_to_zero() {
        // Forward capture but torque_actual is negative (wrong sign).
        assert_eq!(normalize_capture_sample(-367, CaptureDirection::Forward), 0);
    }

    #[test]
    fn high_rate_sample_buffer_push_and_read() {
        let mut buf = HighRateSampleBuffer::new();
        buf.reset();
        assert!(buf.is_empty());
        assert!(buf.push(10));
        assert!(buf.push(20));
        assert!(buf.push(30));
        assert_eq!(buf.len(), 3);
        assert_eq!(buf.samples(), &[10u32, 20, 30]);
    }

    #[test]
    fn high_rate_sample_buffer_rejects_beyond_capacity() {
        let mut buf = HighRateSampleBuffer::new();
        buf.reset();
        for i in 0u32..HIGH_RATE_DELAY_LEN as u32 {
            assert!(buf.push(i), "push {i} should succeed");
        }
        assert!(buf.is_full());
        assert!(!buf.push(9999));
        assert_eq!(buf.len() as usize, HIGH_RATE_DELAY_LEN);
    }

    #[test]
    fn high_rate_delay_offsets_are_range_and_monotonicity_checked() {
        let mut offsets = HighRateDelayOffsets::new();
        assert_eq!(offsets.push(20), Ok(()));
        assert_eq!(offsets.push(40), Ok(()));
        assert_eq!(offsets.offsets_us(), &[20u16, 40]);
        assert_eq!(
            offsets.push(40),
            Err(crate::commissioning::capture_timing::TimingReject::NonMonotonicTime)
        );
        assert_eq!(
            offsets.push(65_536),
            Err(crate::commissioning::capture_timing::TimingReject::TimestampRange)
        );
    }
}
