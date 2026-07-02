//! Shared high-rate electrical capture primitives.

mod estimate;

pub use estimate::{
    FitWindow, InductanceEvidence, LinearFit, OnsetThreshold, RDropModel, detect_theta_onset_us,
    effective_frequency_hz, fit_current_slope, inductance_sensitivity,
};

/// Nominal validation raw-sample capacity.
pub const HIGH_RATE_CAPTURE_SAMPLE_CAPACITY: usize = 512;

/// R perturbation used by the structural gate.
pub const R_SENSITIVITY_PERTURB_PERMILLE: u16 = 200;

/// Maximum accepted L shift under +/-20% R perturbation.
pub const R_SENSITIVITY_MAX_L_SHIFT_PERMILLE: u16 = 30;

/// One timestamped electrical capture sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct ElectricalCaptureSample {
    /// Monotonic timestamp in microseconds from the board `Mono::now()` seam.
    pub timestamp_us: u64,
    /// Commanded q-axis external voltage count.
    pub uq_ext: i16,
    /// Commanded d-axis external voltage count.
    pub ud_ext: i16,
    /// Primary current observation in TMC4671 current counts.
    pub current_primary: i16,
    /// Secondary current observation in TMC4671 current counts.
    pub current_secondary: i16,
    /// Encoder count sampled with the current observation.
    pub encoder_count: u32,
    /// Compact TMC/sample status flags.
    pub status_flags: u32,
}

impl ElectricalCaptureSample {
    /// Build one capture sample.
    pub const fn new(
        timestamp_us: u64,
        uq_ext: i16,
        ud_ext: i16,
        current_primary: i16,
        current_secondary: i16,
        encoder_count: u32,
        status_flags: u32,
    ) -> Self {
        Self {
            timestamp_us,
            uq_ext,
            ud_ext,
            current_primary,
            current_secondary,
            encoder_count,
            status_flags,
        }
    }
}

/// Error returned by fixed-capacity capture storage.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum CaptureBufferError {
    /// The buffer is full.
    Full,
}

/// Fixed-capacity sample buffer for high-rate electrical captures.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ElectricalCaptureBuffer<const N: usize> {
    samples: heapless::Vec<ElectricalCaptureSample, N>,
}

impl<const N: usize> ElectricalCaptureBuffer<N> {
    /// Construct an empty capture buffer.
    pub const fn new() -> Self {
        Self {
            samples: heapless::Vec::new(),
        }
    }

    /// Append one sample.
    pub fn push(&mut self, sample: ElectricalCaptureSample) -> Result<(), CaptureBufferError> {
        self.samples
            .push(sample)
            .map_err(|_| CaptureBufferError::Full)
    }

    /// Return captured samples.
    pub fn as_slice(&self) -> &[ElectricalCaptureSample] {
        self.samples.as_slice()
    }

    /// Return the number of captured samples.
    pub fn len(&self) -> usize {
        self.samples.len()
    }

    /// Return true when no samples have been captured.
    pub fn is_empty(&self) -> bool {
        self.samples.is_empty()
    }
}

impl<const N: usize> Default for ElectricalCaptureBuffer<N> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fixed_buffer_rejects_sample_beyond_capacity() {
        let mut buffer: ElectricalCaptureBuffer<2> = ElectricalCaptureBuffer::new();

        assert!(
            buffer
                .push(ElectricalCaptureSample::new(0, 0, 100, 1, 0, 7, 0))
                .is_ok()
        );
        assert!(
            buffer
                .push(ElectricalCaptureSample::new(1, 0, 100, 2, 0, 7, 0))
                .is_ok()
        );
        assert_eq!(
            buffer.push(ElectricalCaptureSample::new(2, 0, 100, 3, 0, 7, 0)),
            Err(CaptureBufferError::Full)
        );
        assert_eq!(buffer.len(), 2);
    }
}
