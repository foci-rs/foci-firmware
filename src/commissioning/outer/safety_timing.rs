//! Compact trace-only timing evidence for outer safety observations.

use crate::autotune::CommissionError;

use super::velocity::sweep_plan::MAX_SAFETY_OBSERVATION_GAP_US;

const BIN_COUNT_BITS: u32 = 19;
const BIN_COUNT_MASK: u32 = (1 << BIN_COUNT_BITS) - 1;
const CONTEXT_MASK: u16 = (1 << 15) - 1;
const HAS_MAX_BIT: u32 = 1 << 27;
const ZERO_GAP_BIT: u32 = 1 << 28;
const OVER_BOUND_BIT: u32 = 1 << 29;

/// Stage owning one safety observation in a trace velocity workflow.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) enum OuterSafetyTimingStage {
    /// Proportional-gain sweep.
    ///
    /// Only the velocity-integral stage accumulates per-phase safety timing on
    /// the production path; this variant is constructed solely by the
    /// trace/test decode side (`from_packed`).
    #[cfg(any(test, feature = "trace"))]
    Proportional = 0,
    /// Velocity-integral response sweep.
    Integral = 1,
}

/// Stable execution location for a trace velocity safety observation.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) enum OuterSafetyTimingPhase {
    /// Setup or another pre-motion state.
    Setup = 0,
    /// Moving observation capture.
    MovingCapture = 1,
    /// Zero-target settle capture.
    ZeroSettle = 2,
    /// Cooperative analysis or region folding.
    Analysis = 3,
    /// Canonical evidence emission.
    EvidenceEmission = 4,
    /// Recovery preparation before a nonzero target.
    RecoverySetup = 5,
    /// Constant-amplitude recovery motion.
    RecoveryMove = 6,
    /// Recovery braking, settle, or rest analysis.
    RecoverySettle = 7,
    /// Terminal construction after motion has ended.
    Terminalization = 8,
}

impl OuterSafetyTimingPhase {
    #[cfg(any(test, feature = "trace"))]
    fn from_u8(value: u8) -> Option<Self> {
        match value {
            0 => Some(Self::Setup),
            1 => Some(Self::MovingCapture),
            2 => Some(Self::ZeroSettle),
            3 => Some(Self::Analysis),
            4 => Some(Self::EvidenceEmission),
            5 => Some(Self::RecoverySetup),
            6 => Some(Self::RecoveryMove),
            7 => Some(Self::RecoverySettle),
            8 => Some(Self::Terminalization),
            _ => None,
        }
    }
}

/// Exact location attached to the maximum realized observation gap.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) struct OuterSafetyTimingContext {
    /// Owning sweep stage.
    pub(crate) stage: OuterSafetyTimingStage,
    /// Cooperative execution phase.
    pub(crate) phase: OuterSafetyTimingPhase,
    /// Zero-based gain rung.
    pub(crate) rung: u8,
    /// Zero-based signed observation slot.
    pub(crate) slot: u8,
    /// Whether the active velocity target is zero.
    pub(crate) target_zero: bool,
}

impl OuterSafetyTimingContext {
    /// Construct a checked context that fits the audited packed representation.
    pub(crate) fn new(
        stage: OuterSafetyTimingStage,
        phase: OuterSafetyTimingPhase,
        rung: u8,
        slot: u8,
        target_zero: bool,
    ) -> Result<Self, CommissionError> {
        if rung >= 32 || slot >= 16 {
            return Err(CommissionError::InvalidSchedule);
        }
        Ok(Self {
            stage,
            phase,
            rung,
            slot,
            target_zero,
        })
    }

    fn packed(self) -> u16 {
        u16::from(self.phase as u8)
            | (u16::from(self.stage as u8) << 4)
            | (u16::from(self.rung) << 5)
            | (u16::from(self.slot) << 10)
            | (u16::from(self.target_zero) << 14)
    }

    #[cfg(any(test, feature = "trace"))]
    fn from_packed(value: u16) -> Option<Self> {
        let phase = OuterSafetyTimingPhase::from_u8((value & 0x0f) as u8)?;
        let stage = match (value >> 4) & 1 {
            0 => OuterSafetyTimingStage::Proportional,
            1 => OuterSafetyTimingStage::Integral,
            _ => unreachable!(),
        };
        Some(Self {
            stage,
            phase,
            rung: ((value >> 5) & 0x1f) as u8,
            slot: ((value >> 10) & 0x0f) as u8,
            target_zero: ((value >> 14) & 1) != 0,
        })
    }
}

/// Decoded command-lifetime timing evidence emitted after all internal motion.
#[cfg(any(test, feature = "trace"))]
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) struct OuterSafetyTimingSummary {
    /// Count of all realized inter-sample gaps.
    pub(crate) sample_count: u32,
    /// Counts in the four closed 5 ms quarters through the configured bound.
    pub(crate) bin_counts: [u32; 4],
    /// Largest realized gap in microseconds.
    pub(crate) max_gap_us: u32,
    /// First context at which the maximum occurred.
    pub(crate) max_context: Option<OuterSafetyTimingContext>,
    /// Number of zero gaps; at most one because the first faults.
    pub(crate) zero_count: u32,
    /// Number of over-bound gaps; at most one because the first faults.
    pub(crate) over_bound_count: u32,
}

/// Packed command-lifetime timing state sharing the legacy summary-sequence footprint.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) struct OuterSafetyTimingAccumulator {
    words: [u32; 4],
}

impl OuterSafetyTimingAccumulator {
    /// Construct empty timing evidence.
    pub(crate) const fn new() -> Self {
        Self { words: [0; 4] }
    }

    /// Record one realized encoder-read gap before any fault or delta interpretation.
    pub(crate) fn observe(
        &mut self,
        gap_us: u64,
        context: OuterSafetyTimingContext,
    ) -> Result<(), CommissionError> {
        let gap_us = u32::try_from(gap_us).map_err(|_| CommissionError::InvalidSchedule)?;

        if gap_us == 0 {
            self.words[3] |= ZERO_GAP_BIT;
        } else if u64::from(gap_us) > MAX_SAFETY_OBSERVATION_GAP_US {
            self.words[3] |= OVER_BOUND_BIT;
        } else {
            let bin = if gap_us <= 5_000 {
                0
            } else if gap_us <= 10_000 {
                1
            } else if gap_us <= 15_000 {
                2
            } else {
                3
            };
            let count = self.bin_count(bin);
            self.set_bin_count(
                bin,
                count
                    .checked_add(1)
                    .filter(|value| *value <= BIN_COUNT_MASK)
                    .ok_or(CommissionError::InvalidSchedule)?,
            );
        }

        if !self.has_maximum() || gap_us > self.max_gap_us() {
            self.set_maximum(gap_us, context);
        }
        Ok(())
    }

    /// Decode the exact timing summary without consuming it.
    #[cfg(any(test, feature = "trace"))]
    pub(crate) fn summary(&self) -> OuterSafetyTimingSummary {
        let bin_counts = [
            self.bin_count(0),
            self.bin_count(1),
            self.bin_count(2),
            self.bin_count(3),
        ];
        let zero_count = u32::from((self.words[3] & ZERO_GAP_BIT) != 0);
        let over_bound_count = u32::from((self.words[3] & OVER_BOUND_BIT) != 0);
        let sample_count = bin_counts.iter().copied().sum::<u32>() + zero_count + over_bound_count;
        OuterSafetyTimingSummary {
            sample_count,
            bin_counts,
            max_gap_us: self.max_gap_us(),
            max_context: self.has_maximum().then(|| {
                OuterSafetyTimingContext::from_packed(self.packed_context())
                    .expect("timing context was checked before packing")
            }),
            zero_count,
            over_bound_count,
        }
    }

    /// Clear all timing evidence before the storage returns to legacy sequence use.
    #[cfg(any(test, feature = "trace"))]
    pub(crate) fn clear(&mut self) {
        self.words = [0; 4];
    }

    pub(super) fn words_mut(&mut self) -> &mut [u32; 4] {
        &mut self.words
    }

    fn bin_count(&self, index: usize) -> u32 {
        self.words[index] & BIN_COUNT_MASK
    }

    fn set_bin_count(&mut self, index: usize, value: u32) {
        self.words[index] = (self.words[index] & !BIN_COUNT_MASK) | value;
    }

    fn has_maximum(&self) -> bool {
        self.words[3] & HAS_MAX_BIT != 0
    }

    fn max_gap_us(&self) -> u32 {
        (self.words[0] >> 19)
            | ((self.words[1] >> 19) << 13)
            | (((self.words[2] >> 19) & 0x3f) << 26)
    }

    #[cfg(any(test, feature = "trace"))]
    fn packed_context(&self) -> u16 {
        let low = (self.words[2] >> 25) & 0x7f;
        let high = (self.words[3] >> 19) & 0xff;
        ((high << 7) | low) as u16
    }

    fn set_maximum(&mut self, gap_us: u32, context: OuterSafetyTimingContext) {
        let packed_context = u32::from(context.packed() & CONTEXT_MASK);
        self.words[0] = (self.words[0] & BIN_COUNT_MASK) | ((gap_us & 0x1fff) << 19);
        self.words[1] = (self.words[1] & BIN_COUNT_MASK) | (((gap_us >> 13) & 0x1fff) << 19);
        self.words[2] = (self.words[2] & BIN_COUNT_MASK)
            | (((gap_us >> 26) & 0x3f) << 19)
            | ((packed_context & 0x7f) << 25);
        self.words[3] = (self.words[3] & (BIN_COUNT_MASK | ZERO_GAP_BIT | OVER_BOUND_BIT))
            | (((packed_context >> 7) & 0xff) << 19)
            | HAS_MAX_BIT;
    }

    #[cfg(test)]
    pub(crate) fn set_bin_count_for_test(
        &mut self,
        index: usize,
        value: u32,
    ) -> Result<(), CommissionError> {
        if index >= self.words.len() || value > BIN_COUNT_MASK {
            return Err(CommissionError::InvalidSchedule);
        }
        self.set_bin_count(index, value);
        Ok(())
    }
}
