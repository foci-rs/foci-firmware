//! Compact trace-only timing evidence for outer safety observations.

use crate::autotune::CommissionError;

use super::velocity::sweep_plan::MAX_SAFETY_OBSERVATION_GAP_US;

const BIN_COUNT_BITS: u32 = 19;
const BIN_COUNT_MASK: u32 = (1 << BIN_COUNT_BITS) - 1;
const CONTEXT_MASK: u16 = (1 << 15) - 1;
const HAS_MAX_BIT: u32 = 1 << 27;
const ZERO_GAP_BIT: u32 = 1 << 28;
const OVER_BOUND_BIT: u32 = 1 << 29;

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) enum OuterSafetyTimingStage {
    #[cfg(any(test, feature = "trace"))]
    Proportional = 0,
    Integral = 1,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) enum OuterSafetyTimingPhase {
    Setup = 0,
    MovingCapture = 1,
    ZeroSettle = 2,
    Analysis = 3,
    EvidenceEmission = 4,
    RecoverySetup = 5,
    RecoveryMove = 6,
    RecoverySettle = 7,
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

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) struct OuterSafetyTimingContext {
    pub(crate) stage: OuterSafetyTimingStage,
    pub(crate) phase: OuterSafetyTimingPhase,
    /// Zero-based gain rung.
    pub(crate) rung: u8,
    /// Zero-based signed observation slot.
    pub(crate) slot: u8,
    pub(crate) target_zero: bool,
}

impl OuterSafetyTimingContext {
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

#[cfg(any(test, feature = "trace"))]
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) struct OuterSafetyTimingSummary {
    pub(crate) sample_count: u32,
    /// Counts in the four closed 5 ms quarters through the configured bound.
    pub(crate) bin_counts: [u32; 4],
    pub(crate) max_gap_us: u32,
    pub(crate) max_context: Option<OuterSafetyTimingContext>,
    /// Number of zero gaps; at most one because the first faults.
    pub(crate) zero_count: u32,
    /// Number of over-bound gaps; at most one because the first faults.
    pub(crate) over_bound_count: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) struct OuterSafetyTimingAccumulator {
    words: [u32; 4],
}

impl OuterSafetyTimingAccumulator {
    pub(crate) const fn new() -> Self {
        Self { words: [0; 4] }
    }

    pub(crate) fn observe(
        &mut self,
        gap_us: u64,
        context: OuterSafetyTimingContext,
    ) -> Result<(), CommissionError> {
        let gap_us = u32::try_from(gap_us).map_err(|_| CommissionError::SafetyTimingGapInvalid)?;

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
                    .ok_or(CommissionError::EvidenceBufferFull)?,
            );
        }

        if !self.has_maximum() || gap_us > self.max_gap_us() {
            self.set_maximum(gap_us, context);
        }
        Ok(())
    }

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

#[cfg(test)]
mod tests {
    use super::{
        OuterSafetyTimingAccumulator, OuterSafetyTimingContext, OuterSafetyTimingPhase,
        OuterSafetyTimingStage,
    };
    use crate::autotune::CommissionError;

    fn context() -> OuterSafetyTimingContext {
        OuterSafetyTimingContext::new(
            OuterSafetyTimingStage::Integral,
            OuterSafetyTimingPhase::MovingCapture,
            0,
            0,
            false,
        )
        .expect("in-range stage/phase/rung/slot")
    }

    #[test]
    fn observe_rejects_bin_count_overflow() {
        let mut accumulator = OuterSafetyTimingAccumulator::new();
        accumulator
            .set_bin_count_for_test(0, super::BIN_COUNT_MASK)
            .expect("in-range preset");
        let error = accumulator.observe(1_000, context()).unwrap_err();
        assert_eq!(error, CommissionError::EvidenceBufferFull);
    }
}
