//! Outer commissioning trace-summary helpers.

use crate::commissioning::backend::CommissionTrace;
use crate::commissioning::types::EngineKind;
use crate::trace::events::{CommissionTraceEvent, TraceEventLevel, pack_i16_pair, u32_arg};

use super::OuterSafetyTimingAccumulator;
#[cfg(any(test, feature = "trace"))]
use super::OuterSafetyTimingSummary;

#[derive(Debug, Clone, Copy, defmt::Format)]
pub(crate) struct TraceSummarySeqs {
    storage: OuterSafetyTimingAccumulator,
}

impl TraceSummarySeqs {
    pub(crate) const fn new() -> Self {
        Self {
            storage: OuterSafetyTimingAccumulator::new(),
        }
    }

    pub(crate) fn next_restore(&mut self) -> i32 {
        self.next_legacy_sequence(3)
    }

    pub(crate) fn outer_safety_timing_mut(&mut self) -> &mut OuterSafetyTimingAccumulator {
        &mut self.storage
    }

    #[cfg(any(test, feature = "trace"))]
    pub(crate) fn take_outer_safety_timing(&mut self) -> OuterSafetyTimingSummary {
        let summary = self.storage.summary();
        self.storage.clear();
        summary
    }

    fn next_legacy_sequence(&mut self, index: usize) -> i32 {
        let words = self.storage.words_mut();
        let seq = u32_arg(words[index]);
        words[index] = words[index].wrapping_add(1);
        seq
    }
}

fn i16_trace_arg(value: i32) -> i16 {
    value.clamp(i16::MIN as i32, i16::MAX as i32) as i16
}

pub(crate) fn trace_summary_event<B: CommissionTrace>(
    backend: &mut B,
    level: TraceEventLevel,
    event_id: u16,
    summary_seq: i32,
    value: i32,
) {
    backend.record_commission_trace_event(CommissionTraceEvent {
        level,
        event_id,
        engine_kind: EngineKind::OuterTune,
        payload: [summary_seq, value],
        payload_count: 2,
    });
}

pub(crate) fn trace_pack_i16_pair(low: i32, high: i32) -> i32 {
    pack_i16_pair(i16_trace_arg(low), i16_trace_arg(high))
}
