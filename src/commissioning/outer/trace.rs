//! Outer commissioning trace-summary helpers.

use crate::commissioning::backend::CommissionTrace;
use crate::commissioning::types::EngineKind;
use crate::trace::events::{
    CommissionTraceEvent, EVENT_COMMISSION_DIAGNOSTIC_WARNING, TraceEventLevel, pack_i16_pair,
    pack_u16_pair, u32_arg,
};

use super::OUTER_WARNING_NONE;

#[derive(Debug, Clone, Copy, defmt::Format)]
pub(crate) struct TraceSummarySeqs {
    mechanical_id: u32,
    velocity_validate: u32,
    position_tune: u32,
    restore: u32,
}

impl TraceSummarySeqs {
    pub(crate) const fn new() -> Self {
        Self {
            mechanical_id: 0,
            velocity_validate: 0,
            position_tune: 0,
            restore: 0,
        }
    }

    pub(crate) fn next_mechanical_id(&mut self) -> i32 {
        let seq = u32_arg(self.mechanical_id);
        self.mechanical_id = self.mechanical_id.wrapping_add(1);
        seq
    }

    pub(crate) fn next_velocity_validate(&mut self) -> i32 {
        let seq = u32_arg(self.velocity_validate);
        self.velocity_validate = self.velocity_validate.wrapping_add(1);
        seq
    }

    pub(crate) fn next_position_tune(&mut self) -> i32 {
        let seq = u32_arg(self.position_tune);
        self.position_tune = self.position_tune.wrapping_add(1);
        seq
    }

    pub(crate) fn next_restore(&mut self) -> i32 {
        let seq = u32_arg(self.restore);
        self.restore = self.restore.wrapping_add(1);
        seq
    }
}

fn u16_trace_arg(value: u32) -> u16 {
    value.min(u16::MAX as u32) as u16
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

pub(crate) fn trace_diagnostic_warning<B: CommissionTrace>(
    backend: &mut B,
    warning_code: u8,
    detail: u16,
) {
    if warning_code == OUTER_WARNING_NONE {
        return;
    }

    backend.record_commission_trace_event(CommissionTraceEvent {
        level: TraceEventLevel::Warn,
        event_id: EVENT_COMMISSION_DIAGNOSTIC_WARNING,
        engine_kind: EngineKind::OuterTune,
        payload: [i32::from(warning_code), i32::from(detail)],
        payload_count: 2,
    });
}

pub(crate) fn trace_pack_u16_pair(low: u32, high: u32) -> i32 {
    pack_u16_pair(u16_trace_arg(low), u16_trace_arg(high))
}

pub(crate) fn trace_pack_i16_pair(low: i32, high: i32) -> i32 {
    pack_i16_pair(i16_trace_arg(low), i16_trace_arg(high))
}
