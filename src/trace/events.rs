//! Shared trace event contract.

use crate::commissioning::types::EngineKind;

/// Informational USB trace event severity level.
pub const EVENT_LEVEL_INFO: u8 = 1;
/// Warning USB trace event severity level.
pub const EVENT_LEVEL_WARN: u8 = 2;
/// Error USB trace event severity level.
pub const EVENT_LEVEL_ERROR: u8 = 3;

/// USB trace subsystem ID for TMC control events.
pub const SUBSYSTEM_TMC_CONTROL: u8 = 1;
/// USB trace subsystem ID for step timer events.
pub const SUBSYSTEM_STEP_TIMER: u8 = 2;
/// USB trace subsystem ID for USB trace transport events.
pub const SUBSYSTEM_USB_TRACE: u8 = 3;
/// USB trace subsystem ID for Klipper protocol events.
pub const SUBSYSTEM_KLIPPER: u8 = 4;
/// USB trace subsystem ID for homing and endstop events.
pub const SUBSYSTEM_HOMING_ENDSTOP: u8 = 5;
/// USB trace subsystem ID for safety shutdown events.
pub const SUBSYSTEM_SAFETY_SHUTDOWN: u8 = 6;
/// USB trace subsystem ID for commissioning, calibration, selftest, and tune
/// events.
pub const SUBSYSTEM_COMMISSIONING: u8 = 7;

/// One-shot event reporting trace SPI sample duration.
pub const EVENT_SPI_SAMPLE_DURATION: u16 = 11;

/// One-shot event marking an accel/cruise/decel phase-class transition,
/// detected in the step-queue ISR from the loaded segment's `add` sign.
pub const EVENT_PHASE_BOUNDARY: u16 = 12;

/// Engine lifecycle start event.
pub const EVENT_COMMISSION_ENGINE_START: u16 = 100;
/// Phase lifecycle start event.
pub const EVENT_COMMISSION_PHASE_START: u16 = 101;
/// Phase lifecycle end event.
pub const EVENT_COMMISSION_PHASE_END: u16 = 102;
/// Engine lifecycle terminal event.
pub const EVENT_COMMISSION_ENGINE_TERMINAL: u16 = 103;

/// Reserved for historical decoding: the retired mechanical-ID phase's
/// summary result fragment. No live phase emits this event.
pub const EVENT_COMMISSION_MECHANICAL_ID_RESULT: u16 = 110;
/// Reserved for historical decoding: the retired mechanical-ID phase's
/// summary attempt fragment. No live phase emits this event.
pub const EVENT_COMMISSION_MECHANICAL_ID_ATTEMPT: u16 = 111;
/// Reserved for historical decoding: the retired mechanical-ID phase's
/// summary response fragment. No live phase emits this event.
pub const EVENT_COMMISSION_MECHANICAL_ID_RESPONSE: u16 = 112;
/// Reserved for historical decoding: the retired mechanical-ID phase's
/// summary classification fragment. No live phase emits this event.
pub const EVENT_COMMISSION_MECHANICAL_ID_CLASSIFICATION: u16 = 113;

/// Velocity-validation summary result fragment.
pub const EVENT_COMMISSION_VELOCITY_VALIDATE_RESULT: u16 = 120;
/// Velocity-validation summary gain fragment.
pub const EVENT_COMMISSION_VELOCITY_VALIDATE_GAIN: u16 = 121;
/// Velocity-validation summary metrics fragment.
pub const EVENT_COMMISSION_VELOCITY_VALIDATE_METRICS: u16 = 122;
/// Velocity-validation summary limits fragment.
pub const EVENT_COMMISSION_VELOCITY_VALIDATE_LIMITS: u16 = 123;
/// Disturbance-based velocity-sweep immutable plan.
pub const EVENT_COMMISSION_VELOCITY_SWEEP_PLAN: u16 = 124;
/// Disturbance-based velocity-sweep observation summary.
pub const EVENT_COMMISSION_VELOCITY_SWEEP_OBSERVATION: u16 = 125;
/// Disturbance-based velocity-sweep rung verdict.
pub const EVENT_COMMISSION_VELOCITY_SWEEP_RUNG: u16 = 126;
/// Derived Proportional structured-boundary, region, and handoff evidence.
pub const EVENT_COMMISSION_VELOCITY_SWEEP_DERIVED: u16 = 127;
/// Disturbance-based velocity-sweep detailed terminal.
pub const EVENT_COMMISSION_VELOCITY_SWEEP_TERMINAL: u16 = 128;
/// Disturbance-based velocity-sweep complete marker.
pub const EVENT_COMMISSION_VELOCITY_SWEEP_COMPLETE: u16 = 129;

/// Position-tune summary result fragment.
pub const EVENT_COMMISSION_POSITION_TUNE_RESULT: u16 = 130;
/// Position-tune summary target fragment.
pub const EVENT_COMMISSION_POSITION_TUNE_TARGET: u16 = 131;
/// Position-tune summary metrics fragment.
pub const EVENT_COMMISSION_POSITION_TUNE_METRICS: u16 = 132;
/// Position-tune summary limits fragment.
pub const EVENT_COMMISSION_POSITION_TUNE_LIMITS: u16 = 133;

/// Reserved for historical decoding: the retired velocity-integral plan
/// event. No live phase emits this event.
pub const EVENT_COMMISSION_VELOCITY_INTEGRAL_PLAN: u16 = 134;
/// Velocity-integral observation summary.
pub const EVENT_COMMISSION_VELOCITY_INTEGRAL_OBSERVATION: u16 = 135;
/// Velocity-integral rung consensus.
pub const EVENT_COMMISSION_VELOCITY_INTEGRAL_RUNG: u16 = 136;
/// Velocity-integral derived curve and reproduction evidence.
pub const EVENT_COMMISSION_VELOCITY_INTEGRAL_DERIVED: u16 = 137;
/// Velocity-integral detailed terminal.
pub const EVENT_COMMISSION_VELOCITY_INTEGRAL_TERMINAL: u16 = 138;
/// Velocity-integral reproduced-complete marker.
pub const EVENT_COMMISSION_VELOCITY_INTEGRAL_COMPLETE: u16 = 139;

/// Restore summary result fragment.
pub const EVENT_COMMISSION_RESTORE_RESULT: u16 = 140;
/// Restore summary error/drift fragment.
pub const EVENT_COMMISSION_RESTORE_ERROR_DRIFT: u16 = 141;
/// Restore summary final disposition fragment.
pub const EVENT_COMMISSION_RESTORE_DISPOSITION: u16 = 142;
/// One bounded return to the immutable commissioning rung origin.
pub const EVENT_COMMISSION_RUNG_ORIGIN_RECOVERY: u16 = 143;
/// Final command-lifetime outer-safety observation-gap summary marker.
pub const EVENT_COMMISSION_OUTER_SAFETY_TIMING: u16 = 144;
/// Retired: focused velocity-output-limit persistence diagnostic. The id stays
/// reserved (never reassigned) so historical captures still decode.
pub const EVENT_COMMISSION_VELOCITY_LIMIT_LATCH_TEST: u16 = 145;
/// Fixed-I velocity fixed-gain-amplitude immutable plan.
pub const EVENT_COMMISSION_FIXED_GAIN_AMPLITUDE_PLAN: u16 = 146;
/// Fixed-I velocity fixed-gain-amplitude observation evidence.
pub const EVENT_COMMISSION_FIXED_GAIN_AMPLITUDE_OBSERVATION: u16 = 147;
/// Fixed-I velocity per-amplitude consensus.
pub const EVENT_COMMISSION_FIXED_GAIN_AMPLITUDE_AMPLITUDE: u16 = 148;
/// Fixed-I velocity fixed-gain-amplitude terminal.
pub const EVENT_COMMISSION_FIXED_GAIN_AMPLITUDE_TERMINAL: u16 = 149;
/// Selected Proportional recovery lead-in and scored-rest evidence.
pub const EVENT_COMMISSION_SELECTED_RECOVERY_REST: u16 = 150;
/// Exact scored-rest suffix selection diagnostic.
pub const EVENT_COMMISSION_REST_SELECTION_DIAGNOSTIC: u16 = 151;
/// Breakaway-seeded campaign: immutable probe plan.
pub const EVENT_COMMISSION_BREAKAWAY_PROBE_PLAN: u16 = 152;
/// Breakaway-seeded campaign: resolved directional breakaway bracket.
pub const EVENT_COMMISSION_BREAKAWAY_PROBE_RESULT: u16 = 153;
/// Breakaway-seeded campaign: probe-campaign terminal.
pub const EVENT_COMMISSION_BREAKAWAY_PROBE_TERMINAL: u16 = 154;
/// Breakaway-seeded campaign: immutable additive discovery plan, its
/// ceiling's binding source, and its resolved ladder.
pub const EVENT_COMMISSION_BREAKAWAY_DISCOVERY_PLAN: u16 = 155;
/// Breakaway-seeded campaign: discovery terminal and the rung-zero motion
/// diagnostic (its collected in-band rung margins now stream separately under
/// `EVENT_COMMISSION_BREAKAWAY_DISCOVERY_MARGIN`).
pub const EVENT_COMMISSION_BREAKAWAY_DISCOVERY_TERMINAL: u16 = 156;
/// Breakaway-seeded campaign: immutable held-out confirmation plan.
pub const EVENT_COMMISSION_BREAKAWAY_CONFIRMATION_PLAN: u16 = 157;
/// Breakaway-seeded campaign: confirmation terminal and its fresh held-out
/// stroke observations.
pub const EVENT_COMMISSION_BREAKAWAY_CONFIRMATION_TERMINAL: u16 = 158;
/// Breakaway-seeded campaign: canonical closure record.
pub const EVENT_COMMISSION_BREAKAWAY_CAMPAIGN_TERMINAL: u16 = 159;
/// Breakaway-seeded campaign: one collected observation's raw settled-rate
/// evidence, streamed on the tick its stroke completes (discovery and
/// confirmation share this id; the record's `phase` field distinguishes them).
pub const EVENT_COMMISSION_BREAKAWAY_RAW_OBSERVATION: u16 = 160;
/// Breakaway-seeded campaign: one collected in-band rung's nomination margin,
/// streamed on its own tick as the discovery walk folds the rung, so the
/// discovery-terminal tick no longer fans the whole batch (up to 32 records)
/// into the bounded trace motion queue. A non-nominatable in-band rung produces
/// no record, so a gap in `collected_index` is normal.
pub const EVENT_COMMISSION_BREAKAWAY_DISCOVERY_MARGIN: u16 = 161;
/// Breakaway-seeded campaign: one recovered probe decision committed to the
/// adaptive cursor.
pub const EVENT_COMMISSION_BREAKAWAY_PROBE_DECISION: u16 = 162;
/// Breakaway-seeded campaign: one rejected recovery-rest scoring window.
/// Accepted rest windows emit nothing, so this id appears at most a handful of
/// times per run and normally not at all.
pub const EVENT_COMMISSION_BREAKAWAY_RECOVERY_REST_REJECTED: u16 = 163;
/// Breakaway-seeded campaign: one recovery refused because its origin moved
/// between the tick that captured it and the tick recovery began.
pub const EVENT_COMMISSION_BREAKAWAY_RECOVERY_ORIGIN_MISMATCH: u16 = 164;

/// Single-frame commissioning diagnostic warning event.
pub const EVENT_COMMISSION_DIAGNOSTIC_WARNING: u16 = 200;

/// USB trace event severity level.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum TraceEventLevel {
    /// Informational lifecycle or summary event.
    Info = 1,
    /// Warning event that does not necessarily terminate the engine.
    Warn = 2,
    /// Error or terminal-failure event.
    Error = 3,
}

/// Transport-neutral commissioning trace event emitted by shared engine code.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct CommissionTraceEvent {
    /// Event severity level.
    pub level: TraceEventLevel,
    /// Stable USB event ID from this module.
    pub event_id: u16,
    /// Engine kind associated with this event.
    pub engine_kind: EngineKind,
    /// Two transport-neutral payload slots appended after source and engine kind by boards.
    pub payload: [i32; 2],
    /// Number of populated payload slots, from 0 to 2.
    pub payload_count: u8,
}

/// Return the stable USB trace wire code for an engine kind.
pub fn engine_kind_wire_code(kind: EngineKind) -> u8 {
    match kind {
        EngineKind::Commission => 1,
        EngineKind::Calibrate => 2,
        EngineKind::Selftest => 3,
        EngineKind::OuterTune => 4,
    }
}

/// Convert an unsigned value into a positive i32 trace argument with saturation.
pub fn u32_arg(value: u32) -> i32 {
    value.min(i32::MAX as u32) as i32
}

/// Extend a wrapping 32-bit hardware clock into the u64 trace timestamp domain.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct TraceClockExt {
    last_low: u32,
    high: u64,
}

impl TraceClockExt {
    /// Create a zero-initialized trace clock extension state.
    pub const fn new() -> Self {
        Self {
            last_low: 0,
            high: 0,
        }
    }

    /// Extend one sampled low word into the current u64 timestamp epoch.
    pub fn extend_low(&mut self, low: u32) -> u64 {
        const WRAP_THRESHOLD: u32 = 1 << 31;
        let last_low = self.last_low;
        if low < last_low && last_low.wrapping_sub(low) > WRAP_THRESHOLD {
            self.high = self.high.wrapping_add(1u64 << 32);
            self.last_low = low;
        } else if low >= last_low {
            self.last_low = low;
        }
        self.high | u64::from(low)
    }
}

impl Default for TraceClockExt {
    fn default() -> Self {
        Self::new()
    }
}

/// Pack two u16 values into one compact i32 trace argument.
pub fn pack_u16_pair(low: u16, high: u16) -> i32 {
    i32::from_ne_bytes(((low as u32) | ((high as u32) << 16)).to_ne_bytes())
}

/// Pack two i16 values into one compact i32 trace argument.
pub fn pack_i16_pair(low: i16, high: i16) -> i32 {
    pack_u16_pair(low as u16, high as u16)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::commissioning::types::EngineKind;

    #[test]
    fn commissioning_event_ids_are_stable() {
        assert_eq!(EVENT_COMMISSION_ENGINE_START, 100);
        assert_eq!(EVENT_COMMISSION_PHASE_START, 101);
        assert_eq!(EVENT_COMMISSION_PHASE_END, 102);
        assert_eq!(EVENT_COMMISSION_ENGINE_TERMINAL, 103);
        assert_eq!(EVENT_COMMISSION_MECHANICAL_ID_RESULT, 110);
        assert_eq!(EVENT_COMMISSION_VELOCITY_VALIDATE_RESULT, 120);
        assert_eq!(EVENT_COMMISSION_VELOCITY_SWEEP_PLAN, 124);
        assert_eq!(EVENT_COMMISSION_VELOCITY_SWEEP_OBSERVATION, 125);
        assert_eq!(EVENT_COMMISSION_VELOCITY_SWEEP_RUNG, 126);
        assert_eq!(EVENT_COMMISSION_VELOCITY_SWEEP_TERMINAL, 128);
        assert_eq!(EVENT_COMMISSION_VELOCITY_SWEEP_COMPLETE, 129);
        assert_eq!(EVENT_COMMISSION_POSITION_TUNE_RESULT, 130);
        assert_eq!(EVENT_COMMISSION_VELOCITY_INTEGRAL_PLAN, 134);
        assert_eq!(EVENT_COMMISSION_VELOCITY_INTEGRAL_OBSERVATION, 135);
        assert_eq!(EVENT_COMMISSION_VELOCITY_INTEGRAL_RUNG, 136);
        assert_eq!(EVENT_COMMISSION_VELOCITY_INTEGRAL_DERIVED, 137);
        assert_eq!(EVENT_COMMISSION_VELOCITY_INTEGRAL_TERMINAL, 138);
        assert_eq!(EVENT_COMMISSION_VELOCITY_INTEGRAL_COMPLETE, 139);
        assert_eq!(EVENT_COMMISSION_RESTORE_RESULT, 140);
        assert_eq!(EVENT_COMMISSION_VELOCITY_LIMIT_LATCH_TEST, 145);
        assert_eq!(EVENT_COMMISSION_FIXED_GAIN_AMPLITUDE_PLAN, 146);
        assert_eq!(EVENT_COMMISSION_FIXED_GAIN_AMPLITUDE_OBSERVATION, 147);
        assert_eq!(EVENT_COMMISSION_FIXED_GAIN_AMPLITUDE_AMPLITUDE, 148);
        assert_eq!(EVENT_COMMISSION_FIXED_GAIN_AMPLITUDE_TERMINAL, 149);
        assert_eq!(EVENT_COMMISSION_SELECTED_RECOVERY_REST, 150);
        assert_eq!(EVENT_COMMISSION_REST_SELECTION_DIAGNOSTIC, 151);
        assert_eq!(EVENT_COMMISSION_BREAKAWAY_PROBE_PLAN, 152);
        assert_eq!(EVENT_COMMISSION_BREAKAWAY_PROBE_RESULT, 153);
        assert_eq!(EVENT_COMMISSION_BREAKAWAY_PROBE_TERMINAL, 154);
        assert_eq!(EVENT_COMMISSION_BREAKAWAY_DISCOVERY_PLAN, 155);
        assert_eq!(EVENT_COMMISSION_BREAKAWAY_DISCOVERY_TERMINAL, 156);
        assert_eq!(EVENT_COMMISSION_BREAKAWAY_CONFIRMATION_PLAN, 157);
        assert_eq!(EVENT_COMMISSION_BREAKAWAY_CONFIRMATION_TERMINAL, 158);
        assert_eq!(EVENT_COMMISSION_BREAKAWAY_CAMPAIGN_TERMINAL, 159);
        assert_eq!(EVENT_COMMISSION_BREAKAWAY_RAW_OBSERVATION, 160);
        assert_eq!(EVENT_COMMISSION_BREAKAWAY_PROBE_DECISION, 162);
        assert_eq!(EVENT_COMMISSION_DIAGNOSTIC_WARNING, 200);
    }

    #[test]
    fn trace_event_ids_are_stable() {
        assert_eq!(EVENT_COMMISSION_RUNG_ORIGIN_RECOVERY, 143);
        assert_eq!(EVENT_COMMISSION_OUTER_SAFETY_TIMING, 144);
    }

    #[test]
    fn trace_event_level_has_wire_repr() {
        assert_eq!(EVENT_LEVEL_INFO, 1);
        assert_eq!(EVENT_LEVEL_WARN, 2);
        assert_eq!(EVENT_LEVEL_ERROR, 3);
        assert_eq!(TraceEventLevel::Info as u8, EVENT_LEVEL_INFO);
        assert_eq!(TraceEventLevel::Warn as u8, EVENT_LEVEL_WARN);
        assert_eq!(TraceEventLevel::Error as u8, EVENT_LEVEL_ERROR);
    }

    #[test]
    fn trace_subsystem_ids_are_stable() {
        assert_eq!(SUBSYSTEM_TMC_CONTROL, 1);
        assert_eq!(SUBSYSTEM_STEP_TIMER, 2);
        assert_eq!(SUBSYSTEM_USB_TRACE, 3);
        assert_eq!(SUBSYSTEM_KLIPPER, 4);
        assert_eq!(SUBSYSTEM_HOMING_ENDSTOP, 5);
        assert_eq!(SUBSYSTEM_SAFETY_SHUTDOWN, 6);
        assert_eq!(SUBSYSTEM_COMMISSIONING, 7);
        assert_eq!(EVENT_SPI_SAMPLE_DURATION, 11);
        assert_eq!(EVENT_PHASE_BOUNDARY, 12);
    }

    #[test]
    fn engine_kind_wire_code_does_not_use_enum_discriminants() {
        assert_eq!(engine_kind_wire_code(EngineKind::Commission), 1);
        assert_eq!(engine_kind_wire_code(EngineKind::Calibrate), 2);
        assert_eq!(engine_kind_wire_code(EngineKind::Selftest), 3);
        assert_eq!(engine_kind_wire_code(EngineKind::OuterTune), 4);
    }

    #[test]
    fn unsigned_to_i32_saturates() {
        assert_eq!(u32_arg(7), 7);
        assert_eq!(u32_arg(i32::MAX as u32), i32::MAX);
        assert_eq!(u32_arg(u32::MAX), i32::MAX);
    }

    #[test]
    fn trace_clock_extension_handles_interleaved_contexts() {
        let mut ext = TraceClockExt::new();

        assert_eq!(ext.extend_low(0xffff_ff00), 0xffff_ff00);
        assert_eq!(ext.extend_low(0xffff_ff80), 0xffff_ff80);
        assert_eq!(ext.extend_low(0x0000_0040), 0x1_0000_0040);
        assert_eq!(ext.extend_low(0x0000_0080), 0x1_0000_0080);
    }
}
