//! Per-channel record of applied host mutators, reported by the control-state trace frame.

use core::sync::atomic::{AtomicBool, AtomicU8, AtomicU32, Ordering};

use super::stream::{CONTROL_STATE_PAYLOAD_LEN, put_i32, put_u16, put_u32, put_u64};

/// Which host mutator a control-state frame reports.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
#[repr(u8)]
pub enum AppliedRequest {
    /// No mutator; the frame re-emits state after metadata.
    None = 0,
    /// Velocity and position PI gains.
    OuterGains = 1,
    /// Flux and torque PI gains.
    InnerGains = 2,
    /// Velocity biquad filter.
    VelocityFilter = 3,
    /// Torque biquad filter.
    TorqueFilter = 4,
    /// Position biquad filter.
    PositionFilter = 5,
    /// Flux biquad filter.
    FluxFilter = 6,
    /// Velocity feedforward enable and gain.
    VelocityFeedforward = 7,
    /// Acceleration feedforward enable and gains.
    AccelFeedforward = 8,
    /// Run current.
    RunCurrent = 9,
    /// `PIDOUT_UQ_UD_LIMITS` voltage limit.
    VoltageLimit = 10,
}

impl AppliedRequest {
    fn from_u8(raw: u8) -> Self {
        match raw {
            1 => Self::OuterGains,
            2 => Self::InnerGains,
            3 => Self::VelocityFilter,
            4 => Self::TorqueFilter,
            5 => Self::PositionFilter,
            6 => Self::FluxFilter,
            7 => Self::VelocityFeedforward,
            8 => Self::AccelFeedforward,
            9 => Self::RunCurrent,
            10 => Self::VoltageLimit,
            _ => Self::None,
        }
    }
}

/// Application record carried at the head of a control-state frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct ControlStateHeader {
    /// Since-boot sequence of the reported request on this channel; for a metadata
    /// re-emission, the count of requests applied so far.
    pub change_sequence: u32,
    /// Frame-domain tick of the control tick that applied the reported request; 0 for a
    /// metadata re-emission.
    pub applied_tick: u64,
    /// Kind of the reported request.
    pub applied_request: AppliedRequest,
}

/// Unframed applications a channel holds before the oldest is dropped.
pub const CONTROL_STATE_FIFO_CAPACITY: usize = 16;

/// Per-channel FIFO of applied host mutators awaiting their control-state frame. Lives in
/// `TmcChannelState`.
///
/// Entry `n` (1-based, since boot) sits in slot `(n - 1) % CONTROL_STATE_FIFO_CAPACITY`, so
/// an entry's sequence is implied by its position and `change_sequence` doubles as the
/// tail. When the FIFO is full a new record drops the oldest entry; the emitted frames
/// then show a sequence jump, the only loss the decoder is allowed to see.
///
/// The tick is split into two words because the Cortex-M targets have no 64-bit atomics.
///
/// All methods must be called from the priority-1 control-task context: an entry spans
/// several words and `pop_pending` is not atomic with `pending_header`, so cross-context
/// use is unsupported.
pub struct ControlStateRecord {
    change_sequence: AtomicU32,
    framed_sequence: AtomicU32,
    stamped_sequence: AtomicU32,
    kinds: [AtomicU8; CONTROL_STATE_FIFO_CAPACITY],
    ticks_low: [AtomicU32; CONTROL_STATE_FIFO_CAPACITY],
    ticks_high: [AtomicU32; CONTROL_STATE_FIFO_CAPACITY],
    snapshot_pending: AtomicBool,
}

impl Default for ControlStateRecord {
    fn default() -> Self {
        Self::new()
    }
}

// slot() needs 2^32 to be a multiple of the capacity so the index survives u32 wraparound.
const _: () = assert!(CONTROL_STATE_FIFO_CAPACITY.is_power_of_two());

/// Wraps with the `u32` sequence because the capacity divides 2^32.
const fn slot(sequence: u32) -> usize {
    (sequence.wrapping_sub(1) as usize) % CONTROL_STATE_FIFO_CAPACITY
}

impl ControlStateRecord {
    /// Create an empty record: sequence 0, nothing pending.
    pub const fn new() -> Self {
        Self {
            change_sequence: AtomicU32::new(0),
            framed_sequence: AtomicU32::new(0),
            stamped_sequence: AtomicU32::new(0),
            kinds: [const { AtomicU8::new(AppliedRequest::None as u8) };
                CONTROL_STATE_FIFO_CAPACITY],
            ticks_low: [const { AtomicU32::new(0) }; CONTROL_STATE_FIFO_CAPACITY],
            ticks_high: [const { AtomicU32::new(0) }; CONTROL_STATE_FIFO_CAPACITY],
            snapshot_pending: AtomicBool::new(false),
        }
    }

    /// Called by the control task on the tick that applied `request`.
    ///
    /// Appends an entry with its tick pending and advances the sequence by one. A full
    /// FIFO drops its oldest entry first.
    pub fn record(&self, request: AppliedRequest) {
        let sequence = self.change_sequence.load(Ordering::Relaxed).wrapping_add(1);
        let index = slot(sequence);
        self.kinds[index].store(request as u8, Ordering::Relaxed);
        self.ticks_low[index].store(0, Ordering::Relaxed);
        self.ticks_high[index].store(0, Ordering::Relaxed);
        self.change_sequence.store(sequence, Ordering::Relaxed);
        let framed = self.framed_sequence.load(Ordering::Relaxed);
        if sequence.wrapping_sub(framed) as usize > CONTROL_STATE_FIFO_CAPACITY {
            self.framed_sequence.store(
                sequence.wrapping_sub(CONTROL_STATE_FIFO_CAPACITY as u32),
                Ordering::Relaxed,
            );
        }
    }

    /// Called by the board on the same tick with the frame-domain timestamp.
    ///
    /// Assigns `applied_tick` to every unframed entry recorded since the previous stamp;
    /// a no-op when none is waiting.
    pub fn stamp(&self, applied_tick: u64) {
        let tail = self.change_sequence.load(Ordering::Relaxed);
        let stamped = self.stamped_sequence.load(Ordering::Relaxed);
        let framed = self.framed_sequence.load(Ordering::Relaxed);
        let unstamped = tail.wrapping_sub(stamped).min(tail.wrapping_sub(framed));
        for offset in 0..unstamped {
            let index = slot(tail.wrapping_sub(offset));
            self.ticks_low[index].store(applied_tick as u32, Ordering::Relaxed);
            self.ticks_high[index].store((applied_tick >> 32) as u32, Ordering::Relaxed);
        }
        self.stamped_sequence.store(tail, Ordering::Relaxed);
    }

    /// Mark that a metadata emission needs its own `AppliedRequest::None` frame.
    ///
    /// Independent of the mutator FIFO, so pending mutator entries are never overwritten.
    pub fn mark_snapshot(&self) {
        self.snapshot_pending.store(true, Ordering::Release);
    }

    /// Header for the metadata snapshot frame: current sequence, tick 0, request `None`.
    pub fn snapshot_header(&self) -> ControlStateHeader {
        ControlStateHeader {
            change_sequence: self.change_sequence.load(Ordering::Relaxed),
            applied_tick: 0,
            applied_request: AppliedRequest::None,
        }
    }

    /// Return whether a metadata snapshot frame is still owed.
    pub fn is_snapshot_pending(&self) -> bool {
        self.snapshot_pending.load(Ordering::Acquire)
    }

    /// Clear once the snapshot frame was queued.
    pub fn clear_snapshot_pending(&self) {
        self.snapshot_pending.store(false, Ordering::Release);
    }

    /// Return whether any mutator entry still awaits its frame.
    pub fn is_dirty(&self) -> bool {
        self.framed_sequence.load(Ordering::Relaxed) != self.change_sequence.load(Ordering::Relaxed)
    }

    /// Header for the oldest unframed entry, or `None` when the FIFO is empty. Does not pop.
    ///
    /// An entry recorded after the latest `stamp` reports tick 0.
    pub fn pending_header(&self) -> Option<ControlStateHeader> {
        if !self.is_dirty() {
            return None;
        }
        let sequence = self.framed_sequence.load(Ordering::Relaxed).wrapping_add(1);
        let index = slot(sequence);
        let low = u64::from(self.ticks_low[index].load(Ordering::Relaxed));
        let high = u64::from(self.ticks_high[index].load(Ordering::Relaxed));
        Some(ControlStateHeader {
            change_sequence: sequence,
            applied_tick: (high << 32) | low,
            applied_request: AppliedRequest::from_u8(self.kinds[index].load(Ordering::Relaxed)),
        })
    }

    /// Drop the oldest unframed entry once its frame was queued; a no-op when empty.
    pub fn pop_pending(&self) {
        if self.is_dirty() {
            self.framed_sequence.store(
                self.framed_sequence.load(Ordering::Relaxed).wrapping_add(1),
                Ordering::Relaxed,
            );
        }
    }
}

/// Payload of a control-state frame. Little-endian, fixed offsets; see the field comments.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct ControlStateSample {
    /// Sequence, tick and kind of the reported request (offsets 0..13, pad to 16).
    pub header: ControlStateHeader,
    /// Encoder counts per revolution (offset 16).
    pub encoder_counts_per_rev: u32,
    /// PWM frequency in Hz (offset 20).
    pub pwm_hz: u32,
    /// Velocity meter selection (offset 24, pad to 28).
    pub velocity_meter_selection: u8,
    /// Velocity filter corner in Hz (offset 28).
    pub velocity_filter_hz: u16,
    /// Torque filter corner in Hz (offset 30).
    pub torque_filter_hz: u16,
    /// Position filter corner in Hz (offset 32).
    pub position_filter_hz: u16,
    /// Flux filter corner in Hz (offset 34).
    pub flux_filter_hz: u16,
    /// Velocity P gain (offset 36).
    pub velocity_p: i32,
    /// Velocity I gain (offset 40).
    pub velocity_i: i32,
    /// Position P gain (offset 44).
    pub position_p: i32,
    /// Position I gain (offset 48).
    pub position_i: i32,
    /// Torque P gain (offset 52).
    pub torque_p: i32,
    /// Torque I gain (offset 56).
    pub torque_i: i32,
    /// Flux P gain (offset 60).
    pub flux_p: i32,
    /// Flux I gain (offset 64).
    pub flux_i: i32,
    /// Velocity feedforward enable (offset 68, pad to 70).
    pub velocity_ff_enable: u8,
    /// Velocity feedforward gain in permille (offset 70).
    pub velocity_ff_permille: u16,
    /// Acceleration feedforward enable (offset 72, pad to 74).
    pub accel_ff_enable: u8,
    /// Acceleration feedforward gain (offset 74).
    pub accel_ff_gain: u16,
    /// Deceleration feedforward gain (offset 76).
    pub decel_ff_gain: u16,
    /// Run current (offset 78, deliberately unaligned).
    pub run_current: u32,
    /// `PIDOUT_UQ_UD_LIMITS` voltage limit (offset 82); bytes 84..88 are reserved zero.
    pub voltage_limit: u16,
}

/// Encode a control-state payload into an exact 88-byte buffer; padding and reserved bytes
/// are written as zero.
pub fn encode_control_state_payload(
    sample: &ControlStateSample,
    out: &mut [u8; CONTROL_STATE_PAYLOAD_LEN],
) {
    *out = [0; CONTROL_STATE_PAYLOAD_LEN];
    put_u32(out, 0, sample.header.change_sequence);
    put_u64(out, 4, sample.header.applied_tick);
    out[12] = sample.header.applied_request as u8;
    put_u32(out, 16, sample.encoder_counts_per_rev);
    put_u32(out, 20, sample.pwm_hz);
    out[24] = sample.velocity_meter_selection;
    put_u16(out, 28, sample.velocity_filter_hz);
    put_u16(out, 30, sample.torque_filter_hz);
    put_u16(out, 32, sample.position_filter_hz);
    put_u16(out, 34, sample.flux_filter_hz);
    put_i32(out, 36, sample.velocity_p);
    put_i32(out, 40, sample.velocity_i);
    put_i32(out, 44, sample.position_p);
    put_i32(out, 48, sample.position_i);
    put_i32(out, 52, sample.torque_p);
    put_i32(out, 56, sample.torque_i);
    put_i32(out, 60, sample.flux_p);
    put_i32(out, 64, sample.flux_i);
    out[68] = sample.velocity_ff_enable;
    put_u16(out, 70, sample.velocity_ff_permille);
    out[72] = sample.accel_ff_enable;
    put_u16(out, 74, sample.accel_ff_gain);
    put_u16(out, 76, sample.decel_ff_gain);
    put_u32(out, 78, sample.run_current);
    put_u16(out, 82, sample.voltage_limit);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::trace::stream::{FRAME_BUF_CAPACITY, FRAME_TYPE_CONTROL_STATE, HEADER_LEN};

    fn sample() -> ControlStateSample {
        ControlStateSample {
            header: ControlStateHeader {
                change_sequence: 0x0102_0304,
                applied_tick: 0x1112_1314_1516_1718,
                applied_request: AppliedRequest::VoltageLimit,
            },
            encoder_counts_per_rev: 0x2122_2324,
            pwm_hz: 0x3132_3334,
            velocity_meter_selection: 0x41,
            velocity_filter_hz: 0x5152,
            torque_filter_hz: 0x6162,
            position_filter_hz: 0x7172,
            flux_filter_hz: 0x8182,
            velocity_p: 0x0A01_0203,
            velocity_i: -0x0A01_0204,
            position_p: 0x0B01_0203,
            position_i: -0x0B01_0204,
            torque_p: 0x0C01_0203,
            torque_i: -0x0C01_0204,
            flux_p: 0x0D01_0203,
            flux_i: -0x0D01_0204,
            velocity_ff_enable: 0x91,
            velocity_ff_permille: 0x9293,
            accel_ff_enable: 0xA1,
            accel_ff_gain: 0xA2A3,
            decel_ff_gain: 0xB2B3,
            run_current: 0xC1C2_C3C4,
            voltage_limit: 0xD1D2,
        }
    }

    #[test]
    fn control_state_payload_layout_matches_the_wire_table() {
        let s = sample();
        let mut out = [0xFFu8; CONTROL_STATE_PAYLOAD_LEN];
        encode_control_state_payload(&s, &mut out);
        let u16_at = |o: usize| u16::from_le_bytes(out[o..o + 2].try_into().unwrap());
        let u32_at = |o: usize| u32::from_le_bytes(out[o..o + 4].try_into().unwrap());
        let i32_at = |o: usize| i32::from_le_bytes(out[o..o + 4].try_into().unwrap());
        assert_eq!(u32_at(0), s.header.change_sequence);
        assert_eq!(
            u64::from_le_bytes(out[4..12].try_into().unwrap()),
            s.header.applied_tick
        );
        assert_eq!(out[12], s.header.applied_request as u8);
        assert_eq!(&out[13..16], &[0, 0, 0]);
        assert_eq!(u32_at(16), s.encoder_counts_per_rev);
        assert_eq!(u32_at(20), s.pwm_hz);
        assert_eq!(out[24], s.velocity_meter_selection);
        assert_eq!(&out[25..28], &[0, 0, 0]);
        assert_eq!(u16_at(28), s.velocity_filter_hz);
        assert_eq!(u16_at(30), s.torque_filter_hz);
        assert_eq!(u16_at(32), s.position_filter_hz);
        assert_eq!(u16_at(34), s.flux_filter_hz);
        assert_eq!(i32_at(36), s.velocity_p);
        assert_eq!(i32_at(40), s.velocity_i);
        assert_eq!(i32_at(44), s.position_p);
        assert_eq!(i32_at(48), s.position_i);
        assert_eq!(i32_at(52), s.torque_p);
        assert_eq!(i32_at(56), s.torque_i);
        assert_eq!(i32_at(60), s.flux_p);
        assert_eq!(i32_at(64), s.flux_i);
        assert_eq!(out[68], s.velocity_ff_enable);
        assert_eq!(out[69], 0);
        assert_eq!(u16_at(70), s.velocity_ff_permille);
        assert_eq!(out[72], s.accel_ff_enable);
        assert_eq!(out[73], 0);
        assert_eq!(u16_at(74), s.accel_ff_gain);
        assert_eq!(u16_at(76), s.decel_ff_gain);
        assert_eq!(u32_at(78), s.run_current);
        assert_eq!(u16_at(82), s.voltage_limit);
        assert_eq!(&out[84..88], &[0, 0, 0, 0]);
    }

    #[test]
    fn control_state_is_frame_type_10_and_fits_the_frame_buffer() {
        assert_eq!(FRAME_TYPE_CONTROL_STATE, 10);
        assert!(HEADER_LEN + CONTROL_STATE_PAYLOAD_LEN <= FRAME_BUF_CAPACITY);
    }

    fn drain(r: &ControlStateRecord) -> std::vec::Vec<ControlStateHeader> {
        let mut out = std::vec::Vec::new();
        while let Some(h) = r.pending_header() {
            out.push(h);
            r.pop_pending();
        }
        out
    }

    #[test]
    fn record_queues_an_entry_that_the_next_stamp_ticks() {
        let r = ControlStateRecord::new();
        assert!(!r.is_dirty());
        assert_eq!(r.pending_header(), None);
        r.record(AppliedRequest::OuterGains);
        assert!(r.is_dirty());
        r.stamp(0x1_0000_0002);
        assert_eq!(
            r.pending_header(),
            Some(ControlStateHeader {
                change_sequence: 1,
                applied_tick: 0x1_0000_0002,
                applied_request: AppliedRequest::OuterGains,
            })
        );
        r.pop_pending();
        assert!(!r.is_dirty());
        assert_eq!(r.pending_header(), None);
    }

    #[test]
    fn stamp_without_pending_record_is_ignored() {
        let r = ControlStateRecord::new();
        r.stamp(99);
        r.record(AppliedRequest::RunCurrent);
        assert_eq!(r.pending_header().map(|h| h.applied_tick), Some(0));
    }

    #[test]
    fn four_records_before_a_stamp_yield_four_headers_in_request_order() {
        let r = ControlStateRecord::new();
        r.record(AppliedRequest::RunCurrent);
        r.stamp(100);
        r.pop_pending();
        let kinds = [
            AppliedRequest::VelocityFilter,
            AppliedRequest::TorqueFilter,
            AppliedRequest::PositionFilter,
            AppliedRequest::FluxFilter,
        ];
        for kind in kinds {
            r.record(kind);
        }
        r.stamp(500);
        let headers = drain(&r);
        let expected: std::vec::Vec<_> = kinds
            .iter()
            .zip(2u32..)
            .map(|(&applied_request, change_sequence)| ControlStateHeader {
                change_sequence,
                applied_tick: 500,
                applied_request,
            })
            .collect();
        assert_eq!(headers, expected);
        assert!(!r.is_dirty());
    }

    #[test]
    fn stamp_ticks_only_entries_recorded_before_it() {
        let r = ControlStateRecord::new();
        r.record(AppliedRequest::VelocityFilter);
        r.stamp(10);
        r.record(AppliedRequest::TorqueFilter);
        r.pop_pending();
        let h = r.pending_header().expect("torque entry pending");
        assert_eq!(h.applied_request, AppliedRequest::TorqueFilter);
        assert_eq!(h.applied_tick, 0, "not stamped yet");
        r.stamp(20);
        assert_eq!(r.pending_header().map(|h| h.applied_tick), Some(20));
        r.stamp(30);
        assert_eq!(
            r.pending_header().map(|h| h.applied_tick),
            Some(20),
            "a stamped entry keeps its tick"
        );
    }

    #[test]
    fn overflow_drops_the_oldest_entries_and_the_sequence_jump_shows_the_loss() {
        let r = ControlStateRecord::new();
        for _ in 0..CONTROL_STATE_FIFO_CAPACITY + 2 {
            r.record(AppliedRequest::InnerGains);
        }
        r.stamp(9);
        let headers = drain(&r);
        assert_eq!(headers.len(), CONTROL_STATE_FIFO_CAPACITY);
        assert_eq!(headers[0].change_sequence, 3);
        let sequences: std::vec::Vec<u32> = headers.iter().map(|h| h.change_sequence).collect();
        let expected: std::vec::Vec<u32> = (3..=18).collect();
        assert_eq!(sequences, expected);
        assert!(headers.iter().all(|h| h.applied_tick == 9));
        assert_eq!(r.snapshot_header().change_sequence, 18);
    }

    #[test]
    fn overflow_keeps_the_tick_of_surviving_stamped_entries() {
        let r = ControlStateRecord::new();
        for _ in 0..CONTROL_STATE_FIFO_CAPACITY {
            r.record(AppliedRequest::OuterGains);
        }
        r.stamp(5);
        r.record(AppliedRequest::RunCurrent);
        r.stamp(6);
        let headers = drain(&r);
        assert_eq!(headers.len(), CONTROL_STATE_FIFO_CAPACITY);
        assert_eq!(headers[0].change_sequence, 2);
        assert_eq!(headers[0].applied_tick, 5);
        let last = headers.last().expect("non-empty");
        assert_eq!(last.change_sequence, 17);
        assert_eq!(last.applied_tick, 6);
        assert_eq!(last.applied_request, AppliedRequest::RunCurrent);
    }

    #[test]
    fn snapshot_after_metadata_reports_none_and_keeps_the_sequence() {
        let r = ControlStateRecord::new();
        r.record(AppliedRequest::RunCurrent);
        r.stamp(1);
        r.pop_pending();
        r.mark_snapshot();
        assert!(r.is_snapshot_pending());
        assert!(!r.is_dirty());
        let h = r.snapshot_header();
        assert_eq!(h.change_sequence, 1);
        assert_eq!(h.applied_tick, 0);
        assert_eq!(h.applied_request, AppliedRequest::None);
        r.clear_snapshot_pending();
        assert!(!r.is_snapshot_pending());
    }

    #[test]
    fn mark_snapshot_leaves_unemitted_mutator_entries_intact() {
        let r = ControlStateRecord::new();
        r.record(AppliedRequest::OuterGains);
        r.record(AppliedRequest::InnerGains);
        r.stamp(7);
        r.mark_snapshot();
        assert!(r.is_snapshot_pending());
        assert!(r.is_dirty());
        assert_eq!(r.snapshot_header().change_sequence, 2);
        let headers = drain(&r);
        assert_eq!(
            headers,
            [
                ControlStateHeader {
                    change_sequence: 1,
                    applied_tick: 7,
                    applied_request: AppliedRequest::OuterGains,
                },
                ControlStateHeader {
                    change_sequence: 2,
                    applied_tick: 7,
                    applied_request: AppliedRequest::InnerGains,
                },
            ]
        );
        assert!(r.is_snapshot_pending());
    }
}
