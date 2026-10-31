//! Per-channel record of applied host mutators, reported by the control-state trace frame.

use core::sync::atomic::{AtomicBool, AtomicU8, AtomicU32, Ordering};

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
    /// Count of applied mutator requests on this channel since boot.
    pub change_sequence: u32,
    /// Frame-domain tick of the control tick that applied the latest request; 0 for a
    /// metadata re-emission.
    pub applied_tick: u64,
    /// Kind of the latest applied request.
    pub applied_request: AppliedRequest,
}

/// Per-channel record of the latest applied host mutator. Lives in `TmcChannelState`.
///
/// The tick is split into two words because the Cortex-M targets have no 64-bit atomics.
pub struct ControlStateRecord {
    change_sequence: AtomicU32,
    applied_request: AtomicU8,
    applied_tick_low: AtomicU32,
    applied_tick_high: AtomicU32,
    dirty: AtomicBool,
    tick_pending: AtomicBool,
    snapshot_pending: AtomicBool,
}

impl Default for ControlStateRecord {
    fn default() -> Self {
        Self::new()
    }
}

impl ControlStateRecord {
    /// Create an empty record: sequence 0, nothing pending.
    pub const fn new() -> Self {
        Self {
            change_sequence: AtomicU32::new(0),
            applied_request: AtomicU8::new(AppliedRequest::None as u8),
            applied_tick_low: AtomicU32::new(0),
            applied_tick_high: AtomicU32::new(0),
            dirty: AtomicBool::new(false),
            tick_pending: AtomicBool::new(false),
            snapshot_pending: AtomicBool::new(false),
        }
    }

    /// Called by the control task on the tick that applied `request`.
    ///
    /// Increments the sequence, stores the kind, marks dirty and tick-pending.
    pub fn record(&self, request: AppliedRequest) {
        self.change_sequence.fetch_add(1, Ordering::Relaxed);
        self.applied_request.store(request as u8, Ordering::Relaxed);
        self.tick_pending.store(true, Ordering::Release);
        self.dirty.store(true, Ordering::Release);
    }

    /// Called by the board on the same tick with the frame-domain timestamp.
    ///
    /// No-op unless a record is waiting for its tick.
    pub fn stamp(&self, applied_tick: u64) {
        if !self.tick_pending.swap(false, Ordering::AcqRel) {
            return;
        }
        self.applied_tick_low
            .store(applied_tick as u32, Ordering::Relaxed);
        self.applied_tick_high
            .store((applied_tick >> 32) as u32, Ordering::Relaxed);
    }

    /// Mark that a metadata emission needs its own `AppliedRequest::None` frame.
    ///
    /// Independent of the mutator record, so a pending mutator frame is never overwritten.
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

    /// Header for the mutator frame. Does not clear dirty.
    pub fn snapshot(&self) -> ControlStateHeader {
        let low = u64::from(self.applied_tick_low.load(Ordering::Relaxed));
        let high = u64::from(self.applied_tick_high.load(Ordering::Relaxed));
        ControlStateHeader {
            change_sequence: self.change_sequence.load(Ordering::Relaxed),
            applied_tick: (high << 32) | low,
            applied_request: AppliedRequest::from_u8(self.applied_request.load(Ordering::Relaxed)),
        }
    }

    /// Return whether a mutator frame is still owed.
    pub fn is_dirty(&self) -> bool {
        self.dirty.load(Ordering::Acquire)
    }

    /// Clear dirty once the mutator frame was queued.
    pub fn clear_dirty(&self) {
        self.dirty.store(false, Ordering::Release);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn record_increments_sequence_and_sets_dirty_and_tick_pending() {
        let r = ControlStateRecord::new();
        assert!(!r.is_dirty());
        r.record(AppliedRequest::OuterGains);
        let h = r.snapshot();
        assert_eq!(h.change_sequence, 1);
        assert_eq!(h.applied_request, AppliedRequest::OuterGains);
        assert!(r.is_dirty());
        r.stamp(0x1_0000_0002);
        assert_eq!(r.snapshot().applied_tick, 0x1_0000_0002);
    }

    #[test]
    fn stamp_without_pending_record_is_ignored() {
        let r = ControlStateRecord::new();
        r.stamp(99);
        assert_eq!(r.snapshot().applied_tick, 0);
    }

    #[test]
    fn two_records_before_a_stamp_coalesce_with_sequence_advanced_by_two() {
        let r = ControlStateRecord::new();
        r.record(AppliedRequest::VelocityFilter);
        r.record(AppliedRequest::TorqueFilter);
        r.stamp(500);
        let h = r.snapshot();
        assert_eq!(h.change_sequence, 2);
        assert_eq!(h.applied_request, AppliedRequest::TorqueFilter);
        assert_eq!(h.applied_tick, 500);
    }

    #[test]
    fn snapshot_after_metadata_reports_none_and_keeps_the_sequence() {
        let r = ControlStateRecord::new();
        r.record(AppliedRequest::RunCurrent);
        r.stamp(1);
        r.clear_dirty();
        r.mark_snapshot();
        assert!(r.is_snapshot_pending());
        assert!(!r.is_dirty());
        let h = r.snapshot_header();
        assert_eq!(h.change_sequence, 1);
        assert_eq!(h.applied_tick, 0);
        assert_eq!(h.applied_request, AppliedRequest::None);
    }

    #[test]
    fn mark_snapshot_leaves_an_unemitted_mutator_record_intact() {
        let r = ControlStateRecord::new();
        r.record(AppliedRequest::OuterGains);
        r.stamp(7);
        r.mark_snapshot();
        assert!(r.is_snapshot_pending());
        assert!(r.is_dirty());
        let h = r.snapshot();
        assert_eq!(h.applied_request, AppliedRequest::OuterGains);
        assert_eq!(h.applied_tick, 7);
        assert_eq!(h.change_sequence, 1);
    }
}
