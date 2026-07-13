//! Allocation-free attribution of work overlapping a cycle deadline.

const HALF_COUNTER_RANGE: u32 = 1 << 31;

/// Accumulates source work that overlaps an armed deadline.
///
/// All timestamps use the same wrapping `u32` cycle counter. Comparisons are
/// valid when each observed interval is shorter than half the counter range.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DeadlineAttribution {
    due_cycle: u32,
    source_mask: u32,
    accumulated_overlap_cycles: u32,
    max_source_overlap_cycles: u32,
    deadline_expired_before_sleep: bool,
}

impl DeadlineAttribution {
    /// Arms an empty attribution model for `due_cycle`.
    #[must_use]
    pub const fn new(due_cycle: u32) -> Self {
        Self {
            due_cycle,
            source_mask: 0,
            accumulated_overlap_cycles: 0,
            max_source_overlap_cycles: 0,
            deadline_expired_before_sleep: false,
        }
    }

    /// Records one source's entry-to-exit span.
    ///
    /// Only cycles on or after the armed deadline are accumulated. The source
    /// bits are retained only when the span contributes a nonzero overlap.
    /// Spans must be chronological and shorter than half the counter range.
    pub fn record_source_span(&mut self, source_mask: u32, entry_cycle: u32, exit_cycle: u32) {
        let overlap_cycles = source_overlap_cycles(self.due_cycle, entry_cycle, exit_cycle);
        if overlap_cycles == 0 {
            return;
        }

        self.source_mask |= source_mask;
        self.accumulated_overlap_cycles = self
            .accumulated_overlap_cycles
            .saturating_add(overlap_cycles);
        self.max_source_overlap_cycles = self.max_source_overlap_cycles.max(overlap_cycles);
    }

    /// Classifies whether the deadline had expired when board sleep began.
    pub fn mark_sleep_entry(&mut self, sleep_entry_cycle: u32) {
        self.deadline_expired_before_sleep = cycle_at_or_after(sleep_entry_cycle, self.due_cycle);
    }

    /// Returns the accumulated attribution at `resume_cycle`.
    ///
    /// Source spans must be recorded after they exit and no later than the
    /// supplied resume timestamp.
    #[must_use]
    pub fn snapshot(&self, resume_cycle: u32) -> DeadlineAttributionSnapshot {
        let release_lateness_cycles = resume_cycle.wrapping_sub(self.due_cycle);
        let release_lateness_cycles = if release_lateness_cycles < HALF_COUNTER_RANGE {
            release_lateness_cycles
        } else {
            0
        };

        DeadlineAttributionSnapshot {
            release_lateness_cycles,
            source_mask: self.source_mask,
            accumulated_overlap_cycles: self.accumulated_overlap_cycles,
            max_source_overlap_cycles: self.max_source_overlap_cycles,
            deadline_expired_before_sleep: self.deadline_expired_before_sleep,
        }
    }
}

/// Compact deadline-release attribution captured when board execution resumes.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct DeadlineAttributionSnapshot {
    /// Cycles between the armed deadline and board resumption, or zero when
    /// resumption occurred before the deadline.
    pub release_lateness_cycles: u32,
    /// Bitwise union of sources with nonzero post-deadline overlap.
    pub source_mask: u32,
    /// Sum of all recorded post-deadline source spans, saturated at `u32::MAX`.
    pub accumulated_overlap_cycles: u32,
    /// Largest post-deadline overlap contributed by one recorded source span.
    pub max_source_overlap_cycles: u32,
    /// Whether the deadline had expired before the board entered sleep.
    pub deadline_expired_before_sleep: bool,
}

/// Return the portion of a chronological source span on or after `due_cycle`.
///
/// All timestamps use one wrapping `u32` cycle counter. The observed span must
/// be shorter than half the counter range.
#[must_use]
pub fn source_overlap_cycles(due_cycle: u32, entry_cycle: u32, exit_cycle: u32) -> u32 {
    if !cycle_at_or_after(exit_cycle, due_cycle) {
        return 0;
    }

    let overlap_start = if cycle_at_or_after(entry_cycle, due_cycle) {
        entry_cycle
    } else {
        due_cycle
    };
    let overlap_cycles = exit_cycle.wrapping_sub(overlap_start);
    if overlap_cycles >= HALF_COUNTER_RANGE {
        return 0;
    }
    overlap_cycles
}

fn cycle_at_or_after(cycle: u32, reference: u32) -> bool {
    cycle.wrapping_sub(reference) < HALF_COUNTER_RANGE
}

#[cfg(test)]
mod tests {
    use super::DeadlineAttribution;

    const USB_TASK: u32 = 1 << 2;

    #[test]
    fn source_spanning_deadline_attributes_only_post_deadline_work() {
        let mut attribution = DeadlineAttribution::new(100);

        attribution.record_source_span(USB_TASK, 90, 130);

        let snapshot = attribution.snapshot(140);
        assert_eq!(snapshot.release_lateness_cycles, 40);
        assert_eq!(snapshot.source_mask, USB_TASK);
        assert_eq!(snapshot.accumulated_overlap_cycles, 30);
    }

    #[test]
    fn source_starting_after_deadline_attributes_its_entire_span() {
        let mut attribution = DeadlineAttribution::new(100);

        attribution.record_source_span(USB_TASK, 115, 125);

        let snapshot = attribution.snapshot(140);
        assert_eq!(snapshot.source_mask, USB_TASK);
        assert_eq!(snapshot.accumulated_overlap_cycles, 10);
    }

    #[test]
    fn snapshot_retains_largest_single_source_overlap() {
        let mut attribution = DeadlineAttribution::new(100);

        attribution.record_source_span(USB_TASK, 105, 115);
        attribution.record_source_span(USB_TASK, 120, 145);

        let snapshot = attribution.snapshot(150);
        assert_eq!(snapshot.accumulated_overlap_cycles, 35);
        assert_eq!(snapshot.max_source_overlap_cycles, 25);
    }

    #[test]
    fn source_work_entirely_before_deadline_is_not_attributed() {
        let mut attribution = DeadlineAttribution::new(100);

        attribution.record_source_span(USB_TASK, 70, 90);

        let snapshot = attribution.snapshot(140);
        assert_eq!(snapshot.source_mask, 0);
        assert_eq!(snapshot.accumulated_overlap_cycles, 0);
    }

    #[test]
    fn attribution_handles_wrapped_dwt_timestamps() {
        let due_cycle = u32::MAX - 9;
        let mut attribution = DeadlineAttribution::new(due_cycle);

        attribution.record_source_span(USB_TASK, u32::MAX - 14, 10);

        let snapshot = attribution.snapshot(30);
        assert_eq!(snapshot.release_lateness_cycles, 40);
        assert_eq!(snapshot.source_mask, USB_TASK);
        assert_eq!(snapshot.accumulated_overlap_cycles, 20);
    }

    #[test]
    fn late_resume_without_overlapping_source_remains_unattributed() {
        let attribution = DeadlineAttribution::new(100);

        let snapshot = attribution.snapshot(130);
        assert_eq!(snapshot.release_lateness_cycles, 30);
        assert_eq!(snapshot.source_mask, 0);
        assert_eq!(snapshot.accumulated_overlap_cycles, 0);
    }

    #[test]
    fn sleep_entry_at_deadline_is_classified_as_pre_sleep_expiry() {
        let mut expired = DeadlineAttribution::new(100);
        expired.mark_sleep_entry(100);
        let mut not_expired = DeadlineAttribution::new(100);
        not_expired.mark_sleep_entry(99);

        assert!(expired.snapshot(130).deadline_expired_before_sleep);
        assert!(!not_expired.snapshot(130).deadline_expired_before_sleep);
    }
}
