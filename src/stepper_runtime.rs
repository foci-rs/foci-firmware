//! Channel-indexed stepper runtime helpers for board timer dispatch.

/// Maximum step/unstep events a timer IRQ may process before yielding.
pub const MAX_STEP_EVENTS_PER_IRQ: usize = 8;

/// Classification of an observed step deadline against the fatal-lateness policy.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum StepDeadlineStatus {
    OnTimeOrEarly,
    TolerablyLate { ticks: u32 },
    Fatal { ticks: u32 },
}

/// Classify actual step deadline lateness using wrap-safe signed timer ordering.
///
/// `scheduled` and `now` are comparable only when separated by less than half the
/// `u32` timer range; exactly half-range apart is ambiguous under signed-delta ordering.
pub const fn classify_step_deadline(
    scheduled: u32,
    now: u32,
    fatal_lateness_ticks: u32,
) -> StepDeadlineStatus {
    let signed_lateness = now.wrapping_sub(scheduled) as i32;
    if signed_lateness <= 0 {
        return StepDeadlineStatus::OnTimeOrEarly;
    }
    let ticks = signed_lateness as u32;
    if ticks > fatal_lateness_ticks {
        StepDeadlineStatus::Fatal { ticks }
    } else {
        StepDeadlineStatus::TolerablyLate { ticks }
    }
}

/// A selected step event deadline.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct DueStep {
    pub channel: u8,
    pub clock: u32,
}

#[inline]
fn timer_is_before(a: u32, b: u32) -> bool {
    (a as i32).wrapping_sub(b as i32) < 0
}

/// Return the earliest active channel, breaking equal-clock ties by channel index.
///
/// Active deadlines must be comparable within less than `0x8000_0000` ticks; exactly
/// half-range apart is ambiguous under the signed-delta timer ordering used here.
/// The returned `channel` is a board-local `u8`, so callers must use channel counts
/// that fit in `u8`.
pub fn earliest_due<const CHANNELS: usize>(events: &[Option<u32>; CHANNELS]) -> Option<DueStep> {
    let mut selected: Option<DueStep> = None;
    for (idx, event) in events.iter().enumerate() {
        let Some(clock) = *event else {
            continue;
        };
        match selected {
            None => {
                selected = Some(DueStep {
                    channel: idx as u8,
                    clock,
                });
            }
            Some(current)
                if timer_is_before(clock, current.clock)
                    || (clock == current.clock && (idx as u8) < current.channel) =>
            {
                selected = Some(DueStep {
                    channel: idx as u8,
                    clock,
                });
            }
            Some(_) => {}
        }
    }
    selected
}

/// Per-IRQ step event budget.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct StepIrqBudget {
    remaining: usize,
}

impl StepIrqBudget {
    /// Create a budget initialized to `MAX_STEP_EVENTS_PER_IRQ`.
    pub const fn new() -> Self {
        Self {
            remaining: MAX_STEP_EVENTS_PER_IRQ,
        }
    }

    /// Consume one event slot. Returns false when the cap has already been hit.
    pub fn take(&mut self) -> bool {
        if self.remaining == 0 {
            return false;
        }
        self.remaining -= 1;
        true
    }
}

impl Default for StepIrqBudget {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn actual_lateness_is_fatal_only_above_one_millisecond() {
        assert_eq!(
            classify_step_deadline(1_000, 85_000, 84_000),
            StepDeadlineStatus::TolerablyLate { ticks: 84_000 }
        );
        assert_eq!(
            classify_step_deadline(1_000, 85_001, 84_000),
            StepDeadlineStatus::Fatal { ticks: 84_001 }
        );
    }

    #[test]
    fn actual_lateness_classification_is_wrap_safe() {
        assert_eq!(
            classify_step_deadline(0xffff_fff0, 0x0000_0010, 84_000),
            StepDeadlineStatus::TolerablyLate { ticks: 32 }
        );
        assert_eq!(
            classify_step_deadline(0x0000_0010, 0xffff_fff0, 84_000),
            StepDeadlineStatus::OnTimeOrEarly
        );
    }

    #[test]
    fn earliest_due_returns_none_when_all_channels_idle() {
        let events = [None, None];
        assert_eq!(earliest_due(&events), None);
    }

    #[test]
    fn earliest_due_selects_single_active_channel() {
        let events = [None, Some(150), None];
        assert_eq!(
            earliest_due(&events),
            Some(DueStep {
                channel: 1,
                clock: 150,
            })
        );
    }

    #[test]
    fn earliest_due_selects_lowest_clock() {
        let events = [Some(200), Some(100)];
        assert_eq!(
            earliest_due(&events),
            Some(DueStep {
                channel: 1,
                clock: 100,
            })
        );
    }

    #[test]
    fn earliest_due_breaks_ties_by_lowest_channel() {
        let events = [Some(100), Some(100)];
        assert_eq!(
            earliest_due(&events),
            Some(DueStep {
                channel: 0,
                clock: 100,
            })
        );
    }

    #[test]
    fn earliest_due_uses_wrapping_time_order() {
        let events = [Some(0xffff_fffe), Some(2)];
        assert_eq!(
            earliest_due(&events),
            Some(DueStep {
                channel: 0,
                clock: 0xffff_fffe,
            })
        );
    }

    #[test]
    fn earliest_due_uses_wrapping_time_order_in_reverse_channel_order() {
        let events = [Some(2), Some(0xffff_fffe)];
        assert_eq!(
            earliest_due(&events),
            Some(DueStep {
                channel: 1,
                clock: 0xffff_fffe,
            })
        );
    }

    #[test]
    fn irq_budget_caps_events() {
        let mut budget = StepIrqBudget::new();
        for _ in 0..MAX_STEP_EVENTS_PER_IRQ {
            assert!(budget.take());
        }
        assert!(!budget.take());
    }

    #[test]
    fn default_irq_budget_caps_events() {
        let mut budget = StepIrqBudget::default();
        for _ in 0..MAX_STEP_EVENTS_PER_IRQ {
            assert!(budget.take());
        }
        assert!(!budget.take());
    }
}
