//! Channel-indexed stepper runtime helpers for board timer dispatch.

use foci_klipper::stepper::StepMove;

use crate::step_pulse::{StepPulseAdmissionError, StepPulseTiming};

/// Maximum step/unstep events a timer IRQ may process before yielding.
pub const MAX_STEP_EVENTS_PER_IRQ: usize = 8;

/// Admit a complete move segment before any queue insertion.
pub fn admit_step_move(
    timing: &StepPulseTiming,
    move_: &StepMove,
    dir_changed: bool,
) -> Result<(), StepPulseAdmissionError> {
    timing.admit(move_.minimum_interval(), dir_changed)
}

/// A selected step event deadline.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct DueStep {
    /// Board-local channel index.
    pub channel: u8,
    /// Due clock for the selected event.
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
    fn admission_uses_the_segment_minimum_interval() {
        let timing = crate::step_pulse::StepPulseTiming::new(84_000_000, 24, 0).unwrap();
        let move_ = foci_klipper::stepper::StepMove::new(20, 3, -3);

        assert_eq!(
            admit_step_move(&timing, &move_, false),
            Err(crate::step_pulse::StepPulseAdmissionError {
                minimum_interval: 14,
                required_interval: 16,
            })
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
