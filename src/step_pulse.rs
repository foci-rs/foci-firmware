//! Board-neutral physical STEP waveform timing and logical-move admission.

const MAX_STEP_RISING_EDGE_HZ: u32 = 12_500_000;
const HALF_PERIOD_DENOMINATOR_HZ: u32 = 2 * MAX_STEP_RISING_EDGE_HZ;

/// Errors returned while deriving a board's immutable pulse-timing budget.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum StepPulseTimingError {
    /// The step-timer frequency was zero.
    ZeroTimerFrequency,
    /// The measured WCET and margin could not be represented in timer ticks.
    BudgetOverflow,
}

/// Details for a logical move that cannot contain its physical pulse train.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct StepPulseAdmissionError {
    /// Smallest logical interval requested by the move segment.
    pub minimum_interval: u32,
    /// Board timing budget required for this segment.
    pub required_interval: u32,
}

/// Immutable board timing for one atomic zero-, one-, or two-pulse train.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct StepPulseTiming {
    step_timer_hz: u32,
    half_period_ticks: u32,
    dir_setup_ticks: u32,
    measured_wcet_ticks: Option<u32>,
    margin_ticks: u32,
    worst_case_budget_ticks: u32,
}

impl StepPulseTiming {
    /// Derive pulse spacing and the board's admission budget.
    ///
    /// All inputs and derived durations use step-timer ticks. A nonzero
    /// `measured_wcet_ticks` is the measured or provisional total worst-case
    /// P3 handler duration for a direction change and two physical pulses. It
    /// includes GPIO work, DIR setup, and every STEP high/low wait. Admission
    /// therefore uses `max(waveform_worst_case, measured_wcet_ticks) + margin`;
    /// adding the waveform to the measured total would count its waits twice.
    /// Zero selects the waveform-only provisional bound.
    ///
    /// # Errors
    ///
    /// Returns [`StepPulseTimingError::ZeroTimerFrequency`] for a zero timer
    /// frequency and [`StepPulseTimingError::BudgetOverflow`] when a derived
    /// or configured timing value cannot be represented in `u32` ticks.
    pub fn new(
        step_timer_hz: u32,
        measured_wcet_ticks: u32,
        margin_ticks: u32,
    ) -> Result<Self, StepPulseTimingError> {
        if step_timer_hz == 0 {
            return Err(StepPulseTimingError::ZeroTimerFrequency);
        }
        let half_period_ticks = step_timer_hz.div_ceil(HALF_PERIOD_DENOMINATOR_HZ);
        let dir_setup_ticks = half_period_ticks
            .checked_mul(2)
            .ok_or(StepPulseTimingError::BudgetOverflow)?;
        let two_pulse_ticks = half_period_ticks
            .checked_mul(4)
            .ok_or(StepPulseTimingError::BudgetOverflow)?;
        let waveform_worst_case_ticks = dir_setup_ticks
            .checked_add(two_pulse_ticks)
            .ok_or(StepPulseTimingError::BudgetOverflow)?;
        let measured_wcet = (measured_wcet_ticks != 0).then_some(measured_wcet_ticks);
        let selected_wcet = measured_wcet_ticks.max(waveform_worst_case_ticks);
        let worst_case_budget_ticks = selected_wcet
            .checked_add(margin_ticks)
            .ok_or(StepPulseTimingError::BudgetOverflow)?;

        Ok(Self {
            step_timer_hz,
            half_period_ticks,
            dir_setup_ticks,
            measured_wcet_ticks: measured_wcet,
            margin_ticks,
            worst_case_budget_ticks,
        })
    }

    /// Timer ticks in one physical STEP high or low phase.
    pub const fn half_period_ticks(&self) -> u32 {
        self.half_period_ticks
    }

    /// Project-owned DIR-to-STEP setup time in timer ticks.
    pub const fn dir_setup_ticks(&self) -> u32 {
        self.dir_setup_ticks
    }

    /// Waveform-only ticks for zero, one, or two complete pulses.
    pub const fn pulse_train_ticks(&self, pulse_count: u8) -> Option<u32> {
        match pulse_count {
            0 => Some(0),
            1 => Some(self.half_period_ticks * 2),
            2 => Some(self.half_period_ticks * 4),
            _ => None,
        }
    }

    /// Waveform-only two-pulse cost including DIR setup.
    pub const fn waveform_worst_case_ticks(&self) -> u32 {
        self.dir_setup_ticks + self.half_period_ticks * 4
    }

    /// Total direction-change plus two-pulse handler WCET, or `None` while provisional.
    pub const fn measured_wcet_ticks(&self) -> Option<u32> {
        self.measured_wcet_ticks
    }

    /// Explicit board margin applied after the WCET bound.
    pub const fn margin_ticks(&self) -> u32 {
        self.margin_ticks
    }

    /// Required logical interval for a conservative two-pulse event.
    ///
    /// The measured total covers the direction-change path. When direction is
    /// unchanged, only the known DIR setup wait is removed; GPIO, pulse waits,
    /// and all other measured handler overhead remain conservatively budgeted.
    pub const fn required_interval_ticks(&self, dir_changed: bool) -> u32 {
        if dir_changed {
            self.worst_case_budget_ticks
        } else {
            self.worst_case_budget_ticks - self.dir_setup_ticks
        }
    }

    /// Waveform-only logical-event ceiling for the DIR plus two-pulse case.
    pub const fn waveform_ceiling_hz(&self) -> u32 {
        self.step_timer_hz / self.waveform_worst_case_ticks()
    }

    /// Admit a segment's minimum interval against its physical pulse work.
    pub const fn admit(
        &self,
        minimum_interval: u32,
        dir_changed: bool,
    ) -> Result<(), StepPulseAdmissionError> {
        let required_interval = self.required_interval_ticks(dir_changed);
        if minimum_interval < required_interval {
            return Err(StepPulseAdmissionError {
                minimum_interval,
                required_interval,
            });
        }
        Ok(())
    }
}
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn openffboard_nominal_waveform_budget_is_exact() {
        let timing = StepPulseTiming::new(84_000_000, 0, 0).unwrap();
        assert_eq!(timing.half_period_ticks(), 4);
        assert_eq!(timing.dir_setup_ticks(), 8);
        assert_eq!(timing.pulse_train_ticks(0), Some(0));
        assert_eq!(timing.pulse_train_ticks(1), Some(8));
        assert_eq!(timing.pulse_train_ticks(2), Some(16));
        assert_eq!(timing.waveform_worst_case_ticks(), 24);
        assert_eq!(timing.required_interval_ticks(false), 16);
        assert_eq!(timing.required_interval_ticks(true), 24);
        assert_eq!(timing.waveform_ceiling_hz(), 3_500_000);
        assert_eq!(timing.measured_wcet_ticks(), None);
    }

    #[test]
    fn ouroboros_nominal_waveform_budget_is_exact() {
        let timing = StepPulseTiming::new(260_000_000, 0, 0).unwrap();
        assert_eq!(timing.half_period_ticks(), 11);
        assert_eq!(timing.dir_setup_ticks(), 22);
        assert_eq!(timing.waveform_worst_case_ticks(), 66);
        assert_eq!(timing.required_interval_ticks(false), 44);
        assert_eq!(timing.required_interval_ticks(true), 66);
        assert_eq!(timing.waveform_ceiling_hz(), 3_939_393);
    }

    #[test]
    fn total_direction_change_wcet_and_margin_form_checked_admission_budget() {
        let timing = StepPulseTiming::new(84_000_000, 30, 6).unwrap();
        assert_eq!(timing.half_period_ticks(), 4);
        assert_eq!(timing.measured_wcet_ticks(), Some(30));
        assert_eq!(timing.margin_ticks(), 6);
        // The total measured path already includes the 8-tick DIR setup wait.
        assert_eq!(timing.required_interval_ticks(false), 28);
        assert_eq!(timing.required_interval_ticks(true), 36);
        assert_eq!(
            StepPulseTiming::new(84_000_000, u32::MAX, 1),
            Err(StepPulseTimingError::BudgetOverflow)
        );
    }

    #[test]
    fn admission_reports_requested_and_required_intervals() {
        let timing = StepPulseTiming::new(84_000_000, 0, 4).unwrap();
        assert_eq!(timing.admit(28, true), Ok(()));
        assert_eq!(
            timing.admit(27, true),
            Err(StepPulseAdmissionError {
                minimum_interval: 27,
                required_interval: 28,
            })
        );
    }
}
