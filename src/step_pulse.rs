//! Board-neutral physical STEP waveform timing.

const MAX_STEP_RISING_EDGE_HZ: u32 = 12_500_000;
const HALF_PERIOD_DENOMINATOR_HZ: u32 = 2 * MAX_STEP_RISING_EDGE_HZ;

/// Errors returned while deriving a board's immutable pulse-timing budget.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum StepPulseTimingError {
    /// The step-timer frequency was zero.
    ZeroTimerFrequency,
    /// A derived waveform timing value could not be represented in timer ticks.
    BudgetOverflow,
}

/// Immutable board timing for one atomic zero-, one-, or two-pulse train.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct StepPulseTiming {
    step_timer_hz: u32,
    half_period_ticks: u32,
    dir_setup_ticks: u32,
    waveform_worst_case_ticks: u32,
}

impl StepPulseTiming {
    /// Derive immutable pulse spacing and waveform costs.
    ///
    /// # Errors
    ///
    /// Returns [`StepPulseTimingError::ZeroTimerFrequency`] for a zero timer frequency and
    /// [`StepPulseTimingError::BudgetOverflow`] when a derived timing value cannot be represented
    /// in `u32` ticks.
    pub fn new(step_timer_hz: u32) -> Result<Self, StepPulseTimingError> {
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

        Ok(Self {
            step_timer_hz,
            half_period_ticks,
            dir_setup_ticks,
            waveform_worst_case_ticks,
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

    /// Waveform-only two-pulse cost including DIR setup.
    pub const fn waveform_worst_case_ticks(&self) -> u32 {
        self.waveform_worst_case_ticks
    }

    /// One millisecond of actual scheduler lateness in step-timer ticks.
    pub const fn fatal_lateness_ticks(&self) -> u32 {
        self.step_timer_hz / 1_000
    }
}
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zero_timer_frequency_is_rejected() {
        assert_eq!(
            StepPulseTiming::new(0),
            Err(StepPulseTimingError::ZeroTimerFrequency)
        );
    }

    #[test]
    fn openffboard_waveform_and_fatal_lateness_are_exact() {
        let timing = StepPulseTiming::new(84_000_000).unwrap();
        assert_eq!(timing.half_period_ticks(), 4);
        assert_eq!(timing.dir_setup_ticks(), 8);
        assert_eq!(timing.waveform_worst_case_ticks(), 24);
        assert_eq!(timing.fatal_lateness_ticks(), 84_000);
    }

    #[test]
    fn ouroboros_waveform_and_fatal_lateness_are_exact() {
        let timing = StepPulseTiming::new(260_000_000).unwrap();
        assert_eq!(timing.half_period_ticks(), 11);
        assert_eq!(timing.dir_setup_ticks(), 22);
        assert_eq!(timing.waveform_worst_case_ticks(), 66);
        assert_eq!(timing.fatal_lateness_ticks(), 260_000);
    }
}
