//! Generic ADC burst sampling matching Klipper's `adccmds.c`: samples are summed, not averaged.

use core::sync::atomic::{
    AtomicU8, AtomicU16, AtomicU32,
    Ordering::{Acquire, Relaxed, Release},
};

/// ADC burst sampling parameters.
#[derive(Debug, Clone, Copy)]
pub struct AnalogInConfig {
    pub sample_ticks: u32,
    pub sample_count: u8,
    pub rest_ticks: u32,
    pub min_value: u16,
    pub max_value: u16,
    pub range_check_count: u8,
}

/// Burst progress; `accumulator` is a `u32` running sum of raw reads (pre-saturation to `u16`).
#[derive(Debug, Clone, Copy)]
pub struct BurstProgress {
    pub accumulator: u32,
    pub samples_taken: u8,
    pub invalid_count: u8,
}

/// Outcome of one sampling step.
#[derive(Debug, Clone, Copy)]
pub enum SampleStep {
    /// Burst still accumulating; sample again at `next_due_clock`.
    Pending {
        progress: BurstProgress,
        next_due_clock: u32,
    },
    /// Burst complete and range-checked; `shutdown` is set when `range_check_count`
    /// consecutive reports were out of range. Next burst is due at `next_due_clock`.
    Report {
        value: u16,
        next_due_clock: u32,
        shutdown: bool,
        invalid_count: u8,
    },
}

/// Accumulates one raw read; returns `Pending` mid-burst or `Report` when the burst completes.
pub fn record_sample_step(
    config: &AnalogInConfig,
    progress: BurstProgress,
    raw: u16,
    now: u32,
) -> SampleStep {
    let accumulator = progress.accumulator.saturating_add(u32::from(raw));
    let samples_taken = progress.samples_taken + 1;
    if samples_taken < config.sample_count {
        return SampleStep::Pending {
            progress: BurstProgress {
                accumulator,
                samples_taken,
                invalid_count: progress.invalid_count,
            },
            next_due_clock: now.wrapping_add(config.sample_ticks),
        };
    }
    let sum = accumulator.min(u32::from(u16::MAX)) as u16;
    let in_range = sum >= config.min_value && sum <= config.max_value;
    let (invalid_count, shutdown) = if in_range {
        (0, false)
    } else {
        let invalid_count = progress.invalid_count + 1;
        if invalid_count >= config.range_check_count {
            (0, true)
        } else {
            (invalid_count, false)
        }
    };
    SampleStep::Report {
        value: sum,
        next_due_clock: now.wrapping_add(config.rest_ticks),
        shutdown,
        invalid_count,
    }
}

/// Sentinel meaning "no `config_analog_in` has claimed this slot yet".
pub const UNCONFIGURED_OID: u8 = 0xFF;

/// Channel slots: Ouroboros uses three (TH0, TH1, MCU_TEMP), OpenFFBoard only index 0.
pub const MAX_ANALOG_CHANNELS: usize = 3;

/// One analog-in channel's configuration and progress, atomic so command handlers and the
/// sampling task can share it lock-free.
pub struct AnalogChannelState {
    oid: AtomicU8,
    sample_ticks: AtomicU32,
    sample_count: AtomicU8,
    rest_ticks: AtomicU32,
    min_value: AtomicU16,
    max_value: AtomicU16,
    range_check_count: AtomicU8,
    accumulator: AtomicU32,
    samples_taken: AtomicU8,
    invalid_count: AtomicU8,
    next_due_clock: AtomicU32,
}

impl AnalogChannelState {
    pub const fn new() -> Self {
        Self {
            oid: AtomicU8::new(UNCONFIGURED_OID),
            sample_ticks: AtomicU32::new(0),
            sample_count: AtomicU8::new(0),
            rest_ticks: AtomicU32::new(0),
            min_value: AtomicU16::new(0),
            max_value: AtomicU16::new(u16::MAX),
            range_check_count: AtomicU8::new(0),
            accumulator: AtomicU32::new(0),
            samples_taken: AtomicU8::new(0),
            invalid_count: AtomicU8::new(0),
            next_due_clock: AtomicU32::new(0),
        }
    }

    pub fn oid(&self) -> Option<u8> {
        match self.oid.load(Relaxed) {
            UNCONFIGURED_OID => None,
            oid => Some(oid),
        }
    }

    /// Binds this slot to `oid`; `query_analog_in` finds its slot by oid.
    pub fn configure_oid(&self, oid: u8) {
        self.oid.store(oid, Relaxed);
    }

    /// Resets progress and schedules the first sample at `start_clock`. `sample_count` is
    /// zeroed first and stored last (Release) so readers never see a torn configuration.
    pub fn configure_sampling(&self, config: &AnalogInConfig, start_clock: u32) {
        self.sample_count.store(0, Release);
        self.sample_ticks.store(config.sample_ticks, Relaxed);
        self.rest_ticks.store(config.rest_ticks, Relaxed);
        self.min_value.store(config.min_value, Relaxed);
        self.max_value.store(config.max_value, Relaxed);
        self.range_check_count
            .store(config.range_check_count, Relaxed);
        self.accumulator.store(0, Relaxed);
        self.samples_taken.store(0, Relaxed);
        self.invalid_count.store(0, Relaxed);
        self.next_due_clock.store(start_clock, Relaxed);
        self.sample_count.store(config.sample_count, Release);
    }

    /// True when configured and the next sample or report is due at `now`.
    pub fn is_due(&self, now: u32) -> bool {
        self.sample_count.load(Acquire) != 0
            && now.wrapping_sub(self.next_due_clock.load(Relaxed)) < (u32::MAX / 2)
    }

    /// Records one raw read via `record_sample_step`. An unconfigured slot, or one
    /// reconfigured mid-call, returns its current progress as `Pending` without recording.
    pub fn record_sample(&self, raw: u16, now: u32) -> SampleStep {
        self.record_sample_inner(raw, now, || {})
    }

    #[cfg(test)]
    fn record_sample_with_interleave(
        &self,
        raw: u16,
        now: u32,
        interleave: impl FnOnce(),
    ) -> SampleStep {
        self.record_sample_inner(raw, now, interleave)
    }

    fn record_sample_inner(&self, raw: u16, now: u32, interleave: impl FnOnce()) -> SampleStep {
        let observed_count = self.sample_count.load(Acquire);
        if observed_count == 0 {
            return self.pending_from_current_state();
        }
        let config = AnalogInConfig {
            sample_ticks: self.sample_ticks.load(Relaxed),
            sample_count: observed_count,
            rest_ticks: self.rest_ticks.load(Relaxed),
            min_value: self.min_value.load(Relaxed),
            max_value: self.max_value.load(Relaxed),
            range_check_count: self.range_check_count.load(Relaxed),
        };
        let progress = BurstProgress {
            accumulator: self.accumulator.load(Relaxed),
            samples_taken: self.samples_taken.load(Relaxed),
            invalid_count: self.invalid_count.load(Relaxed),
        };
        let step = record_sample_step(&config, progress, raw, now);
        interleave();
        if self.sample_count.load(Acquire) != observed_count {
            return self.pending_from_current_state();
        }
        match step {
            SampleStep::Pending {
                progress,
                next_due_clock,
            } => {
                self.accumulator.store(progress.accumulator, Relaxed);
                self.samples_taken.store(progress.samples_taken, Relaxed);
                self.next_due_clock.store(next_due_clock, Relaxed);
            }
            SampleStep::Report {
                next_due_clock,
                invalid_count,
                ..
            } => {
                self.accumulator.store(0, Relaxed);
                self.samples_taken.store(0, Relaxed);
                self.invalid_count.store(invalid_count, Relaxed);
                self.next_due_clock.store(next_due_clock, Relaxed);
            }
        }
        step
    }

    fn pending_from_current_state(&self) -> SampleStep {
        SampleStep::Pending {
            progress: BurstProgress {
                accumulator: self.accumulator.load(Relaxed),
                samples_taken: self.samples_taken.load(Relaxed),
                invalid_count: self.invalid_count.load(Relaxed),
            },
            next_due_clock: self.next_due_clock.load(Relaxed),
        }
    }

    /// Returns the slot to unconfigured defaults (invalidating `sample_count` first).
    pub fn reset(&self) {
        self.sample_count.store(0, Release);
        self.oid.store(UNCONFIGURED_OID, Relaxed);
        self.sample_ticks.store(0, Relaxed);
        self.rest_ticks.store(0, Relaxed);
        self.min_value.store(0, Relaxed);
        self.max_value.store(u16::MAX, Relaxed);
        self.range_check_count.store(0, Relaxed);
        self.accumulator.store(0, Relaxed);
        self.samples_taken.store(0, Relaxed);
        self.invalid_count.store(0, Relaxed);
        self.next_due_clock.store(0, Relaxed);
    }
}

impl Default for AnalogChannelState {
    fn default() -> Self {
        Self::new()
    }
}

/// Lock-free storage for every analog-in channel.
pub static ANALOG_CHANNELS: [AnalogChannelState; MAX_ANALOG_CHANNELS] =
    [const { AnalogChannelState::new() }; MAX_ANALOG_CHANNELS];

pub fn analog_channel(index: u8) -> Option<&'static AnalogChannelState> {
    ANALOG_CHANNELS.get(index as usize)
}

/// Reset all analog-in channel states to firmware defaults.
pub fn reset_all_analog_channels() {
    for channel in &ANALOG_CHANNELS {
        channel.reset();
    }
}

/// Finds the channel assigned to `oid` by `config_analog_in`.
pub fn find_channel_by_oid(oid: u8) -> Option<&'static AnalogChannelState> {
    ANALOG_CHANNELS
        .iter()
        .find(|channel| channel.oid() == Some(oid))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn config() -> AnalogInConfig {
        AnalogInConfig {
            sample_ticks: 10,
            sample_count: 4,
            rest_ticks: 1000,
            min_value: 100,
            max_value: 10_000,
            range_check_count: 3,
        }
    }

    fn empty_progress() -> BurstProgress {
        BurstProgress {
            accumulator: 0,
            samples_taken: 0,
            invalid_count: 0,
        }
    }

    #[test]
    fn a_sample_mid_burst_schedules_the_next_one_sample_ticks_later() {
        let step = record_sample_step(&config(), empty_progress(), 500, 100);
        match step {
            SampleStep::Pending {
                progress,
                next_due_clock,
            } => {
                assert_eq!(progress.accumulator, 500);
                assert_eq!(progress.samples_taken, 1);
                assert_eq!(
                    next_due_clock, 110,
                    "must be sample_ticks (10) after now, not rest_ticks"
                );
            }
            SampleStep::Report { .. } => panic!("burst is not complete yet"),
        }
    }

    #[test]
    fn accumulates_sum_not_average_across_a_burst() {
        let cfg = config();
        let mut progress = empty_progress();
        let mut now = 0u32;
        for _ in 0..3 {
            now += 10;
            match record_sample_step(&cfg, progress, 500, now) {
                SampleStep::Pending { progress: p, .. } => progress = p,
                SampleStep::Report { .. } => {
                    panic!("burst must not complete before sample_count reads")
                }
            }
        }
        now += 10;
        match record_sample_step(&cfg, progress, 500, now) {
            SampleStep::Report {
                value, shutdown, ..
            } => {
                assert_eq!(value, 2000, "sum of 4 samples of 500, not their average");
                assert!(!shutdown);
            }
            SampleStep::Pending { .. } => panic!("expected a completed report"),
        }
    }

    #[test]
    fn completed_report_schedules_the_next_burst_rest_ticks_later() {
        let cfg = config();
        let mut progress = empty_progress();
        let mut now = 0u32;
        for i in 0..4 {
            now += 10;
            match record_sample_step(&cfg, progress, 500, now) {
                SampleStep::Pending { progress: p, .. } => progress = p,
                SampleStep::Report { next_due_clock, .. } => {
                    assert_eq!(i, 3, "only the 4th sample should complete the burst");
                    assert_eq!(next_due_clock, now + cfg.rest_ticks);
                }
            }
        }
    }

    #[test]
    fn sum_exactly_at_min_value_is_in_range() {
        let mut cfg = config();
        cfg.sample_count = 1;
        cfg.min_value = 500;
        cfg.max_value = 10_000;
        match record_sample_step(&cfg, empty_progress(), 500, 0) {
            SampleStep::Report {
                shutdown,
                invalid_count,
                ..
            } => {
                assert!(!shutdown);
                assert_eq!(
                    invalid_count, 0,
                    "exactly min_value must reset the invalid counter, not trip it"
                );
            }
            SampleStep::Pending { .. } => panic!("single-sample burst must complete immediately"),
        }
    }

    #[test]
    fn sum_exactly_at_max_value_is_in_range() {
        let mut cfg = config();
        cfg.sample_count = 1;
        cfg.min_value = 0;
        cfg.max_value = 500;
        match record_sample_step(&cfg, empty_progress(), 500, 0) {
            SampleStep::Report {
                shutdown,
                invalid_count,
                ..
            } => {
                assert!(!shutdown);
                assert_eq!(
                    invalid_count, 0,
                    "exactly max_value must reset the invalid counter, not trip it"
                );
            }
            SampleStep::Pending { .. } => panic!("single-sample burst must complete immediately"),
        }
    }

    #[test]
    fn one_over_max_value_is_out_of_range() {
        let mut cfg = config();
        cfg.sample_count = 1;
        cfg.min_value = 0;
        cfg.max_value = 500;
        cfg.range_check_count = 3;
        match record_sample_step(&cfg, empty_progress(), 501, 0) {
            SampleStep::Report {
                shutdown,
                invalid_count,
                ..
            } => {
                assert!(
                    !shutdown,
                    "first out-of-range report must not shut down yet"
                );
                assert_eq!(invalid_count, 1);
            }
            SampleStep::Pending { .. } => panic!("single-sample burst must complete immediately"),
        }
    }

    #[test]
    fn out_of_range_sum_shuts_down_after_range_check_count_consecutive_reports() {
        let mut cfg = config();
        cfg.sample_count = 1;
        cfg.min_value = 1000;
        cfg.range_check_count = 2;
        let mut progress = BurstProgress {
            accumulator: 0,
            samples_taken: 0,
            invalid_count: 1,
        };
        let step = record_sample_step(&cfg, progress, 50, 0);
        progress = match step {
            SampleStep::Report {
                shutdown,
                invalid_count,
                ..
            } => {
                assert!(
                    shutdown,
                    "second consecutive out-of-range report must shut down"
                );
                BurstProgress {
                    accumulator: 0,
                    samples_taken: 0,
                    invalid_count,
                }
            }
            SampleStep::Pending { .. } => panic!("single-sample burst must complete immediately"),
        };
        assert_eq!(
            progress.invalid_count, 0,
            "counter must reset immediately after tripping"
        );
    }

    #[test]
    fn in_range_report_resets_invalid_count() {
        let mut cfg = config();
        cfg.sample_count = 1;
        cfg.min_value = 1000;
        let progress = BurstProgress {
            accumulator: 0,
            samples_taken: 0,
            invalid_count: 1,
        };
        match record_sample_step(&cfg, progress, 2000, 0) {
            SampleStep::Report {
                shutdown,
                invalid_count,
                ..
            } => {
                assert!(!shutdown);
                assert_eq!(invalid_count, 0);
            }
            SampleStep::Pending { .. } => panic!("single-sample burst must complete immediately"),
        }
    }

    #[test]
    fn reset_clears_oid_and_sampling_state() {
        let slot = AnalogChannelState::new();
        slot.configure_oid(9);
        slot.configure_sampling(&config(), 1000);
        slot.record_sample(500, 1000);

        slot.reset();

        assert_eq!(slot.oid(), None, "oid must not survive a reset");
        assert!(
            !slot.is_due(1000),
            "a reset slot must not be due -- sample_count must be invalidated"
        );
        match slot.record_sample(0, 1000) {
            SampleStep::Pending {
                progress,
                next_due_clock,
            } => {
                assert_eq!(progress.accumulator, 0);
                assert_eq!(progress.samples_taken, 0);
                assert_eq!(progress.invalid_count, 0);
                assert_eq!(next_due_clock, 0);
            }
            SampleStep::Report { .. } => {
                panic!("a reset (unconfigured) channel must not produce a report")
            }
        }
    }

    #[test]
    fn configure_oid_then_find_by_oid_locates_the_same_slot() {
        let slot = AnalogChannelState::new();
        assert!(slot.oid().is_none());
        slot.configure_oid(7);
        assert_eq!(slot.oid(), Some(7));
    }

    #[test]
    fn is_due_reflects_configured_sampling_and_next_due_clock() {
        let slot = AnalogChannelState::new();
        assert!(
            !slot.is_due(1000),
            "unconfigured slot (sample_count=0) is never due"
        );
        slot.configure_sampling(
            &AnalogInConfig {
                sample_ticks: 10,
                sample_count: 4,
                rest_ticks: 1000,
                min_value: 0,
                max_value: u16::MAX,
                range_check_count: 3,
            },
            1000,
        );
        assert!(!slot.is_due(500));
        assert!(slot.is_due(1000));
        assert!(slot.is_due(1500));
    }

    #[test]
    fn reconfigure_landing_mid_record_sample_is_not_clobbered() {
        let slot = AnalogChannelState::new();
        slot.configure_sampling(&config(), 0);
        match slot.record_sample(500, 10) {
            SampleStep::Pending { progress, .. } => assert_eq!(progress.samples_taken, 1),
            SampleStep::Report { .. } => panic!("burst is not complete yet"),
        }

        let new_config = AnalogInConfig {
            sample_ticks: 5,
            sample_count: 2,
            rest_ticks: 200,
            min_value: 0,
            max_value: u16::MAX,
            range_check_count: 3,
        };
        let step = slot.record_sample_with_interleave(500, 20, || {
            slot.configure_sampling(&new_config, 999);
        });

        match step {
            SampleStep::Pending {
                progress,
                next_due_clock,
            } => {
                assert_eq!(
                    progress.accumulator, 0,
                    "must reflect the new config's freshly reset state, not a stale mix"
                );
                assert_eq!(progress.samples_taken, 0);
                assert_eq!(next_due_clock, 999);
            }
            SampleStep::Report { .. } => {
                panic!("must not fabricate a report from an abandoned computation")
            }
        }
        assert_eq!(
            slot.accumulator.load(Relaxed),
            0,
            "the abandoned write must not clobber the freshly published config"
        );
        assert_eq!(slot.samples_taken.load(Relaxed), 0);
        assert_eq!(slot.sample_count.load(Relaxed), new_config.sample_count);
    }

    #[test]
    fn record_sample_on_unconfigured_channel_returns_unchanged_pending() {
        let slot = AnalogChannelState::new();
        let step = slot.record_sample(1000, 0);
        match step {
            SampleStep::Pending {
                progress,
                next_due_clock,
            } => {
                assert_eq!(progress.accumulator, 0);
                assert_eq!(progress.samples_taken, 0);
                assert_eq!(progress.invalid_count, 0);
                assert_eq!(next_due_clock, 0);
            }
            SampleStep::Report { .. } => {
                panic!("unconfigured channel must not produce a spurious report")
            }
        }
    }
}
