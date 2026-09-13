//! Board-agnostic generic ADC sampling: a pure state-transition function
//! (`record_sample_step`) plus a lock-free atomics-backed slot type
//! (`AnalogChannelState`) that wraps it for production use. Mirrors
//! Klipper's reference C firmware (`src/adccmds.c`) exactly -- a burst of
//! `sample_count` raw reads is summed (never averaged), individual
//! samples within a burst are spaced `sample_ticks` apart (not the
//! caller's poll interval), and a consecutive-invalid-report counter
//! trips a caller-visible shutdown signal at `range_check_count`,
//! resetting on any in-range report -- see
//! `docs/specs/2026-09-09-foci-367-thermistor-mcu-temp-support.md`.

use core::sync::atomic::{
    AtomicU8, AtomicU16, AtomicU32,
    Ordering::{Acquire, Relaxed, Release},
};

/// Configuration for generic ADC burst sampling: sample spacing, count, rest
/// interval, and value-range check parameters.
#[derive(Debug, Clone, Copy)]
pub struct AnalogInConfig {
    pub sample_ticks: u32,
    pub sample_count: u8,
    pub rest_ticks: u32,
    pub min_value: u16,
    pub max_value: u16,
    pub range_check_count: u8,
}

/// Accumulating progress state during an active ADC burst: raw sum of
/// samples, count taken so far, and consecutive out-of-range report counter.
/// The accumulator is a running `u32` sum of raw ADC reads (before saturation
/// to `u16` and range checking), which avoids overflow while accumulating up
/// to `sample_count` reads even when each is near `u16::MAX`.
#[derive(Debug, Clone, Copy)]
pub struct BurstProgress {
    pub accumulator: u32,
    pub samples_taken: u8,
    pub invalid_count: u8,
}

/// Outcome of one ADC sampling step: either advance within a burst, or deliver
/// a complete report.
#[derive(Debug, Clone, Copy)]
pub enum SampleStep {
    /// Burst is still accumulating. The caller should schedule the next sample
    /// at `next_due_clock`.
    Pending {
        progress: BurstProgress,
        next_due_clock: u32,
    },
    /// Burst is complete; the sum has been range-checked. The caller should
    /// report the value and may trigger shutdown if the shutdown flag is set.
    /// The sampling task resets and schedules the next burst at `next_due_clock`.
    Report {
        value: u16,
        next_due_clock: u32,
        shutdown: bool,
        invalid_count: u8,
    },
}

/// Execute one ADC sampling step on pure state: accumulate a raw read and
/// either return Pending (within-burst) or Report (burst complete). Mirrors
/// Klipper's `analog_in_event` exactly: samples are summed (not averaged),
/// bursts are spaced `rest_ticks` apart, individual samples within a burst
/// are spaced `sample_ticks` apart, and consecutive out-of-range reports
/// trigger shutdown at `range_check_count`, resetting immediately after.
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

/// Ouroboros's channel count (TH0, TH1, MCU_TEMP); OpenFFBoard only ever
/// uses index 0. Sized to the larger board, exactly like `state/tmc.rs`'s
/// `MAX_TMC_CHANNELS` covers the larger board's channel count.
pub const MAX_ANALOG_CHANNELS: usize = 3;

/// One board-local analog-in channel's live configuration and sampling
/// progress. Every field is an atomic (see `state/tmc.rs`'s
/// `TmcChannelState` for the established idiom this follows) so ankyra
/// command handlers (running in the USB/serial dispatch context) and the
/// RTIC sampling task (an independent execution context) can both reach
/// the same slot through a `&'static` accessor with no RTIC-priority
/// lock.
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

    /// Called by `config_analog_in` to remember which oid this slot
    /// belongs to -- `query_analog_in` does not repeat `pin` on the wire,
    /// so it must find its slot by oid (`find_channel_by_oid`) instead.
    pub fn configure_oid(&self, oid: u8) {
        self.oid.store(oid, Relaxed);
    }

    /// Called by `query_analog_in` once its owning slot is found by oid.
    /// Resets all sampling progress and schedules the first sample at
    /// `start_clock`.
    ///
    /// Uses a release/acquire gate (sample_count) to prevent concurrent
    /// `is_due`/`record_sample` calls from observing a torn read of the
    /// configuration group. This follows `state/tmc.rs`'s own
    /// `configure_motion_scale` idiom: sample_count=0 invalidates before the
    /// write group, then the real value validates after. Readers check
    /// sample_count with Acquire to synchronize the entire group.
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

    /// Whether this slot is configured (`query_analog_in` has run) and its
    /// next scheduled sample or report is due at `now`. The Acquire load on
    /// `sample_count` ensures a concurrent `configure_sampling` write is
    /// never observed partially.
    pub fn is_due(&self, now: u32) -> bool {
        self.sample_count.load(Acquire) != 0
            && now.wrapping_sub(self.next_due_clock.load(Relaxed)) < (u32::MAX / 2)
    }

    /// Record one raw ADC read for this slot via the pure
    /// `record_sample_step`, persisting the resulting state. The Acquire load
    /// on `sample_count` ensures the configuration snapshot is not torn by a
    /// concurrent `configure_sampling` call. If the channel is unconfigured
    /// (sample_count == 0, the invalidation sentinel), returns the current
    /// progress unchanged to signal "try again shortly" without recording.
    pub fn record_sample(&self, raw: u16, now: u32) -> SampleStep {
        let sample_count = self.sample_count.load(Acquire);
        if sample_count == 0 {
            return SampleStep::Pending {
                progress: BurstProgress {
                    accumulator: self.accumulator.load(Relaxed),
                    samples_taken: self.samples_taken.load(Relaxed),
                    invalid_count: self.invalid_count.load(Relaxed),
                },
                next_due_clock: self.next_due_clock.load(Relaxed),
            };
        }
        let config = AnalogInConfig {
            sample_ticks: self.sample_ticks.load(Relaxed),
            sample_count,
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
}

impl Default for AnalogChannelState {
    fn default() -> Self {
        Self::new()
    }
}

/// Fixed-size, lock-free storage for every analog-in channel across both
/// boards -- exactly `state/tmc.rs`'s `TMC_CHANNELS` idiom.
pub static ANALOG_CHANNELS: [AnalogChannelState; MAX_ANALOG_CHANNELS] =
    [const { AnalogChannelState::new() }; MAX_ANALOG_CHANNELS];

pub fn analog_channel(index: u8) -> Option<&'static AnalogChannelState> {
    ANALOG_CHANNELS.get(index as usize)
}

/// Find the channel `config_analog_in` previously assigned this `oid` to.
/// A plain linear scan over at most 3 slots -- this codebase has no
/// general oid-to-arbitrary-data map anywhere (see `state/trsync.rs`'s
/// equivalent small-N bounded-index idiom), and a hash map would be both
/// unnecessary and `alloc`-requiring.
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
        }; // one prior out-of-range report already recorded
        let step = record_sample_step(&cfg, progress, 50, 0); // sum=50 < min_value 1000
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
