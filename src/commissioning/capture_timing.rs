//! Allocation-free timing quality accounting for electrical captures.

/// Timing-quality rejection from deadline selection or capture accounting.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum TimingReject {
    /// Checked deadline or interval arithmetic overflowed.
    ArithmeticOverflow,
    /// A timestamp moved backwards or violated poll ordering.
    NonMonotonicTime,
    /// Sample-start lateness exceeded one quarter of the requested period.
    ExcessiveLateness,
    /// Complete sample service took longer than one requested period.
    ServiceOverrun,
    /// The capture missed more sample slots than its method permits.
    MissedSamples,
    /// One observation gap skipped more consecutive logical sample slots than permitted.
    ConsecutiveMisses,
    /// Time attributed to SPI exceeded the complete poll wall time.
    SpiWallExceedsPoll,
    /// A telemetry counter or duration saturated its storage type.
    TelemetryOverflow,
    /// A timestamp offset cannot be represented by its capture container.
    TimestampRange,
}

/// Method-specific missed-slot acceptance limits.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct TimingEnvelope {
    max_missed_samples: Option<u16>,
    max_consecutive_misses: u8,
}

impl TimingEnvelope {
    /// Resistance permits one total missed slot and a chain of at most one.
    pub const RESISTANCE: Self = Self {
        max_missed_samples: Some(1),
        max_consecutive_misses: 1,
    };

    /// Inductance defers total coverage to the estimator, with chains capped at two.
    pub const INDUCTANCE: Self = Self {
        max_missed_samples: None,
        max_consecutive_misses: 2,
    };

    #[cfg(test)]
    pub const DELAY: Self = Self {
        max_missed_samples: Some(0),
        max_consecutive_misses: 0,
    };
}

/// The one sample slot selected for a delayed scheduler wake.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct SampleSlot {
    /// Number of elapsed slots skipped before this observation.
    pub missed: u32,
    /// Absolute deadline associated with the one sample to take now.
    pub sample_deadline_us: u64,
    /// Absolute deadline for the following logical sample.
    pub next_deadline_us: u64,
}

/// Pre-SPI disposition for one selected sample slot.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) enum SampleStart {
    /// The selected slot is inside the start-lateness envelope.
    Sample,
    /// The selected slot was too late and the caller must wait for this deadline.
    Skip { next_deadline_us: u64 },
}

/// Select one current sample slot without performing catch-up reads.
///
/// # Errors
///
/// Returns [`TimingReject::NonMonotonicTime`] when `now_us` precedes the
/// current deadline, or [`TimingReject::ArithmeticOverflow`] when checked
/// period/deadline arithmetic cannot be represented.
pub fn sample_slot(due_us: u64, period_us: u16, now_us: u64) -> Result<SampleSlot, TimingReject> {
    if now_us < due_us {
        return Err(TimingReject::NonMonotonicTime);
    }
    let period_us = u64::from(period_us);
    if period_us == 0 {
        return Err(TimingReject::ArithmeticOverflow);
    }
    let missed = (now_us - due_us) / period_us;
    let sample_deadline_us = missed
        .checked_mul(period_us)
        .and_then(|elapsed| due_us.checked_add(elapsed))
        .ok_or(TimingReject::ArithmeticOverflow)?;
    let next_deadline_us = sample_deadline_us
        .checked_add(period_us)
        .ok_or(TimingReject::ArithmeticOverflow)?;
    let missed = u32::try_from(missed).map_err(|_| TimingReject::ArithmeticOverflow)?;

    Ok(SampleSlot {
        missed,
        sample_deadline_us,
        next_deadline_us,
    })
}

/// Compact completed timing evidence retained after a capture finishes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct CaptureTimingEvidence {
    /// Requested absolute sample period in microseconds.
    pub requested_period_us: u16,
    /// Number of accepted logical samples.
    pub valid_samples: u16,
    /// Number of skipped sample slots.
    pub missed_samples: u16,
    /// Largest number of consecutive logical sample slots skipped in one observation gap.
    pub max_consecutive_misses: u8,
    /// Maximum sample-start lateness in microseconds.
    pub max_lateness_us: u32,
    /// Maximum interval between accepted observations in microseconds.
    pub max_interval_us: u32,
    /// Maximum complete poll wall time in microseconds.
    pub max_poll_wall_us: u32,
    /// Maximum SPI-await wall time within one poll in microseconds.
    pub max_spi_wall_us: u32,
}

impl CaptureTimingEvidence {
    /// Zero-filled evidence for a timing method that did not run.
    pub const EMPTY: Self = Self {
        requested_period_us: 0,
        valid_samples: 0,
        missed_samples: 0,
        max_consecutive_misses: 0,
        max_lateness_us: 0,
        max_interval_us: 0,
        max_poll_wall_us: 0,
        max_spi_wall_us: 0,
    };
}

/// Streaming timing-quality accumulator for one electrical capture.
#[derive(Debug, Clone, Copy, defmt::Format)]
pub struct CaptureTiming {
    requested_period_us: u16,
    valid_samples: u16,
    missed_samples: u16,
    consecutive_misses: u8,
    max_consecutive_misses: u8,
    max_lateness_us: u32,
    max_interval_us: u32,
    max_poll_wall_us: u32,
    max_spi_wall_us: u32,
    previous_observation_us: u64,
    overflowed: bool,
    envelope: TimingEnvelope,
}

impl CaptureTiming {
    /// Create an empty timing accumulator for `requested_period_us`.
    #[must_use]
    pub const fn new(requested_period_us: u16, envelope: TimingEnvelope) -> Self {
        Self {
            requested_period_us,
            valid_samples: 0,
            missed_samples: 0,
            consecutive_misses: 0,
            max_consecutive_misses: 0,
            max_lateness_us: 0,
            max_interval_us: 0,
            max_poll_wall_us: 0,
            max_spi_wall_us: 0,
            previous_observation_us: 0,
            overflowed: requested_period_us == 0,
            envelope,
        }
    }

    /// Classify sample-start lateness before performing any sample I/O.
    ///
    /// An excessively late selected slot is recorded as one additional miss
    /// beyond any fully elapsed slots. The caller advances to the returned
    /// absolute deadline without taking a catch-up sample.
    pub(crate) fn classify_sample_start(
        &mut self,
        due_us: u64,
        poll_entry_us: u64,
    ) -> Result<SampleStart, TimingReject> {
        let slot = sample_slot(due_us, self.requested_period_us, poll_entry_us)?;
        let lateness_us = poll_entry_us
            .checked_sub(slot.sample_deadline_us)
            .ok_or(TimingReject::NonMonotonicTime)?;
        if lateness_us <= u64::from(self.requested_period_us) / 4 {
            return Ok(SampleStart::Sample);
        }

        update_max_u32(&mut self.max_lateness_us, lateness_us, &mut self.overflowed);
        let missed = slot
            .missed
            .checked_add(1)
            .ok_or(TimingReject::TelemetryOverflow)?;
        self.record_skipped_slots(missed);
        self.check_miss_envelope()?;
        Ok(SampleStart::Skip {
            next_deadline_us: slot.next_deadline_us,
        })
    }

    /// Record one scheduled sample poll and its SPI contribution.
    ///
    /// `due_us` is the prior absolute deadline. `poll_entry_us`,
    /// `observation_us`, and `poll_completion_us` must be nondecreasing.
    ///
    /// # Errors
    ///
    /// Returns a [`TimingReject`] for invalid chronology, excessive lateness,
    /// invalid slot arithmetic, or SPI time outside the poll wall interval.
    /// [`TimingReject::ServiceOverrun`] reports complete service longer than
    /// one requested period.
    pub fn record_sample(
        &mut self,
        due_us: u64,
        poll_entry_us: u64,
        observation_us: u64,
        poll_completion_us: u64,
        spi_wall_us: u32,
    ) -> Result<SampleSlot, TimingReject> {
        let poll_wall_us = poll_completion_us
            .checked_sub(poll_entry_us)
            .ok_or(TimingReject::NonMonotonicTime)?;
        update_max_u32(
            &mut self.max_poll_wall_us,
            poll_wall_us,
            &mut self.overflowed,
        );
        self.max_spi_wall_us = self.max_spi_wall_us.max(spi_wall_us);
        if observation_us < poll_entry_us
            || poll_completion_us < observation_us
            || (self.valid_samples != 0 && observation_us <= self.previous_observation_us)
        {
            return Err(TimingReject::NonMonotonicTime);
        }

        let slot = sample_slot(due_us, self.requested_period_us, poll_entry_us)?;
        let lateness_us = poll_entry_us
            .checked_sub(slot.sample_deadline_us)
            .ok_or(TimingReject::NonMonotonicTime)?;
        update_max_u32(&mut self.max_lateness_us, lateness_us, &mut self.overflowed);
        if u64::from(spi_wall_us) > poll_wall_us {
            return Err(TimingReject::SpiWallExceedsPoll);
        }
        if lateness_us > u64::from(self.requested_period_us) / 4 {
            return Err(TimingReject::ExcessiveLateness);
        }
        if poll_wall_us > u64::from(self.requested_period_us) {
            return Err(TimingReject::ServiceOverrun);
        }

        self.valid_samples = saturating_add_u16(self.valid_samples, 1, &mut self.overflowed);
        self.record_misses_before_accepted_sample(slot.missed);

        if self.valid_samples > 1 {
            let interval_us = observation_us
                .checked_sub(self.previous_observation_us)
                .ok_or(TimingReject::NonMonotonicTime)?;
            update_max_u32(&mut self.max_interval_us, interval_us, &mut self.overflowed);
        }
        self.previous_observation_us = observation_us;

        Ok(slot)
    }

    fn record_skipped_slots(&mut self, missed: u32) {
        self.missed_samples = saturating_add_u16(self.missed_samples, missed, &mut self.overflowed);
        let missed = match u8::try_from(missed) {
            Ok(missed) => missed,
            Err(_) => {
                self.overflowed = true;
                u8::MAX
            }
        };
        self.consecutive_misses = match self.consecutive_misses.checked_add(missed) {
            Some(consecutive) => consecutive,
            None => {
                self.overflowed = true;
                u8::MAX
            }
        };
        self.max_consecutive_misses = self.max_consecutive_misses.max(self.consecutive_misses);
    }

    fn record_misses_before_accepted_sample(&mut self, missed: u32) {
        self.missed_samples = saturating_add_u16(self.missed_samples, missed, &mut self.overflowed);
        let missed = match u8::try_from(missed) {
            Ok(missed) => missed,
            Err(_) => {
                self.overflowed = true;
                u8::MAX
            }
        };
        let consecutive_misses = match self.consecutive_misses.checked_add(missed) {
            Some(consecutive) => consecutive,
            None => {
                self.overflowed = true;
                u8::MAX
            }
        };
        self.max_consecutive_misses = self.max_consecutive_misses.max(consecutive_misses);
        self.consecutive_misses = 0;
    }

    /// Mark an external timing source overflow and retain its saturated SPI span.
    pub(crate) fn mark_source_overflow(&mut self, max_spi_wall_us: u32) {
        self.overflowed = true;
        self.max_spi_wall_us = self.max_spi_wall_us.max(max_spi_wall_us);
    }

    /// Apply the method envelope and return compact completed evidence.
    ///
    /// # Errors
    ///
    /// Returns the first applicable telemetry, missed-slot, or consecutive
    /// miss rejection.
    pub fn finish(self) -> Result<CaptureTimingEvidence, TimingReject> {
        self.check_miss_envelope()?;
        Ok(self.snapshot().0)
    }

    fn check_miss_envelope(&self) -> Result<(), TimingReject> {
        if self.overflowed {
            return Err(TimingReject::TelemetryOverflow);
        }
        if let Some(max_missed_samples) = self.envelope.max_missed_samples
            && self.missed_samples > max_missed_samples
        {
            return Err(TimingReject::MissedSamples);
        }
        if self.max_consecutive_misses > self.envelope.max_consecutive_misses {
            return Err(TimingReject::ConsecutiveMisses);
        }
        Ok(())
    }

    /// Return the latest saturating evidence and its overflow flag without applying gates.
    #[must_use]
    pub const fn snapshot(&self) -> (CaptureTimingEvidence, bool) {
        (
            CaptureTimingEvidence {
                requested_period_us: self.requested_period_us,
                valid_samples: self.valid_samples,
                missed_samples: self.missed_samples,
                max_consecutive_misses: self.max_consecutive_misses,
                max_lateness_us: self.max_lateness_us,
                max_interval_us: self.max_interval_us,
                max_poll_wall_us: self.max_poll_wall_us,
                max_spi_wall_us: self.max_spi_wall_us,
            },
            self.overflowed,
        )
    }
}

fn saturating_add_u16(value: u16, add: u32, overflowed: &mut bool) -> u16 {
    let sum = u32::from(value).saturating_add(add);
    if sum > u32::from(u16::MAX) {
        *overflowed = true;
        u16::MAX
    } else {
        sum as u16
    }
}

fn update_max_u32(target: &mut u32, value: u64, overflowed: &mut bool) {
    let value = match u32::try_from(value) {
        Ok(value) => value,
        Err(_) => {
            *overflowed = true;
            u32::MAX
        }
    };
    *target = (*target).max(value);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn delayed_wakeup_skips_slots_without_catch_up() {
        let slot = sample_slot(1_000, 160, 1_500).unwrap();

        assert_eq!(
            (slot.missed, slot.sample_deadline_us, slot.next_deadline_us,),
            (3, 1_480, 1_640),
        );
    }

    #[test]
    fn delay_rejects_any_miss() {
        let mut timing = CaptureTiming::new(160, TimingEnvelope::DELAY);
        timing
            .record_sample(1_000, 1_160, 1_170, 1_176, 16)
            .unwrap();

        assert_eq!(timing.finish(), Err(TimingReject::MissedSamples));
    }

    #[test]
    fn resistance_accepts_one_miss_but_rejects_a_second_separated_miss() {
        let mut one_miss = CaptureTiming::new(100, TimingEnvelope::RESISTANCE);
        one_miss
            .record_sample(1_000, 1_100, 1_100, 1_110, 10)
            .unwrap();

        let evidence = one_miss.finish().unwrap();
        assert_eq!(evidence.missed_samples, 1);
        assert_eq!(evidence.max_consecutive_misses, 1);

        let mut two_misses = CaptureTiming::new(100, TimingEnvelope::RESISTANCE);
        two_misses
            .record_sample(1_000, 1_100, 1_100, 1_110, 10)
            .unwrap();
        two_misses
            .record_sample(1_200, 1_300, 1_300, 1_310, 10)
            .unwrap();

        assert_eq!(two_misses.finish(), Err(TimingReject::MissedSamples));
    }

    #[test]
    fn observation_after_quarter_period_is_accepted_when_sample_start_is_on_time() {
        let mut timing = CaptureTiming::new(100, TimingEnvelope::DELAY);
        timing
            .record_sample(1_000, 1_000, 1_080, 1_090, 80)
            .unwrap();

        let evidence = timing.finish().unwrap();
        assert_eq!(evidence.max_lateness_us, 0);
    }

    #[test]
    fn sample_start_lateness_above_quarter_period_is_rejected() {
        let mut timing = CaptureTiming::new(100, TimingEnvelope::DELAY);

        assert_eq!(
            timing.record_sample(1_000, 1_026, 1_080, 1_090, 54),
            Err(TimingReject::ExcessiveLateness),
        );
        let (evidence, overflowed) = timing.snapshot();

        assert_eq!(evidence.valid_samples, 0);
        assert_eq!(evidence.max_lateness_us, 26);
        assert_eq!(evidence.max_poll_wall_us, 64);
        assert_eq!(evidence.max_spi_wall_us, 54);
        assert!(!overflowed);
    }

    #[test]
    fn sample_completion_exactly_at_next_deadline_is_accepted() {
        let mut timing = CaptureTiming::new(100, TimingEnvelope::INDUCTANCE);

        timing
            .record_sample(1_000, 1_000, 1_080, 1_100, 80)
            .unwrap();

        let evidence = timing.finish().unwrap();
        assert_eq!(evidence.valid_samples, 1);
        assert_eq!(evidence.max_poll_wall_us, 100);
        assert_eq!(evidence.max_spi_wall_us, 80);
    }

    #[test]
    fn scheduler_lateness_does_not_consume_the_service_budget() {
        let mut timing = CaptureTiming::new(100, TimingEnvelope::INDUCTANCE);

        timing
            .record_sample(1_000, 1_017, 1_090, 1_105, 73)
            .unwrap();

        let evidence = timing.finish().unwrap();
        assert_eq!(evidence.valid_samples, 1);
        assert_eq!(evidence.max_lateness_us, 17);
        assert_eq!(evidence.max_poll_wall_us, 88);
        assert_eq!(evidence.max_spi_wall_us, 73);
    }

    #[test]
    fn sample_service_over_one_period_rejects_with_service_evidence() {
        let mut timing = CaptureTiming::new(100, TimingEnvelope::INDUCTANCE);

        assert_eq!(
            timing.record_sample(1_000, 1_000, 1_080, 1_101, 80),
            Err(TimingReject::ServiceOverrun),
        );
        let (evidence, overflowed) = timing.snapshot();

        assert_eq!(evidence.valid_samples, 0);
        assert_eq!(evidence.max_lateness_us, 0);
        assert_eq!(evidence.max_poll_wall_us, 101);
        assert_eq!(evidence.max_spi_wall_us, 80);
        assert!(!overflowed);
    }

    #[test]
    fn source_overflow_sets_provenance_and_saturates_spi_span() {
        let mut timing = CaptureTiming::new(160, TimingEnvelope::DELAY);

        timing.mark_source_overflow(u32::MAX);
        let (evidence, overflowed) = timing.snapshot();

        assert!(overflowed);
        assert_eq!(evidence.max_spi_wall_us, u32::MAX);
        assert_eq!(timing.finish(), Err(TimingReject::TelemetryOverflow));
    }

    #[test]
    fn slot_arithmetic_overflow_is_rejected() {
        assert_eq!(
            sample_slot(u64::MAX - 10, 20, u64::MAX - 10),
            Err(TimingReject::ArithmeticOverflow),
        );
    }

    #[test]
    fn non_monotonic_time_is_rejected() {
        let mut timing = CaptureTiming::new(100, TimingEnvelope::INDUCTANCE);
        timing
            .record_sample(1_000, 1_000, 1_010, 1_020, 10)
            .unwrap();

        assert_eq!(
            timing.record_sample(1_100, 1_100, 1_009, 1_110, 10),
            Err(TimingReject::NonMonotonicTime),
        );
    }

    #[test]
    fn serviced_non_monotonic_sample_retains_poll_and_spi_evidence() {
        let mut timing = CaptureTiming::new(100, TimingEnvelope::INDUCTANCE);

        assert_eq!(
            timing.record_sample(1_000, 1_000, 999, 1_010, 8),
            Err(TimingReject::NonMonotonicTime),
        );
        let (evidence, overflowed) = timing.snapshot();
        assert_eq!(evidence.max_poll_wall_us, 10);
        assert_eq!(evidence.max_spi_wall_us, 8);
        assert!(!overflowed);
    }

    #[test]
    fn misses_separated_by_accepted_samples_are_not_consecutive() {
        let mut timing = CaptureTiming::new(100, TimingEnvelope::INDUCTANCE);
        timing
            .record_sample(1_000, 1_100, 1_100, 1_110, 10)
            .unwrap();
        timing
            .record_sample(1_200, 1_300, 1_300, 1_310, 10)
            .unwrap();

        let evidence = timing.finish().unwrap();
        assert_eq!(evidence.missed_samples, 2);
        assert_eq!(evidence.max_consecutive_misses, 1);
    }

    #[test]
    fn inductance_accepts_two_slots_skipped_in_one_wakeup() {
        let mut timing = CaptureTiming::new(100, TimingEnvelope::INDUCTANCE);
        timing
            .record_sample(1_000, 1_200, 1_200, 1_210, 10)
            .unwrap();

        let evidence = timing.finish().unwrap();
        assert_eq!(evidence.missed_samples, 2);
        assert_eq!(evidence.max_consecutive_misses, 2);
    }

    #[test]
    fn inductance_defers_total_miss_acceptance_to_the_coverage_gate() {
        let mut timing = CaptureTiming::new(100, TimingEnvelope::INDUCTANCE);
        timing
            .record_sample(1_000, 1_200, 1_200, 1_210, 10)
            .unwrap();
        timing
            .record_sample(1_300, 1_300, 1_310, 1_320, 10)
            .unwrap();
        for (due_us, entry_us) in [(1_400, 1_500), (1_600, 1_700), (1_800, 1_900)] {
            timing
                .record_sample(due_us, entry_us, entry_us + 10, entry_us + 20, 10)
                .unwrap();
        }

        let evidence = timing.finish().expect("isolated misses remain admissible");
        assert_eq!(evidence.missed_samples, 5);
        assert_eq!(evidence.max_consecutive_misses, 2);

        timing
            .record_sample(2_000, 2_100, 2_110, 2_120, 10)
            .unwrap();
        let evidence = timing
            .finish()
            .expect("final estimator coverage decides total misses");
        assert_eq!(evidence.missed_samples, 6);
        assert_eq!(evidence.max_consecutive_misses, 2);
    }

    #[test]
    fn inductance_rejects_three_consecutive_misses() {
        let mut timing = CaptureTiming::new(100, TimingEnvelope::INDUCTANCE);
        timing
            .record_sample(1_000, 1_300, 1_300, 1_310, 10)
            .unwrap();

        assert_eq!(timing.finish(), Err(TimingReject::ConsecutiveMisses));
    }

    #[test]
    fn spi_wall_time_cannot_exceed_poll_wall_time() {
        let mut timing = CaptureTiming::new(100, TimingEnvelope::INDUCTANCE);

        assert_eq!(
            timing.record_sample(1_000, 1_000, 1_005, 1_010, 11),
            Err(TimingReject::SpiWallExceedsPoll),
        );
        let (evidence, overflowed) = timing.snapshot();
        assert_eq!(evidence.max_poll_wall_us, 10);
        assert_eq!(evidence.max_spi_wall_us, 11);
        assert!(!overflowed);
    }

    #[test]
    fn excessive_selected_slot_lateness_skips_one_slot() {
        let mut timing = CaptureTiming::new(100, TimingEnvelope::INDUCTANCE);

        let decision = timing.classify_sample_start(1_000, 1_092).unwrap();

        assert_eq!(
            decision,
            SampleStart::Skip {
                next_deadline_us: 1_100
            }
        );
        let evidence = timing.finish().unwrap();
        assert_eq!(evidence.valid_samples, 0);
        assert_eq!(evidence.missed_samples, 1);
        assert_eq!(evidence.max_consecutive_misses, 1);
        assert_eq!(evidence.max_lateness_us, 92);
    }

    #[test]
    fn elapsed_and_selected_slot_skips_retain_two_misses() {
        let mut timing = CaptureTiming::new(100, TimingEnvelope::INDUCTANCE);

        assert_eq!(
            timing.classify_sample_start(1_000, 1_168).unwrap(),
            SampleStart::Skip {
                next_deadline_us: 1_200
            },
        );
        let evidence = timing.finish().unwrap();
        assert_eq!(evidence.valid_samples, 0);
        assert_eq!(evidence.missed_samples, 2);
        assert_eq!(evidence.max_consecutive_misses, 2);
        assert_eq!(evidence.max_lateness_us, 68);
    }

    #[test]
    fn successive_selected_slot_skips_retain_two_misses() {
        let mut timing = CaptureTiming::new(100, TimingEnvelope::INDUCTANCE);

        assert_eq!(
            timing.classify_sample_start(1_000, 1_092).unwrap(),
            SampleStart::Skip {
                next_deadline_us: 1_100
            }
        );
        assert_eq!(
            timing.classify_sample_start(1_100, 1_192).unwrap(),
            SampleStart::Skip {
                next_deadline_us: 1_200
            },
        );
        let evidence = timing.finish().unwrap();
        assert_eq!(evidence.missed_samples, 2);
        assert_eq!(evidence.max_consecutive_misses, 2);
    }

    #[test]
    fn selected_skip_then_elapsed_slot_finishes_with_two_misses() {
        let mut timing = CaptureTiming::new(100, TimingEnvelope::INDUCTANCE);
        assert_eq!(
            timing.classify_sample_start(1_000, 1_092).unwrap(),
            SampleStart::Skip {
                next_deadline_us: 1_100
            }
        );

        timing.record_sample(1_100, 1_200, 1_205, 1_210, 5).unwrap();

        let evidence = timing.finish().unwrap();
        assert_eq!(evidence.valid_samples, 1);
        assert_eq!(evidence.missed_samples, 2);
        assert_eq!(evidence.max_consecutive_misses, 2);
    }

    #[test]
    fn saturated_counters_reject_at_finish() {
        let mut timing = CaptureTiming::new(1, TimingEnvelope::INDUCTANCE);
        timing.valid_samples = u16::MAX;
        timing.record_sample(1_000, 1_000, 1_000, 1_000, 0).unwrap();

        assert_eq!(timing.finish(), Err(TimingReject::TelemetryOverflow));
    }
}
