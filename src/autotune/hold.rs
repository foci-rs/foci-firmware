use super::{
    CURRENT_LOOP_FAILURE_HOLD_POSITION_SPAN, CURRENT_LOOP_FAILURE_HOLD_SAMPLE,
    CURRENT_LOOP_FAILURE_HOLD_STATUS_FLAGS, current::current_loop_actionable_status_mask,
};

/// Default sustained-hold dwell duration.
pub const HOLD_GATE_DWELL_US: u32 = 250_000;
/// Fine-cadence diagnostic hold dwell duration.
pub const HOLD_GATE_FINE_DWELL_US: u32 = 50_000;
/// Fine-cadence diagnostic sample period.
pub const HOLD_GATE_FINE_SAMPLE_PERIOD_US: u32 = 250;
/// Maximum bounded samples in the sustained-hold accumulator.
pub const HOLD_GATE_MAX_SAMPLES: u16 = 250;
/// Evidence threshold for sustained-hold position span.
pub const HOLD_GATE_POSITION_SPAN_LIMIT_COUNTS: u32 = 16;
/// Default sustained-hold sample period.
pub const HOLD_GATE_SAMPLE_PERIOD_US: u32 = 1_000;

/// Hold gate has not run.
pub const HOLD_GATE_STATUS_NOT_RUN: u8 = 0;
/// Hold gate passed hard validation.
pub const HOLD_GATE_STATUS_PASS: u8 = 1;
/// Reserved legacy code; sustained-hold position span is warning-only.
pub const HOLD_GATE_STATUS_FAIL_POSITION_SPAN: u8 = 2;
/// Hold gate failed because actionable status flags were sampled.
pub const HOLD_GATE_STATUS_FAIL_STATUS_FLAGS: u8 = 3;
/// Hold gate failed because sampling or setup failed.
pub const HOLD_GATE_STATUS_FAIL_SAMPLE_ERROR: u8 = 4;

/// Sampled status flags included non-actionable telemetry.
pub const HOLD_GATE_WARNING_NON_ACTIONABLE_STATUS: u16 = 1 << 0;
/// Sampling cadence was slower than requested.
pub const HOLD_GATE_WARNING_SAMPLE_OVERRUN: u16 = 1 << 1;
/// Torque current RMS exceeded the diagnostic threshold.
pub const HOLD_GATE_WARNING_TORQUE_RMS: u16 = 1 << 2;
/// Flux current RMS exceeded the diagnostic threshold.
pub const HOLD_GATE_WARNING_FLUX_RMS: u16 = 1 << 3;
/// Torque current span exceeded the diagnostic threshold.
pub const HOLD_GATE_WARNING_TORQUE_SPAN: u16 = 1 << 4;
/// Flux current span exceeded the diagnostic threshold.
pub const HOLD_GATE_WARNING_FLUX_SPAN: u16 = 1 << 5;
/// Torque current had diagnostic crossing activity.
pub const HOLD_GATE_WARNING_TORQUE_CROSSINGS: u16 = 1 << 6;
/// Flux current had diagnostic crossing activity.
pub const HOLD_GATE_WARNING_FLUX_CROSSINGS: u16 = 1 << 7;
/// Position span exceeded the evidence threshold.
pub const HOLD_GATE_WARNING_POSITION_SPAN: u16 = 1 << 8;

/// Map a sustained-hold status code to a stable current-loop failure reason.
pub const fn hold_status_failure_reason(status: u8) -> Option<u8> {
    match status {
        HOLD_GATE_STATUS_FAIL_POSITION_SPAN => Some(CURRENT_LOOP_FAILURE_HOLD_POSITION_SPAN),
        HOLD_GATE_STATUS_FAIL_STATUS_FLAGS => Some(CURRENT_LOOP_FAILURE_HOLD_STATUS_FLAGS),
        HOLD_GATE_STATUS_FAIL_SAMPLE_ERROR => Some(CURRENT_LOOP_FAILURE_HOLD_SAMPLE),
        _ => None,
    }
}

/// One sustained-hold sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct HoldGateSample {
    /// Elapsed time since the hold window started.
    pub elapsed_us: u32,
    /// Actual position in encoder counts.
    pub position_count: i32,
    /// Actual torque current in TMC4671 counts.
    pub torque_count: i16,
    /// Actual flux current in TMC4671 counts.
    pub flux_count: i16,
    /// TMC4671 status flags sampled with this point.
    pub status_flags: u32,
}

/// Final sustained-hold metrics.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct HoldGateMetrics {
    /// `HOLD_GATE_STATUS_*` result.
    pub status: u8,
    /// ORed `HOLD_GATE_WARNING_*` flags.
    pub warning_flags: u16,
    /// Actual elapsed hold-window time.
    pub elapsed_us: u32,
    /// Number of accepted samples.
    pub sample_count: u16,
    /// Requested sample period for the run.
    pub requested_sample_period_us: u16,
    /// First sampled position.
    pub position_start_count: i32,
    /// Last sampled position.
    pub position_end_count: i32,
    /// Minimum sampled position.
    pub position_min_count: i32,
    /// Maximum sampled position.
    pub position_max_count: i32,
    /// Span between minimum and maximum sampled position.
    pub position_span_count: u32,
    /// Absolute drift between first and last sampled position.
    pub position_drift_count: u32,
    /// Mean torque current.
    pub torque_mean_count: i16,
    /// RMS torque current about the integer mean.
    pub torque_rms_count: u16,
    /// Torque current peak-to-peak span.
    pub torque_peak_to_peak_count: u16,
    /// Raw torque-current zero-crossing count.
    pub torque_crossing_count: u16,
    /// Mean flux current.
    pub flux_mean_count: i16,
    /// RMS flux current about the integer mean.
    pub flux_rms_count: u16,
    /// Flux current peak-to-peak span.
    pub flux_peak_to_peak_count: u16,
    /// Raw flux-current zero-crossing count.
    pub flux_crossing_count: u16,
    /// OR of sampled TMC4671 status flags.
    pub status_flags_or: u32,
    /// Number of samples containing actionable status flags.
    pub actionable_status_count: u16,
}

/// No-heap accumulator for sustained-hold metrics.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct HoldGateAccumulator {
    requested_sample_period_us: u16,
    actionable_status_mask: u32,
    sample_count: u16,
    position_start_count: i32,
    position_end_count: i32,
    position_min_count: i32,
    position_max_count: i32,
    torque_sum_count: i64,
    torque_sum_square_count: u64,
    torque_min_count: i16,
    torque_max_count: i16,
    torque_last_sign: i8,
    torque_crossing_count: u16,
    flux_sum_count: i64,
    flux_sum_square_count: u64,
    flux_min_count: i16,
    flux_max_count: i16,
    flux_last_sign: i8,
    flux_crossing_count: u16,
    status_flags_or: u32,
    actionable_status_count: u16,
    sample_overrun: bool,
    last_elapsed_us: u32,
}

impl HoldGateAccumulator {
    /// Build an accumulator with an explicit sample period and actionable mask.
    pub const fn new(requested_sample_period_us: u16, actionable_status_mask: u32) -> Self {
        Self {
            requested_sample_period_us,
            actionable_status_mask,
            sample_count: 0,
            position_start_count: 0,
            position_end_count: 0,
            position_min_count: 0,
            position_max_count: 0,
            torque_sum_count: 0,
            torque_sum_square_count: 0,
            torque_min_count: 0,
            torque_max_count: 0,
            torque_last_sign: 0,
            torque_crossing_count: 0,
            flux_sum_count: 0,
            flux_sum_square_count: 0,
            flux_min_count: 0,
            flux_max_count: 0,
            flux_last_sign: 0,
            flux_crossing_count: 0,
            status_flags_or: 0,
            actionable_status_count: 0,
            sample_overrun: false,
            last_elapsed_us: 0,
        }
    }

    /// Build an accumulator with the current-loop actionable status mask.
    pub const fn commissioning(
        requested_sample_period_us: u16,
        feedback_config: crate::config::FeedbackConfig,
    ) -> Self {
        Self::new(
            requested_sample_period_us,
            current_loop_actionable_status_mask(feedback_config),
        )
    }

    /// Add one sample to the accumulator.
    pub fn push(&mut self, sample: HoldGateSample) {
        if self.sample_count >= HOLD_GATE_MAX_SAMPLES {
            self.sample_overrun = true;
            return;
        }

        if self.sample_count == 0 {
            self.position_start_count = sample.position_count;
            self.position_min_count = sample.position_count;
            self.position_max_count = sample.position_count;
            self.torque_min_count = sample.torque_count;
            self.torque_max_count = sample.torque_count;
            self.flux_min_count = sample.flux_count;
            self.flux_max_count = sample.flux_count;
        } else if self.requested_sample_period_us > 0
            && sample.elapsed_us.saturating_sub(self.last_elapsed_us)
                > u32::from(self.requested_sample_period_us)
        {
            self.sample_overrun = true;
        }

        self.sample_count = self.sample_count.saturating_add(1);
        self.last_elapsed_us = sample.elapsed_us;
        self.position_end_count = sample.position_count;
        self.position_min_count = self.position_min_count.min(sample.position_count);
        self.position_max_count = self.position_max_count.max(sample.position_count);
        self.torque_sum_count += i64::from(sample.torque_count);
        self.torque_sum_square_count += current_square(sample.torque_count);
        self.torque_min_count = self.torque_min_count.min(sample.torque_count);
        self.torque_max_count = self.torque_max_count.max(sample.torque_count);
        count_crossing(
            &mut self.torque_last_sign,
            &mut self.torque_crossing_count,
            sample.torque_count,
        );

        self.flux_sum_count += i64::from(sample.flux_count);
        self.flux_sum_square_count += current_square(sample.flux_count);
        self.flux_min_count = self.flux_min_count.min(sample.flux_count);
        self.flux_max_count = self.flux_max_count.max(sample.flux_count);
        count_crossing(
            &mut self.flux_last_sign,
            &mut self.flux_crossing_count,
            sample.flux_count,
        );

        self.status_flags_or |= sample.status_flags;
        if sample.status_flags & self.actionable_status_mask != 0 {
            self.actionable_status_count = self.actionable_status_count.saturating_add(1);
        }
    }

    /// Return the number of accepted samples.
    pub const fn sample_count(&self) -> u16 {
        self.sample_count
    }

    /// Finish the run and return accumulated metrics.
    pub fn finish(&self, elapsed_us: u32, sample_error: bool) -> HoldGateMetrics {
        let position_span_count = self.position_max_count.abs_diff(self.position_min_count);
        let position_drift_count = self.position_end_count.abs_diff(self.position_start_count);
        let torque = finish_current_metrics(
            self.torque_sum_count,
            self.torque_sum_square_count,
            self.torque_min_count,
            self.torque_max_count,
            self.sample_count,
        );
        let flux = finish_current_metrics(
            self.flux_sum_count,
            self.flux_sum_square_count,
            self.flux_min_count,
            self.flux_max_count,
            self.sample_count,
        );

        let status = if sample_error {
            HOLD_GATE_STATUS_FAIL_SAMPLE_ERROR
        } else if self.sample_count == 0 {
            HOLD_GATE_STATUS_NOT_RUN
        } else if self.actionable_status_count > 0 {
            HOLD_GATE_STATUS_FAIL_STATUS_FLAGS
        } else {
            HOLD_GATE_STATUS_PASS
        };

        HoldGateMetrics {
            status,
            warning_flags: self.warning_flags(),
            elapsed_us,
            sample_count: self.sample_count,
            requested_sample_period_us: self.requested_sample_period_us,
            position_start_count: self.position_start_count,
            position_end_count: self.position_end_count,
            position_min_count: self.position_min_count,
            position_max_count: self.position_max_count,
            position_span_count,
            position_drift_count,
            torque_mean_count: torque.mean_count,
            torque_rms_count: torque.rms_count,
            torque_peak_to_peak_count: torque.peak_to_peak_count,
            torque_crossing_count: self.torque_crossing_count,
            flux_mean_count: flux.mean_count,
            flux_rms_count: flux.rms_count,
            flux_peak_to_peak_count: flux.peak_to_peak_count,
            flux_crossing_count: self.flux_crossing_count,
            status_flags_or: self.status_flags_or,
            actionable_status_count: self.actionable_status_count,
        }
    }

    fn warning_flags(&self) -> u16 {
        let mut flags = 0;

        if self.status_flags_or & !self.actionable_status_mask != 0 {
            flags |= HOLD_GATE_WARNING_NON_ACTIONABLE_STATUS;
        }
        if self.sample_overrun {
            flags |= HOLD_GATE_WARNING_SAMPLE_OVERRUN;
        }
        if self.position_max_count.abs_diff(self.position_min_count)
            > HOLD_GATE_POSITION_SPAN_LIMIT_COUNTS
        {
            flags |= HOLD_GATE_WARNING_POSITION_SPAN;
        }

        flags
    }
}

struct CurrentMetrics {
    mean_count: i16,
    rms_count: u16,
    peak_to_peak_count: u16,
}

fn current_square(value: i16) -> u64 {
    let value = i64::from(value);
    (value * value) as u64
}

fn current_sign(value: i16) -> i8 {
    match value.cmp(&0) {
        core::cmp::Ordering::Less => -1,
        core::cmp::Ordering::Equal => 0,
        core::cmp::Ordering::Greater => 1,
    }
}

fn count_crossing(last_sign: &mut i8, crossing_count: &mut u16, value: i16) {
    let sign = current_sign(value);
    if sign == 0 {
        return;
    }
    if *last_sign != 0 && *last_sign != sign {
        *crossing_count = crossing_count.saturating_add(1);
    }
    *last_sign = sign;
}

fn finish_current_metrics(
    sum_count: i64,
    sum_square_count: u64,
    min_count: i16,
    max_count: i16,
    sample_count: u16,
) -> CurrentMetrics {
    if sample_count == 0 {
        return CurrentMetrics {
            mean_count: 0,
            rms_count: 0,
            peak_to_peak_count: 0,
        };
    }

    let sample_count = i64::from(sample_count);
    let mean = sum_count / sample_count;
    let centered_sum_square =
        sum_square_count as i64 - 2 * mean * sum_count + sample_count * mean * mean;
    let variance = centered_sum_square / sample_count;
    let rms = integer_sqrt(variance.max(0) as u64).min(u32::from(u16::MAX)) as u16;
    let peak_to_peak_count = i32::from(max_count).abs_diff(i32::from(min_count)) as u16;

    CurrentMetrics {
        mean_count: mean.clamp(i64::from(i16::MIN), i64::from(i16::MAX)) as i16,
        rms_count: rms,
        peak_to_peak_count,
    }
}

fn integer_sqrt(value: u64) -> u32 {
    let mut remaining = value;
    let mut root = 0u64;
    let mut bit = 1u64 << 62;

    while bit > remaining {
        bit >>= 2;
    }

    while bit != 0 {
        if remaining >= root + bit {
            remaining -= root + bit;
            root = (root >> 1) + bit;
        } else {
            root >>= 1;
        }
        bit >>= 2;
    }

    root as u32
}

#[cfg(test)]
mod tests {
    use super::*;

    const CURRENT_WARNING_FLAGS: u16 = HOLD_GATE_WARNING_TORQUE_RMS
        | HOLD_GATE_WARNING_FLUX_RMS
        | HOLD_GATE_WARNING_TORQUE_SPAN
        | HOLD_GATE_WARNING_FLUX_SPAN
        | HOLD_GATE_WARNING_TORQUE_CROSSINGS
        | HOLD_GATE_WARNING_FLUX_CROSSINGS;

    #[test]
    fn hold_status_codes_match_stable_plan_values() {
        assert_eq!(HOLD_GATE_STATUS_NOT_RUN, 0);
        assert_eq!(HOLD_GATE_STATUS_PASS, 1);
        assert_eq!(HOLD_GATE_STATUS_FAIL_POSITION_SPAN, 2);
        assert_eq!(HOLD_GATE_STATUS_FAIL_STATUS_FLAGS, 3);
        assert_eq!(HOLD_GATE_STATUS_FAIL_SAMPLE_ERROR, 4);
    }

    #[test]
    fn status_failure_maps_to_current_loop_reason() {
        assert_eq!(
            hold_status_failure_reason(HOLD_GATE_STATUS_FAIL_POSITION_SPAN),
            Some(CURRENT_LOOP_FAILURE_HOLD_POSITION_SPAN)
        );
        assert_eq!(
            hold_status_failure_reason(HOLD_GATE_STATUS_FAIL_STATUS_FLAGS),
            Some(CURRENT_LOOP_FAILURE_HOLD_STATUS_FLAGS)
        );
        assert_eq!(
            hold_status_failure_reason(HOLD_GATE_STATUS_FAIL_SAMPLE_ERROR),
            Some(CURRENT_LOOP_FAILURE_HOLD_SAMPLE)
        );
        assert_eq!(hold_status_failure_reason(HOLD_GATE_STATUS_PASS), None);
    }

    #[test]
    fn hold_warning_bits_match_stable_plan_values() {
        assert_eq!(HOLD_GATE_WARNING_NON_ACTIONABLE_STATUS, 1 << 0);
        assert_eq!(HOLD_GATE_WARNING_SAMPLE_OVERRUN, 1 << 1);
        assert_eq!(HOLD_GATE_WARNING_TORQUE_RMS, 1 << 2);
        assert_eq!(HOLD_GATE_WARNING_FLUX_RMS, 1 << 3);
        assert_eq!(HOLD_GATE_WARNING_TORQUE_SPAN, 1 << 4);
        assert_eq!(HOLD_GATE_WARNING_FLUX_SPAN, 1 << 5);
        assert_eq!(HOLD_GATE_WARNING_TORQUE_CROSSINGS, 1 << 6);
        assert_eq!(HOLD_GATE_WARNING_FLUX_CROSSINGS, 1 << 7);
        assert_eq!(HOLD_GATE_WARNING_POSITION_SPAN, 1 << 8);
    }

    #[test]
    fn hold_sample_period_constants_are_u32() {
        fn require_u32(_: u32) {}

        require_u32(HOLD_GATE_FINE_SAMPLE_PERIOD_US);
        require_u32(HOLD_GATE_SAMPLE_PERIOD_US);
    }

    #[test]
    fn quiet_hold_reports_zero_rms_and_passes() {
        let mut acc = HoldGateAccumulator::new(
            0,
            current_loop_actionable_status_mask(crate::config::FeedbackConfig::abn_abn()),
        );
        acc.push(HoldGateSample {
            elapsed_us: 0,
            position_count: 10,
            torque_count: 42,
            flux_count: -7,
            status_flags: 0,
        });
        acc.push(HoldGateSample {
            elapsed_us: 1_000,
            position_count: 10,
            torque_count: 42,
            flux_count: -7,
            status_flags: 0,
        });

        let metrics = acc.finish(1_000, false);

        assert_eq!(metrics.status, HOLD_GATE_STATUS_PASS);
        assert_eq!(metrics.sample_count, 2);
        assert_eq!(metrics.position_span_count, 0);
        assert_eq!(metrics.position_drift_count, 0);
        assert_eq!(metrics.torque_mean_count, 42);
        assert_eq!(metrics.torque_rms_count, 0);
        assert_eq!(metrics.torque_peak_to_peak_count, 0);
        assert_eq!(metrics.flux_mean_count, -7);
        assert_eq!(metrics.flux_rms_count, 0);
        assert_eq!(metrics.warning_flags, 0);
    }

    #[test]
    fn alternating_current_reports_rms_span_and_crossings() {
        let mut acc = HoldGateAccumulator::new(
            0,
            current_loop_actionable_status_mask(crate::config::FeedbackConfig::abn_abn()),
        );
        for (idx, torque) in [100, -100, 100, -100].iter().copied().enumerate() {
            acc.push(HoldGateSample {
                elapsed_us: (idx as u32) * 1_000,
                position_count: 0,
                torque_count: torque,
                flux_count: 0,
                status_flags: 0,
            });
        }

        let metrics = acc.finish(3_000, false);

        assert_eq!(metrics.status, HOLD_GATE_STATUS_PASS);
        assert_eq!(metrics.torque_mean_count, 0);
        assert_eq!(metrics.torque_rms_count, 100);
        assert_eq!(metrics.torque_peak_to_peak_count, 200);
        assert_eq!(metrics.torque_crossing_count, 3);
    }

    #[test]
    fn dc_biased_alternation_reports_raw_not_demeaned_crossings() {
        let mut acc = HoldGateAccumulator::new(
            0,
            current_loop_actionable_status_mask(crate::config::FeedbackConfig::abn_abn()),
        );
        for (idx, torque) in [100, 200, 100, 200].iter().copied().enumerate() {
            acc.push(HoldGateSample {
                elapsed_us: (idx as u32) * 1_000,
                position_count: 0,
                torque_count: torque,
                flux_count: -torque,
                status_flags: 0,
            });
        }

        let metrics = acc.finish(3_000, false);

        // Exact demeaned crossings would require the final mean during sampling.
        // The accumulator keeps no sample storage, so it reports raw zero crossings only.
        assert_eq!(metrics.torque_mean_count, 150);
        assert_eq!(metrics.torque_rms_count, 50);
        assert_eq!(metrics.torque_peak_to_peak_count, 100);
        assert_eq!(metrics.torque_crossing_count, 0);
        assert_eq!(metrics.flux_crossing_count, 0);
        assert_eq!(metrics.warning_flags, 0);
    }

    #[test]
    fn high_positive_dc_one_count_ripple_rms_uses_reported_mean() {
        let mut acc = HoldGateAccumulator::new(
            0,
            current_loop_actionable_status_mask(crate::config::FeedbackConfig::abn_abn()),
        );
        for (idx, torque) in [32766, 32767].iter().copied().enumerate() {
            acc.push(HoldGateSample {
                elapsed_us: (idx as u32) * 1_000,
                position_count: 0,
                torque_count: torque,
                flux_count: 0,
                status_flags: 0,
            });
        }

        let metrics = acc.finish(1_000, false);

        assert_eq!(metrics.torque_mean_count, 32766);
        assert_eq!(metrics.torque_rms_count, 0);
        assert_eq!(metrics.torque_peak_to_peak_count, 1);
    }

    #[test]
    fn high_negative_dc_one_count_ripple_rms_uses_reported_mean() {
        let mut acc = HoldGateAccumulator::new(
            0,
            current_loop_actionable_status_mask(crate::config::FeedbackConfig::abn_abn()),
        );
        for (idx, torque) in [-32768, -32767].iter().copied().enumerate() {
            acc.push(HoldGateSample {
                elapsed_us: (idx as u32) * 1_000,
                position_count: 0,
                torque_count: torque,
                flux_count: 0,
                status_flags: 0,
            });
        }

        let metrics = acc.finish(1_000, false);

        assert_eq!(metrics.torque_mean_count, -32767);
        assert_eq!(metrics.torque_rms_count, 0);
        assert_eq!(metrics.torque_peak_to_peak_count, 1);
    }

    #[test]
    fn noisy_current_metrics_do_not_warn_before_threshold_selection() {
        let mut acc = HoldGateAccumulator::new(
            0,
            current_loop_actionable_status_mask(crate::config::FeedbackConfig::abn_abn()),
        );
        for (idx, torque) in [600, -600, 600, -600].iter().copied().enumerate() {
            acc.push(HoldGateSample {
                elapsed_us: (idx as u32) * 1_000,
                position_count: 0,
                torque_count: torque,
                flux_count: -torque,
                status_flags: 0,
            });
        }

        let metrics = acc.finish(3_000, false);

        assert_eq!(metrics.torque_rms_count, 600);
        assert_eq!(metrics.torque_peak_to_peak_count, 1200);
        assert_eq!(metrics.flux_rms_count, 600);
        assert_eq!(metrics.flux_peak_to_peak_count, 1200);
        assert_eq!(metrics.warning_flags, 0);
    }

    #[test]
    fn non_actionable_status_sets_only_status_warning() {
        let mut acc = HoldGateAccumulator::new(
            0,
            current_loop_actionable_status_mask(crate::config::FeedbackConfig::abn_abn()),
        );
        acc.push(HoldGateSample {
            elapsed_us: 0,
            position_count: 0,
            torque_count: 600,
            flux_count: -600,
            status_flags: 1 << 0,
        });

        let metrics = acc.finish(0, false);

        assert_eq!(
            metrics.warning_flags,
            HOLD_GATE_WARNING_NON_ACTIONABLE_STATUS
        );
        assert_eq!(metrics.warning_flags & CURRENT_WARNING_FLAGS, 0);
    }

    #[test]
    fn sample_overrun_sets_only_overrun_warning() {
        let mut acc = HoldGateAccumulator::new(
            1_000,
            current_loop_actionable_status_mask(crate::config::FeedbackConfig::abn_abn()),
        );
        acc.push(HoldGateSample {
            elapsed_us: 0,
            position_count: 0,
            torque_count: 600,
            flux_count: -600,
            status_flags: 0,
        });
        acc.push(HoldGateSample {
            elapsed_us: 1_001,
            position_count: 0,
            torque_count: -600,
            flux_count: 600,
            status_flags: 0,
        });

        let metrics = acc.finish(1_001, false);

        assert_eq!(metrics.warning_flags, HOLD_GATE_WARNING_SAMPLE_OVERRUN);
        assert_eq!(metrics.warning_flags & CURRENT_WARNING_FLAGS, 0);
    }

    #[test]
    fn samples_beyond_cap_set_overrun_warning() {
        let mut acc = HoldGateAccumulator::new(
            0,
            current_loop_actionable_status_mask(crate::config::FeedbackConfig::abn_abn()),
        );
        for idx in 0..HOLD_GATE_MAX_SAMPLES {
            acc.push(HoldGateSample {
                elapsed_us: u32::from(idx),
                position_count: 0,
                torque_count: 0,
                flux_count: 0,
                status_flags: 0,
            });
        }
        acc.push(HoldGateSample {
            elapsed_us: u32::from(HOLD_GATE_MAX_SAMPLES),
            position_count: HOLD_GATE_POSITION_SPAN_LIMIT_COUNTS as i32 + 1,
            torque_count: 0,
            flux_count: 0,
            status_flags: 1 << 8,
        });

        let metrics = acc.finish(u32::from(HOLD_GATE_MAX_SAMPLES), false);

        assert_eq!(metrics.sample_count, HOLD_GATE_MAX_SAMPLES);
        assert_eq!(metrics.status, HOLD_GATE_STATUS_PASS);
        assert_eq!(metrics.warning_flags, HOLD_GATE_WARNING_SAMPLE_OVERRUN);
        assert_eq!(metrics.warning_flags & CURRENT_WARNING_FLAGS, 0);
    }

    #[test]
    fn position_span_above_limit_is_evidence_warning() {
        let mut acc = HoldGateAccumulator::new(
            0,
            current_loop_actionable_status_mask(crate::config::FeedbackConfig::abn_abn()),
        );
        acc.push(HoldGateSample {
            elapsed_us: 0,
            position_count: -8,
            torque_count: 0,
            flux_count: 0,
            status_flags: 0,
        });
        acc.push(HoldGateSample {
            elapsed_us: 1_000,
            position_count: 9,
            torque_count: 0,
            flux_count: 0,
            status_flags: 0,
        });

        let metrics = acc.finish(1_000, false);

        assert_eq!(metrics.status, HOLD_GATE_STATUS_PASS);
        assert_eq!(metrics.warning_flags, HOLD_GATE_WARNING_POSITION_SPAN);
        assert_eq!(metrics.position_span_count, 17);
    }

    #[test]
    fn actionable_status_flag_is_hard_failure() {
        let mut acc = HoldGateAccumulator::new(0, 1 << 8);
        acc.push(HoldGateSample {
            elapsed_us: 0,
            position_count: 0,
            torque_count: 0,
            flux_count: 0,
            status_flags: 1 << 8,
        });

        let metrics = acc.finish(0, false);

        assert_eq!(metrics.status, HOLD_GATE_STATUS_FAIL_STATUS_FLAGS);
        assert_eq!(metrics.status_flags_or, 1 << 8);
        assert_eq!(metrics.actionable_status_count, 1);
    }
}
