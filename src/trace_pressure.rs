//! Shared trace pressure counters and policy.
//!
//! This module owns board-global trace-health telemetry. It deliberately does
//! not own USB queues, DWT reads, frame emission, metadata, status source
//! layout, or board-specific step timing.

use core::sync::atomic::{AtomicBool, AtomicU16, AtomicU32, Ordering};

use crate::trace_events::{EVENT_LEVEL_INFO, EVENT_SPI_SAMPLE_DURATION, SUBSYSTEM_TMC_CONTROL};

/// Board-global trace pressure and status-throttle runtime.
pub struct TracePressureRuntime {
    spi_error_count: AtomicU32,
    status_interrupt_count: AtomicU32,
    suppressed_status_change_count: AtomicU32,
    pending_status_change: AtomicBool,
    last_status_change_emit_tick: AtomicU32,
    first_spi_sample_cycles: AtomicU32,
    max_spi_sample_cycles: AtomicU32,
    first_spi_duration_reported: AtomicBool,
    skip_next_control_detail: AtomicBool,
    trace_deadline_pressure_count: AtomicU32,
    last_command_service_tick_low: AtomicU32,
    last_command_service_tick_high: AtomicU32,
    latest_queue_depth: AtomicU16,
}

impl TracePressureRuntime {
    /// Create a zero-initialized trace pressure runtime.
    pub const fn new() -> Self {
        Self {
            spi_error_count: AtomicU32::new(0),
            status_interrupt_count: AtomicU32::new(0),
            suppressed_status_change_count: AtomicU32::new(0),
            pending_status_change: AtomicBool::new(false),
            last_status_change_emit_tick: AtomicU32::new(0),
            first_spi_sample_cycles: AtomicU32::new(0),
            max_spi_sample_cycles: AtomicU32::new(0),
            first_spi_duration_reported: AtomicBool::new(false),
            skip_next_control_detail: AtomicBool::new(false),
            trace_deadline_pressure_count: AtomicU32::new(0),
            last_command_service_tick_low: AtomicU32::new(0),
            last_command_service_tick_high: AtomicU32::new(0),
            latest_queue_depth: AtomicU16::new(0),
        }
    }

    /// Record a trace-only TMC SPI error.
    pub fn record_spi_error(&self) {
        self.spi_error_count.fetch_add(1, Ordering::Relaxed);
        self.pending_status_change.store(true, Ordering::Relaxed);
    }

    /// Record a TMC STATUS interrupt observed by the board.
    pub fn record_status_interrupt(&self) {
        self.status_interrupt_count.fetch_add(1, Ordering::Relaxed);
        self.pending_status_change.store(true, Ordering::Relaxed);
    }

    /// Record one trace sample await span in DWT cycles.
    pub fn record_spi_sample_cycles(&self, cycles: u32, threshold_cycles: u32) {
        let mut current = self.max_spi_sample_cycles.load(Ordering::Relaxed);
        while cycles > current {
            match self.max_spi_sample_cycles.compare_exchange_weak(
                current,
                cycles,
                Ordering::Relaxed,
                Ordering::Relaxed,
            ) {
                Ok(_) => break,
                Err(next) => current = next,
            }
        }
        let _ = self.first_spi_sample_cycles.compare_exchange(
            0,
            cycles,
            Ordering::Relaxed,
            Ordering::Relaxed,
        );
        if cycles > threshold_cycles {
            self.skip_next_control_detail.store(true, Ordering::Relaxed);
            self.trace_deadline_pressure_count
                .fetch_add(1, Ordering::Relaxed);
            self.pending_status_change.store(true, Ordering::Relaxed);
        }
    }

    /// Take the one-shot control-detail skip flag.
    pub fn take_control_detail_skip(&self) -> bool {
        self.skip_next_control_detail.swap(false, Ordering::Relaxed)
    }

    /// Store the last command service tick for critical trace frames.
    pub fn store_last_command_service_tick(&self, ticks: u64) {
        self.last_command_service_tick_low
            .store(ticks as u32, Ordering::Relaxed);
        self.last_command_service_tick_high
            .store((ticks >> 32) as u32, Ordering::Relaxed);
    }

    /// Load the last command service tick for critical trace frames.
    pub fn last_command_service_tick(&self) -> u64 {
        u64::from(self.last_command_service_tick_low.load(Ordering::Relaxed))
            | (u64::from(self.last_command_service_tick_high.load(Ordering::Relaxed)) << 32)
    }

    /// Record the latest accepted step queue depth.
    pub fn record_queue_depth(&self, queue_len: usize) {
        self.latest_queue_depth
            .store(queue_len.min(u16::MAX as usize) as u16, Ordering::Relaxed);
    }

    /// Return the latest accepted step queue depth.
    pub fn latest_queue_depth(&self) -> u16 {
        self.latest_queue_depth.load(Ordering::Relaxed)
    }

    /// Return the one-shot SPI-duration event payload when available.
    pub fn take_spi_duration_event(&self) -> Option<TracePressureEvent> {
        if self
            .first_spi_duration_reported
            .swap(true, Ordering::Relaxed)
        {
            return None;
        }
        let cycles = self.first_spi_sample_cycles.load(Ordering::Relaxed);
        if cycles == 0 {
            self.first_spi_duration_reported
                .store(false, Ordering::Relaxed);
            return None;
        }
        Some(TracePressureEvent {
            level: EVENT_LEVEL_INFO,
            subsystem_id: SUBSYSTEM_TMC_CONTROL,
            event_id: EVENT_SPI_SAMPLE_DURATION,
            args: [
                cycles as i32,
                self.max_spi_sample_cycles.load(Ordering::Relaxed) as i32,
                0,
                0,
            ],
            arg_count: 2,
        })
    }

    /// Decide whether a status sample should be emitted for this tick.
    pub fn should_emit_status(
        &self,
        control_tick: u32,
        regular_tick: bool,
        consume_pending_change: bool,
    ) -> bool {
        let pending_change = if consume_pending_change {
            self.pending_status_change.swap(false, Ordering::Relaxed)
        } else {
            false
        };
        if !regular_tick && !pending_change {
            return false;
        }
        if pending_change {
            let previous = self.last_status_change_emit_tick.load(Ordering::Relaxed);
            if control_tick.wrapping_sub(previous) < 10 {
                // Preserve the existing behavior: a throttled status-change
                // event also suppresses an otherwise regular status sample.
                self.suppressed_status_change_count
                    .fetch_add(1, Ordering::Relaxed);
                return false;
            }
            self.last_status_change_emit_tick
                .store(control_tick, Ordering::Relaxed);
        }
        true
    }

    /// Snapshot trace pressure counters for a board status frame.
    pub fn status_counters(&self, inputs: TracePressureStatusInputs) -> TracePressureStatus {
        TracePressureStatus {
            spi_error_count: self.spi_error_count.load(Ordering::Relaxed),
            status_interrupt_count: self.status_interrupt_count.load(Ordering::Relaxed),
            watchdog_or_step_overrun_count: inputs.missed_or_deferred_step_count,
            latest_queue_depth: self.latest_queue_depth.load(Ordering::Relaxed),
            usb_trace_queue_high_water: inputs.usb_trace_queue_high_water,
            dropped_motion_sample_count: inputs.dropped_motion_sample_count,
            dropped_event_log_count: inputs.dropped_event_log_count,
            endpoint_not_ready_count: inputs
                .endpoint_not_ready_count
                .saturating_add(self.trace_deadline_pressure_count.load(Ordering::Relaxed)),
            suppressed_status_change_count: self
                .suppressed_status_change_count
                .load(Ordering::Relaxed),
        }
    }
}

impl Default for TracePressureRuntime {
    fn default() -> Self {
        Self::new()
    }
}

/// Trace event data produced by the shared pressure runtime.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct TracePressureEvent {
    /// Event severity level.
    pub level: u8,
    /// Event subsystem ID.
    pub subsystem_id: u8,
    /// Event ID.
    pub event_id: u16,
    /// Event arguments.
    pub args: [i32; 4],
    /// Number of populated event arguments.
    pub arg_count: u8,
}

/// Board-owned counters folded into a trace pressure status snapshot.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, defmt::Format)]
pub struct TracePressureStatusInputs {
    /// USB endpoint-not-ready count from the transport.
    pub endpoint_not_ready_count: u32,
    /// Motion trace queue high-water mark.
    pub usb_trace_queue_high_water: u16,
    /// Dropped motion sample count.
    pub dropped_motion_sample_count: u32,
    /// Dropped event log count.
    pub dropped_event_log_count: u32,
    /// Missed or deferred step-timer event count.
    pub missed_or_deferred_step_count: u32,
}

/// Trace pressure status counters for board-owned status frame assembly.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, defmt::Format)]
pub struct TracePressureStatus {
    /// Trace-only SPI error count.
    pub spi_error_count: u32,
    /// STATUS interrupt count.
    pub status_interrupt_count: u32,
    /// Step watchdog or step overrun count.
    pub watchdog_or_step_overrun_count: u32,
    /// Latest accepted step queue depth.
    pub latest_queue_depth: u16,
    /// Motion trace queue high-water mark.
    pub usb_trace_queue_high_water: u16,
    /// Dropped motion sample count.
    pub dropped_motion_sample_count: u32,
    /// Dropped event log count.
    pub dropped_event_log_count: u32,
    /// Endpoint-not-ready count plus trace deadline pressure.
    pub endpoint_not_ready_count: u32,
    /// Suppressed status-change sample count.
    pub suppressed_status_change_count: u32,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn spi_error_increments_and_makes_status_pending() {
        let runtime = TracePressureRuntime::new();

        runtime.record_spi_error();

        let status = runtime.status_counters(TracePressureStatusInputs::default());
        assert_eq!(status.spi_error_count, 1);
        assert!(runtime.should_emit_status(10, false, true));
    }

    #[test]
    fn status_interrupt_increments_and_makes_status_pending() {
        let runtime = TracePressureRuntime::new();

        runtime.record_status_interrupt();

        let status = runtime.status_counters(TracePressureStatusInputs::default());
        assert_eq!(status.status_interrupt_count, 1);
        assert!(runtime.should_emit_status(10, false, true));
    }

    #[test]
    fn spi_sample_cycles_record_first_and_maximum() {
        let runtime = TracePressureRuntime::new();

        runtime.record_spi_sample_cycles(40, 100);
        runtime.record_spi_sample_cycles(25, 100);
        runtime.record_spi_sample_cycles(90, 100);

        let event = runtime.take_spi_duration_event().unwrap();
        assert_eq!(event.args[0], 40);
        assert_eq!(event.args[1], 90);
    }

    #[test]
    fn long_spi_sample_sets_one_detail_skip_and_pressure_count() {
        let runtime = TracePressureRuntime::new();

        runtime.record_spi_sample_cycles(101, 100);

        assert!(runtime.take_control_detail_skip());
        assert!(!runtime.take_control_detail_skip());
        let status = runtime.status_counters(TracePressureStatusInputs::default());
        assert_eq!(status.endpoint_not_ready_count, 1);
        assert!(runtime.should_emit_status(10, false, true));
    }

    #[test]
    fn spi_duration_event_is_one_shot_after_first_sample() {
        let runtime = TracePressureRuntime::new();

        assert_eq!(runtime.take_spi_duration_event(), None);

        runtime.record_spi_sample_cycles(55, 100);
        let event = runtime.take_spi_duration_event().unwrap();

        assert_eq!(event.level, crate::trace_events::EVENT_LEVEL_INFO);
        assert_eq!(
            event.subsystem_id,
            crate::trace_events::SUBSYSTEM_TMC_CONTROL
        );
        assert_eq!(
            event.event_id,
            crate::trace_events::EVENT_SPI_SAMPLE_DURATION
        );
        assert_eq!(event.args, [55, 55, 0, 0]);
        assert_eq!(event.arg_count, 2);
        assert_eq!(runtime.take_spi_duration_event(), None);
    }

    #[test]
    fn status_emission_requires_regular_tick_or_pending_change() {
        let runtime = TracePressureRuntime::new();

        assert!(!runtime.should_emit_status(10, false, true));
        assert!(runtime.should_emit_status(10, true, true));
    }

    #[test]
    fn pending_status_change_is_throttled_for_ten_ticks() {
        let runtime = TracePressureRuntime::new();

        runtime.record_spi_error();
        assert!(runtime.should_emit_status(10, false, true));

        runtime.record_spi_error();
        assert!(!runtime.should_emit_status(19, false, true));

        let status = runtime.status_counters(TracePressureStatusInputs::default());
        assert_eq!(status.suppressed_status_change_count, 1);
    }

    #[test]
    fn pending_status_change_inside_window_suppresses_regular_tick() {
        let runtime = TracePressureRuntime::new();

        runtime.record_status_interrupt();
        assert!(runtime.should_emit_status(10, false, true));

        runtime.record_status_interrupt();
        assert!(!runtime.should_emit_status(15, true, true));

        let status = runtime.status_counters(TracePressureStatusInputs::default());
        assert_eq!(status.suppressed_status_change_count, 1);
    }

    #[test]
    fn consume_pending_false_does_not_clear_pending_status_change() {
        let runtime = TracePressureRuntime::new();

        runtime.record_spi_error();

        assert!(!runtime.should_emit_status(10, false, false));
        assert!(runtime.should_emit_status(10, false, true));
    }

    #[test]
    fn status_snapshot_saturates_endpoint_pressure_sum() {
        let runtime = TracePressureRuntime::new();

        runtime.record_spi_sample_cycles(101, 100);

        let status = runtime.status_counters(TracePressureStatusInputs {
            endpoint_not_ready_count: u32::MAX,
            ..TracePressureStatusInputs::default()
        });
        assert_eq!(status.endpoint_not_ready_count, u32::MAX);
    }

    #[test]
    fn status_snapshot_carries_board_inputs() {
        let runtime = TracePressureRuntime::new();

        runtime.record_queue_depth(7);
        let status = runtime.status_counters(TracePressureStatusInputs {
            endpoint_not_ready_count: 1,
            usb_trace_queue_high_water: 2,
            dropped_motion_sample_count: 3,
            dropped_event_log_count: 4,
            missed_or_deferred_step_count: 5,
        });

        assert_eq!(status.latest_queue_depth, 7);
        assert_eq!(status.endpoint_not_ready_count, 1);
        assert_eq!(status.usb_trace_queue_high_water, 2);
        assert_eq!(status.dropped_motion_sample_count, 3);
        assert_eq!(status.dropped_event_log_count, 4);
        assert_eq!(status.watchdog_or_step_overrun_count, 5);
    }

    #[test]
    fn command_service_tick_round_trips_split_storage() {
        let runtime = TracePressureRuntime::new();

        runtime.store_last_command_service_tick(0x1122_3344_5566_7788);

        assert_eq!(runtime.last_command_service_tick(), 0x1122_3344_5566_7788);
    }

    #[test]
    fn queue_depth_saturates_to_u16_max() {
        let runtime = TracePressureRuntime::new();

        runtime.record_queue_depth(usize::MAX);

        assert_eq!(runtime.latest_queue_depth(), u16::MAX);
    }
}
