//! Structured diagnostic codes emitted before fatal shutdowns when transport is usable.

use core::sync::atomic::{AtomicI32, AtomicU32, Ordering::Relaxed};

/// Fatal fault class carried by `foci_diagnostic`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
#[repr(u8)]
pub enum FaultClass {
    /// A queue could not accept an operation.
    QueueFull = 1,
    /// A command referenced an unknown OID.
    UnknownOid = 2,
    /// A reset was requested while a channel was active.
    ActiveReset = 3,
    /// A command referenced an invalid channel.
    InvalidChannel = 4,
}

/// Operation code carried by `foci_diagnostic`.
pub mod operation {
    /// `queue_step`.
    pub const QUEUE_STEP: u8 = 1;
    /// `set_next_step_dir`.
    pub const SET_NEXT_STEP_DIR: u8 = 2;
    /// `reset_step_clock`.
    pub const RESET_STEP_CLOCK: u8 = 3;
    /// `tmc_control`.
    pub const TMC_CONTROL: u8 = 4;
    /// `stepper_stop`.
    pub const STEPPER_STOP: u8 = 5;
    /// Stepper OID/channel lookup before a motion command.
    pub const STEPPER_LOOKUP: u8 = 6;
}

/// Bounded stepper runtime event reason carried by `foci_stepper_event`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
#[repr(u8)]
pub enum StepperEventReason {
    /// Step timer found no more queued moves and disarmed itself.
    QueueEmpty = 1,
    /// First step of a newly loaded segment missed the deadline window.
    MissedDeadlineLoad = 2,
    /// A step/unstep event inside an active segment missed the deadline window.
    MissedDeadlineStep = 3,
    /// A TrSync trigger stopped the physical stepper.
    TrsyncStop = 4,
    /// P1 control code stopped the physical stepper.
    P1Stop = 5,
    /// Host reset the step-clock epoch.
    ResetStepClock = 6,
    /// TMC fault/disable signal stopped the motor.
    TmcDisableSignal = 7,
}

impl StepperEventReason {
    /// Stable short name for host log formatting.
    pub const fn name(self) -> &'static str {
        match self {
            Self::QueueEmpty => "queue_empty",
            Self::MissedDeadlineLoad => "missed_deadline_load",
            Self::MissedDeadlineStep => "missed_deadline_step",
            Self::TrsyncStop => "trsync_stop",
            Self::P1Stop => "p1_stop",
            Self::ResetStepClock => "reset_step_clock",
            Self::TmcDisableSignal => "tmc_disable_signal",
        }
    }
}

/// Snapshot sent to the host when a bounded stepper runtime event occurs.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct StepperDiagnosticEvent {
    /// Event reason.
    pub reason: StepperEventReason,
    /// Board-local stepper channel.
    pub channel: u8,
    /// Firmware step counter at event time.
    pub position: i32,
    /// TIM5/Klipper clock at event time.
    pub clock: u32,
    /// Non-zero when the board believes the step timer is active.
    pub timer_active: bool,
    /// Board-local queue length, or `u16::MAX` if unknown at the call site.
    pub queue_len: u16,
    /// Current direction state.
    pub direction: bool,
    /// Reason-specific value.
    pub data0: u32,
    /// Reason-specific value.
    pub data1: u32,
}

/// Snapshot of board-local step queue and execution counters.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct StepperExecutionSnapshot {
    /// Board-local stepper channel.
    pub channel: u8,
    /// Firmware step counter at query time.
    pub position: i32,
    /// Accepted `queue_step` segments since boot.
    pub queued_segments: u32,
    /// Accepted motor steps since boot.
    pub queued_steps: u32,
    /// Segments loaded by the timer ISR since boot.
    pub loaded_segments: u32,
    /// Motor steps loaded by the timer ISR since boot.
    pub loaded_steps: u32,
    /// Executed positive-direction motor steps since boot.
    pub executed_pos_steps: u32,
    /// Executed negative-direction motor steps since boot.
    pub executed_neg_steps: u32,
    /// Number of times the board armed the step timer.
    pub activation_count: u32,
    /// TIM5 clock when the step timer was most recently armed.
    pub last_activation_clock: u32,
    /// TIM5 clock when the first segment of the last activation loaded.
    pub first_load_now: u32,
    /// Scheduled first-step clock from the first loaded segment.
    pub first_load_scheduled: u32,
    /// Compare clock selected for the first loaded segment.
    pub first_load_compare: u32,
    /// `first_load_scheduled - first_load_now`, interpreted as signed ticks.
    pub first_load_lead_ticks: i32,
    /// `first_load_compare - first_load_now`, interpreted as signed ticks.
    pub first_load_compare_delay_ticks: i32,
    /// TIM5 clock observed at the first rising edge of the last activation.
    pub first_step_clock: u32,
    /// `first_step_clock - last_activation_clock`, interpreted as signed ticks.
    pub first_step_delay_ticks: i32,
    /// Segments discarded because the stepper awaited `reset_step_clock`.
    pub discarded_segments: u32,
    /// Motor steps discarded because the stepper awaited `reset_step_clock`.
    pub discarded_steps: u32,
    /// Times the step timer went idle because the queue was empty.
    pub queue_empty_count: u32,
    /// Combined missed-deadline count from segment-load and in-segment checks.
    pub missed_deadline_count: u32,
    /// Physical stepper stop count from TrSync or P1 stop paths.
    pub stop_count: u32,
    /// Queued segments drained by physical stop paths since boot.
    pub stop_drained_segments: u32,
    /// Queued motor steps drained by physical stop paths since boot.
    pub stop_drained_steps: u32,
    /// Accepted `reset_step_clock` count.
    pub reset_count: u32,
    /// Queued segments drained by `reset_step_clock` since boot.
    pub reset_drained_segments: u32,
    /// Queued motor steps drained by `reset_step_clock` since boot.
    pub reset_drained_steps: u32,
    /// [`StepperEventReason`] code for the most recent physical stop.
    pub last_stop_reason: u8,
    /// Active timer events remaining at the most recent physical stop.
    pub last_stop_remaining_events: u32,
    /// Queue length observed at the most recent physical stop.
    pub last_stop_queue_len: u16,
    /// Queued segments drained by the most recent physical stop.
    pub last_stop_drained_segments: u32,
    /// Queued motor steps drained by the most recent physical stop.
    pub last_stop_drained_steps: u32,
    /// Non-zero when the board believes the step timer is active.
    pub timer_active: bool,
    /// Current board-local queue length.
    pub queue_len: u16,
}

struct StepperExecutionCounters {
    queued_segments: AtomicU32,
    queued_steps: AtomicU32,
    loaded_segments: AtomicU32,
    loaded_steps: AtomicU32,
    executed_pos_steps: AtomicU32,
    executed_neg_steps: AtomicU32,
    activation_count: AtomicU32,
    last_activation_clock: AtomicU32,
    activation_loaded_segments: AtomicU32,
    activation_executed_steps: AtomicU32,
    first_load_now: AtomicU32,
    first_load_scheduled: AtomicU32,
    first_load_compare: AtomicU32,
    first_load_lead_ticks: AtomicI32,
    first_load_compare_delay_ticks: AtomicI32,
    first_step_clock: AtomicU32,
    first_step_delay_ticks: AtomicI32,
    discarded_segments: AtomicU32,
    discarded_steps: AtomicU32,
    queue_empty_count: AtomicU32,
    missed_deadline_count: AtomicU32,
    stop_count: AtomicU32,
    stop_drained_segments: AtomicU32,
    stop_drained_steps: AtomicU32,
    reset_count: AtomicU32,
    reset_drained_segments: AtomicU32,
    reset_drained_steps: AtomicU32,
    last_stop_reason: AtomicU32,
    last_stop_remaining_events: AtomicU32,
    last_stop_queue_len: AtomicU32,
    last_stop_drained_segments: AtomicU32,
    last_stop_drained_steps: AtomicU32,
}

impl StepperExecutionCounters {
    const fn new() -> Self {
        Self {
            queued_segments: AtomicU32::new(0),
            queued_steps: AtomicU32::new(0),
            loaded_segments: AtomicU32::new(0),
            loaded_steps: AtomicU32::new(0),
            executed_pos_steps: AtomicU32::new(0),
            executed_neg_steps: AtomicU32::new(0),
            activation_count: AtomicU32::new(0),
            last_activation_clock: AtomicU32::new(0),
            activation_loaded_segments: AtomicU32::new(0),
            activation_executed_steps: AtomicU32::new(0),
            first_load_now: AtomicU32::new(0),
            first_load_scheduled: AtomicU32::new(0),
            first_load_compare: AtomicU32::new(0),
            first_load_lead_ticks: AtomicI32::new(0),
            first_load_compare_delay_ticks: AtomicI32::new(0),
            first_step_clock: AtomicU32::new(0),
            first_step_delay_ticks: AtomicI32::new(0),
            discarded_segments: AtomicU32::new(0),
            discarded_steps: AtomicU32::new(0),
            queue_empty_count: AtomicU32::new(0),
            missed_deadline_count: AtomicU32::new(0),
            stop_count: AtomicU32::new(0),
            stop_drained_segments: AtomicU32::new(0),
            stop_drained_steps: AtomicU32::new(0),
            reset_count: AtomicU32::new(0),
            reset_drained_segments: AtomicU32::new(0),
            reset_drained_steps: AtomicU32::new(0),
            last_stop_reason: AtomicU32::new(0),
            last_stop_remaining_events: AtomicU32::new(0),
            last_stop_queue_len: AtomicU32::new(0),
            last_stop_drained_segments: AtomicU32::new(0),
            last_stop_drained_steps: AtomicU32::new(0),
        }
    }
}

/// Shared board-agnostic step queue and execution counters.
///
/// The counters are diagnostic best-effort views over relaxed atomics. Snapshot
/// callers may observe fields from different instants while P3 is updating
/// them; this avoids adding locks or critical sections to the stepper hot path.
pub struct StepperExecutionStats<const CHANNELS: usize> {
    counters: [StepperExecutionCounters; CHANNELS],
}

impl<const CHANNELS: usize> StepperExecutionStats<CHANNELS> {
    /// Create zeroed execution counters for `CHANNELS` stepper channels.
    pub const fn new() -> Self {
        Self {
            counters: [const { StepperExecutionCounters::new() }; CHANNELS],
        }
    }

    fn counters(&self, channel: u8) -> Option<&StepperExecutionCounters> {
        self.counters.get(channel as usize)
    }

    /// Record that the board armed the step timer for a new activation.
    pub fn record_timer_activation(&self, channel: u8, clock: u32) {
        if let Some(counters) = self.counters(channel) {
            counters.activation_count.fetch_add(1, Relaxed);
            counters.last_activation_clock.store(clock, Relaxed);
            counters.activation_loaded_segments.store(0, Relaxed);
            counters.activation_executed_steps.store(0, Relaxed);
            counters.first_load_now.store(0, Relaxed);
            counters.first_load_scheduled.store(0, Relaxed);
            counters.first_load_compare.store(0, Relaxed);
            counters.first_load_lead_ticks.store(0, Relaxed);
            counters.first_load_compare_delay_ticks.store(0, Relaxed);
            counters.first_step_clock.store(0, Relaxed);
            counters.first_step_delay_ticks.store(0, Relaxed);
        }
    }

    /// Record an accepted `queue_step` segment.
    pub fn record_queued_segment(&self, channel: u8, steps: u32) {
        if let Some(counters) = self.counters(channel) {
            counters.queued_segments.fetch_add(1, Relaxed);
            counters.queued_steps.fetch_add(steps, Relaxed);
        }
    }

    /// Record a segment loaded by the timer ISR.
    pub fn record_loaded_segment(&self, channel: u8, steps: u32) {
        if let Some(counters) = self.counters(channel) {
            counters.loaded_segments.fetch_add(1, Relaxed);
            counters.loaded_steps.fetch_add(steps, Relaxed);
        }
    }

    /// Record a segment loaded by the timer ISR with scheduling context.
    pub fn record_loaded_segment_timing(
        &self,
        channel: u8,
        steps: u32,
        now: u32,
        scheduled: u32,
        compare: u32,
    ) {
        self.record_loaded_segment(channel, steps);
        if let Some(counters) = self.counters(channel)
            && counters.activation_loaded_segments.fetch_add(1, Relaxed) == 0
        {
            counters.first_load_now.store(now, Relaxed);
            counters.first_load_scheduled.store(scheduled, Relaxed);
            counters.first_load_compare.store(compare, Relaxed);
            counters
                .first_load_lead_ticks
                .store(scheduled.wrapping_sub(now) as i32, Relaxed);
            counters
                .first_load_compare_delay_ticks
                .store(compare.wrapping_sub(now) as i32, Relaxed);
        }
    }

    /// Record one executed motor step and first-step timing for the activation.
    pub fn record_executed_step(&self, channel: u8, forward: bool, clock: u32) {
        if let Some(counters) = self.counters(channel) {
            if forward {
                counters.executed_pos_steps.fetch_add(1, Relaxed);
            } else {
                counters.executed_neg_steps.fetch_add(1, Relaxed);
            }
            if counters.activation_executed_steps.fetch_add(1, Relaxed) == 0 {
                let activation_clock = counters.last_activation_clock.load(Relaxed);
                counters.first_step_clock.store(clock, Relaxed);
                counters
                    .first_step_delay_ticks
                    .store(clock.wrapping_sub(activation_clock) as i32, Relaxed);
            }
        }
    }

    /// Record a `queue_step` segment discarded while awaiting reset.
    pub fn record_discarded_segment(&self, channel: u8, steps: u32) {
        if let Some(counters) = self.counters(channel) {
            counters.discarded_segments.fetch_add(1, Relaxed);
            counters.discarded_steps.fetch_add(steps, Relaxed);
        }
    }

    /// Record a queue-empty step timer stop.
    pub fn record_queue_empty(&self, channel: u8) {
        if let Some(counters) = self.counters(channel) {
            counters.queue_empty_count.fetch_add(1, Relaxed);
        }
    }

    /// Record a missed step deadline.
    pub fn record_missed_deadline(&self, channel: u8) {
        if let Some(counters) = self.counters(channel) {
            counters.missed_deadline_count.fetch_add(1, Relaxed);
        }
    }

    /// Record a physical stop snapshot before the active segment is reset.
    pub fn record_physical_stop(
        &self,
        channel: u8,
        reason: StepperEventReason,
        remaining_events: u32,
        queue_len: u16,
        drained_segments: u32,
        drained_steps: u32,
    ) {
        if let Some(counters) = self.counters(channel) {
            counters.stop_count.fetch_add(1, Relaxed);
            counters
                .stop_drained_segments
                .fetch_add(drained_segments, Relaxed);
            counters
                .stop_drained_steps
                .fetch_add(drained_steps, Relaxed);
            counters.last_stop_reason.store(reason as u32, Relaxed);
            counters
                .last_stop_remaining_events
                .store(remaining_events, Relaxed);
            counters
                .last_stop_queue_len
                .store(u32::from(queue_len), Relaxed);
            counters
                .last_stop_drained_segments
                .store(drained_segments, Relaxed);
            counters
                .last_stop_drained_steps
                .store(drained_steps, Relaxed);
        }
    }

    /// Record an accepted step-clock reset.
    pub fn record_reset(&self, channel: u8, drained_segments: u32, drained_steps: u32) {
        if let Some(counters) = self.counters(channel) {
            counters.reset_count.fetch_add(1, Relaxed);
            counters
                .reset_drained_segments
                .fetch_add(drained_segments, Relaxed);
            counters
                .reset_drained_steps
                .fetch_add(drained_steps, Relaxed);
        }
    }

    /// Return a no-motion snapshot of stepper execution counters.
    pub fn snapshot(
        &self,
        channel: u8,
        position: i32,
        timer_active: bool,
        queue_len: u16,
    ) -> Option<StepperExecutionSnapshot> {
        self.counters(channel)
            .map(|counters| StepperExecutionSnapshot {
                channel,
                position,
                queued_segments: counters.queued_segments.load(Relaxed),
                queued_steps: counters.queued_steps.load(Relaxed),
                loaded_segments: counters.loaded_segments.load(Relaxed),
                loaded_steps: counters.loaded_steps.load(Relaxed),
                executed_pos_steps: counters.executed_pos_steps.load(Relaxed),
                executed_neg_steps: counters.executed_neg_steps.load(Relaxed),
                activation_count: counters.activation_count.load(Relaxed),
                last_activation_clock: counters.last_activation_clock.load(Relaxed),
                first_load_now: counters.first_load_now.load(Relaxed),
                first_load_scheduled: counters.first_load_scheduled.load(Relaxed),
                first_load_compare: counters.first_load_compare.load(Relaxed),
                first_load_lead_ticks: counters.first_load_lead_ticks.load(Relaxed),
                first_load_compare_delay_ticks: counters
                    .first_load_compare_delay_ticks
                    .load(Relaxed),
                first_step_clock: counters.first_step_clock.load(Relaxed),
                first_step_delay_ticks: counters.first_step_delay_ticks.load(Relaxed),
                discarded_segments: counters.discarded_segments.load(Relaxed),
                discarded_steps: counters.discarded_steps.load(Relaxed),
                queue_empty_count: counters.queue_empty_count.load(Relaxed),
                missed_deadline_count: counters.missed_deadline_count.load(Relaxed),
                stop_count: counters.stop_count.load(Relaxed),
                stop_drained_segments: counters.stop_drained_segments.load(Relaxed),
                stop_drained_steps: counters.stop_drained_steps.load(Relaxed),
                reset_count: counters.reset_count.load(Relaxed),
                reset_drained_segments: counters.reset_drained_segments.load(Relaxed),
                reset_drained_steps: counters.reset_drained_steps.load(Relaxed),
                last_stop_reason: counters.last_stop_reason.load(Relaxed) as u8,
                last_stop_remaining_events: counters.last_stop_remaining_events.load(Relaxed),
                last_stop_queue_len: counters.last_stop_queue_len.load(Relaxed) as u16,
                last_stop_drained_segments: counters.last_stop_drained_segments.load(Relaxed),
                last_stop_drained_steps: counters.last_stop_drained_steps.load(Relaxed),
                timer_active,
                queue_len,
            })
    }
}

impl<const CHANNELS: usize> Default for StepperExecutionStats<CHANNELS> {
    fn default() -> Self {
        Self::new()
    }
}

const PERF_SITE_QUEUE_STEP: u8 = 1;

struct StepperPerfCounterState {
    sample_count: AtomicU32,
    crit_count: AtomicU32,
    crit_max_cycles: AtomicU32,
    crit_max_site: AtomicU32,
    crit_over_10us: AtomicU32,
    crit_over_50us: AtomicU32,
    crit_over_100us: AtomicU32,
    crit_over_1000us: AtomicU32,
    queue_step_count: AtomicU32,
    queue_step_max_cycles: AtomicU32,
    tim5_activation_count: AtomicU32,
    tim5_irq_max_cycles: AtomicU32,
    tim5_dispatch_max_cycles: AtomicU32,
    tim5_dispatch_max_cycles_events: AtomicU32,
    tim5_events_max_per_irq: AtomicU32,
    tim5_event_count_total: AtomicU32,
    tim5_defer_count: AtomicU32,
    tim5_empty_count: AtomicU32,
    tim5_events_last_activation: AtomicU32,
    tim5_burst_cycles_per_event_max: AtomicU32,
    tim5_burst_cycles_per_event_max_cycles: AtomicU32,
    tim5_burst_cycles_per_event_max_events: AtomicU32,
    tim5_burst_cycles_per_event_floor3_max: AtomicU32,
    tim5_entry_latency_max_ticks: AtomicU32,
    tim5_pop_lateness_max_ticks: AtomicU32,
    scheduler_cycles_max: AtomicU32,
    scheduler_cycles_events_at_max: AtomicU32,
    scheduler_cycles_per_event_max: AtomicU32,
    scheduler_cycles_per_event_max_cycles: AtomicU32,
    scheduler_cycles_per_event_max_events: AtomicU32,
    scheduler_cycles_per_event_floor3_max: AtomicU32,
    scheduler_full_count: AtomicU32,
    stepper_load_lateness_max_ticks: AtomicU32,
    stepper_load_lateness_last_ticks: AtomicI32,
}

impl StepperPerfCounterState {
    const fn new() -> Self {
        Self {
            sample_count: AtomicU32::new(0),
            crit_count: AtomicU32::new(0),
            crit_max_cycles: AtomicU32::new(0),
            crit_max_site: AtomicU32::new(0),
            crit_over_10us: AtomicU32::new(0),
            crit_over_50us: AtomicU32::new(0),
            crit_over_100us: AtomicU32::new(0),
            crit_over_1000us: AtomicU32::new(0),
            queue_step_count: AtomicU32::new(0),
            queue_step_max_cycles: AtomicU32::new(0),
            tim5_activation_count: AtomicU32::new(0),
            tim5_irq_max_cycles: AtomicU32::new(0),
            tim5_dispatch_max_cycles: AtomicU32::new(0),
            tim5_dispatch_max_cycles_events: AtomicU32::new(0),
            tim5_events_max_per_irq: AtomicU32::new(0),
            tim5_event_count_total: AtomicU32::new(0),
            tim5_defer_count: AtomicU32::new(0),
            tim5_empty_count: AtomicU32::new(0),
            tim5_events_last_activation: AtomicU32::new(0),
            tim5_burst_cycles_per_event_max: AtomicU32::new(0),
            tim5_burst_cycles_per_event_max_cycles: AtomicU32::new(0),
            tim5_burst_cycles_per_event_max_events: AtomicU32::new(0),
            tim5_burst_cycles_per_event_floor3_max: AtomicU32::new(0),
            tim5_entry_latency_max_ticks: AtomicU32::new(0),
            tim5_pop_lateness_max_ticks: AtomicU32::new(0),
            scheduler_cycles_max: AtomicU32::new(0),
            scheduler_cycles_events_at_max: AtomicU32::new(0),
            scheduler_cycles_per_event_max: AtomicU32::new(0),
            scheduler_cycles_per_event_max_cycles: AtomicU32::new(0),
            scheduler_cycles_per_event_max_events: AtomicU32::new(0),
            scheduler_cycles_per_event_floor3_max: AtomicU32::new(0),
            scheduler_full_count: AtomicU32::new(0),
            stepper_load_lateness_max_ticks: AtomicU32::new(0),
            stepper_load_lateness_last_ticks: AtomicI32::new(0),
        }
    }

    fn reset(&self) {
        self.sample_count.store(0, Relaxed);
        self.crit_count.store(0, Relaxed);
        self.crit_max_cycles.store(0, Relaxed);
        self.crit_max_site.store(0, Relaxed);
        self.crit_over_10us.store(0, Relaxed);
        self.crit_over_50us.store(0, Relaxed);
        self.crit_over_100us.store(0, Relaxed);
        self.crit_over_1000us.store(0, Relaxed);
        self.queue_step_count.store(0, Relaxed);
        self.queue_step_max_cycles.store(0, Relaxed);
        self.tim5_activation_count.store(0, Relaxed);
        self.tim5_irq_max_cycles.store(0, Relaxed);
        self.tim5_dispatch_max_cycles.store(0, Relaxed);
        self.tim5_dispatch_max_cycles_events.store(0, Relaxed);
        self.tim5_events_max_per_irq.store(0, Relaxed);
        self.tim5_event_count_total.store(0, Relaxed);
        self.tim5_defer_count.store(0, Relaxed);
        self.tim5_empty_count.store(0, Relaxed);
        self.tim5_events_last_activation.store(0, Relaxed);
        self.tim5_burst_cycles_per_event_max.store(0, Relaxed);
        self.tim5_burst_cycles_per_event_max_cycles
            .store(0, Relaxed);
        self.tim5_burst_cycles_per_event_max_events
            .store(0, Relaxed);
        self.tim5_burst_cycles_per_event_floor3_max
            .store(0, Relaxed);
        self.tim5_entry_latency_max_ticks.store(0, Relaxed);
        self.tim5_pop_lateness_max_ticks.store(0, Relaxed);
        self.scheduler_cycles_max.store(0, Relaxed);
        self.scheduler_cycles_events_at_max.store(0, Relaxed);
        self.scheduler_cycles_per_event_max.store(0, Relaxed);
        self.scheduler_cycles_per_event_max_cycles.store(0, Relaxed);
        self.scheduler_cycles_per_event_max_events.store(0, Relaxed);
        self.scheduler_cycles_per_event_floor3_max.store(0, Relaxed);
        self.scheduler_full_count.store(0, Relaxed);
        self.stepper_load_lateness_max_ticks.store(0, Relaxed);
        self.stepper_load_lateness_last_ticks.store(0, Relaxed);
    }
}

/// Shared board-agnostic step-dispatch performance counters.
///
/// The counters are diagnostic best-effort views over relaxed atomics. Snapshot
/// callers may observe fields from different instants while P3 updates them;
/// this avoids adding locks or critical sections to stepper hot paths.
pub struct StepperPerfCounters<const CPU_CYCLES_PER_US: u32> {
    counters: StepperPerfCounterState,
}

impl<const CPU_CYCLES_PER_US: u32> StepperPerfCounters<CPU_CYCLES_PER_US> {
    /// Create zeroed step-dispatch performance counters.
    pub const fn new() -> Self {
        Self {
            counters: StepperPerfCounterState::new(),
        }
    }

    /// Record one trigger-domain critical-section duration.
    pub fn record_critical_section(&self, site: u8, cycles: u32) {
        let counters = &self.counters;
        counters.sample_count.fetch_add(1, Relaxed);
        counters.crit_count.fetch_add(1, Relaxed);
        record_max_with_site(
            &counters.crit_max_cycles,
            &counters.crit_max_site,
            cycles,
            site,
        );
        if cycles >= 10 * CPU_CYCLES_PER_US {
            counters.crit_over_10us.fetch_add(1, Relaxed);
        }
        if cycles >= 50 * CPU_CYCLES_PER_US {
            counters.crit_over_50us.fetch_add(1, Relaxed);
        }
        if cycles >= 100 * CPU_CYCLES_PER_US {
            counters.crit_over_100us.fetch_add(1, Relaxed);
        }
        if cycles >= 1000 * CPU_CYCLES_PER_US {
            counters.crit_over_1000us.fetch_add(1, Relaxed);
        }
        if site == PERF_SITE_QUEUE_STEP {
            counters.queue_step_count.fetch_add(1, Relaxed);
            record_max(&counters.queue_step_max_cycles, cycles);
        }
    }

    /// Record lateness observed at TIM5 ISR body entry before dispatch starts.
    pub fn record_tim5_entry_latency(&self, lateness_ticks: u32) {
        record_max(&self.counters.tim5_entry_latency_max_ticks, lateness_ticks);
    }

    /// Record that TIM5 dispatch deferred after hitting the repeat guard.
    pub fn record_tim5_defer(&self) {
        self.counters.tim5_defer_count.fetch_add(1, Relaxed);
    }

    /// Record scheduler pop lateness measured before event callback work.
    pub fn record_scheduler_pop_lateness(&self, lateness_ticks: u32) {
        record_max(&self.counters.tim5_pop_lateness_max_ticks, lateness_ticks);
    }

    /// Record a failed scheduler insertion caused by a full event table.
    pub fn record_scheduler_full(&self) {
        self.counters.scheduler_full_count.fetch_add(1, Relaxed);
    }

    /// Record TIM5-dispatch scheduler bookkeeping cycles.
    pub fn record_scheduler_cycles(&self, scheduler_cycles: u32, event_count: u32) {
        let counters = &self.counters;
        record_max_with_event_count(
            &counters.scheduler_cycles_max,
            &counters.scheduler_cycles_events_at_max,
            scheduler_cycles,
            event_count,
        );
        if let Some(cycles_per_event) = scheduler_cycles.checked_div(event_count) {
            record_max_burst_ratio(
                &counters.scheduler_cycles_per_event_max,
                &counters.scheduler_cycles_per_event_max_cycles,
                &counters.scheduler_cycles_per_event_max_events,
                cycles_per_event,
                scheduler_cycles,
                event_count,
            );
            if event_count >= 3 {
                record_max(
                    &counters.scheduler_cycles_per_event_floor3_max,
                    cycles_per_event,
                );
            }
        }
    }

    /// Record one TIM5 IRQ dispatch sample.
    pub fn record_tim5_irq(&self, irq_cycles: u32, dispatch_cycles: u32, event_count: u32) {
        let counters = &self.counters;
        counters.sample_count.fetch_add(1, Relaxed);
        counters.tim5_activation_count.fetch_add(1, Relaxed);
        counters
            .tim5_event_count_total
            .fetch_add(event_count, Relaxed);
        counters
            .tim5_events_last_activation
            .store(event_count, Relaxed);
        if let Some(cycles_per_event) = dispatch_cycles.checked_div(event_count) {
            record_max_burst_ratio(
                &counters.tim5_burst_cycles_per_event_max,
                &counters.tim5_burst_cycles_per_event_max_cycles,
                &counters.tim5_burst_cycles_per_event_max_events,
                cycles_per_event,
                dispatch_cycles,
                event_count,
            );
            if event_count >= 3 {
                record_max(
                    &counters.tim5_burst_cycles_per_event_floor3_max,
                    cycles_per_event,
                );
            }
        } else {
            counters.tim5_empty_count.fetch_add(1, Relaxed);
        }
        record_max(&counters.tim5_irq_max_cycles, irq_cycles);
        record_max_with_event_count(
            &counters.tim5_dispatch_max_cycles,
            &counters.tim5_dispatch_max_cycles_events,
            dispatch_cycles,
            event_count,
        );
        record_max(&counters.tim5_events_max_per_irq, event_count);
    }

    /// Record signed segment-load lateness observed by the stepper ISR.
    pub fn record_stepper_load_lateness(&self, lateness_ticks: i32) {
        self.counters
            .stepper_load_lateness_last_ticks
            .store(lateness_ticks, Relaxed);
        if lateness_ticks > 0 {
            record_max(
                &self.counters.stepper_load_lateness_max_ticks,
                lateness_ticks as u32,
            );
        }
    }

    /// Return a no-motion snapshot of step-dispatch performance counters.
    pub fn snapshot(
        &self,
        channel: u8,
        clear: bool,
        build_trace_enabled: bool,
    ) -> StepperPerfSnapshot {
        let counters = &self.counters;
        let snapshot = StepperPerfSnapshot {
            channel,
            sample_count: counters.sample_count.load(Relaxed),
            crit_count: counters.crit_count.load(Relaxed),
            crit_max_cycles: counters.crit_max_cycles.load(Relaxed),
            crit_max_site: counters.crit_max_site.load(Relaxed) as u8,
            crit_over_10us: counters.crit_over_10us.load(Relaxed),
            crit_over_50us: counters.crit_over_50us.load(Relaxed),
            crit_over_100us: counters.crit_over_100us.load(Relaxed),
            crit_over_1000us: counters.crit_over_1000us.load(Relaxed),
            queue_step_count: counters.queue_step_count.load(Relaxed),
            queue_step_max_cycles: counters.queue_step_max_cycles.load(Relaxed),
            tim5_activation_count: counters.tim5_activation_count.load(Relaxed),
            tim5_irq_max_cycles: counters.tim5_irq_max_cycles.load(Relaxed),
            tim5_dispatch_max_cycles: counters.tim5_dispatch_max_cycles.load(Relaxed),
            tim5_dispatch_max_cycles_events: counters.tim5_dispatch_max_cycles_events.load(Relaxed),
            tim5_events_max_per_irq: counters.tim5_events_max_per_irq.load(Relaxed),
            tim5_event_count_total: counters.tim5_event_count_total.load(Relaxed),
            tim5_defer_count: counters.tim5_defer_count.load(Relaxed),
            tim5_empty_count: counters.tim5_empty_count.load(Relaxed),
            tim5_events_last_activation: counters.tim5_events_last_activation.load(Relaxed),
            tim5_burst_cycles_per_event_max: counters.tim5_burst_cycles_per_event_max.load(Relaxed),
            tim5_burst_cycles_per_event_max_cycles: counters
                .tim5_burst_cycles_per_event_max_cycles
                .load(Relaxed),
            tim5_burst_cycles_per_event_max_events: counters
                .tim5_burst_cycles_per_event_max_events
                .load(Relaxed),
            tim5_burst_cycles_per_event_floor3_max: counters
                .tim5_burst_cycles_per_event_floor3_max
                .load(Relaxed),
            tim5_entry_latency_max_ticks: counters.tim5_entry_latency_max_ticks.load(Relaxed),
            tim5_pop_lateness_max_ticks: counters.tim5_pop_lateness_max_ticks.load(Relaxed),
            scheduler_cycles_max: counters.scheduler_cycles_max.load(Relaxed),
            scheduler_cycles_events_at_max: counters.scheduler_cycles_events_at_max.load(Relaxed),
            scheduler_cycles_per_event_max: counters.scheduler_cycles_per_event_max.load(Relaxed),
            scheduler_cycles_per_event_max_cycles: counters
                .scheduler_cycles_per_event_max_cycles
                .load(Relaxed),
            scheduler_cycles_per_event_max_events: counters
                .scheduler_cycles_per_event_max_events
                .load(Relaxed),
            scheduler_cycles_per_event_floor3_max: counters
                .scheduler_cycles_per_event_floor3_max
                .load(Relaxed),
            scheduler_full_count: counters.scheduler_full_count.load(Relaxed),
            stepper_load_lateness_max_ticks: counters.stepper_load_lateness_max_ticks.load(Relaxed),
            stepper_load_lateness_last_ticks: counters
                .stepper_load_lateness_last_ticks
                .load(Relaxed),
            build_trace_enabled,
        };
        if clear {
            counters.reset();
        }
        snapshot
    }
}

impl<const CPU_CYCLES_PER_US: u32> Default for StepperPerfCounters<CPU_CYCLES_PER_US> {
    fn default() -> Self {
        Self::new()
    }
}

fn record_max(max: &AtomicU32, value: u32) {
    let mut current = max.load(Relaxed);
    while value > current {
        match max.compare_exchange_weak(current, value, Relaxed, Relaxed) {
            Ok(_) => return,
            Err(next) => current = next,
        }
    }
}

fn record_max_with_event_count(
    max: &AtomicU32,
    events_at_max: &AtomicU32,
    value: u32,
    event_count: u32,
) {
    let mut current = max.load(Relaxed);
    while value > current {
        match max.compare_exchange_weak(current, value, Relaxed, Relaxed) {
            Ok(_) => {
                events_at_max.store(event_count, Relaxed);
                return;
            }
            Err(next) => current = next,
        }
    }
}

fn record_max_burst_ratio(
    max: &AtomicU32,
    cycles_at_max: &AtomicU32,
    events_at_max: &AtomicU32,
    ratio: u32,
    cycles: u32,
    event_count: u32,
) {
    let mut current = max.load(Relaxed);
    while ratio > current {
        match max.compare_exchange_weak(current, ratio, Relaxed, Relaxed) {
            Ok(_) => {
                cycles_at_max.store(cycles, Relaxed);
                events_at_max.store(event_count, Relaxed);
                return;
            }
            Err(next) => current = next,
        }
    }
}

fn record_max_with_site(max: &AtomicU32, site: &AtomicU32, value: u32, site_code: u8) {
    let mut current = max.load(Relaxed);
    while value > current {
        match max.compare_exchange_weak(current, value, Relaxed, Relaxed) {
            Ok(_) => {
                site.store(u32::from(site_code), Relaxed);
                return;
            }
            Err(next) => current = next,
        }
    }
}

/// Snapshot of board-local step-dispatch performance counters.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct StepperPerfSnapshot {
    /// Board-local stepper channel associated with the query.
    pub channel: u8,
    /// Total profiler samples accumulated since the last reset.
    pub sample_count: u32,
    /// Trigger-domain critical sections measured since the last reset.
    pub crit_count: u32,
    /// Longest trigger-domain critical section in CPU cycles.
    pub crit_max_cycles: u32,
    /// Site code that produced `crit_max_cycles`.
    pub crit_max_site: u8,
    /// Critical sections longer than 10 us.
    pub crit_over_10us: u32,
    /// Critical sections longer than 50 us.
    pub crit_over_50us: u32,
    /// Critical sections longer than 100 us.
    pub crit_over_100us: u32,
    /// Critical sections longer than 1000 us.
    pub crit_over_1000us: u32,
    /// `queue_step` trigger-domain sections measured since the last reset.
    pub queue_step_count: u32,
    /// Longest `queue_step` trigger-domain section in CPU cycles.
    pub queue_step_max_cycles: u32,
    /// TIM5 IRQ dispatch activations since the last reset.
    pub tim5_activation_count: u32,
    /// Longest TIM5 IRQ handler duration in CPU cycles.
    pub tim5_irq_max_cycles: u32,
    /// Longest TIM5 scheduler dispatch duration in CPU cycles.
    pub tim5_dispatch_max_cycles: u32,
    /// Events processed in the activation that set `tim5_dispatch_max_cycles`.
    pub tim5_dispatch_max_cycles_events: u32,
    /// Largest number of due events processed by one TIM5 IRQ.
    pub tim5_events_max_per_irq: u32,
    /// Total scheduler events processed by TIM5 since the last reset.
    pub tim5_event_count_total: u32,
    /// TIM5 activations that hit the repeat guard and deferred.
    pub tim5_defer_count: u32,
    /// TIM5 activations that processed no scheduler event.
    pub tim5_empty_count: u32,
    /// Events processed in the most recent TIM5 activation.
    pub tim5_events_last_activation: u32,
    /// Max per-activation `dispatch_cycles / processed_events`.
    pub tim5_burst_cycles_per_event_max: u32,
    /// Raw dispatch cycles for the activation that set `tim5_burst_cycles_per_event_max`.
    pub tim5_burst_cycles_per_event_max_cycles: u32,
    /// Events processed by the activation that set `tim5_burst_cycles_per_event_max`.
    pub tim5_burst_cycles_per_event_max_events: u32,
    /// Max per-activation cycles/event for activations with at least three events.
    pub tim5_burst_cycles_per_event_floor3_max: u32,
    /// Max TIM5 entry lateness in TIM5 ticks.
    pub tim5_entry_latency_max_ticks: u32,
    /// Max positive scheduler pop lateness in TIM5 ticks.
    pub tim5_pop_lateness_max_ticks: u32,
    /// Longest TIM5-dispatch scheduler bookkeeping duration in CPU cycles.
    pub scheduler_cycles_max: u32,
    /// Events processed in the activation that set `scheduler_cycles_max`.
    pub scheduler_cycles_events_at_max: u32,
    /// Max per-activation `scheduler_cycles / processed_events`.
    pub scheduler_cycles_per_event_max: u32,
    /// Raw scheduler cycles for the activation that set the per-event max.
    pub scheduler_cycles_per_event_max_cycles: u32,
    /// Events processed by the activation that set the per-event max.
    pub scheduler_cycles_per_event_max_events: u32,
    /// Max scheduler cycles/event for activations with at least three events.
    pub scheduler_cycles_per_event_floor3_max: u32,
    /// Scheduler insertions dropped because the fixed table was full.
    pub scheduler_full_count: u32,
    /// Max positive stepper segment-load lateness in TIM5 ticks.
    pub stepper_load_lateness_max_ticks: u32,
    /// Last stepper segment-load lateness as signed TIM5 ticks.
    pub stepper_load_lateness_last_ticks: i32,
    /// Non-zero when the board was built with trace instrumentation.
    pub build_trace_enabled: bool,
}

/// Fatal step-dispatch event with a performance-counter snapshot.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct StepperPerfDiagnosticEvent {
    /// Event reason that caused the snapshot.
    pub reason: StepperEventReason,
    /// TIM5/Klipper clock at event time.
    pub clock: u32,
    /// Dispatch performance counters captured near event time.
    pub snapshot: StepperPerfSnapshot,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fault_class_codes_are_stable() {
        assert_eq!(FaultClass::QueueFull as u8, 1);
        assert_eq!(FaultClass::UnknownOid as u8, 2);
        assert_eq!(FaultClass::ActiveReset as u8, 3);
        assert_eq!(FaultClass::InvalidChannel as u8, 4);
    }

    #[test]
    fn operation_codes_are_stable() {
        assert_eq!(operation::QUEUE_STEP, 1);
        assert_eq!(operation::SET_NEXT_STEP_DIR, 2);
        assert_eq!(operation::RESET_STEP_CLOCK, 3);
        assert_eq!(operation::TMC_CONTROL, 4);
        assert_eq!(operation::STEPPER_STOP, 5);
        assert_eq!(operation::STEPPER_LOOKUP, 6);
    }

    #[test]
    fn stepper_event_reason_codes_are_stable() {
        assert_eq!(StepperEventReason::QueueEmpty as u8, 1);
        assert_eq!(StepperEventReason::MissedDeadlineLoad as u8, 2);
        assert_eq!(StepperEventReason::MissedDeadlineStep as u8, 3);
        assert_eq!(StepperEventReason::TrsyncStop as u8, 4);
        assert_eq!(StepperEventReason::P1Stop as u8, 5);
        assert_eq!(StepperEventReason::ResetStepClock as u8, 6);
        assert_eq!(StepperEventReason::TmcDisableSignal as u8, 7);
    }

    #[test]
    fn stepper_event_reason_names_are_short_and_stable() {
        assert_eq!(StepperEventReason::QueueEmpty.name(), "queue_empty");
        assert_eq!(
            StepperEventReason::MissedDeadlineLoad.name(),
            "missed_deadline_load"
        );
        assert_eq!(
            StepperEventReason::MissedDeadlineStep.name(),
            "missed_deadline_step"
        );
        assert_eq!(StepperEventReason::TrsyncStop.name(), "trsync_stop");
        assert_eq!(StepperEventReason::P1Stop.name(), "p1_stop");
        assert_eq!(
            StepperEventReason::ResetStepClock.name(),
            "reset_step_clock"
        );
        assert_eq!(
            StepperEventReason::TmcDisableSignal.name(),
            "tmc_disable_signal"
        );
    }

    #[test]
    fn stepper_execution_snapshot_reports_motion_counters() {
        let snapshot = StepperExecutionSnapshot {
            channel: 0,
            position: -1173,
            queued_segments: 196,
            queued_steps: 45150,
            loaded_segments: 57,
            loaded_steps: 14352,
            executed_pos_steps: 14352,
            executed_neg_steps: 0,
            activation_count: 2,
            last_activation_clock: 123_456,
            first_load_now: 123_584,
            first_load_scheduled: 124_000,
            first_load_compare: 124_000,
            first_load_lead_ticks: 416,
            first_load_compare_delay_ticks: 416,
            first_step_clock: 124_002,
            first_step_delay_ticks: 546,
            discarded_segments: 2,
            discarded_steps: 500,
            queue_empty_count: 0,
            missed_deadline_count: 0,
            stop_count: 1,
            stop_drained_segments: 44,
            stop_drained_steps: 11_128,
            reset_count: 1,
            reset_drained_segments: 3,
            reset_drained_steps: 750,
            last_stop_reason: StepperEventReason::TrsyncStop as u8,
            last_stop_remaining_events: 61596,
            last_stop_queue_len: 138,
            last_stop_drained_segments: 44,
            last_stop_drained_steps: 11_128,
            timer_active: false,
            queue_len: 0,
        };

        assert_eq!(snapshot.channel, 0);
        assert_eq!(snapshot.queued_steps, 45150);
        assert_eq!(snapshot.loaded_steps, 14352);
        assert_eq!(snapshot.executed_pos_steps, 14352);
        assert_eq!(snapshot.activation_count, 2);
        assert_eq!(snapshot.first_load_lead_ticks, 416);
        assert_eq!(snapshot.first_step_delay_ticks, 546);
        assert_eq!(snapshot.discarded_steps, 500);
        assert_eq!(snapshot.stop_drained_steps, 11_128);
        assert_eq!(snapshot.reset_drained_steps, 750);
        assert_eq!(
            snapshot.last_stop_reason,
            StepperEventReason::TrsyncStop as u8
        );
        assert_eq!(snapshot.last_stop_remaining_events, 61596);
        assert_eq!(snapshot.last_stop_drained_steps, 11_128);
        assert!(!snapshot.timer_active);
    }

    #[test]
    fn stepper_execution_stats_tracks_per_channel_motion_and_stop_state() {
        let stats = StepperExecutionStats::<2>::new();

        stats.record_queued_segment(0, 12);
        stats.record_queued_segment(0, 8);
        stats.record_discarded_segment(1, 5);
        stats.record_loaded_segment(0, 7);
        stats.record_executed_step(0, true, 1200);
        stats.record_executed_step(0, false, 1250);
        stats.record_queue_empty(0);
        stats.record_missed_deadline(1);
        stats.record_reset(1, 2, 44);
        stats.record_physical_stop(1, StepperEventReason::TrsyncStop, 6, 9, 3, 77);

        let channel0 = stats
            .snapshot(0, -11, true, 4)
            .expect("channel 0 snapshot exists");
        assert_eq!(channel0.channel, 0);
        assert_eq!(channel0.position, -11);
        assert_eq!(channel0.queued_segments, 2);
        assert_eq!(channel0.queued_steps, 20);
        assert_eq!(channel0.loaded_segments, 1);
        assert_eq!(channel0.loaded_steps, 7);
        assert_eq!(channel0.executed_pos_steps, 1);
        assert_eq!(channel0.executed_neg_steps, 1);
        assert_eq!(channel0.queue_empty_count, 1);
        assert!(channel0.timer_active);
        assert_eq!(channel0.queue_len, 4);

        let channel1 = stats
            .snapshot(1, 33, false, 0)
            .expect("channel 1 snapshot exists");
        assert_eq!(channel1.channel, 1);
        assert_eq!(channel1.position, 33);
        assert_eq!(channel1.queued_segments, 0);
        assert_eq!(channel1.discarded_segments, 1);
        assert_eq!(channel1.discarded_steps, 5);
        assert_eq!(channel1.missed_deadline_count, 1);
        assert_eq!(channel1.reset_count, 1);
        assert_eq!(channel1.reset_drained_segments, 2);
        assert_eq!(channel1.reset_drained_steps, 44);
        assert_eq!(channel1.stop_count, 1);
        assert_eq!(channel1.stop_drained_segments, 3);
        assert_eq!(channel1.stop_drained_steps, 77);
        assert_eq!(
            channel1.last_stop_reason,
            StepperEventReason::TrsyncStop as u8
        );
        assert_eq!(channel1.last_stop_remaining_events, 6);
        assert_eq!(channel1.last_stop_queue_len, 9);
        assert_eq!(channel1.last_stop_drained_segments, 3);
        assert_eq!(channel1.last_stop_drained_steps, 77);
        assert!(!channel1.timer_active);
        assert_eq!(channel1.queue_len, 0);

        assert!(stats.snapshot(2, 0, false, 0).is_none());
        stats.record_queued_segment(2, 99);
        assert!(stats.snapshot(2, 0, false, 0).is_none());
    }

    #[test]
    fn stepper_execution_stats_re_latches_per_activation_timing() {
        let stats = StepperExecutionStats::<1>::new();

        stats.record_timer_activation(0, 1000);
        stats.record_loaded_segment_timing(0, 3, 1010, 1030, 1040);
        stats.record_loaded_segment_timing(0, 4, 1020, 1050, 1060);
        stats.record_executed_step(0, true, 1070);
        stats.record_executed_step(0, true, 1080);

        let first = stats
            .snapshot(0, 0, true, 0)
            .expect("first activation snapshot exists");
        assert_eq!(first.activation_count, 1);
        assert_eq!(first.last_activation_clock, 1000);
        assert_eq!(first.loaded_segments, 2);
        assert_eq!(first.loaded_steps, 7);
        assert_eq!(first.executed_pos_steps, 2);
        assert_eq!(first.first_load_now, 1010);
        assert_eq!(first.first_load_scheduled, 1030);
        assert_eq!(first.first_load_compare, 1040);
        assert_eq!(first.first_load_lead_ticks, 20);
        assert_eq!(first.first_load_compare_delay_ticks, 30);
        assert_eq!(first.first_step_clock, 1070);
        assert_eq!(first.first_step_delay_ticks, 70);

        stats.record_timer_activation(0, 2000);
        let reset = stats
            .snapshot(0, 0, true, 0)
            .expect("reset activation snapshot exists");
        assert_eq!(reset.activation_count, 2);
        assert_eq!(reset.last_activation_clock, 2000);
        assert_eq!(reset.first_load_now, 0);
        assert_eq!(reset.first_load_scheduled, 0);
        assert_eq!(reset.first_load_compare, 0);
        assert_eq!(reset.first_load_lead_ticks, 0);
        assert_eq!(reset.first_load_compare_delay_ticks, 0);
        assert_eq!(reset.first_step_clock, 0);
        assert_eq!(reset.first_step_delay_ticks, 0);

        stats.record_loaded_segment_timing(0, 5, 2010, 2060, 2075);
        stats.record_executed_step(0, false, 2090);
        let second = stats
            .snapshot(0, 0, true, 0)
            .expect("second activation snapshot exists");
        assert_eq!(second.activation_count, 2);
        assert_eq!(second.loaded_segments, 3);
        assert_eq!(second.loaded_steps, 12);
        assert_eq!(second.executed_pos_steps, 2);
        assert_eq!(second.executed_neg_steps, 1);
        assert_eq!(second.first_load_now, 2010);
        assert_eq!(second.first_load_scheduled, 2060);
        assert_eq!(second.first_load_compare, 2075);
        assert_eq!(second.first_load_lead_ticks, 50);
        assert_eq!(second.first_load_compare_delay_ticks, 65);
        assert_eq!(second.first_step_clock, 2090);
        assert_eq!(second.first_step_delay_ticks, 90);
    }

    #[test]
    fn stepper_perf_snapshot_reports_dispatch_counters() {
        let snapshot = StepperPerfSnapshot {
            channel: 0,
            sample_count: 42,
            crit_count: 37,
            crit_max_cycles: 190_000,
            crit_max_site: 1,
            crit_over_10us: 9,
            crit_over_50us: 4,
            crit_over_100us: 2,
            crit_over_1000us: 1,
            queue_step_count: 23,
            queue_step_max_cycles: 175_000,
            tim5_activation_count: 203,
            tim5_irq_max_cycles: 2_400,
            tim5_dispatch_max_cycles: 2_100,
            tim5_dispatch_max_cycles_events: 5,
            tim5_events_max_per_irq: 3,
            tim5_event_count_total: 401,
            tim5_defer_count: 7,
            tim5_empty_count: 2,
            tim5_events_last_activation: 4,
            tim5_burst_cycles_per_event_max: 900,
            tim5_burst_cycles_per_event_max_cycles: 2_700,
            tim5_burst_cycles_per_event_max_events: 3,
            tim5_burst_cycles_per_event_floor3_max: 900,
            tim5_entry_latency_max_ticks: 512,
            tim5_pop_lateness_max_ticks: 128,
            scheduler_cycles_max: 777,
            scheduler_cycles_events_at_max: 4,
            scheduler_cycles_per_event_max: 333,
            scheduler_cycles_per_event_max_cycles: 999,
            scheduler_cycles_per_event_max_events: 3,
            scheduler_cycles_per_event_floor3_max: 333,
            scheduler_full_count: 2,
            stepper_load_lateness_max_ticks: 256,
            stepper_load_lateness_last_ticks: -32,
            build_trace_enabled: cfg!(feature = "trace"),
        };

        assert_eq!(snapshot.channel, 0);
        assert_eq!(snapshot.crit_max_cycles, 190_000);
        assert_eq!(snapshot.queue_step_max_cycles, 175_000);
        assert_eq!(snapshot.tim5_dispatch_max_cycles_events, 5);
        assert_eq!(snapshot.tim5_events_max_per_irq, 3);
        assert_eq!(snapshot.tim5_event_count_total, 401);
        assert_eq!(snapshot.tim5_activation_count, 203);
        assert_eq!(snapshot.tim5_defer_count, 7);
        assert_eq!(snapshot.tim5_empty_count, 2);
        assert_eq!(snapshot.tim5_events_last_activation, 4);
        assert_eq!(snapshot.tim5_burst_cycles_per_event_max, 900);
        assert_eq!(snapshot.tim5_burst_cycles_per_event_max_cycles, 2_700);
        assert_eq!(snapshot.tim5_burst_cycles_per_event_max_events, 3);
        assert_eq!(snapshot.tim5_burst_cycles_per_event_floor3_max, 900);
        assert_eq!(snapshot.tim5_entry_latency_max_ticks, 512);
        assert_eq!(snapshot.tim5_pop_lateness_max_ticks, 128);
        assert_eq!(snapshot.scheduler_cycles_max, 777);
        assert_eq!(snapshot.scheduler_cycles_events_at_max, 4);
        assert_eq!(snapshot.scheduler_cycles_per_event_max, 333);
        assert_eq!(snapshot.scheduler_cycles_per_event_max_cycles, 999);
        assert_eq!(snapshot.scheduler_cycles_per_event_max_events, 3);
        assert_eq!(snapshot.scheduler_cycles_per_event_floor3_max, 333);
        assert_eq!(snapshot.scheduler_full_count, 2);
        assert_eq!(snapshot.stepper_load_lateness_max_ticks, 256);
        assert_eq!(snapshot.stepper_load_lateness_last_ticks, -32);
        assert_eq!(snapshot.build_trace_enabled, cfg!(feature = "trace"));
    }

    #[test]
    fn stepper_perf_event_carries_reason_clock_and_snapshot() {
        let snapshot = StepperPerfSnapshot {
            channel: 0,
            sample_count: 42,
            crit_count: 37,
            crit_max_cycles: 190_000,
            crit_max_site: 1,
            crit_over_10us: 9,
            crit_over_50us: 4,
            crit_over_100us: 2,
            crit_over_1000us: 1,
            queue_step_count: 23,
            queue_step_max_cycles: 175_000,
            tim5_activation_count: 203,
            tim5_irq_max_cycles: 2_400,
            tim5_dispatch_max_cycles: 2_100,
            tim5_dispatch_max_cycles_events: 5,
            tim5_events_max_per_irq: 3,
            tim5_event_count_total: 401,
            tim5_defer_count: 7,
            tim5_empty_count: 2,
            tim5_events_last_activation: 4,
            tim5_burst_cycles_per_event_max: 900,
            tim5_burst_cycles_per_event_max_cycles: 2_700,
            tim5_burst_cycles_per_event_max_events: 3,
            tim5_burst_cycles_per_event_floor3_max: 900,
            tim5_entry_latency_max_ticks: 512,
            tim5_pop_lateness_max_ticks: 128,
            scheduler_cycles_max: 777,
            scheduler_cycles_events_at_max: 4,
            scheduler_cycles_per_event_max: 333,
            scheduler_cycles_per_event_max_cycles: 999,
            scheduler_cycles_per_event_max_events: 3,
            scheduler_cycles_per_event_floor3_max: 333,
            scheduler_full_count: 2,
            stepper_load_lateness_max_ticks: 256,
            stepper_load_lateness_last_ticks: -32,
            build_trace_enabled: cfg!(feature = "trace"),
        };
        let event = StepperPerfDiagnosticEvent {
            reason: StepperEventReason::MissedDeadlineLoad,
            clock: 123_456,
            snapshot,
        };

        assert_eq!(event.reason, StepperEventReason::MissedDeadlineLoad);
        assert_eq!(event.clock, 123_456);
        assert_eq!(event.snapshot.crit_over_1000us, 1);
    }

    #[test]
    fn stepper_perf_counters_track_critical_sections_and_queue_site() {
        let counters = StepperPerfCounters::<520>::new();

        counters.record_critical_section(0, 999);
        counters.record_critical_section(1, 10_000);
        counters.record_critical_section(2, 100_000);

        let snapshot = counters.snapshot(3, false, true);
        assert_eq!(snapshot.channel, 3);
        assert_eq!(snapshot.sample_count, 3);
        assert_eq!(snapshot.crit_count, 3);
        assert_eq!(snapshot.crit_max_cycles, 100_000);
        assert_eq!(snapshot.crit_max_site, 2);
        assert_eq!(snapshot.crit_over_10us, 2);
        assert_eq!(snapshot.crit_over_50us, 1);
        assert_eq!(snapshot.crit_over_100us, 1);
        assert_eq!(snapshot.crit_over_1000us, 0);
        assert_eq!(snapshot.queue_step_count, 1);
        assert_eq!(snapshot.queue_step_max_cycles, 10_000);
        assert!(snapshot.build_trace_enabled);
    }

    #[test]
    fn stepper_perf_counters_track_tim5_irq_and_empty_samples() {
        let counters = StepperPerfCounters::<168>::new();

        counters.record_tim5_irq(500, 300, 3);
        counters.record_tim5_irq(900, 450, 0);
        counters.record_tim5_defer();
        counters.record_tim5_entry_latency(27);

        let snapshot = counters.snapshot(0, false, false);
        assert_eq!(snapshot.sample_count, 2);
        assert_eq!(snapshot.tim5_activation_count, 2);
        assert_eq!(snapshot.tim5_irq_max_cycles, 900);
        assert_eq!(snapshot.tim5_dispatch_max_cycles, 450);
        assert_eq!(snapshot.tim5_dispatch_max_cycles_events, 0);
        assert_eq!(snapshot.tim5_events_max_per_irq, 3);
        assert_eq!(snapshot.tim5_event_count_total, 3);
        assert_eq!(snapshot.tim5_defer_count, 1);
        assert_eq!(snapshot.tim5_empty_count, 1);
        assert_eq!(snapshot.tim5_events_last_activation, 0);
        assert_eq!(snapshot.tim5_burst_cycles_per_event_max, 100);
        assert_eq!(snapshot.tim5_burst_cycles_per_event_max_cycles, 300);
        assert_eq!(snapshot.tim5_burst_cycles_per_event_max_events, 3);
        assert_eq!(snapshot.tim5_burst_cycles_per_event_floor3_max, 100);
        assert_eq!(snapshot.tim5_entry_latency_max_ticks, 27);
        assert!(!snapshot.build_trace_enabled);
    }

    #[test]
    fn stepper_perf_counters_track_scheduler_and_load_lateness() {
        let counters = StepperPerfCounters::<168>::new();

        counters.record_scheduler_cycles(120, 2);
        counters.record_scheduler_cycles(330, 3);
        counters.record_scheduler_full();
        counters.record_scheduler_pop_lateness(44);
        counters.record_stepper_load_lateness(-7);
        counters.record_stepper_load_lateness(19);

        let snapshot = counters.snapshot(1, false, false);
        assert_eq!(snapshot.scheduler_cycles_max, 330);
        assert_eq!(snapshot.scheduler_cycles_events_at_max, 3);
        assert_eq!(snapshot.scheduler_cycles_per_event_max, 110);
        assert_eq!(snapshot.scheduler_cycles_per_event_max_cycles, 330);
        assert_eq!(snapshot.scheduler_cycles_per_event_max_events, 3);
        assert_eq!(snapshot.scheduler_cycles_per_event_floor3_max, 110);
        assert_eq!(snapshot.scheduler_full_count, 1);
        assert_eq!(snapshot.tim5_pop_lateness_max_ticks, 44);
        assert_eq!(snapshot.stepper_load_lateness_last_ticks, 19);
        assert_eq!(snapshot.stepper_load_lateness_max_ticks, 19);
    }

    #[test]
    fn stepper_perf_counters_clear_after_snapshot_when_requested() {
        let counters = StepperPerfCounters::<168>::new();

        counters.record_critical_section(1, 20_000);
        counters.record_tim5_irq(700, 600, 2);
        counters.record_scheduler_full();
        counters.record_stepper_load_lateness(11);

        let before_clear = counters.snapshot(7, true, true);
        assert_eq!(before_clear.channel, 7);
        assert_eq!(before_clear.crit_count, 1);
        assert_eq!(before_clear.tim5_activation_count, 1);
        assert_eq!(before_clear.scheduler_full_count, 1);
        assert_eq!(before_clear.stepper_load_lateness_max_ticks, 11);
        assert!(before_clear.build_trace_enabled);

        let after_clear = counters.snapshot(7, false, false);
        assert_eq!(after_clear.crit_count, 0);
        assert_eq!(after_clear.tim5_activation_count, 0);
        assert_eq!(after_clear.scheduler_full_count, 0);
        assert_eq!(after_clear.stepper_load_lateness_max_ticks, 0);
        assert!(!after_clear.build_trace_enabled);
    }
}
