//! Structured diagnostic codes emitted before fatal shutdowns when transport is usable.

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
}
