//! Trace sampling input construction and planning.

use crate::tmc_control::{TmcControlTraceState, TmcControlTraceTick};

/// Minimum remaining commissioning tick budget required before USB trace SPI
/// sampling may start.
pub const COMMISSION_TRACE_SAMPLE_MIN_BUDGET_US: u64 = 250;

/// Runtime values copied into trace samples.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct TraceSampleInputs {
    /// Last command rate in TMC position counts per PWM cycle.
    pub command_velocity_reference: i32,
    /// Last velocity feedforward offset emitted to trace sampling.
    pub cached_velocity_offset: i32,
    /// Velocity feedforward offset was clamped on the last tick.
    pub cached_velocity_clamped: bool,
    /// Last acceleration torque offset emitted to trace sampling.
    pub cached_accel_torque_offset: i32,
    /// Last acceleration feedforward gain emitted to trace sampling, in permille.
    pub accel_feedforward_gain_permille: u16,
    /// Last acceleration trace flags.
    pub accel_feedforward_flags: u8,
    /// Trace sample tick used for critical/current-path/control-detail cadence.
    pub control_tick: u32,
}

impl TraceSampleInputs {
    /// Build an all-zero commissioning sample input set for `control_tick`.
    pub const fn commissioning_idle(control_tick: u32) -> Self {
        Self {
            command_velocity_reference: 0,
            cached_velocity_offset: 0,
            cached_velocity_clamped: false,
            cached_accel_torque_offset: 0,
            accel_feedforward_gain_permille: 0,
            accel_feedforward_flags: 0,
            control_tick,
        }
    }

    /// Build trace sample inputs from runtime trace state.
    pub fn from_runtime_trace(
        trace_state: &TmcControlTraceState,
        trace_tick: TmcControlTraceTick,
    ) -> Self {
        Self {
            command_velocity_reference: trace_state.cached_velocity_reference,
            cached_velocity_offset: trace_state.cached_velocity_offset,
            cached_velocity_clamped: trace_state.cached_velocity_clamped,
            cached_accel_torque_offset: trace_state.cached_accel_torque_offset,
            accel_feedforward_gain_permille: trace_state.cached_accel_gain_permille,
            accel_feedforward_flags: trace_state.cached_accel_flags,
            control_tick: trace_tick.sample_tick,
        }
    }
}

/// Board-local action selected by the shared trace sampling planner.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum TraceSamplePlan {
    /// Trace is not configured; skip silently and do not count a drop.
    SkipUnconfigured,
    /// Trace is configured but the current tick has too little budget left.
    DropForBudget,
    /// Trace is configured and budget remains for SPI sampling.
    Sample(TraceSampleInputs),
}

/// Runtime trace emission mode, selected by the host over G-code.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
#[repr(u8)]
pub enum TraceMode {
    /// Control detail, status, metadata and events only. Boot value.
    Dormant = 0,
    /// Dormant plus critical frames at 1 kHz.
    Critical = 1,
    /// Critical plus current-path frames at their existing cadence.
    Diagnostic = 2,
}

impl TraceMode {
    /// Decode a wire byte; `None` for anything outside 0..=2.
    pub const fn from_wire(byte: u8) -> Option<Self> {
        match byte {
            0 => Some(Self::Dormant),
            1 => Some(Self::Critical),
            2 => Some(Self::Diagnostic),
            _ => None,
        }
    }

    /// Nominal critical rate for metadata: 1000 or 0.
    pub const fn critical_hz(self) -> u32 {
        match self {
            Self::Dormant => 0,
            Self::Critical | Self::Diagnostic => 1000,
        }
    }

    /// Whether the critical SPI sample runs this tick.
    pub const fn critical_sampling(self) -> bool {
        match self {
            Self::Dormant => false,
            Self::Critical | Self::Diagnostic => true,
        }
    }

    /// Whether current-path frames are enabled.
    pub const fn current_path(self) -> bool {
        match self {
            Self::Dormant | Self::Critical => false,
            Self::Diagnostic => true,
        }
    }
}

/// Plan one commissioning closed-loop-entry trace sample.
///
/// `Dormant` mode skips without counting a drop, regardless of budget.
pub const fn commissioning_trace_sample_plan(
    trace_configured: bool,
    mode: TraceMode,
    past_deadline: bool,
    remaining_budget_us: u64,
    control_tick: u32,
) -> TraceSamplePlan {
    if !trace_configured || !mode.critical_sampling() {
        return TraceSamplePlan::SkipUnconfigured;
    }
    if past_deadline || remaining_budget_us < COMMISSION_TRACE_SAMPLE_MIN_BUDGET_US {
        return TraceSamplePlan::DropForBudget;
    }
    TraceSamplePlan::Sample(TraceSampleInputs::commissioning_idle(control_tick))
}

#[cfg(test)]
mod tests {
    use super::{
        COMMISSION_TRACE_SAMPLE_MIN_BUDGET_US, TraceMode, TraceSampleInputs, TraceSamplePlan,
        commissioning_trace_sample_plan,
    };
    use crate::tmc_control::{TmcControlTraceState, TmcControlTraceTick};

    #[test]
    fn commissioning_idle_inputs_are_zero_except_control_tick() {
        assert_eq!(
            TraceSampleInputs::commissioning_idle(17),
            TraceSampleInputs {
                command_velocity_reference: 0,
                cached_velocity_offset: 0,
                cached_velocity_clamped: false,
                cached_accel_torque_offset: 0,
                accel_feedforward_gain_permille: 0,
                accel_feedforward_flags: 0,
                control_tick: 17,
            },
        );
    }

    #[test]
    fn runtime_trace_inputs_copy_cached_fields_and_sample_tick() {
        let state = TmcControlTraceState {
            control_tick: 999,
            cached_velocity_reference: -11,
            cached_velocity_offset: 22,
            cached_velocity_clamped: true,
            cached_accel_torque_offset: 44,
            cached_accel_gain_permille: 55,
            cached_accel_flags: 0xA5,
        };
        let tick = TmcControlTraceTick {
            sample_tick: 123,
            control_tick: 456,
            status_detail_due: true,
            metadata_due: true,
        };

        assert_eq!(
            TraceSampleInputs::from_runtime_trace(&state, tick),
            TraceSampleInputs {
                command_velocity_reference: -11,
                cached_velocity_offset: 22,
                cached_velocity_clamped: true,
                cached_accel_torque_offset: 44,
                accel_feedforward_gain_permille: 55,
                accel_feedforward_flags: 0xA5,
                control_tick: 123,
            },
        );
    }

    #[test]
    fn runtime_trace_inputs_do_not_use_status_control_tick() {
        let state = TmcControlTraceState::new();
        let tick = TmcControlTraceTick {
            sample_tick: 10,
            control_tick: 20,
            status_detail_due: false,
            metadata_due: false,
        };

        assert_eq!(
            TraceSampleInputs::from_runtime_trace(&state, tick).control_tick,
            10,
        );
    }

    #[test]
    fn commissioning_plan_preserves_budget_policy() {
        assert_eq!(
            commissioning_trace_sample_plan(false, TraceMode::Critical, false, 1_000, 9),
            TraceSamplePlan::SkipUnconfigured,
        );
        assert_eq!(
            commissioning_trace_sample_plan(true, TraceMode::Critical, true, 1_000, 9),
            TraceSamplePlan::DropForBudget,
        );
        assert_eq!(
            commissioning_trace_sample_plan(
                true,
                TraceMode::Critical,
                false,
                COMMISSION_TRACE_SAMPLE_MIN_BUDGET_US - 1,
                9,
            ),
            TraceSamplePlan::DropForBudget,
        );
        assert_eq!(
            commissioning_trace_sample_plan(
                true,
                TraceMode::Critical,
                false,
                COMMISSION_TRACE_SAMPLE_MIN_BUDGET_US,
                9
            ),
            TraceSamplePlan::Sample(TraceSampleInputs::commissioning_idle(9)),
        );
        assert_eq!(
            commissioning_trace_sample_plan(
                true,
                TraceMode::Critical,
                false,
                COMMISSION_TRACE_SAMPLE_MIN_BUDGET_US + 1,
                10,
            ),
            TraceSamplePlan::Sample(TraceSampleInputs::commissioning_idle(10)),
        );
    }

    #[test]
    fn from_wire_accepts_only_the_three_modes() {
        assert_eq!(TraceMode::from_wire(0), Some(TraceMode::Dormant));
        assert_eq!(TraceMode::from_wire(1), Some(TraceMode::Critical));
        assert_eq!(TraceMode::from_wire(2), Some(TraceMode::Diagnostic));
        assert_eq!(TraceMode::from_wire(3), None);
        assert_eq!(TraceMode::from_wire(0xff), None);
    }

    #[test]
    fn critical_sampling_is_off_only_in_dormant() {
        assert!(!TraceMode::Dormant.critical_sampling());
        assert!(TraceMode::Critical.critical_sampling());
        assert!(TraceMode::Diagnostic.critical_sampling());
        assert_eq!(TraceMode::Dormant.critical_hz(), 0);
        assert_eq!(TraceMode::Critical.critical_hz(), 1000);
        assert_eq!(TraceMode::Diagnostic.critical_hz(), 1000);
    }

    #[test]
    fn current_path_is_on_only_in_diagnostic() {
        assert!(!TraceMode::Dormant.current_path());
        assert!(!TraceMode::Critical.current_path());
        assert!(TraceMode::Diagnostic.current_path());
    }

    #[test]
    fn dormant_plan_skips_without_counting_a_drop() {
        let plan = commissioning_trace_sample_plan(true, TraceMode::Dormant, false, 10_000, 7);
        assert_eq!(plan, TraceSamplePlan::SkipUnconfigured);
    }

    #[test]
    fn critical_plan_samples_with_budget_and_drops_without() {
        assert!(matches!(
            commissioning_trace_sample_plan(true, TraceMode::Critical, false, 10_000, 7),
            TraceSamplePlan::Sample(_)
        ));
        assert_eq!(
            commissioning_trace_sample_plan(true, TraceMode::Critical, false, 10, 7),
            TraceSamplePlan::DropForBudget
        );
    }
}
