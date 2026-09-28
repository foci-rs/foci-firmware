//! Pure trace sampling input construction and planning helpers.
//!
//! This module owns board-agnostic sampling inputs and planning decisions. It
//! does not own USB queues, timestamps, source identity, SPI reads, trace
//! counters, or frame encoding.

use crate::tmc_control::{TmcControlTraceState, TmcControlTraceTick};

/// Minimum remaining commissioning tick budget required before USB trace SPI
/// sampling may start.
pub const COMMISSION_TRACE_SAMPLE_MIN_BUDGET_US: u64 = 250;

/// Runtime-owned values copied into critical/current-path/control-detail trace
/// samples by board-local trace integrations.
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

/// Plan one commissioning closed-loop-entry trace sample.
pub const fn commissioning_trace_sample_plan(
    trace_configured: bool,
    past_deadline: bool,
    remaining_budget_us: u64,
    control_tick: u32,
) -> TraceSamplePlan {
    if !trace_configured {
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
        COMMISSION_TRACE_SAMPLE_MIN_BUDGET_US, TraceSampleInputs, TraceSamplePlan,
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
            commissioning_trace_sample_plan(false, false, 1_000, 9),
            TraceSamplePlan::SkipUnconfigured,
        );
        assert_eq!(
            commissioning_trace_sample_plan(true, true, 1_000, 9),
            TraceSamplePlan::DropForBudget,
        );
        assert_eq!(
            commissioning_trace_sample_plan(
                true,
                false,
                COMMISSION_TRACE_SAMPLE_MIN_BUDGET_US - 1,
                9,
            ),
            TraceSamplePlan::DropForBudget,
        );
        assert_eq!(
            commissioning_trace_sample_plan(true, false, COMMISSION_TRACE_SAMPLE_MIN_BUDGET_US, 9,),
            TraceSamplePlan::Sample(TraceSampleInputs::commissioning_idle(9)),
        );
        assert_eq!(
            commissioning_trace_sample_plan(
                true,
                false,
                COMMISSION_TRACE_SAMPLE_MIN_BUDGET_US + 1,
                10,
            ),
            TraceSamplePlan::Sample(TraceSampleInputs::commissioning_idle(10)),
        );
    }
}
