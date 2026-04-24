//! Board-agnostic post-processing for `CommissioningEngine::step()` results.
//!
//! Lifted from `openffboard-fw::tmc_control` per the design at
//! `docs/specs/2026-04-23-commission-dispatch-extraction-design.md`.
//!
//! Public surface: a single `drive_commission_result` function plus the
//! types it returns. The board calls this once per engine.step() result
//! and matches `CommissionOutcome` to fire RTIC primitives + emit replies.

// pub(crate) so sibling test modules (tmc_ops, commissioning, …) can
// import MockBackend + helpers via crate::commission_dispatch::test_support::*.
#[cfg(test)]
pub(crate) mod test_support;
#[cfg(test)]
mod tests;

use crate::autotune::PhaseId;
// CommissioningEngine lives in commissioning/mod.rs (the engine impl); the
// data types live in commissioning/types.rs.
use crate::commissioning::{
    CommissioningEngine,
    types::{
        AdcCalResult, CommissionResult, CommissioningBackend, EngineKind, OuterResult, StepResult,
    },
};
use crate::pid::InnerLoopGains;
use crate::selftest::SelftestResults;

/// Borrowed view of the per-engine task locals the dispatch needs to mutate.
///
/// The board's `tmc_control` task owns these as task locals; per-dispatch
/// it constructs a `DispatchState` so the firmware can mutate them in
/// place without copy-back.
pub struct DispatchState<'a> {
    pub calibrated: &'a mut bool,
    pub motor_enabled: &'a mut bool,
    pub stored_gains: &'a mut Option<InnerLoopGains>,
}

/// What the board should do after the dispatch returns.
pub enum CommissionOutcome {
    /// Engine wants more time. Board: trace_tick, Mono::delay(1ms), continue.
    Continue,
    /// A phase transition occurred. Board: update trace_buf phase (and
    /// conditionally start trace at ClosedLoopEntry for Commission), emit
    /// FociCommissionPhase reply, Mono::delay(1ms), continue.
    PhaseChanged {
        oid: u8,
        phase_id: PhaseId,
        engine_kind: EngineKind,
    },
    /// Engine reached a terminal state. Firmware has already executed all
    /// SPI restore, atomic stores, filter re-apply, and arm/disarm motor.
    /// Board: execute PostActions in order, emit reply.
    Terminal {
        reply: ReplyPayload,
        actions: PostActions,
    },
}

/// RTIC-bound primitives the board fires after a Terminal outcome.
/// Order: stop_trace -> stop_stepper -> spawn_arm_check -> emit reply.
#[derive(Debug, Clone, Copy, defmt::Format)]
pub struct PostActions {
    /// Stop the trace buffer. True for InnerFailed, OuterDone, OuterFailed,
    /// CommissionDone. False for CalibrateDone, SelftestDone.
    pub stop_trace: bool,
    /// Hard-stop the stepper across tasks. True for OuterFailed safety
    /// faults (Spi, Shutdown, SafetyEnvelopeViolation) and OuterDone
    /// restore SPI failure.
    pub stop_stepper: bool,
    /// Spawn arm_check to re-arm step-timer processing. True for
    /// CalibrateDone and CommissionDone (motor newly armed).
    pub spawn_arm_check: bool,
}

impl PostActions {
    pub const NONE: Self = Self {
        stop_trace: false,
        stop_stepper: false,
        spawn_arm_check: false,
    };
}

/// Wire-reply payload for the board to emit via `klipper_reply_from!`.
pub enum ReplyPayload {
    /// FociCommissionPhase: phase change reply (foci_commission failures
    /// during inner phases).
    CommissionPhase { oid: u8, phase: u8, status: u8 },
    /// FociCommissionResult: Stage 1 commission complete.
    CommissionResult {
        oid: u8,
        status: u8,
        result: CommissionResult,
    },
    /// FociTuneResult: Stage 2 tune complete (success or any failure mode).
    /// Zero-filled OuterResult on failure paths.
    TuneResult {
        oid: u8,
        status: u8,
        result: OuterResult,
    },
    /// FociCalibrateResult: ADC calibration complete.
    CalibrateResult {
        oid: u8,
        status: u8,
        result: AdcCalResult,
    },
    /// FociSelftestResult per stage + FociSelftestDone terminal.
    /// Board emits one Result message per populated stage, then Done.
    Selftest {
        oid: u8,
        results: SelftestResults,
        done_status: u8,
    },
}

/// Helper: zero-filled OuterResult for failure-path TuneResult emission.
fn zero_outer_result() -> OuterResult {
    OuterResult {
        status: crate::autotune::CommissionStatus::Failed(crate::autotune::CommissionError::Spi),
        velocity_p: 0,
        velocity_i: 0,
        velocity_limit: 0,
        position_p: 0,
        position_i: 0,
        velocity_filter_hz: 0,
        position_filter_hz: 0,
        flux_filter_hz: 0,
        torque_filter_hz: 0,
        j_eff: 0,
        b_eff: 0,
        warning_code: 0,
        mech_torque_step: 0,
        mech_fwd_accel: 0,
        mech_rev_accel: 0,
        mech_fwd_peak_velocity: 0,
        mech_rev_peak_velocity: 0,
        mech_fwd_travel: 0,
        mech_rev_travel: 0,
        synth_velocity_p: 0,
        synth_velocity_i: 0,
        synth_position_p: 0,
        synth_position_i: 0,
    }
}

/// Helper: zero-filled AdcCalResult for failure-path CalibrateResult emission.
fn zero_adc_cal_result() -> AdcCalResult {
    AdcCalResult {
        adc_i0: 0,
        adc_i1: 0,
        encoder_count: 0,
    }
}

/// Dispatch a single `StepResult` from `CommissioningEngine::step()` to the
/// appropriate outcome, executing any required SPI work, state mutations, and
/// atomic stores along the way.
///
/// # Parameters
/// - `result`: The `StepResult` returned by the most recent engine step.
/// - `engine`: Mutable reference to the `Option<CommissioningEngine>` task-local
///   slot. Terminal variants consume the engine (`engine.take()`); non-terminal
///   variants leave it in place.
/// - `backend`: SPI + GPIO backend for register access.
/// - `state`: Borrowed task-local mutable state (calibrated flag, motor-enabled
///   flag, stored PID gains).
/// - `pwm_maxcnt`: PWM period counter used when re-applying filters or limits
///   after a terminal outcome.
pub async fn drive_commission_result<B: CommissioningBackend>(
    result: StepResult,
    engine: &mut Option<CommissioningEngine>,
    backend: &mut B,
    state: &mut DispatchState<'_>,
    pwm_maxcnt: u16,
) -> CommissionOutcome {
    let _ = (state, pwm_maxcnt); // suppress unused warnings until all arms land
    match result {
        StepResult::Continue => CommissionOutcome::Continue,
        StepResult::PhaseChanged(phase_id) => {
            let engine_ref = engine
                .as_ref()
                .expect("engine slot is None on PhaseChanged");
            CommissionOutcome::PhaseChanged {
                oid: engine_ref.oid(),
                phase_id,
                engine_kind: engine_ref.kind(),
            }
        }
        StepResult::SelftestDone => {
            let mut consumed = engine.take().expect("engine slot None on SelftestDone");
            let oid = consumed.oid();
            let results = consumed.take_selftest_results();
            crate::state::tmc::COMMISSION_IN_FLIGHT
                .store(false, core::sync::atomic::Ordering::Relaxed);
            CommissionOutcome::Terminal {
                reply: ReplyPayload::Selftest {
                    oid,
                    results,
                    done_status: 0,
                },
                actions: PostActions::NONE,
            }
        }
        StepResult::InnerFailed(error) => {
            let mut consumed = engine.take().expect("engine slot None on InnerFailed");
            let oid = consumed.oid();
            let kind = consumed.kind();
            let selftest_results = consumed.take_selftest_results();
            crate::state::tmc::COMMISSION_IN_FLIGHT
                .store(false, core::sync::atomic::Ordering::Relaxed);
            crate::selftest::restore_safe_state_via_backend(backend).await;

            let reply = match kind {
                EngineKind::Calibrate => ReplyPayload::CalibrateResult {
                    oid,
                    status: error.status_code(),
                    result: zero_adc_cal_result(),
                },
                EngineKind::Selftest => ReplyPayload::Selftest {
                    oid,
                    results: selftest_results,
                    done_status: error.status_code(),
                },
                EngineKind::Commission | EngineKind::OuterTune => ReplyPayload::CommissionPhase {
                    oid,
                    phase: 0,
                    status: error.status_code(),
                },
            };
            CommissionOutcome::Terminal {
                reply,
                actions: PostActions {
                    stop_trace: true,
                    stop_stepper: false,
                    spawn_arm_check: false,
                },
            }
        }
        // Other arms added in Tasks 15-18.
        _ => todo!("implemented per-variant in Tasks 15-18"),
    }
}

// Suppress unused-helper warnings until the dispatch arms consume them.
#[allow(dead_code)]
const _: fn() = || {
    let _ = zero_outer_result;
};
