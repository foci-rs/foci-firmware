//! PositionTune: two-bound position-P sweep commissioning phase.
//!
//! See `docs/specs/2026-09-13-foci-340-position-tune-two-bound-design.md` for
//! the full design. This module drives the position-P ladder over the
//! step-queue stimulus: `Predict` computes a config-derived start rung from
//! the tracking bound and the planned homing/motion strokes, `Stroke` runs
//! each rung's homing and motion legs through the trapezoid planner with
//! production feedforward on the motion leg, `AwaitRestore` waits for the
//! feedforward surfaces to settle, and `StandstillHold` captures the rung's
//! dither before `selection::decide` picks the lowest rung meeting both
//! bounds under the overshoot and dither guards.

pub(crate) mod bounds;
pub(crate) mod ceiling;
pub(crate) mod metrics;
pub(crate) mod selection;

use crate::autotune::{
    CommissionError, CommissionPhase, CommissionProfile, CommissionStatus, PositionTuneOutcome,
    TRANSIENT_LEN, TransientBuffer,
};
use crate::commissioning::OuterGainRollbackSnapshot;
use crate::commissioning::backend::CommissioningBackend;
use crate::commissioning::outer::{
    OuterMotionBudget, OuterPhase, OuterSafetyLimits, fail_outer_position_tune,
};
use crate::commissioning::schedule::next_grid_deadline;
use crate::commissioning::stimulus::trapezoid::{LegPlan, StrokeRequest, plan_leg, plan_stroke};
use crate::commissioning::types::{
    OuterResult, PositionTuneProvenance, StepResult, TuneProvenance,
};
use ceiling::{ceiling_trips, compute_k};
use foci_tmc4671::pid::{PidGainPair, PidIGain, PidPGain};
use foci_tmc4671::registers::{
    MODE_RAMP_MODE_MOTION, PID_ERROR_PID_POSITION_ERROR, PID_ERROR_PID_POSITION_ERROR_SUM,
    PID_POSITION_ACTUAL, PID_POSITION_P_POSITION_I, PID_POSITION_TARGET, PID_TORQUE_FLUX_TARGET,
    PID_VELOCITY_P_VELOCITY_I, PID_VELOCITY_TARGET, PidTorqueFluxTarget, Register, SubRegister,
};
use metrics::{LegMetrics, StrokeTiming};
use selection::{RungDecision, RungMeasurement, next_rung};

/// Standstill-dither observations per rung (spec: `M = 5`, odd so the 3-of-5
/// majority vote can never tie).
pub(crate) const STANDSTILL_CAPTURE_COUNT: u8 = 5;

/// `STANDSTILL_HOLD_US` / `STANDSTILL_SAMPLE_PERIOD_US`: a 100 ms hold at
/// 1 kHz, short enough that five captures per rung plus the two stroke legs
/// stay within a reasonable per-rung budget, long enough to observe multiple
/// dither cycles.
pub(crate) const STANDSTILL_HOLD_US: u32 = 100_000;
pub(crate) const STANDSTILL_SAMPLE_PERIOD_US: u32 = 1_000;

/// Ramped-stroke repeats captured per rung, per leg kind; `selection::decide`
/// acts on their per-field medians.
pub(crate) const RAMPED_STROKE_REPEATS: u8 = 3;

/// Lead time between arming the step-queue stimulus and the stroke's first
/// queued step, so the capture window's start lines up with the first
/// commanded step rather than with when the segment was merely queued.
pub(crate) const STROKE_LEAD_US: u32 = 20_000;

/// Shortest cruise a planned stroke leg must keep (see
/// `stimulus::trapezoid::StrokeRequest::min_cruise_us`).
pub(crate) const MIN_CRUISE_US: u32 = 50_000;

/// Longest wait for `stimulus_feedforward_idle()` to report every
/// feedforward surface restored after a motion leg, before the phase reports
/// `RestorationTimeout`.
pub(crate) const RESTORE_WAIT_US: u32 = 50_000;

/// Poll grid for the commission-unit tick while a stroke or restore wait is
/// active. Samples are pushed only on the derived sample period, not on
/// every poll.
pub(crate) const STROKE_POLL_US: u32 = 1_000;

/// `MODE_RAMP_MODE_MOTION` value for position mode with the advanced PI
/// representation bit set (bit 31), matching `integral_sweep.rs`'s
/// `OUTER_VELOCITY_MODE` convention.
const POSITION_TUNE_POSITION_MODE: u32 = 0x8000_0003;

/// `2*pi` as a Q16.16 fixed-point integer, the same scaled-integer
/// convention `ceiling::compute_k`'s `k_q16` uses. `6.283185307 * 65536 =
/// 411774.86`, so this is exact to within 1.4e-7 relative -- far below the
/// 1 Hz quantization of the field it feeds.
const TAU_Q16: i64 = 411_775;

/// Which directional leg of a rung's stroke is being driven.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) enum StrokeKind {
    /// The homing-speed leg: no feedforward, the loop tracks entirely on P.
    Homing,
    /// The machine-ceiling leg: production velocity feedforward opted in.
    Motion,
}

/// Which sub-step of one rung's evaluation is in progress.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) enum PositionTuneStep {
    /// Computing the tracking bound, the planned strokes, and the start rung
    /// (runs once, before the first rung; no capture).
    Predict,
    /// Driving one rung's homing or motion leg through the step-queue
    /// stimulus, `repeat` of `RAMPED_STROKE_REPEATS`.
    Stroke {
        rung: PidPGain,
        kind: StrokeKind,
        repeat: u8,
    },
    /// Waiting for the motion leg's feedforward surfaces to report idle
    /// before the standstill dither captures start.
    AwaitRestore { rung: PidPGain, since_us: u64 },
    /// Capturing the current rung's `STANDSTILL_CAPTURE_COUNT` standstill
    /// dither observations.
    StandstillHold { rung: PidPGain, capture_index: u8 },
    /// Terminal: an accepted P.
    Done,
}

/// Phase-local state for the two-bound position-P ladder sweep.
#[derive(Debug, defmt::Format)]
pub(crate) struct PositionTuneState {
    pub(crate) step: PositionTuneStep,
    pub(crate) k_q16: i64,
    pub(crate) bound_units: u32,
    pub(crate) predicted_start: PidPGain,
    pub(crate) homing_leg: Option<LegPlan>,
    pub(crate) motion_leg: Option<LegPlan>,
    /// The active stroke's emitted boundaries, captured once per stroke from
    /// the planned profile so the capture windows follow the schedule the
    /// step-queue ISR actually runs.
    pub(crate) stroke_timing: Option<StrokeTiming>,
    pub(crate) ff_permille: u32,
    pub(crate) dither_captures: [i32; STANDSTILL_CAPTURE_COUNT as usize],
    pub(crate) homing_repeats: [Option<LegMetrics>; RAMPED_STROKE_REPEATS as usize],
    pub(crate) motion_repeats: [Option<LegMetrics>; RAMPED_STROKE_REPEATS as usize],
    /// The rung that produced the running-minimum motion overshoot, and
    /// that overshoot itself; `None` before the first `Advance`.
    pub(crate) min_overshoot_so_far: Option<(PidPGain, u32)>,
    pub(crate) rungs_measured: u8,
    pub(crate) last_measurement: Option<RungMeasurement>,
    /// Feedforward paths live on the last admitted motion stroke (from
    /// `backend.feedforward_path_flags()` at `arm_stroke` admission); zero
    /// until a motion stroke has been admitted.
    pub(crate) feedforward_paths_active: u8,
    /// Live diagnostic evidence, rebuilt at the end of `predict`, after
    /// every stroke admission, and in `resolve_completed_rung`, so every
    /// failure exit can report the last rung actually measured instead of a
    /// zeroed block.
    pub(crate) provenance: PositionTuneProvenance,
    /// The accepted rung, retained so the terminal is idempotent if the
    /// engine steps a completed phase again.
    accepted: Option<RungMeasurement>,
    /// Outer gains as they stood at Stage 2 entry.
    ///
    /// This phase tunes position P and nothing else, so every other field of
    /// the terminal `OuterResult` is reported from here. The `OuterDone`
    /// restore path stores `velocity_limit` and the four biquad cutoffs to the
    /// channel config unconditionally, with no zero guard, so a default-filled
    /// result would clamp the velocity controller's input to zero and leave the
    /// machine unable to move after a *successful* tune.
    entry: OuterGainRollbackSnapshot,
    /// Velocity P as read from the chip at phase entry: the gain the
    /// ceiling's `k` is actually measured against.
    velocity_p: PidPGain,
    /// Commissioning profile, read once at phase entry and used to derive
    /// the tracking-bound margin.
    profile: CommissionProfile,
    sample_period_us: u32,
    /// False when a sub-step has been entered but its capture is not armed
    /// yet, so the next tick runs that sub-step's setup instead of sampling.
    capture_running: bool,
    next_sample_us: u64,
    next_poll_us: u64,
    deadline_us: u64,
}

impl PositionTuneState {
    /// Seed state for a fresh sweep.
    pub(crate) const fn new(entry: OuterGainRollbackSnapshot, profile: CommissionProfile) -> Self {
        Self {
            step: PositionTuneStep::Predict,
            k_q16: 0,
            bound_units: 0,
            predicted_start: PidPGain::ZERO,
            homing_leg: None,
            motion_leg: None,
            stroke_timing: None,
            ff_permille: 0,
            dither_captures: [0; STANDSTILL_CAPTURE_COUNT as usize],
            homing_repeats: [None; RAMPED_STROKE_REPEATS as usize],
            motion_repeats: [None; RAMPED_STROKE_REPEATS as usize],
            min_overshoot_so_far: None,
            rungs_measured: 0,
            last_measurement: None,
            feedforward_paths_active: 0,
            provenance: PositionTuneProvenance {
                predicted_start_p: 0,
                last_rung_p: 0,
                min_overshoot_p: 0,
                min_overshoot_units: 0,
                bound_units: 0,
                homing_speed_mrev_s: 0,
                motion_speed_mrev_s: 0,
                homing_peak_abs_units: 0,
                motion_cruise_mean_abs_units: 0,
                motion_overshoot_units: 0,
                motion_cruise_rms_units: 0,
                nominal_bandwidth_hz: 0,
                dither_margin_milli: 0,
                rungs_measured: 0,
                stimulus_feedforward_paths: 0,
            },
            accepted: None,
            entry,
            velocity_p: PidPGain::ZERO,
            profile,
            sample_period_us: 0,
            capture_running: false,
            next_sample_us: 0,
            next_poll_us: 0,
            deadline_us: 0,
        }
    }
}

/// A failure this phase reports: either an engine-level error shared with
/// every other outer phase, or one of this phase's own diagnostic outcomes
/// behind `CommissionError::PositionTuneFailed`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
enum PositionTuneFault {
    Engine(CommissionError),
    Outcome(PositionTuneOutcome),
}

impl From<CommissionError> for PositionTuneFault {
    fn from(error: CommissionError) -> Self {
        Self::Engine(error)
    }
}

impl From<PositionTuneOutcome> for PositionTuneFault {
    fn from(outcome: PositionTuneOutcome) -> Self {
        Self::Outcome(outcome)
    }
}

/// Halt the step-queue stimulus and clear the feedforward opt-in.
///
/// Must be the first effect at every site that calls it: the step timer must
/// be halted and the queue drained before that caller zeroes excitation or
/// disables the driver, so no queued segment can keep pulsing a channel
/// whose driver has just been cut. Called first in `fail_position_tune`;
/// first in each of `arm_stroke`'s three stroke-arming failure paths (queue
/// overflow, enqueue refusal, arm rejection); at the top of all three of
/// `dispatch.rs`'s outer-safety exits (invalid-budget, safety-fault,
/// check-error); unconditionally at the top of `CommissioningEngine::step`'s
/// shutdown branch, which bypasses `step_inner`/`step_outer` (and so every
/// one of the sites above) entirely; unconditionally at the top of
/// `apply_shutdown_safe_state`, both boards' shared shutdown-loop helper;
/// and, in `TmcControlRuntime::drive_operator_cancel`, first in each of its
/// two active-cancellation branches (a pending restore, or an installed
/// commission engine) -- never called at all when an operator cancel finds
/// neither, so an idle cancel does not stop unrelated in-flight motion.
pub(crate) fn abort_stimulus<B: CommissioningBackend>(backend: &mut B) {
    backend.stimulus_stop();
    backend.set_stimulus_feedforward(false);
}

/// Zero excitation, report the outcome, and transition the phase to a terminal
/// failure state.
///
/// The entry-gain snapshot restore itself runs in
/// `commission_dispatch::restore_machine`, which every `StepResult::OuterFailed`
/// already drives; this is the same contract the other outer phases use. One
/// of this phase's own outcomes additionally retains that outcome on the
/// terminal phase, so the reply can report *which* sweep failure occurred --
/// `CommissionError::PositionTuneFailed` alone collapses all of them into one
/// status code.
async fn fail_position_tune<B: CommissioningBackend>(
    phase: &mut OuterPhase,
    backend: &mut B,
    fault: PositionTuneFault,
) -> StepResult {
    abort_stimulus(backend);
    let _ = backend.write_register(PID_VELOCITY_TARGET, 0).await;
    let _ = backend.write_register(PID_TORQUE_FLUX_TARGET, 0).await;
    let _ = backend.write_register(MODE_RAMP_MODE_MOTION, 0).await;
    let provenance = match &*phase {
        OuterPhase::PositionTune { state, .. } => state.provenance,
        _ => PositionTuneProvenance::default(),
    };
    match fault {
        PositionTuneFault::Engine(error) => {
            fail_outer_position_tune(phase, error, None, provenance)
        }
        PositionTuneFault::Outcome(outcome) => {
            defmt::warn!(
                "position tune failed: outcome={} wire_code={}",
                outcome,
                outcome.to_wire_code(),
            );
            fail_outer_position_tune(
                phase,
                CommissionError::PositionTuneFailed,
                Some(outcome),
                provenance,
            )
        }
    }
}

async fn read_sub_register<B: CommissioningBackend>(
    backend: &mut B,
    sub: SubRegister,
) -> Result<u32, CommissionError> {
    backend
        .write_register(sub.addr, u32::from(sub.sub_addr))
        .await
        .map_err(|_| CommissionError::Spi)?;
    backend
        .read_register(sub.data)
        .await
        .map_err(|_| CommissionError::Spi)
}

async fn write_sub_register<B: CommissioningBackend>(
    backend: &mut B,
    sub: SubRegister,
    value: u32,
) -> Result<(), CommissionError> {
    backend
        .write_register(sub.addr, u32::from(sub.sub_addr))
        .await
        .map_err(|_| CommissionError::Spi)?;
    backend
        .write_register(sub.data, value)
        .await
        .map_err(|_| CommissionError::Spi)
}

async fn write_register<B: CommissioningBackend>(
    backend: &mut B,
    register: Register,
    value: u32,
) -> Result<(), CommissionError> {
    backend
        .write_register(register, value)
        .await
        .map_err(|_| CommissionError::Spi)
}

async fn read_capture<B: CommissioningBackend>(
    backend: &mut B,
    register: Register,
) -> Result<u32, CommissionError> {
    backend
        .read_capture_register(register)
        .await
        .map_err(|_| CommissionError::Spi)
}

fn start_capture(
    state: &mut PositionTuneState,
    now_us: u64,
    period_us: u32,
    window_us: u32,
) -> Result<(), CommissionError> {
    state.next_sample_us = now_us
        .checked_add(u64::from(period_us))
        .ok_or(CommissionError::ScheduleDeadlineOverflow)?;
    state.deadline_us = now_us
        .checked_add(u64::from(window_us))
        .ok_or(CommissionError::ScheduleDeadlineOverflow)?;
    state.capture_running = true;
    Ok(())
}

fn advance_sample(
    state: &mut PositionTuneState,
    period_us: u32,
    now_us: u64,
) -> Result<u64, CommissionError> {
    let next = next_grid_deadline(state.next_sample_us, u64::from(period_us), now_us)?;
    state.next_sample_us = next.min(state.deadline_us);
    Ok(state.next_sample_us)
}

/// Read the velocity loop's already-selected gains, which the ceiling's `k`
/// constant is derived from.
async fn read_velocity_gains<B: CommissioningBackend>(
    backend: &mut B,
) -> Result<PidGainPair, PositionTuneFault> {
    let raw = backend
        .read_register(PID_VELOCITY_P_VELOCITY_I)
        .await
        .map_err(|_| PositionTuneFault::Engine(CommissionError::Spi))?;
    PidGainPair::try_from_register(raw)
        .map_err(|_| PositionTuneFault::Engine(CommissionError::PidGainInvalid))
}

/// Compute the tracking bound, plan both stroke legs, and derive the
/// closed-form start rung -- no capture, runs once before the first rung.
async fn predict<B: CommissioningBackend>(
    state: &mut PositionTuneState,
    safety: &mut OuterSafetyLimits,
    backend: &mut B,
) -> Result<(), PositionTuneFault> {
    if backend.is_shutdown_requested() {
        return Err(CommissionError::Shutdown.into());
    }
    if backend.stimulus_step_timer_hz() == 0 || backend.stimulus_is_trigger_blocked() {
        return Err(PositionTuneOutcome::ConfigurationInvalid.into());
    }
    let counts_per_rev = backend.mechanical_encoder_quadrature_counts_per_rev();
    safety
        .configure_velocity_sweep_encoder_resolution(counts_per_rev)
        .map_err(|_| CommissionError::EncoderFault)?;
    let encoder_raw = backend
        .try_read_mechanical_encoder_position_counts()
        .await
        .map_err(|_| CommissionError::Spi)?;
    safety
        .set_encoder_baseline(encoder_raw)
        .map_err(|_| CommissionError::EncoderFault)?;
    safety.torque_cap = u16::from(backend.current_limit());
    let velocity_gains = read_velocity_gains(backend).await?;
    state.velocity_p = velocity_gains.p;
    // `compute_k` takes encoder pulses per revolution; the safety domain and
    // the backend both work in quadrature counts, which are four per pulse.
    state.k_q16 = compute_k(counts_per_rev / 4, velocity_gains.p)?;

    let budget = safety.budget;
    state.bound_units = bounds::bound_units(
        backend.stall_threshold_units(),
        bounds::margin_permille(state.profile),
    )?;
    state.ff_permille = backend.velocity_feedforward_gain_permille();
    let steps_per_rev = 65_536
        / backend
            .configured_physical_step_width()
            .filter(|width| *width > 0)
            .ok_or(PositionTuneOutcome::ConfigurationInvalid)?;
    let excursion_mrev = budget.max_stroke_travel_mrev / 2;
    let request = |speed_mrev_s: u32| StrokeRequest {
        speed_mrev_s,
        accel_mrev_s2: budget.max_accel_mrev_s2,
        excursion_mrev,
        steps_per_rev,
        step_timer_hz: backend.stimulus_step_timer_hz(),
        min_cruise_us: MIN_CRUISE_US,
    };
    let homing = plan_leg(&request(budget.homing_speed_mrev_s))
        .map_err(|_| PositionTuneOutcome::ConfigurationInvalid)?;
    let motion = plan_leg(&request(budget.machine_velocity_ceiling_mrev_s))
        .map_err(|_| PositionTuneOutcome::ConfigurationInvalid)?;
    state.predicted_start = bounds::start_rung(
        homing.speed_mrev_s,
        motion.speed_mrev_s,
        state.ff_permille,
        state.bound_units,
    )?;
    state.homing_leg = Some(homing);
    state.motion_leg = Some(motion);
    safety.arm_safety_observation(backend.elapsed_us())?;
    state.step = PositionTuneStep::Stroke {
        rung: state.predicted_start,
        kind: StrokeKind::Homing,
        repeat: 0,
    };
    refresh_provenance(state);
    defmt::info!(
        "position tune predict: bound={} start_p={} homing={}mrev/s motion={}mrev/s ff={} predicted_homing_error={} predicted_motion_error={}",
        state.bound_units,
        state.predicted_start.raw(),
        homing.speed_mrev_s,
        motion.speed_mrev_s,
        state.ff_permille,
        bounds::predicted_error_units(homing.speed_mrev_s, 0, state.predicted_start.raw()),
        bounds::predicted_error_units(
            motion.speed_mrev_s,
            state.ff_permille,
            state.predicted_start.raw()
        ),
    );
    Ok(())
}

/// Build the `StrokeRequest` a leg at `speed_mrev_s` is planned from: every
/// field but the speed comes straight from `budget`, `steps_per_rev`, and
/// `step_timer_hz`. Both this phase's own stroke arm and the dispatch-side
/// duration envelope (`commission_dispatch::position_tune`) call this, so
/// the disclosed worst-case/nominal durations and the actual stroke agree on
/// the same request shape.
pub(crate) fn stroke_request(
    speed_mrev_s: u32,
    budget: &OuterMotionBudget,
    steps_per_rev: u32,
    step_timer_hz: u32,
) -> StrokeRequest {
    StrokeRequest {
        speed_mrev_s,
        accel_mrev_s2: budget.max_accel_mrev_s2,
        excursion_mrev: budget.max_stroke_travel_mrev / 2,
        steps_per_rev,
        step_timer_hz,
        min_cruise_us: MIN_CRUISE_US,
    }
}

/// Install one rung's position P, plan its stroke, and queue it on the
/// step-queue stimulus.
async fn arm_stroke<B: CommissioningBackend>(
    state: &mut PositionTuneState,
    safety: &mut OuterSafetyLimits,
    workspace: &mut TransientBuffer,
    backend: &mut B,
    rung: PidPGain,
    kind: StrokeKind,
) -> Result<(), PositionTuneFault> {
    let leg = match kind {
        StrokeKind::Homing => state.homing_leg,
        StrokeKind::Motion => state.motion_leg,
    }
    .expect("legs are planned before the first stroke");
    write_register(
        backend,
        PID_POSITION_P_POSITION_I,
        PidGainPair::new(rung, PidIGain::ZERO).register_value(),
    )
    .await?;
    write_sub_register(backend, PID_ERROR_PID_POSITION_ERROR_SUM, 0).await?;
    write_register(backend, MODE_RAMP_MODE_MOTION, POSITION_TUNE_POSITION_MODE).await?;
    // A soft position loop overshoots the slower homing leg's cruise speed at
    // the end of its ramp by about as much as the per-leg margin allows, so
    // both legs share the faster leg's runaway bound.
    let envelope_speed_mrev_s = [state.homing_leg, state.motion_leg]
        .into_iter()
        .flatten()
        .map(|planned| planned.speed_mrev_s)
        .max()
        .unwrap_or(leg.speed_mrev_s);
    safety.configure_velocity_envelope(envelope_speed_mrev_s)?;

    backend.stimulus_stop();
    if backend.stimulus_is_trigger_blocked() {
        return Err(PositionTuneOutcome::ConfigurationInvalid.into());
    }
    let lead_ticks = (u64::from(backend.stimulus_step_timer_hz()) * u64::from(STROKE_LEAD_US)
        / 1_000_000) as u32;
    let base_clock = backend.stimulus_step_clock_now().wrapping_add(lead_ticks);
    backend
        .stimulus_reset_step_clock(base_clock)
        .map_err(|_| PositionTuneOutcome::StimulusRejected)?;
    backend.set_stimulus_feedforward(matches!(kind, StrokeKind::Motion));
    if matches!(kind, StrokeKind::Motion) {
        state.feedforward_paths_active = backend.feedforward_path_flags();
    }

    let steps_per_rev = 65_536
        / backend
            .configured_physical_step_width()
            .filter(|width| *width > 0)
            .ok_or(PositionTuneOutcome::ConfigurationInvalid)?;
    let request = stroke_request(
        leg.speed_mrev_s,
        &safety.budget,
        steps_per_rev,
        backend.stimulus_step_timer_hz(),
    );
    let profile = plan_stroke(&request).map_err(|_| PositionTuneOutcome::StimulusRejected)?;
    if profile.segments.len() > backend.stimulus_queue_free() {
        defmt::warn!(
            "position tune stroke: {} segments exceed {} free queue slots",
            profile.segments.len(),
            backend.stimulus_queue_free()
        );
        abort_stimulus(backend);
        return Err(PositionTuneOutcome::StimulusRejected.into());
    }
    state.stroke_timing = Some(StrokeTiming::from_profile(&profile));
    let mut current_dir = None;
    for segment in profile.segments.iter() {
        if current_dir != Some(segment.forward) {
            backend.stimulus_set_direction(segment.forward);
            current_dir = Some(segment.forward);
        }
        if let Err(error) = backend.stimulus_queue_segment(segment.step_move()) {
            defmt::warn!("position tune stroke: enqueue refused {}", error);
            abort_stimulus(backend);
            return Err(PositionTuneOutcome::StimulusRejected.into());
        }
    }
    if backend.stimulus_arm().is_err() {
        abort_stimulus(backend);
        return Err(PositionTuneOutcome::StimulusRejected.into());
    }

    let started_us = backend.elapsed_us() + u64::from(STROKE_LEAD_US);
    state.sample_period_us = metrics::sample_period_us(profile.stroke_us());
    safety.begin_primitive(backend.elapsed_us())?;
    workspace.reset(state.sample_period_us as u16);
    start_capture(
        state,
        started_us,
        state.sample_period_us,
        profile.stroke_us(),
    )?;
    state.next_poll_us = backend.elapsed_us() + u64::from(STROKE_POLL_US);
    refresh_provenance(state);
    Ok(())
}

/// Peak-to-peak excursion of one standstill hold's torque-command capture.
fn standstill_excursion(workspace: &TransientBuffer) -> Result<i32, PositionTuneOutcome> {
    let samples = workspace.samples();
    if samples.is_empty() {
        return Err(PositionTuneOutcome::CaptureEmpty);
    }
    let mut minimum = i64::MAX;
    let mut maximum = i64::MIN;
    for &raw in samples {
        let value = i64::from(raw as i32);
        minimum = minimum.min(value);
        maximum = maximum.max(value);
    }
    Ok(i32::try_from(maximum - minimum).unwrap_or(i32::MAX))
}

/// Arm one standstill hold: the position target is a fresh
/// `PID_POSITION_ACTUAL` read (the stroke ended at rest wherever it did) and
/// the velocity target is zero, so the only torque command left is the
/// loop's own dither.
async fn arm_standstill_hold<B: CommissioningBackend>(
    state: &mut PositionTuneState,
    safety: &mut OuterSafetyLimits,
    workspace: &mut TransientBuffer,
    backend: &mut B,
) -> Result<(), PositionTuneFault> {
    write_register(backend, PID_VELOCITY_TARGET, 0).await?;
    let position_actual = read_capture(backend, PID_POSITION_ACTUAL).await?;
    write_register(backend, PID_POSITION_TARGET, position_actual).await?;

    let started_us = backend.elapsed_us();
    safety.begin_primitive(started_us)?;
    workspace.reset(STANDSTILL_SAMPLE_PERIOD_US as u16);
    start_capture(
        state,
        started_us,
        STANDSTILL_SAMPLE_PERIOD_US,
        STANDSTILL_HOLD_US,
    )?;
    Ok(())
}

/// Per-field median of three repeats' `LegMetrics`.
///
/// Panics if any of the three slots is still `None`: the caller only calls
/// this once all three repeats have completed.
fn median_metrics(repeats: &[Option<LegMetrics>; RAMPED_STROKE_REPEATS as usize]) -> LegMetrics {
    let filled: [LegMetrics; RAMPED_STROKE_REPEATS as usize] =
        repeats.map(|entry| entry.expect("every repeat slot is filled before the median is read"));
    LegMetrics {
        cruise_mean_abs_units: metrics::median3([
            filled[0].cruise_mean_abs_units,
            filled[1].cruise_mean_abs_units,
            filled[2].cruise_mean_abs_units,
        ]),
        peak_abs_units: metrics::median3([
            filled[0].peak_abs_units,
            filled[1].peak_abs_units,
            filled[2].peak_abs_units,
        ]),
        overshoot_units: metrics::median3([
            filled[0].overshoot_units,
            filled[1].overshoot_units,
            filled[2].overshoot_units,
        ]),
        cruise_rms_units: metrics::median3([
            filled[0].cruise_rms_units,
            filled[1].cruise_rms_units,
            filled[2].cruise_rms_units,
        ]),
    }
}

/// Fold one completed rung's medianed homing/motion metrics and dither
/// captures into a decision.
fn resolve_completed_rung(
    state: &mut PositionTuneState,
    rung: PidPGain,
) -> Result<(), PositionTuneFault> {
    let homing = median_metrics(&state.homing_repeats);
    let motion = median_metrics(&state.motion_repeats);
    let (ceiling_tripped, ceiling_ratios_milli) =
        ceiling_trips(&state.dither_captures, state.k_q16, rung);
    let measurement = RungMeasurement {
        position_p: rung,
        homing_peak_abs_units: homing.peak_abs_units,
        motion_cruise_mean_abs_units: motion.cruise_mean_abs_units,
        motion_overshoot_units: motion.overshoot_units,
        motion_cruise_rms_units: motion.cruise_rms_units,
        ceiling_ratios_milli,
    };
    let decision = selection::decide(
        &measurement,
        state.bound_units,
        state.min_overshoot_so_far.map(|(_, units)| units),
        ceiling_tripped,
    );
    state.rungs_measured = state.rungs_measured.saturating_add(1);
    state.last_measurement = Some(measurement);
    defmt::info!(
        "position tune rung: p={} homing_peak={} motion_cruise_mean={} motion_overshoot={} motion_rms={} dither_margin_milli={} decision={}",
        measurement.position_p.raw(),
        measurement.homing_peak_abs_units,
        measurement.motion_cruise_mean_abs_units,
        measurement.motion_overshoot_units,
        measurement.motion_cruise_rms_units,
        dither_margin_milli(measurement.ceiling_ratios_milli),
        decision,
    );
    let result = match decision {
        RungDecision::Accept => {
            state.accepted = Some(measurement);
            state.step = PositionTuneStep::Done;
            state.capture_running = false;
            Ok(())
        }
        RungDecision::Conflict(_) => Err(PositionTuneOutcome::Conflict.into()),
        RungDecision::Advance => {
            state.min_overshoot_so_far = Some(match state.min_overshoot_so_far {
                Some((prev_rung, prev_units)) if prev_units <= motion.overshoot_units => {
                    (prev_rung, prev_units)
                }
                _ => (rung, motion.overshoot_units),
            });
            match next_rung(rung) {
                Some(next) => {
                    state.homing_repeats = [None; RAMPED_STROKE_REPEATS as usize];
                    state.motion_repeats = [None; RAMPED_STROKE_REPEATS as usize];
                    state.step = PositionTuneStep::Stroke {
                        rung: next,
                        kind: StrokeKind::Homing,
                        repeat: 0,
                    };
                    // The completed rung's last capture (its fifth standstill
                    // hold) left `capture_running` set; the walk's next
                    // sub-step is a fresh `Stroke` arm, not a continuation of
                    // that capture.
                    state.capture_running = false;
                    Ok(())
                }
                None => Err(PositionTuneOutcome::SweepExhausted.into()),
            }
        }
    };
    refresh_provenance(state);
    result
}

/// Rebuild `state.provenance` from the walk's current evidence: the
/// predicted start rung, the config-derived bound and planned stroke speeds,
/// the last measured rung (or the accepted one), the running-minimum
/// overshoot and its rung, and the feedforward paths live on the last
/// admitted motion stroke. Called at the end of `predict`, after every
/// stroke admission, and in `resolve_completed_rung`, so every failure exit
/// reads real evidence instead of a zeroed block.
fn refresh_provenance(state: &mut PositionTuneState) {
    let measurement = state.last_measurement.or(state.accepted);
    let last_rung_p = measurement.map_or(0, |m| m.position_p.raw());
    let homing_peak_abs_units = measurement.map_or(0, |m| m.homing_peak_abs_units);
    let motion_cruise_mean_abs_units = measurement.map_or(0, |m| m.motion_cruise_mean_abs_units);
    let motion_overshoot_units = measurement.map_or(0, |m| m.motion_overshoot_units);
    let motion_cruise_rms_units = measurement.map_or(0, |m| m.motion_cruise_rms_units);
    let dither_margin = measurement.map_or(0, |m| dither_margin_milli(m.ceiling_ratios_milli));
    let (min_overshoot_p, min_overshoot_units) = state
        .min_overshoot_so_far
        .map_or((0, 0), |(p, units)| (p.raw(), units));

    state.provenance = PositionTuneProvenance {
        predicted_start_p: state.predicted_start.raw(),
        last_rung_p,
        min_overshoot_p,
        min_overshoot_units,
        bound_units: state.bound_units,
        homing_speed_mrev_s: state.homing_leg.map_or(0, |leg| leg.speed_mrev_s),
        motion_speed_mrev_s: state.motion_leg.map_or(0, |leg| leg.speed_mrev_s),
        homing_peak_abs_units,
        motion_cruise_mean_abs_units,
        motion_overshoot_units,
        motion_cruise_rms_units,
        nominal_bandwidth_hz: nominal_bandwidth_hz(last_rung_p),
        dither_margin_milli: dither_margin,
        rungs_measured: state.rungs_measured,
        stimulus_feedforward_paths: state.feedforward_paths_active,
    };
}

/// Nominal position-loop bandwidth: a pure function of the accepted
/// position P and unit constants, no torque constant or inertia (spec,
/// Reporting section).
///
/// `bandwidth_hz = (position_p/256) * 65536 / (60 * 2*pi)`, evaluated in
/// integer arithmetic throughout: the numerator is scaled by the same 65536
/// `TAU_Q16` carries before the single divide, so the Q16.16 factor cancels
/// and no intermediate is truncated. Rounded to nearest rather than
/// truncated, matching the half-up rounding the field reported before.
fn nominal_bandwidth_hz(position_p_raw: u16) -> u32 {
    // (p * 65536 / 256) * 65536 / (60 * TAU_Q16), with the /256 folded into
    // the literal: 65536/256 * 65536 = 16_777_216. At PidPGain::MAX this
    // numerator is ~5.5e11, three orders below i64::MAX.
    let numerator = i64::from(position_p_raw) * 16_777_216;
    let denominator = 60 * TAU_Q16;
    let rounded = (numerator + denominator / 2) / denominator;
    u32::try_from(rounded).unwrap_or(u32::MAX)
}

/// Median of the accepted rung's 5 per-capture ceiling ratios (milli-ratios),
/// reported as the rung's diagnostic dither margin (spec, Reporting section).
/// The majority-vote trip decision itself never uses this median -- see
/// `ceiling::ceiling_trips`'s own doc comment.
fn dither_margin_milli(ceiling_ratios_milli: [i64; 5]) -> i32 {
    let mut sorted = ceiling_ratios_milli;
    sorted.sort_unstable();
    sorted[2].clamp(i64::from(i32::MIN), i64::from(i32::MAX)) as i32
}

/// Build the phase's success terminal from the accepted rung.
///
/// Joins the accepted position P with the entry snapshot's remaining gains,
/// velocity limit, and biquad cutoffs, the same way
/// `robustness_reversal::robustness_pass_outer_result` does: the `OuterDone`
/// restore path writes those fields straight to the channel config without a
/// zero guard, so anything this phase does not tune must be carried through
/// rather than defaulted. `state.provenance` is already current for the
/// accepted rung: `resolve_completed_rung` refreshes it before returning.
fn position_tune_terminal(state: &PositionTuneState, accepted: RungMeasurement) -> StepResult {
    defmt::info!(
        "position tune accepted: p={} bound_units={} rungs_measured={}",
        accepted.position_p.raw(),
        state.bound_units,
        state.rungs_measured,
    );
    let position_tune = state.provenance;
    StepResult::OuterDone(OuterResult {
        status: CommissionStatus::Accepted,
        velocity_p: state.entry.velocity_p.raw(),
        velocity_i: state.entry.velocity_i.raw(),
        velocity_limit: state.entry.velocity_limit,
        position_p: accepted.position_p.raw(),
        position_i: state.entry.position_i.raw(),
        velocity_filter_hz: state.entry.velocity_filter_hz,
        position_filter_hz: state.entry.position_filter_hz,
        flux_filter_hz: state.entry.flux_filter_hz,
        torque_filter_hz: state.entry.torque_filter_hz,
        warning_code: 0,
        provenance: TuneProvenance::default(),
        position_tune,
        position_tune_outcome_code: 0,
    })
}

/// One cooperative tick of the two-bound position-P ladder sweep.
///
/// Mirrors `integral_sweep::step_velocity_integral_with_trace`'s shape: each
/// sub-step either arms its capture, takes one sample and reports
/// `StepResult::At(next_sample_us)`, or folds the completed capture and moves
/// on. Every failure path zeroes excitation and reports `OuterFailed`, which
/// drives the same `restore_machine` entry-gain restore every other outer
/// phase uses.
pub(crate) async fn step_position_tune<B: CommissioningBackend>(
    phase: &mut OuterPhase,
    workspace: &mut TransientBuffer,
    backend: &mut B,
) -> StepResult {
    let (state, safety) = match phase {
        OuterPhase::PositionTune { state, safety } => (state, safety),
        _ => return StepResult::ReadyNow,
    };

    match state.step {
        PositionTuneStep::Predict => match predict(state, safety, backend).await {
            Ok(()) => StepResult::PhaseChanged(CommissionPhase::PositionTune),
            Err(fault) => fail_position_tune(phase, backend, fault).await,
        },
        PositionTuneStep::Stroke { rung, kind, repeat } => {
            if !state.capture_running {
                return match arm_stroke(state, safety, workspace, backend, rung, kind).await {
                    Ok(()) => StepResult::At(state.next_poll_us),
                    Err(fault) => fail_position_tune(phase, backend, fault).await,
                };
            }
            let now_us = backend.elapsed_us();
            if now_us < state.next_poll_us {
                return StepResult::At(state.next_poll_us);
            }
            state.next_poll_us = now_us + u64::from(STROKE_POLL_US);
            if now_us >= state.next_sample_us && now_us < state.deadline_us {
                let position_error = match read_sub_register(backend, PID_ERROR_PID_POSITION_ERROR)
                    .await
                {
                    Ok(value) => value,
                    Err(error) => return fail_position_tune(phase, backend, error.into()).await,
                };
                workspace.push(position_error);
                if let Err(error) = advance_sample(state, state.sample_period_us, now_us) {
                    return fail_position_tune(phase, backend, error.into()).await;
                }
            }
            if now_us < state.deadline_us {
                return StepResult::At(state.next_poll_us);
            }
            safety.end_primitive();
            backend.stimulus_stop();
            let timing = state
                .stroke_timing
                .expect("timing is captured at admission");
            let samples = workspace.samples();
            let mut signed = [0_i32; TRANSIENT_LEN];
            for (slot, &raw) in signed.iter_mut().zip(samples) {
                *slot = raw as i32;
            }
            let [forward, reverse] =
                metrics::leg_windows(&timing, state.sample_period_us, samples.len());
            let measured = match (
                metrics::leg_metrics(&signed[..samples.len()], &forward),
                metrics::leg_metrics(&signed[..samples.len()], &reverse),
            ) {
                (Ok(a), Ok(b)) => metrics::worst_of(&a, &b),
                (Err(outcome), _) | (_, Err(outcome)) => {
                    return fail_position_tune(phase, backend, outcome.into()).await;
                }
            };
            let repeats = match kind {
                StrokeKind::Homing => &mut state.homing_repeats,
                StrokeKind::Motion => &mut state.motion_repeats,
            };
            repeats[usize::from(repeat)] = Some(measured);
            state.capture_running = false;
            state.step = match (kind, repeat + 1 < RAMPED_STROKE_REPEATS) {
                (kind, true) => PositionTuneStep::Stroke {
                    rung,
                    kind,
                    repeat: repeat + 1,
                },
                (StrokeKind::Homing, false) => PositionTuneStep::Stroke {
                    rung,
                    kind: StrokeKind::Motion,
                    repeat: 0,
                },
                (StrokeKind::Motion, false) => {
                    backend.set_stimulus_feedforward(false);
                    PositionTuneStep::AwaitRestore {
                        rung,
                        since_us: now_us,
                    }
                }
            };
            StepResult::ReadyNow
        }
        PositionTuneStep::AwaitRestore { rung, since_us } => {
            let now_us = backend.elapsed_us();
            if backend.stimulus_feedforward_idle() {
                state.step = PositionTuneStep::StandstillHold {
                    rung,
                    capture_index: 0,
                };
                return StepResult::ReadyNow;
            }
            if now_us.saturating_sub(since_us) >= u64::from(RESTORE_WAIT_US) {
                return fail_position_tune(
                    phase,
                    backend,
                    PositionTuneOutcome::RestorationTimeout.into(),
                )
                .await;
            }
            StepResult::At(now_us + u64::from(STROKE_POLL_US))
        }
        PositionTuneStep::StandstillHold {
            rung,
            capture_index,
        } => {
            if !state.capture_running {
                return match arm_standstill_hold(state, safety, workspace, backend).await {
                    Ok(()) => StepResult::At(state.next_sample_us),
                    Err(fault) => fail_position_tune(phase, backend, fault).await,
                };
            }
            let now_us = backend.elapsed_us();
            if now_us < state.next_sample_us {
                return StepResult::At(state.next_sample_us);
            }
            let raw_target = match read_capture(backend, PID_TORQUE_FLUX_TARGET).await {
                Ok(value) => value,
                Err(error) => return fail_position_tune(phase, backend, error.into()).await,
            };
            let torque_target = PidTorqueFluxTarget::from(raw_target).pid_torque_target();
            workspace.push(i32::from(torque_target) as u32);
            let now_us = backend.elapsed_us();
            if now_us < state.deadline_us {
                return match advance_sample(state, STANDSTILL_SAMPLE_PERIOD_US, now_us) {
                    Ok(next_sample_us) => StepResult::At(next_sample_us),
                    Err(error) => fail_position_tune(phase, backend, error.into()).await,
                };
            }
            safety.end_primitive();
            let excursion = match standstill_excursion(workspace) {
                Ok(value) => value,
                Err(outcome) => return fail_position_tune(phase, backend, outcome.into()).await,
            };
            let Some(slot) = state.dither_captures.get_mut(usize::from(capture_index)) else {
                return fail_position_tune(
                    phase,
                    backend,
                    CommissionError::RungIndexOutOfRange.into(),
                )
                .await;
            };
            *slot = excursion;
            if capture_index + 1 < STANDSTILL_CAPTURE_COUNT {
                state.step = PositionTuneStep::StandstillHold {
                    rung,
                    capture_index: capture_index + 1,
                };
                state.capture_running = false;
                return StepResult::ReadyNow;
            }
            match resolve_completed_rung(state, rung) {
                Ok(()) => match state.accepted {
                    Some(accepted) => position_tune_terminal(state, accepted),
                    None => StepResult::ReadyNow,
                },
                Err(fault) => fail_position_tune(phase, backend, fault).await,
            }
        }
        PositionTuneStep::Done => match state.accepted {
            Some(accepted) => position_tune_terminal(state, accepted),
            // Unreachable: `Done` is only entered together with the accepted
            // measurement; every failure path replaces the phase instead.
            None => {
                fail_position_tune(phase, backend, CommissionError::MissingCaptureState.into())
                    .await
            }
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::commission_dispatch::test_support::{
        MockBackend, MockEvent, SpiOp, StimulusOp, current_limit, sample_entry_snapshot,
        sample_outer_params,
    };
    use crate::commissioning::outer::OuterMotionBudget;
    use core::sync::atomic::Ordering::Relaxed;
    use foci_firmware_macros::fixture_test;
    use foci_tmc4671::pid::PidPGain;

    fn position_tune_budget() -> OuterMotionBudget {
        // Small excursion and accel so a stroke leg's cruise sits near the
        // `MIN_CRUISE_US` floor -- keeps each stroke short enough that these
        // tests drive to a terminal in a modest tick budget.
        OuterMotionBudget::test_budget(200, 2_000, 3_000, 1_000, 200_000)
    }

    fn position_tune_entry_snapshot() -> OuterGainRollbackSnapshot {
        OuterGainRollbackSnapshot {
            velocity_p: PidPGain::new(350).unwrap(),
            velocity_i: PidIGain::new(1_280).unwrap(),
            position_p: PidPGain::new(640).unwrap(),
            position_i: PidIGain::new(7).unwrap(),
            velocity_limit: 500_000,
            voltage_limit: 16_000,
            velocity_filter_hz: 1_100,
            torque_filter_hz: 1_200,
            position_filter_hz: 1_300,
            flux_filter_hz: 1_400,
        }
    }

    fn position_tune_phase() -> OuterPhase {
        OuterPhase::PositionTune {
            state: PositionTuneState::new(
                position_tune_entry_snapshot(),
                CommissionProfile::Balanced,
            ),
            safety: OuterSafetyLimits::new(0, position_tune_budget()),
        }
    }

    fn position_tune_backend() -> MockBackend {
        let mut backend = MockBackend::new();
        backend.spi_elapsed_step_us = 0;
        backend.elapsed_us_value = 1_000_000;
        backend.current_limit_value = current_limit(5_000);
        backend.mechanical_encoder_quadrature_counts_per_rev_value = 4_000;
        backend.configured_physical_width = Some(8);
        let _ = backend.registers.insert(
            PID_VELOCITY_P_VELOCITY_I.addr(),
            PidGainPair::new(PidPGain::new(350).unwrap(), PidIGain::ZERO).register_value(),
        );
        backend
    }

    fn torque_target_register(torque: i32) -> u32 {
        PidTorqueFluxTarget::new()
            .with_pid_torque_target(torque as i16)
            .into()
    }

    /// Quantization-only dither prediction at `rung_raw` for the given
    /// `k_q16`, matching `ceiling::compute_k`'s own convention -- a
    /// non-tripping standstill capture.
    fn quantization_only(rung_raw: u16, k_q16: i64) -> i32 {
        i32::try_from(k_q16.saturating_mul(i64::from(rung_raw)) / 65_536).unwrap_or(i32::MAX)
    }

    /// Drive the phase to a terminal `StepResult`.
    ///
    /// `error_for` scripts the position-error register read during a
    /// `Stroke` capture, keyed by the current phase state and the sample
    /// index within the active capture (`workspace.len()`, reset at each
    /// stroke's arm). `dither_for` scripts the `PID_TORQUE_FLUX_TARGET`
    /// dither amplitude during a `StandstillHold`, keyed by the rung's raw P
    /// and the phase's own `k_q16`. `AwaitRestore` is answered immediately
    /// (the mock's `feedforward_surfaces_idle` mirror is set true as soon as
    /// the phase enters that step), so these tests never exercise the
    /// restoration wait itself -- see `a_restoration_timeout_fails_the_phase`
    /// for that path.
    fn drive_sweep(
        phase: &mut OuterPhase,
        backend: &mut MockBackend,
        error_for: impl Fn(&PositionTuneState, u16) -> i32,
        dither_for: impl Fn(u16, i64) -> i32,
    ) -> StepResult {
        let mut workspace = TransientBuffer::new();
        let mut clock_us = backend.elapsed_us_value;

        for _ in 0..20_000 {
            let OuterPhase::PositionTune { state, .. } = &*phase else {
                panic!("the phase must stay resident until it reports a terminal");
            };
            match state.step {
                PositionTuneStep::Predict => {}
                PositionTuneStep::Stroke { .. } => {
                    let sample_index = workspace.len();
                    let error = error_for(state, sample_index);
                    let _ = backend
                        .registers
                        .insert(PID_ERROR_PID_POSITION_ERROR.data.addr(), error as u32);
                }
                PositionTuneStep::AwaitRestore { .. } => {
                    backend
                        .tmc_state
                        .feedforward_surfaces_idle
                        .store(true, Relaxed);
                }
                PositionTuneStep::StandstillHold { rung, .. } => {
                    let sample_index = workspace.len();
                    let predicted = dither_for(rung.raw(), state.k_q16);
                    let torque = if sample_index % 2 == 0 { 0 } else { predicted };
                    let _ = backend.registers.insert(
                        PID_TORQUE_FLUX_TARGET.addr(),
                        torque_target_register(torque),
                    );
                }
                PositionTuneStep::Done => {}
            }

            backend.elapsed_us_value = clock_us;
            let result = pollster::block_on(step_position_tune(phase, &mut workspace, backend));
            match result {
                StepResult::At(next_us) => clock_us = next_us,
                StepResult::ReadyNow | StepResult::PhaseChanged(_) => {}
                terminal => return terminal,
            }
        }
        panic!("the sweep did not reach a terminal within the tick budget");
    }

    /// Drive one `Predict` tick and return the resulting `predicted_start`
    /// raw P, so a test can script its lag table against the real value
    /// rather than an assumed one.
    fn predicted_start_of(phase: &mut OuterPhase, backend: &mut MockBackend) -> u16 {
        let mut scratch = TransientBuffer::new();
        let result = pollster::block_on(step_position_tune(phase, &mut scratch, backend));
        assert!(
            matches!(result, StepResult::PhaseChanged(_)),
            "predict must report a phase change, got {result:?}"
        );
        let OuterPhase::PositionTune { state, .. } = &*phase else {
            panic!("phase stays PositionTune after predict");
        };
        state.predicted_start.raw()
    }

    /// `Stop` is the last stimulus op that is not itself a `Feedforward`
    /// toggle, and the final `Feedforward` op is `false` -- the ordering
    /// every failure path (`fail_position_tune`'s `abort_stimulus` call)
    /// guarantees.
    fn assert_ends_with_stop_and_feedforward_false(log: &[StimulusOp]) {
        let last_non_feedforward = log
            .iter()
            .rev()
            .find(|op| !matches!(op, StimulusOp::Feedforward(_)));
        assert_eq!(
            last_non_feedforward,
            Some(&StimulusOp::Stop),
            "expected Stop as the last non-feedforward stimulus op, log: {log:?}"
        );
        let last_feedforward = log
            .iter()
            .rev()
            .find(|op| matches!(op, StimulusOp::Feedforward(_)));
        assert_eq!(
            last_feedforward,
            Some(&StimulusOp::Feedforward(false)),
            "expected the final Feedforward op to be false, log: {log:?}"
        );
    }

    #[fixture_test]
    fn homing_strokes_share_the_motion_legs_velocity_envelope() {
        let mut backend = position_tune_backend();
        let mut phase = position_tune_phase();
        let mut workspace = TransientBuffer::new();
        let mut clock_us = backend.elapsed_us_value;
        let mut homing_cap = None;
        let mut motion_cap = None;

        for _ in 0..20_000 {
            backend.elapsed_us_value = clock_us;
            let _ = backend
                .registers
                .insert(PID_ERROR_PID_POSITION_ERROR.data.addr(), 50);
            match pollster::block_on(step_position_tune(&mut phase, &mut workspace, &mut backend)) {
                StepResult::At(next_us) => clock_us = next_us,
                StepResult::ReadyNow | StepResult::PhaseChanged(_) => {}
                terminal => panic!("unexpected terminal {terminal:?}"),
            }
            let OuterPhase::PositionTune { state, safety } = &phase else {
                panic!("the phase stays resident");
            };
            if let (PositionTuneStep::Stroke { kind, .. }, true) =
                (state.step, state.capture_running)
            {
                let cap = safety.active_velocity_cap_milli_counts_per_s();
                match kind {
                    StrokeKind::Homing => homing_cap = homing_cap.or(Some(cap)),
                    StrokeKind::Motion => motion_cap = motion_cap.or(Some(cap)),
                }
            }
            if motion_cap.is_some() {
                break;
            }
        }

        let OuterPhase::PositionTune { state, .. } = &phase else {
            panic!("the phase stays resident");
        };
        assert!(
            state.homing_leg.expect("planned").speed_mrev_s
                < state.motion_leg.expect("planned").speed_mrev_s,
            "fixture must plan a slower homing leg"
        );
        assert_eq!(
            homing_cap.expect("a homing stroke armed"),
            motion_cap.expect("a motion stroke armed")
        );
    }

    #[fixture_test]
    fn a_rung_inside_the_bound_is_accepted_on_the_start_rung() {
        let mut backend = position_tune_backend();
        let mut phase = position_tune_phase();

        let terminal = drive_sweep(
            &mut phase,
            &mut backend,
            |_state, _idx| 50,
            |rung, k_q16| quantization_only(rung, k_q16),
        );

        let StepResult::OuterDone(result) = terminal else {
            panic!("expected OuterDone, got {terminal:?}");
        };
        let OuterPhase::PositionTune { state, .. } = &phase else {
            panic!("the phase stays resident after a successful terminal");
        };
        assert_eq!(result.position_p, state.predicted_start.raw());

        // Each stroke: Stop, ResetClock, SetDirection(true), Queue x n,
        // SetDirection(false), Queue x n, Arm, ..., Stop -- with the two
        // Feedforward toggles filtered out since their exact placement is
        // covered separately below.
        let filtered: std::vec::Vec<StimulusOp> = backend
            .stimulus_log
            .iter()
            .copied()
            .filter(|op| !matches!(op, StimulusOp::Feedforward(_)))
            .collect();
        let mut idx = 0;
        let mut stroke_count = 0;
        while idx < filtered.len() {
            assert_eq!(
                filtered[idx],
                StimulusOp::Stop,
                "stroke must open with Stop"
            );
            idx += 1;
            assert!(
                matches!(filtered[idx], StimulusOp::ResetClock(_)),
                "expected ResetClock, got {:?}",
                filtered[idx]
            );
            idx += 1;
            assert_eq!(filtered[idx], StimulusOp::SetDirection(true));
            idx += 1;
            let mut queued_forward = 0;
            while matches!(filtered[idx], StimulusOp::Queue(_)) {
                idx += 1;
                queued_forward += 1;
            }
            assert!(
                queued_forward > 0,
                "forward leg must queue at least one segment"
            );
            assert_eq!(filtered[idx], StimulusOp::SetDirection(false));
            idx += 1;
            let mut queued_reverse = 0;
            while matches!(filtered[idx], StimulusOp::Queue(_)) {
                idx += 1;
                queued_reverse += 1;
            }
            assert!(
                queued_reverse > 0,
                "reverse leg must queue at least one segment"
            );
            assert_eq!(filtered[idx], StimulusOp::Arm);
            idx += 1;
            assert_eq!(
                filtered[idx],
                StimulusOp::Stop,
                "stroke must close with Stop"
            );
            idx += 1;
            stroke_count += 1;
        }
        assert_eq!(
            stroke_count, 6,
            "expected 3 homing + 3 motion repeats for one accepted rung"
        );

        let first_feedforward_true = backend
            .stimulus_log
            .iter()
            .position(|op| *op == StimulusOp::Feedforward(true));
        assert!(
            first_feedforward_true.is_some(),
            "motion strokes must opt in feedforward"
        );
        let last_feedforward_false = backend
            .stimulus_log
            .iter()
            .rposition(|op| *op == StimulusOp::Feedforward(false));
        assert!(
            first_feedforward_true.unwrap()
                < last_feedforward_false.expect("feedforward is cleared"),
            "Feedforward(true) must precede the trailing Feedforward(false)"
        );
    }

    #[fixture_test]
    fn the_walk_advances_while_the_homing_peak_is_outside_the_bound() {
        let mut backend = position_tune_backend();
        let mut phase = position_tune_phase();
        let start_raw = predicted_start_of(&mut phase, &mut backend);
        let expected = next_rung(PidPGain::new(start_raw).unwrap())
            .expect("a representable next rung")
            .raw();

        let terminal = drive_sweep(
            &mut phase,
            &mut backend,
            move |state, _idx| {
                let PositionTuneStep::Stroke { rung, kind, .. } = state.step else {
                    return 0;
                };
                if rung.raw() == start_raw {
                    match kind {
                        StrokeKind::Homing => 10_000,
                        StrokeKind::Motion => 50,
                    }
                } else {
                    50
                }
            },
            |rung, k_q16| quantization_only(rung, k_q16),
        );

        let StepResult::OuterDone(result) = terminal else {
            panic!("expected OuterDone, got {terminal:?}");
        };
        assert_eq!(result.position_p, expected);
    }

    #[fixture_test]
    fn an_overshoot_ratio_trip_reports_conflict_with_the_last_rung() {
        let mut backend = position_tune_backend();
        let mut phase = position_tune_phase();
        let start_raw = predicted_start_of(&mut phase, &mut backend);

        // rung 1 (the start rung) is forced to advance on a homing-bound
        // violation, which still records its own motion overshoot (40) as
        // the walk's running minimum; rung 2's motion overshoot (200, 5x)
        // then trips the overshoot-ratio guard.
        let terminal = drive_sweep(
            &mut phase,
            &mut backend,
            move |state, sample_index| {
                let PositionTuneStep::Stroke { rung, kind, .. } = state.step else {
                    return 0;
                };
                match kind {
                    StrokeKind::Homing => {
                        if rung.raw() == start_raw {
                            10_000
                        } else {
                            50
                        }
                    }
                    StrokeKind::Motion => {
                        let spike_over_steady = if rung.raw() == start_raw { 40 } else { 200 };
                        let ramp_samples = state
                            .stroke_timing
                            .map(|timing| timing.ramp_end_us / state.sample_period_us.max(1))
                            .unwrap_or(0);
                        let spike_index = (ramp_samples / 2).max(1);
                        if sample_index == 0 {
                            0
                        } else if u32::from(sample_index) == spike_index {
                            50 + spike_over_steady
                        } else {
                            50
                        }
                    }
                }
            },
            |rung, k_q16| quantization_only(rung, k_q16),
        );

        assert!(matches!(
            terminal,
            StepResult::OuterFailed(CommissionError::PositionTuneFailed)
        ));
        assert!(matches!(
            phase,
            OuterPhase::PositionTuneFailed {
                error: CommissionError::PositionTuneFailed,
                outcome: Some(PositionTuneOutcome::Conflict),
                ..
            }
        ));
    }

    #[fixture_test]
    fn a_conflict_terminal_carries_both_rungs() {
        let mut backend = position_tune_backend();
        let mut phase = position_tune_phase();
        let start_raw = predicted_start_of(&mut phase, &mut backend);
        let second_rung_raw = next_rung(PidPGain::new(start_raw).unwrap())
            .expect("a representable next rung")
            .raw();

        let terminal = drive_sweep(
            &mut phase,
            &mut backend,
            move |state, sample_index| {
                let PositionTuneStep::Stroke { rung, kind, .. } = state.step else {
                    return 0;
                };
                match kind {
                    StrokeKind::Homing => {
                        if rung.raw() == start_raw {
                            10_000
                        } else {
                            50
                        }
                    }
                    StrokeKind::Motion => {
                        let spike_over_steady = if rung.raw() == start_raw { 40 } else { 200 };
                        let ramp_samples = state
                            .stroke_timing
                            .map(|timing| timing.ramp_end_us / state.sample_period_us.max(1))
                            .unwrap_or(0);
                        let spike_index = (ramp_samples / 2).max(1);
                        if sample_index == 0 {
                            0
                        } else if u32::from(sample_index) == spike_index {
                            50 + spike_over_steady
                        } else {
                            50
                        }
                    }
                }
            },
            |rung, k_q16| quantization_only(rung, k_q16),
        );

        assert!(matches!(
            terminal,
            StepResult::OuterFailed(CommissionError::PositionTuneFailed)
        ));
        let OuterPhase::PositionTuneFailed { provenance, .. } = &phase else {
            panic!("expected PositionTuneFailed, got {phase:?}");
        };
        assert_eq!(provenance.rungs_measured, 2);
        assert_eq!(provenance.last_rung_p, second_rung_raw);
        assert_eq!(provenance.min_overshoot_p, start_raw);
        // The ratio guard only trips when the later rung's overshoot grows
        // well past the running minimum, so this ordering alone confirms
        // both rungs' evidence landed on the fields the spec names for them.
        assert!(provenance.motion_overshoot_units > provenance.min_overshoot_units);
    }

    #[fixture_test]
    fn a_refused_enqueue_stops_and_reports_stimulus_rejected() {
        let mut backend = position_tune_backend();
        backend.stimulus_refuse_after = Some(0);
        let mut phase = position_tune_phase();

        let terminal = drive_sweep(&mut phase, &mut backend, |_state, _idx| 50, |_rung, _k| 0);

        assert!(matches!(
            terminal,
            StepResult::OuterFailed(CommissionError::PositionTuneFailed)
        ));
        assert!(matches!(
            phase,
            OuterPhase::PositionTuneFailed {
                error: CommissionError::PositionTuneFailed,
                outcome: Some(PositionTuneOutcome::StimulusRejected),
                ..
            }
        ));
        assert!(!backend.stimulus_log.iter().any(|op| *op == StimulusOp::Arm));
        assert_ends_with_stop_and_feedforward_false(&backend.stimulus_log);
    }

    #[fixture_test]
    fn insufficient_queue_capacity_is_rejected_before_any_enqueue() {
        let mut backend = position_tune_backend();
        backend.stimulus_queue_free = 1;
        let mut phase = position_tune_phase();

        let terminal = drive_sweep(&mut phase, &mut backend, |_state, _idx| 50, |_rung, _k| 0);

        assert!(matches!(
            terminal,
            StepResult::OuterFailed(CommissionError::PositionTuneFailed)
        ));
        assert!(matches!(
            phase,
            OuterPhase::PositionTuneFailed {
                error: CommissionError::PositionTuneFailed,
                outcome: Some(PositionTuneOutcome::StimulusRejected),
                ..
            }
        ));
        assert!(
            !backend
                .stimulus_log
                .iter()
                .any(|op| matches!(op, StimulusOp::Queue(_))),
            "no segment may be queued once admission is refused"
        );
        assert_ends_with_stop_and_feedforward_false(&backend.stimulus_log);
    }

    #[fixture_test]
    fn an_outer_safety_fault_mid_stroke_stops_the_stimulus_before_the_generic_path() {
        let mut backend = position_tune_backend();
        let params = sample_outer_params();
        let entry = sample_entry_snapshot();
        let mut engine =
            crate::commissioning::CommissioningEngine::new_outer_for_tmc_with_position_tune(
                7,
                crate::state::tmc::tmc_channel0(),
                params,
                entry,
                0,
            );

        // Drive Predict, the first stroke's admission, and a handful of poll
        // ticks mid-capture -- comfortably short of the stroke's own
        // deadline given the small excursion `sample_outer_params` requests.
        // The clock must be advanced explicitly (mirroring `drive_sweep`):
        // `MockBackend::elapsed_us_value` is otherwise inert, so a bare
        // `engine.step` loop would never reach the tick at which the outer
        // safety envelope's next observation is actually due.
        let mut clock_us = backend.elapsed_us_value;
        for _ in 0..6 {
            backend.elapsed_us_value = clock_us;
            let result = pollster::block_on(engine.step(&mut backend));
            match result {
                StepResult::At(next_us) => clock_us = next_us,
                StepResult::ReadyNow | StepResult::PhaseChanged(_) => {}
                other => panic!("unexpected early terminal: {other:?}"),
            }
        }

        // Trip the quarter-turn aliasing guard: a single observation whose
        // |delta| exceeds one quarter of the configured encoder resolution
        // (4_000 counts/rev in `position_tune_backend`).
        backend.mechanical_encoder_position_counts_value = 5_000;
        backend.elapsed_us_value = clock_us;

        let terminal = pollster::block_on(engine.step(&mut backend));
        assert!(matches!(
            terminal,
            StepResult::OuterFailed(CommissionError::SafetyEnvelopeViolation)
        ));

        // `arm_stroke`'s own admission (the first `engine.step` in the drive
        // loop above) already emits a `Stop` and a `Feedforward` toggle, so
        // asserting on the *first* occurrence of either op in the whole log
        // would pass unconditionally -- it says nothing about where the
        // safety exit's own `abort_stimulus` call landed. Instead, find the
        // safety exit's own zeroing write and require the event pair
        // immediately before it to be exactly `abort_stimulus`'s own two
        // ops, adjacent, nothing else between them and the write: this form
        // fails the instant anything is inserted ahead of the abort, or the
        // abort is moved after the write.
        let write_zero_index = backend
            .event_log
            .iter()
            .position(|event| {
                matches!(event, MockEvent::Spi(SpiOp::Write(addr, 0)) if *addr == PID_TORQUE_FLUX_TARGET.addr())
            })
            .expect("the safety-fault exit's PID_TORQUE_FLUX_TARGET zero write must be recorded");
        let preceding = &backend.event_log[..write_zero_index];
        assert!(
            preceding.len() >= 2,
            "expected abort_stimulus's pair before the safety exit's write, got {preceding:?}"
        );
        assert_eq!(
            &preceding[preceding.len() - 2..],
            [
                MockEvent::Stimulus(StimulusOp::Stop),
                MockEvent::Stimulus(StimulusOp::Feedforward(false)),
            ],
            "abort_stimulus's Stop then Feedforward(false) must be the last events before the \
             safety exit's excitation zeroing, with nothing else between them and the write"
        );
    }

    #[fixture_test]
    fn a_safety_fault_after_a_completed_rung_keeps_its_evidence() {
        // Drives through `step_outer_with_trace` directly (not the full
        // `CommissioningEngine`) so the test can keep scripting registers
        // from the resident phase, exactly as `drive_sweep` does.
        let mut backend = position_tune_backend();
        let mut phase = position_tune_phase();
        let start_raw = predicted_start_of(&mut phase, &mut backend);

        let mut workspace = TransientBuffer::new();
        let mut trace_summary_seqs = crate::commissioning::outer::TraceSummarySeqs::new();
        let mut clock_us = backend.elapsed_us_value;
        let mut second_rung_entered = false;

        for _ in 0..20_000 {
            let OuterPhase::PositionTune { state, .. } = &phase else {
                panic!("the phase must stay resident until it reports a terminal");
            };
            match state.step {
                PositionTuneStep::Predict => {}
                PositionTuneStep::Stroke { rung, kind, .. } => {
                    let error = if rung.raw() == start_raw {
                        match kind {
                            StrokeKind::Homing => 10_000,
                            StrokeKind::Motion => 50,
                        }
                    } else {
                        50
                    };
                    let _ = backend
                        .registers
                        .insert(PID_ERROR_PID_POSITION_ERROR.data.addr(), error as u32);
                    if rung.raw() != start_raw && !second_rung_entered {
                        backend.mechanical_encoder_position_counts_value = 5_000;
                        second_rung_entered = true;
                    }
                }
                PositionTuneStep::AwaitRestore { .. } => {
                    backend
                        .tmc_state
                        .feedforward_surfaces_idle
                        .store(true, Relaxed);
                }
                PositionTuneStep::StandstillHold { rung, .. } => {
                    let sample_index = workspace.len();
                    let predicted = quantization_only(rung.raw(), state.k_q16);
                    let torque = if sample_index % 2 == 0 { 0 } else { predicted };
                    let _ = backend.registers.insert(
                        PID_TORQUE_FLUX_TARGET.addr(),
                        torque_target_register(torque),
                    );
                }
                PositionTuneStep::Done => {}
            }

            backend.elapsed_us_value = clock_us;
            let result = pollster::block_on(crate::commissioning::outer::step_outer_with_trace(
                &mut phase,
                &mut workspace,
                CommissionProfile::Balanced,
                crate::autotune::OperatingMode::Nominal,
                0,
                0,
                0,
                0,
                &mut trace_summary_seqs,
                &mut backend,
            ));
            match result {
                StepResult::At(next_us) => clock_us = next_us,
                StepResult::ReadyNow | StepResult::PhaseChanged(_) => {}
                terminal => {
                    assert!(matches!(
                        terminal,
                        StepResult::OuterFailed(CommissionError::SafetyEnvelopeViolation)
                    ));
                    let OuterPhase::FailedWithSafety { position_tune, .. } = &phase else {
                        panic!("expected FailedWithSafety, got {phase:?}");
                    };
                    assert_eq!(position_tune.rungs_measured, 1);
                    assert_eq!(position_tune.last_rung_p, start_raw);
                    return;
                }
            }
        }
        panic!("expected a terminal within the tick budget");
    }

    #[fixture_test]
    fn a_shutdown_mid_stroke_stops_the_stimulus_before_the_engines_safe_state_restore() {
        // `CommissioningEngine::step`'s shutdown branch bypasses
        // `step_inner`/`step_outer` entirely -- and so every one of
        // `dispatch.rs`'s outer-safety exits and `fail_position_tune` -- so
        // it needs its own `abort_stimulus` call, independent of the one
        // this module's own failure path and the outer safety layer make.
        let mut backend = position_tune_backend();
        let params = sample_outer_params();
        let entry = sample_entry_snapshot();
        let mut engine =
            crate::commissioning::CommissioningEngine::new_outer_for_tmc_with_position_tune(
                7,
                crate::state::tmc::tmc_channel0(),
                params,
                entry,
                0,
            );

        let mut clock_us = backend.elapsed_us_value;
        for _ in 0..6 {
            backend.elapsed_us_value = clock_us;
            let result = pollster::block_on(engine.step(&mut backend));
            match result {
                StepResult::At(next_us) => clock_us = next_us,
                StepResult::ReadyNow | StepResult::PhaseChanged(_) => {}
                other => panic!("unexpected early terminal: {other:?}"),
            }
        }

        backend.shutdown_requested = true;
        backend.elapsed_us_value = clock_us;
        let terminal = pollster::block_on(engine.step(&mut backend));
        assert!(matches!(
            terminal,
            StepResult::OuterFailed(CommissionError::Shutdown)
        ));

        // The safe-state restore sequence's first write is always
        // `MODE_RAMP_MODE_MOTION = 0` (see `tmc_safe_state`'s step 0).
        let restore_write_index = backend
            .event_log
            .iter()
            .position(|event| {
                matches!(event, MockEvent::Spi(SpiOp::Write(addr, 0)) if *addr == MODE_RAMP_MODE_MOTION.addr())
            })
            .expect("the safe-state restore's MODE_RAMP_MODE_MOTION zero write must be recorded");
        let preceding = &backend.event_log[..restore_write_index];
        assert!(
            preceding.len() >= 2,
            "expected abort_stimulus's pair before the safe-state restore, got {preceding:?}"
        );
        assert_eq!(
            &preceding[preceding.len() - 2..],
            [
                MockEvent::Stimulus(StimulusOp::Stop),
                MockEvent::Stimulus(StimulusOp::Feedforward(false)),
            ],
            "the shutdown branch's abort_stimulus call must stop the stimulus and clear \
             feedforward before the safe-state restore begins"
        );
    }

    #[fixture_test]
    fn a_restoration_timeout_fails_the_phase() {
        let mut backend = position_tune_backend();
        let mut phase = position_tune_phase();
        let mut workspace = TransientBuffer::new();
        let mut clock_us = backend.elapsed_us_value;

        for _ in 0..20_000 {
            let OuterPhase::PositionTune { state, .. } = &phase else {
                panic!("the phase must stay resident until it reports a terminal");
            };
            match state.step {
                PositionTuneStep::Stroke { .. } => {
                    let _ = backend
                        .registers
                        .insert(PID_ERROR_PID_POSITION_ERROR.data.addr(), 50);
                }
                PositionTuneStep::StandstillHold { rung, .. } => {
                    let sample_index = workspace.len();
                    let predicted = quantization_only(rung.raw(), state.k_q16);
                    let torque = if sample_index % 2 == 0 { 0 } else { predicted };
                    let _ = backend.registers.insert(
                        PID_TORQUE_FLUX_TARGET.addr(),
                        torque_target_register(torque),
                    );
                }
                // Never mark the restoration mirror idle: the phase must
                // eventually time out waiting for it.
                PositionTuneStep::AwaitRestore { .. }
                | PositionTuneStep::Predict
                | PositionTuneStep::Done => {}
            }

            backend.elapsed_us_value = clock_us;
            let result =
                pollster::block_on(step_position_tune(&mut phase, &mut workspace, &mut backend));
            match result {
                StepResult::At(next_us) => clock_us = next_us,
                StepResult::ReadyNow | StepResult::PhaseChanged(_) => {}
                terminal => {
                    assert!(matches!(
                        terminal,
                        StepResult::OuterFailed(CommissionError::PositionTuneFailed)
                    ));
                    assert!(matches!(
                        phase,
                        OuterPhase::PositionTuneFailed {
                            error: CommissionError::PositionTuneFailed,
                            outcome: Some(PositionTuneOutcome::RestorationTimeout),
                            ..
                        }
                    ));
                    assert_ends_with_stop_and_feedforward_false(&backend.stimulus_log);
                    return;
                }
            }
        }
        panic!("expected a terminal within the tick budget");
    }

    #[fixture_test]
    fn a_trigger_blocked_stepper_fails_at_admission() {
        let mut backend = position_tune_backend();
        backend.stimulus_trigger_blocked = true;
        let mut phase = position_tune_phase();

        let terminal = drive_sweep(&mut phase, &mut backend, |_state, _idx| 50, |_rung, _k| 0);

        assert!(matches!(
            terminal,
            StepResult::OuterFailed(CommissionError::PositionTuneFailed)
        ));
        assert!(matches!(
            phase,
            OuterPhase::PositionTuneFailed {
                error: CommissionError::PositionTuneFailed,
                outcome: Some(PositionTuneOutcome::ConfigurationInvalid),
                ..
            }
        ));
        assert!(
            !backend
                .stimulus_log
                .iter()
                .any(|op| matches!(op, StimulusOp::Queue(_))),
            "no segment may be queued once the stepper is trigger-blocked"
        );
    }

    #[fixture_test]
    fn a_failure_before_the_first_motion_stroke_reports_no_feedforward_paths() {
        // A refused enqueue on the first (homing) stroke -- no motion stroke
        // has ever been admitted, so the failure's evidence must not carry a
        // configured-but-never-run feedforward mask.
        let mut backend = position_tune_backend();
        backend.stimulus_refuse_after = Some(0);
        let mut phase = position_tune_phase();

        let terminal = drive_sweep(&mut phase, &mut backend, |_state, _idx| 50, |_rung, _k| 0);

        assert!(matches!(
            terminal,
            StepResult::OuterFailed(CommissionError::PositionTuneFailed)
        ));
        let OuterPhase::PositionTuneFailed { provenance, .. } = &phase else {
            panic!("expected PositionTuneFailed, got {phase:?}");
        };
        assert_eq!(provenance.stimulus_feedforward_paths, 0);

        // A completed rung's accepted terminal carries whatever feedforward
        // paths were live on its last (motion) stroke admission.
        let mut backend = position_tune_backend();
        backend.feedforward_path_flags_value = 0b0_0101; // velocity + acceleration
        let mut phase = position_tune_phase();

        let terminal = drive_sweep(
            &mut phase,
            &mut backend,
            |_state, _idx| 50,
            |rung, k_q16| quantization_only(rung, k_q16),
        );

        let StepResult::OuterDone(result) = terminal else {
            panic!("expected OuterDone, got {terminal:?}");
        };
        assert_eq!(result.position_tune.stimulus_feedforward_paths, 0b0_0101);
    }

    #[fixture_test]
    fn every_failure_path_stops_the_stimulus_and_clears_feedforward() {
        // 3: overshoot-ratio conflict.
        {
            let mut backend = position_tune_backend();
            let mut phase = position_tune_phase();
            let start_raw = predicted_start_of(&mut phase, &mut backend);
            let _ = drive_sweep(
                &mut phase,
                &mut backend,
                move |state, sample_index| {
                    let PositionTuneStep::Stroke { rung, kind, .. } = state.step else {
                        return 0;
                    };
                    match kind {
                        StrokeKind::Homing => {
                            if rung.raw() == start_raw {
                                10_000
                            } else {
                                50
                            }
                        }
                        StrokeKind::Motion => {
                            let spike_over_steady = if rung.raw() == start_raw { 40 } else { 200 };
                            let ramp_samples = state
                                .stroke_timing
                                .map(|timing| timing.ramp_end_us / state.sample_period_us.max(1))
                                .unwrap_or(0);
                            let spike_index = (ramp_samples / 2).max(1);
                            if sample_index == 0 {
                                0
                            } else if u32::from(sample_index) == spike_index {
                                50 + spike_over_steady
                            } else {
                                50
                            }
                        }
                    }
                },
                |rung, k_q16| quantization_only(rung, k_q16),
            );
            assert_ends_with_stop_and_feedforward_false(&backend.stimulus_log);
        }
        // 4: refused enqueue.
        {
            let mut backend = position_tune_backend();
            backend.stimulus_refuse_after = Some(0);
            let mut phase = position_tune_phase();
            let _ = drive_sweep(&mut phase, &mut backend, |_state, _idx| 50, |_rung, _k| 0);
            assert_ends_with_stop_and_feedforward_false(&backend.stimulus_log);
        }
        // 4b: insufficient queue capacity.
        {
            let mut backend = position_tune_backend();
            backend.stimulus_queue_free = 1;
            let mut phase = position_tune_phase();
            let _ = drive_sweep(&mut phase, &mut backend, |_state, _idx| 50, |_rung, _k| 0);
            assert_ends_with_stop_and_feedforward_false(&backend.stimulus_log);
        }
        // 4c: outer safety fault mid-stroke -- covered by its own dedicated
        // test above (`an_outer_safety_fault_mid_stroke_...`), which asserts
        // the same Stop-then-Feedforward(false)-before-excitation-zeroing
        // ordering through the outer dispatch path rather than this
        // module's bare `step_position_tune`.
        // 5: restoration timeout -- covered by its own dedicated test above
        // (`a_restoration_timeout_fails_the_phase`), which already asserts
        // this same ending.
        // 6: trigger-blocked at admission.
        {
            let mut backend = position_tune_backend();
            backend.stimulus_trigger_blocked = true;
            let mut phase = position_tune_phase();
            let _ = drive_sweep(&mut phase, &mut backend, |_state, _idx| 50, |_rung, _k| 0);
            assert_ends_with_stop_and_feedforward_false(&backend.stimulus_log);
        }
    }

    #[fixture_test]
    fn the_terminal_carries_every_untuned_entry_gain() {
        // The OuterDone restore path stores velocity_limit and the four
        // biquad cutoffs to the channel config with no zero guard, so a
        // terminal built from OuterResult::default() would leave the
        // velocity controller's input clamped to zero after a *successful*
        // tune.
        let entry = position_tune_entry_snapshot();
        let mut backend = position_tune_backend();
        let mut phase = position_tune_phase();

        let terminal = drive_sweep(
            &mut phase,
            &mut backend,
            |_state, _idx| 50,
            |rung, k_q16| quantization_only(rung, k_q16),
        );

        let StepResult::OuterDone(result) = terminal else {
            panic!("a completed sweep reports OuterDone, got {terminal:?}");
        };
        assert_eq!(result.velocity_limit, entry.velocity_limit);
        assert_eq!(result.velocity_p, entry.velocity_p.raw());
        assert_eq!(result.velocity_i, entry.velocity_i.raw());
        assert_eq!(result.position_i, entry.position_i.raw());
        assert_eq!(result.velocity_filter_hz, entry.velocity_filter_hz);
        assert_eq!(result.position_filter_hz, entry.position_filter_hz);
        assert_eq!(result.flux_filter_hz, entry.flux_filter_hz);
        assert_eq!(result.torque_filter_hz, entry.torque_filter_hz);
        // Position P is the one field this phase tunes, so it comes from the
        // accepted rung rather than from the snapshot.
        assert_ne!(result.position_p, entry.position_p.raw());
    }
}
