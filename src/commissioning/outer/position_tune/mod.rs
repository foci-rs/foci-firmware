//! PositionTune: two-bound position-P sweep commissioning phase.
//!
//! See `docs/specs/2026-09-13-foci-340-position-tune-two-bound-design.md` for
//! the full design. This module wires the pure computations in `floor`,
//! `ceiling`, and `selection` into the outer commissioning engine's
//! phase/capture/safety conventions, following `velocity/integral_sweep.rs`'s
//! rung-stepping and `StepResult` shape.

pub(crate) mod ceiling;
pub(crate) mod floor;
pub(crate) mod selection;

use crate::autotune::{
    CommissionError, CommissionPhase, CommissionStatus, PositionTuneOutcome, TRANSIENT_LEN,
    TransientBuffer,
};
use crate::commissioning::OuterGainRollbackSnapshot;
use crate::commissioning::backend::CommissioningBackend;
use crate::commissioning::outer::{OuterMotionBudget, OuterPhase, OuterSafetyLimits, fail_outer};
use crate::commissioning::schedule::next_grid_deadline;
use crate::commissioning::types::{
    OuterResult, PositionTuneProvenance, StepResult, TuneProvenance,
};
use ceiling::{ceiling_trips, compute_k};
use floor::compute_position_floor;
use foci_tmc4671::pid::{PidGainPair, PidIGain, PidPGain};
use foci_tmc4671::registers::{
    MODE_RAMP_MODE_MOTION, PID_ERROR_PID_POSITION_ERROR, PID_ERROR_PID_POSITION_ERROR_SUM,
    PID_POSITION_ACTUAL, PID_POSITION_P_POSITION_I, PID_POSITION_TARGET, PID_TORQUE_FLUX_TARGET,
    PID_VELOCITY_ACTUAL, PID_VELOCITY_P_VELOCITY_I, PID_VELOCITY_TARGET, PidTorqueFluxTarget,
    Register, SubRegister,
};
use selection::{RungOutcome, SweepDecision, is_lag_plateau, next_rung, resolve_rung};

/// Fixed capture parameters for this phase. All are conservative starting
/// points to be validated against real captures on hardware, not values
/// derived from a specific machine.
///
/// - `P_ONLY_TORQUE_PERMILLE_OF_CAP`: a fixed 20% of `safety.torque_cap`. A
///   zero result (a misconfigured or already-zero cap) is rejected before
///   anything is commanded, rather than commanding a zero torque step.
/// - `P_ONLY_CAPTURE_US` / `P_ONLY_SAMPLE_PERIOD_US`: a 4 ms onset window at
///   1 kHz. The fast rotor-only response is characterized within the first
///   ~2 ms, so 4 ms leaves margin without extending the phase unnecessarily.
/// - `RAMPED_STROKE_CAPTURE_US` / `RAMPED_STROKE_SAMPLE_PERIOD_US`: the outer
///   velocity sweep's own stroke duration (512 ms) reused, but sampled at
///   500 Hz rather than 1 kHz: `TransientBuffer`'s capacity is
///   `TRANSIENT_LEN = 256`, and 512 ms at 1 kHz would silently truncate the
///   last half of the capture. 512 ms / 2000 us is exactly 256 samples.
/// - `STANDSTILL_HOLD_US` / `STANDSTILL_SAMPLE_PERIOD_US`: a 100 ms hold at
///   1 kHz, short enough that five captures per rung plus the ramped stroke
///   stay within a reasonable per-rung budget, long enough to observe
///   multiple dither cycles.
pub(crate) const P_ONLY_TORQUE_PERMILLE_OF_CAP: u32 = 200;
pub(crate) const P_ONLY_CAPTURE_US: u32 = 4_000;
pub(crate) const P_ONLY_SAMPLE_PERIOD_US: u32 = 1_000;
pub(crate) const RAMPED_STROKE_CAPTURE_US: u32 = 512_000;
pub(crate) const RAMPED_STROKE_SAMPLE_PERIOD_US: u32 = 2_000;
pub(crate) const STANDSTILL_HOLD_US: u32 = 100_000;
pub(crate) const STANDSTILL_SAMPLE_PERIOD_US: u32 = 1_000;

/// Standstill-dither observations per rung (spec: `M = 5`, odd so the 3-of-5
/// majority vote can never tie).
const STANDSTILL_CAPTURE_COUNT: u8 = 5;

/// Samples the floor's onset window yields on an un-slipped sample grid.
///
/// The fit reconstructs each sample's timestamp from its index, so a capture
/// that collected any other number of samples has a grid slip somewhere in it
/// and its implied slope would be wrong.
const FLOOR_EXPECTED_SAMPLES: usize = (P_ONLY_CAPTURE_US / P_ONLY_SAMPLE_PERIOD_US) as usize;

/// `MODE_RAMP_MODE_MOTION` values: advanced PI representation (bit 31) plus
/// the motion mode, matching `integral_sweep.rs`'s `OUTER_VELOCITY_MODE`.
const POSITION_TUNE_TORQUE_MODE: u32 = 0x8000_0001;
const POSITION_TUNE_POSITION_MODE: u32 = 0x8000_0003;

/// One rung's outcome bundled with its ramped-stroke response metrics, both
/// computed from that rung's own captures before either capture's
/// `TransientBuffer` is reused for the next rung.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) struct RungEvidence {
    pub(crate) outcome: RungOutcome,
    pub(crate) metrics: RampedStrokeMetrics,
}

/// What the sweep does once one rung's evidence is complete.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) enum RungResolution {
    /// Neither stop condition tripped and a next rung is representable.
    /// `retained` becomes the next rung's `previous_evidence`.
    Advance {
        next: PidPGain,
        retained: RungEvidence,
    },
    /// A stop condition tripped; this is the accepted rung's full evidence.
    Accept(RungEvidence),
    /// The sweep cannot produce an accepted rung.
    Failed(PositionTuneOutcome),
}

/// Apply both stop conditions to one completed rung's evidence.
///
/// `previous` is `None` only at the floor rung. The accepted rung is always
/// the rung *before* the condition that tripped, for both stop conditions, so
/// this hands `resolve_rung` the previous rung's outcome and, on `Accept`,
/// returns the previous rung's retained evidence -- never the tripping rung's.
fn resolve_rung_evidence(
    previous: Option<RungEvidence>,
    current: RungEvidence,
    ceiling_tripped: bool,
) -> RungResolution {
    let lag_plateaued =
        previous.is_some_and(|prev| is_lag_plateau(prev.outcome.lag_us, current.outcome.lag_us));
    // At the floor rung `resolve_rung`'s only reachable results are `Conflict`
    // (a ceiling trip) and `Continue`, neither of which reads
    // `accepted_outcome`: there is no previous lag to plateau against, so this
    // rung's own outcome is a well-typed placeholder, not a claim that the
    // floor rung could accept itself.
    let accepted_outcome = previous.map_or(current.outcome, |prev| prev.outcome);
    match resolve_rung(
        previous.is_none(),
        ceiling_tripped,
        lag_plateaued,
        accepted_outcome,
    ) {
        SweepDecision::Conflict => {
            RungResolution::Failed(PositionTuneOutcome::FloorCeilingConflict)
        }
        SweepDecision::Accept(_) => {
            debug_assert!(
                previous.is_some(),
                "Accept is unreachable at the floor rung"
            );
            RungResolution::Accept(previous.unwrap_or(current))
        }
        SweepDecision::Continue => match next_rung(current.outcome.position_p) {
            Some(next) => RungResolution::Advance {
                next,
                retained: current,
            },
            None => RungResolution::Failed(PositionTuneOutcome::SweepExhausted),
        },
    }
}

/// Measured response of one rung's ramped reversal stroke, derived from a
/// `TransientBuffer` capture of `PID_ERROR_PID_POSITION_ERROR` samples across
/// the full `RAMPED_STROKE_CAPTURE_US` window.
///
/// A ramped reversal stroke at nonzero commanded velocity has a nonzero
/// steady-state following error by design, so every metric here is measured
/// relative to the tail-window steady-state reference, never relative to zero.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) struct RampedStrokeMetrics {
    /// Elapsed time, from stroke start, at which `|position_error -
    /// steady_state|` first drops below 10% of the observed peak absolute
    /// deviation from `steady_state` and stays below it for the remainder of
    /// the capture. Censored to the full window when it never does — settling
    /// time is a diagnostic, not a stop condition.
    pub settling_time_us: u32,
    /// Peak excursion **past** `steady_state`, in the direction opposite the
    /// stroke's initial approach. A monotonic approach that never crosses past
    /// `steady_state` reports `0`, not the approach distance.
    pub overshoot_counts: u32,
    /// The last recorded `position_error` sample, signed, in counts relative
    /// to zero — the actual final tracking error reported to the operator.
    pub final_position_error_counts: i32,
    /// True when `(position_error - steady_state)` crosses zero more than 2
    /// times after the peak absolute deviation from `steady_state`.
    pub oscillation_detected: bool,
}

fn compute_ramped_stroke_metrics(
    samples: &[i32],
    sample_period_us: u32,
) -> Result<RampedStrokeMetrics, PositionTuneOutcome> {
    // A failed, aborted, or never-started capture producing zero samples must
    // be a reported failure, not a panic -- `tail_len` below would otherwise
    // be 0, and dividing by it would panic.
    if samples.is_empty() {
        return Err(PositionTuneOutcome::CaptureEmpty);
    }

    // Steady-state reference: mean of the last 10% of samples (at least 1
    // sample). This is what "settled" means for this stroke -- whatever
    // following error persists once any transient has decayed, not zero.
    let tail_len = (samples.len() / 10).max(1).min(samples.len());
    let tail = &samples[samples.len() - tail_len..];
    let steady_state =
        i32::try_from(tail.iter().map(|&s| i64::from(s)).sum::<i64>() / tail.len() as i64)
            .unwrap_or(0);

    // Peak ABSOLUTE deviation from steady_state, in either direction -- used
    // below to anchor the oscillation and settling checks. Deliberately
    // undirected and distinct from `overshoot_counts`: it answers "how far did
    // the signal ever get from where it settles", which is what "has this
    // decayed yet" needs, regardless of which side that excursion was on.
    let mut peak_deviation: u32 = 0;
    let mut peak_index: usize = 0;
    for (i, &sample) in samples.iter().enumerate() {
        let deviation = i64::from(sample) - i64::from(steady_state);
        let deviation_abs = u32::try_from(deviation.unsigned_abs()).unwrap_or(u32::MAX);
        if deviation_abs > peak_deviation {
            peak_deviation = deviation_abs;
            peak_index = i;
        }
    }

    // Determine the approach side from the FIRST sample whose deviation from
    // steady_state is nonzero, not simply `samples[0]` -- an integer capture
    // can quantize its first sample to exactly steady_state, which would
    // misclassify the direction and report the following dip as overshoot.
    let approaching_from_below = samples
        .iter()
        .find(|&&s| s != steady_state)
        .map(|&s| s < steady_state);
    let Some(approaching_from_below) = approaching_from_below else {
        // Every sample equals steady_state: zero overshoot, skip the loop
        // below entirely rather than picking an arbitrary direction.
        return Ok(RampedStrokeMetrics {
            settling_time_us: 0,
            overshoot_counts: 0,
            final_position_error_counts: *samples.last().unwrap_or(&0),
            oscillation_detected: false,
        });
    };
    let mut overshoot: i64 = 0;
    for &sample in samples {
        let signed_excess = if approaching_from_below {
            i64::from(sample) - i64::from(steady_state)
        } else {
            i64::from(steady_state) - i64::from(sample)
        };
        if signed_excess > overshoot {
            overshoot = signed_excess;
        }
    }
    let overshoot_counts = u32::try_from(overshoot.max(0)).unwrap_or(u32::MAX);

    // Oscillation: zero-crossings of (sample - steady_state) strictly after
    // the peak-deviation sample -- ringing around the eventual settled value,
    // not around literal zero.
    let mut crossings = 0u8;
    for window in samples[peak_index..].windows(2) {
        let a = i64::from(window[0]) - i64::from(steady_state);
        let b = i64::from(window[1]) - i64::from(steady_state);
        if (a >= 0) != (b >= 0) {
            crossings = crossings.saturating_add(1);
        }
    }
    let oscillation_detected = crossings > 2;

    // Settling time: first index after which every remaining sample stays
    // within 10% of peak_deviation of steady_state, converted to elapsed
    // microseconds. Censored to the full window if it never settles.
    let settle_threshold = i64::from(peak_deviation) / 10;
    let mut settle_index = samples.len();
    'outer: for start in 0..samples.len() {
        for &sample in &samples[start..] {
            let deviation = (i64::from(sample) - i64::from(steady_state)).unsigned_abs();
            if i64::try_from(deviation).unwrap_or(i64::MAX) > settle_threshold {
                continue 'outer;
            }
        }
        settle_index = start;
        break;
    }
    let settling_time_us = u32::try_from(settle_index)
        .unwrap_or(u32::MAX)
        .saturating_mul(sample_period_us);

    Ok(RampedStrokeMetrics {
        settling_time_us,
        overshoot_counts,
        final_position_error_counts: *samples.last().unwrap_or(&0),
        oscillation_detected,
    })
}

/// Which sub-step of one rung's evaluation is in progress.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) enum PositionTuneStep {
    /// Measuring the acceleration-per-torque fit (runs once, before the first
    /// rung).
    MeasuringFloor,
    /// Capturing the current rung's ramped reversal stroke.
    RampedStroke { rung: PidPGain },
    /// Capturing the current rung's 5 standstill-dither observations.
    StandstillHold { rung: PidPGain, capture_index: u8 },
    /// Terminal: an accepted P.
    Done,
}

/// Phase-local state for the two-bound position-P ladder sweep.
#[derive(Debug, defmt::Format)]
pub(crate) struct PositionTuneState {
    pub(crate) step: PositionTuneStep,
    pub(crate) k_q16: i64,
    /// Retained from `compute_position_floor` at phase entry -- the reported
    /// inner-bandwidth field has no other producer.
    pub(crate) bw_inner_rad_s: i64,
    pub(crate) dither_captures: [i32; STANDSTILL_CAPTURE_COUNT as usize],
    /// The previous rung's full evidence -- outcome AND metrics together,
    /// computed and stored immediately after that rung's own captures
    /// complete, before its `TransientBuffer` is reused for the next rung's
    /// capture. `None` only before the floor rung's own evidence is computed.
    pub(crate) previous_evidence: Option<RungEvidence>,
    /// The *current* rung's `RampedStrokeMetrics`, computed the moment its
    /// `RampedStroke` capture completes -- before the five `StandstillHold`
    /// sub-steps that follow reuse the same `TransientBuffer`. Taken (and
    /// combined with the current rung's `RungOutcome` into a `RungEvidence`)
    /// once all five standstill captures complete. Deferring this until
    /// `RungOutcome` exists would read the buffer after the standstill
    /// captures have already overwritten the stroke.
    pub(crate) current_ramped_metrics: Option<RampedStrokeMetrics>,
    /// The accepted rung, retained so the terminal is idempotent if the
    /// engine steps a completed phase again.
    accepted: Option<RungEvidence>,
    /// Outer gains as they stood at Stage 2 entry.
    ///
    /// This phase tunes position P and nothing else, so every other field of
    /// the terminal `OuterResult` is reported from here. The `OuterDone`
    /// restore path stores `velocity_limit` and the four biquad cutoffs to the
    /// channel config unconditionally, with no zero guard, so a default-filled
    /// result would clamp the velocity controller's input to zero and leave the
    /// machine unable to move after a *successful* tune.
    entry: OuterGainRollbackSnapshot,
    /// Velocity P as read from the chip at phase entry: the gain the floor fit
    /// and the ceiling's `k` are actually measured against, which is not
    /// necessarily what the terminal reports.
    velocity_p: PidPGain,
    commanded_torque_count: i16,
    commanded_velocity_counts_per_s: i32,
    current_lag_us: u32,
    position_target: i32,
    /// False when a sub-step has been entered but its capture is not armed
    /// yet, so the next tick runs that sub-step's setup instead of sampling.
    capture_running: bool,
    next_sample_us: u64,
    deadline_us: u64,
    /// Absolute timestamp at which the ramped stroke reverses direction.
    reversal_us: u64,
}

impl PositionTuneState {
    /// Seed state for a fresh sweep.
    pub(crate) const fn new(entry: OuterGainRollbackSnapshot) -> Self {
        Self {
            step: PositionTuneStep::MeasuringFloor,
            k_q16: 0,
            bw_inner_rad_s: 0,
            dither_captures: [0; STANDSTILL_CAPTURE_COUNT as usize],
            previous_evidence: None,
            current_ramped_metrics: None,
            accepted: None,
            entry,
            velocity_p: PidPGain::ZERO,
            commanded_torque_count: 0,
            commanded_velocity_counts_per_s: 0,
            current_lag_us: 0,
            position_target: 0,
            capture_running: false,
            next_sample_us: 0,
            deadline_us: 0,
            reversal_us: 0,
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

/// Zero excitation, report the outcome, and transition the phase to `Failed`.
///
/// The entry-gain snapshot restore itself runs in
/// `commission_dispatch::restore_machine`, which every `StepResult::OuterFailed`
/// already drives; this is the same contract the other outer phases use.
async fn fail_position_tune<B: CommissioningBackend>(
    phase: &mut OuterPhase,
    backend: &mut B,
    fault: PositionTuneFault,
) -> StepResult {
    let _ = backend.write_register(PID_VELOCITY_TARGET, 0).await;
    let _ = backend.write_register(PID_TORQUE_FLUX_TARGET, 0).await;
    let _ = backend.write_register(MODE_RAMP_MODE_MOTION, 0).await;
    let error = match fault {
        PositionTuneFault::Engine(error) => error,
        PositionTuneFault::Outcome(outcome) => {
            defmt::warn!(
                "position tune failed: outcome={} wire_code={}",
                outcome,
                outcome.to_wire_code(),
            );
            CommissionError::PositionTuneFailed
        }
    };
    fail_outer(phase, error)
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

/// The fixed torque step the floor's acceleration-per-torque fit is measured
/// against, as a permille fraction of the configured current limit.
///
/// A zero result means the phase has no usable excitation to command at all,
/// which the fit would otherwise have to reject after energizing the motor for
/// nothing; report it as a rejected fit before commanding anything.
fn p_only_torque_count(torque_cap: u16) -> Result<i16, PositionTuneOutcome> {
    let scaled = u32::from(torque_cap) * P_ONLY_TORQUE_PERMILLE_OF_CAP / 1_000;
    i16::try_from(scaled)
        .ok()
        .filter(|count| *count > 0)
        .ok_or(PositionTuneOutcome::AccelerationFitRejected)
}

/// Commanded velocity for the ramped reversal stroke, in mechanical
/// millirevolutions per second.
///
/// The stroke runs at a constant commanded velocity for half the capture
/// window in each direction, so its peak excursion from the start position is
/// that velocity times half the window. Size the commanded velocity so that
/// peak stays inside half the host-declared stroke window, leaving the rest as
/// margin for the reversal transient, rather than relying on the per-tick
/// envelope check to abort a stroke that was planned too large.
fn ramp_velocity_mrev_s(budget: OuterMotionBudget) -> Result<u32, PositionTuneOutcome> {
    let half_stroke_us = u64::from(RAMPED_STROKE_CAPTURE_US / 2);
    let excursion_allowance_mrev = u64::from(budget.max_stroke_travel_mrev / 2);
    let allowance_mrev_s = excursion_allowance_mrev.saturating_mul(1_000_000) / half_stroke_us;
    let velocity_mrev_s = u64::from(budget.requested_velocity_mrev_s).min(allowance_mrev_s);
    u32::try_from(velocity_mrev_s)
        .ok()
        .filter(|value| *value > 0)
        .ok_or(PositionTuneOutcome::AccelerationFitRejected)
}

/// The same commanded velocity in the encoder-count domain the position
/// target and the tracking-lag conversion both work in.
fn velocity_counts_per_s(
    velocity_mrev_s: u32,
    counts_per_rev: u32,
) -> Result<i32, PositionTuneOutcome> {
    let scaled = u64::from(velocity_mrev_s).saturating_mul(u64::from(counts_per_rev)) / 1_000;
    i32::try_from(scaled)
        .ok()
        .filter(|value| *value > 0)
        .ok_or(PositionTuneOutcome::AccelerationFitRejected)
}

/// Position-target increment per ramp tick, in encoder counts.
fn ramp_step_counts(velocity_counts_per_s: i32, period_us: u32) -> i32 {
    let scaled = i64::from(velocity_counts_per_s).saturating_mul(i64::from(period_us)) / 1_000_000;
    i32::try_from(scaled).unwrap_or(i32::MAX)
}

/// Convert one signed position-error sample and the commanded velocity active
/// during that stroke into a time-domain tracking lag.
///
/// Both inputs are signed (a reversal stroke commands both directions, and
/// leading versus following error have opposite signs); the plateau check only
/// compares lag *magnitudes*, so take absolute values in a widened domain
/// before dividing. `None` when the commanded velocity is zero -- a rejected
/// capture, never a silent lag of zero.
fn position_lag_us(
    position_error_counts: i32,
    commanded_velocity_counts_per_s: i32,
) -> Option<u32> {
    let abs_position_error = i64::from(position_error_counts).unsigned_abs();
    let abs_velocity = i64::from(commanded_velocity_counts_per_s).unsigned_abs();
    if abs_velocity == 0 {
        return None;
    }
    let lag_us = abs_position_error.saturating_mul(1_000_000) / abs_velocity;
    Some(u32::try_from(lag_us).unwrap_or(u32::MAX))
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

/// Reconstruct the signed ramped-stroke capture and reduce it to metrics.
///
/// Deliberately a synchronous helper: its full-length sample array is a plain
/// stack frame here, and never becomes part of the phase's async future.
fn ramped_metrics(workspace: &TransientBuffer) -> Result<RampedStrokeMetrics, PositionTuneOutcome> {
    let captured = workspace.samples();
    let mut samples = [0_i32; TRANSIENT_LEN];
    for (slot, &raw) in samples.iter_mut().zip(captured) {
        *slot = raw as i32;
    }
    compute_ramped_stroke_metrics(&samples[..captured.len()], RAMPED_STROKE_SAMPLE_PERIOD_US)
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

/// Read the velocity loop's already-selected gains, which both the floor fit
/// and the ceiling's `k` constant are derived from.
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

/// Arm the one-shot P-only torque step the floor fit is measured from.
async fn arm_floor_capture<B: CommissioningBackend>(
    state: &mut PositionTuneState,
    safety: &mut OuterSafetyLimits,
    workspace: &mut TransientBuffer,
    backend: &mut B,
) -> Result<(), PositionTuneFault> {
    if backend.is_shutdown_requested() {
        return Err(CommissionError::Shutdown.into());
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

    let ramp_velocity_mrev_s = ramp_velocity_mrev_s(safety.budget)?;
    safety.configure_velocity_envelope(ramp_velocity_mrev_s)?;
    state.commanded_velocity_counts_per_s =
        velocity_counts_per_s(ramp_velocity_mrev_s, counts_per_rev)?;

    safety.torque_cap = u16::from(backend.current_limit());
    state.commanded_torque_count = p_only_torque_count(safety.torque_cap)?;

    let velocity_gains = read_velocity_gains(backend).await?;
    state.velocity_p = velocity_gains.p;
    // `compute_k` takes encoder pulses per revolution; the safety domain and
    // the backend both work in quadrature counts, which are four per pulse.
    state.k_q16 = compute_k(counts_per_rev / 4, velocity_gains.p)?;

    safety.arm_safety_observation(backend.elapsed_us())?;

    let torque_target = PidTorqueFluxTarget::new()
        .with_pid_flux_target(0)
        .with_pid_torque_target(state.commanded_torque_count);
    write_register(backend, PID_TORQUE_FLUX_TARGET, torque_target.into()).await?;
    write_register(backend, MODE_RAMP_MODE_MOTION, POSITION_TUNE_TORQUE_MODE).await?;

    let started_us = backend.elapsed_us();
    safety.begin_primitive(started_us)?;
    workspace.reset(P_ONLY_SAMPLE_PERIOD_US as u16);
    start_capture(
        state,
        started_us,
        P_ONLY_SAMPLE_PERIOD_US,
        P_ONLY_CAPTURE_US,
    )?;
    Ok(())
}

/// Stop the torque step and fit the floor from the captured onset slope.
async fn finish_floor_capture<B: CommissioningBackend>(
    state: &mut PositionTuneState,
    safety: &mut OuterSafetyLimits,
    workspace: &TransientBuffer,
    backend: &mut B,
) -> Result<(), PositionTuneFault> {
    write_register(backend, PID_TORQUE_FLUX_TARGET, 0).await?;
    write_register(backend, MODE_RAMP_MODE_MOTION, 0).await?;
    safety.end_primitive();

    let captured = workspace.samples();
    if captured.is_empty() {
        return Err(PositionTuneOutcome::CaptureEmpty.into());
    }
    if captured.len() != FLOOR_EXPECTED_SAMPLES {
        return Err(PositionTuneOutcome::AccelerationFitRejected.into());
    }
    let mut samples = [(0_u32, 0_i32); FLOOR_EXPECTED_SAMPLES];
    for (index, (slot, &raw)) in samples.iter_mut().zip(captured).enumerate() {
        *slot = (index as u32 * P_ONLY_SAMPLE_PERIOD_US, raw as i32);
    }
    let floor_result = compute_position_floor(
        &samples,
        i32::from(state.commanded_torque_count),
        state.velocity_p,
    )?;
    state.bw_inner_rad_s = floor_result.bw_inner_rad_s;
    defmt::info!(
        "position tune floor: raw={} bw_inner_rad_s={}",
        floor_result.floor.raw(),
        floor_result.bw_inner_rad_s,
    );
    state.step = PositionTuneStep::RampedStroke {
        rung: floor_result.floor,
    };
    state.capture_running = false;
    Ok(())
}

/// Install one rung's position P and arm its ramped reversal stroke.
async fn arm_ramped_stroke<B: CommissioningBackend>(
    state: &mut PositionTuneState,
    safety: &mut OuterSafetyLimits,
    workspace: &mut TransientBuffer,
    backend: &mut B,
    rung: PidPGain,
) -> Result<(), PositionTuneFault> {
    write_register(
        backend,
        PID_POSITION_P_POSITION_I,
        PidGainPair::new(rung, PidIGain::ZERO).register_value(),
    )
    .await?;
    write_sub_register(backend, PID_ERROR_PID_POSITION_ERROR_SUM, 0).await?;
    let position_actual = read_capture(backend, PID_POSITION_ACTUAL).await? as i32;
    state.position_target = position_actual;
    write_register(backend, PID_POSITION_TARGET, position_actual as u32).await?;
    write_register(backend, PID_VELOCITY_TARGET, 0).await?;
    write_register(backend, MODE_RAMP_MODE_MOTION, POSITION_TUNE_POSITION_MODE).await?;

    let started_us = backend.elapsed_us();
    safety.begin_primitive(started_us)?;
    state.reversal_us = started_us
        .checked_add(u64::from(RAMPED_STROKE_CAPTURE_US / 2))
        .ok_or(CommissionError::ScheduleDeadlineOverflow)?;
    workspace.reset(RAMPED_STROKE_SAMPLE_PERIOD_US as u16);
    start_capture(
        state,
        started_us,
        RAMPED_STROKE_SAMPLE_PERIOD_US,
        RAMPED_STROKE_CAPTURE_US,
    )?;
    Ok(())
}

/// Arm one standstill hold: the position target stays where the stroke left
/// it and the velocity target is zero, so the only torque command left is the
/// loop's own dither.
async fn arm_standstill_hold<B: CommissioningBackend>(
    state: &mut PositionTuneState,
    safety: &mut OuterSafetyLimits,
    workspace: &mut TransientBuffer,
    backend: &mut B,
) -> Result<(), PositionTuneFault> {
    write_register(backend, PID_VELOCITY_TARGET, 0).await?;
    write_register(backend, PID_POSITION_TARGET, state.position_target as u32).await?;

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

/// Fold one completed rung's five standstill captures and its already-retained
/// ramped-stroke metrics into a decision.
fn resolve_completed_rung(
    state: &mut PositionTuneState,
    rung: PidPGain,
) -> Result<(), PositionTuneFault> {
    let (ceiling_tripped, ceiling_ratios_milli) =
        ceiling_trips(&state.dither_captures, state.k_q16, rung);
    let Some(metrics) = state.current_ramped_metrics.take() else {
        debug_assert!(
            false,
            "the ramped-stroke metrics are retained before the standstill captures start"
        );
        return Err(CommissionError::MissingCaptureState.into());
    };
    let evidence = RungEvidence {
        outcome: RungOutcome {
            position_p: rung,
            lag_us: state.current_lag_us,
            ceiling_ratios_milli,
        },
        metrics,
    };
    match resolve_rung_evidence(state.previous_evidence, evidence, ceiling_tripped) {
        RungResolution::Advance { next, retained } => {
            state.previous_evidence = Some(retained);
            state.step = PositionTuneStep::RampedStroke { rung: next };
            state.capture_running = false;
            Ok(())
        }
        RungResolution::Accept(accepted) => {
            state.accepted = Some(accepted);
            state.step = PositionTuneStep::Done;
            state.capture_running = false;
            Ok(())
        }
        RungResolution::Failed(outcome) => Err(outcome.into()),
    }
}

/// Nominal position-loop bandwidth: a pure function of the accepted
/// position P and unit constants, no torque constant or inertia (spec,
/// Reporting section).
fn nominal_bandwidth_hz(position_p_raw: u16) -> u32 {
    let bandwidth_hz =
        (f32::from(position_p_raw) / 256.0) * 65_536.0 / (60.0 * 2.0 * core::f32::consts::PI);
    if bandwidth_hz.is_finite() && bandwidth_hz > 0.0 {
        (bandwidth_hz + 0.5) as u32
    } else {
        0
    }
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
/// rather than defaulted.
fn position_tune_terminal(state: &PositionTuneState, accepted: RungEvidence) -> StepResult {
    defmt::info!(
        "position tune accepted: p={} bw_inner_rad_s={} lag_us={} settling_us={} overshoot={} final_error={} oscillation={}",
        accepted.outcome.position_p.raw(),
        state.bw_inner_rad_s,
        accepted.outcome.lag_us,
        accepted.metrics.settling_time_us,
        accepted.metrics.overshoot_counts,
        accepted.metrics.final_position_error_counts,
        accepted.metrics.oscillation_detected,
    );
    let position_tune = PositionTuneProvenance {
        nominal_bandwidth_hz: nominal_bandwidth_hz(accepted.outcome.position_p.raw()),
        inner_bandwidth_rad_s: state.bw_inner_rad_s.clamp(0, i64::from(u32::MAX)) as u32,
        dither_margin_milli: dither_margin_milli(accepted.outcome.ceiling_ratios_milli),
        settling_time_us: accepted.metrics.settling_time_us,
        overshoot_counts: accepted.metrics.overshoot_counts,
        final_position_error_counts: accepted.metrics.final_position_error_counts,
        oscillation_detected: accepted.metrics.oscillation_detected,
    };
    StepResult::OuterDone(OuterResult {
        status: CommissionStatus::Accepted,
        velocity_p: state.entry.velocity_p.raw(),
        velocity_i: state.entry.velocity_i.raw(),
        velocity_limit: state.entry.velocity_limit,
        position_p: accepted.outcome.position_p.raw(),
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
        PositionTuneStep::MeasuringFloor => {
            if !state.capture_running {
                return match arm_floor_capture(state, safety, workspace, backend).await {
                    Ok(()) => StepResult::PhaseChanged(CommissionPhase::PositionTune),
                    Err(fault) => fail_position_tune(phase, backend, fault).await,
                };
            }
            let now_us = backend.elapsed_us();
            if now_us < state.next_sample_us {
                return StepResult::At(state.next_sample_us);
            }
            let velocity_rpm = match read_capture(backend, PID_VELOCITY_ACTUAL).await {
                Ok(value) => value,
                Err(error) => return fail_position_tune(phase, backend, error.into()).await,
            };
            workspace.push(velocity_rpm);
            let now_us = backend.elapsed_us();
            if now_us < state.deadline_us {
                return match advance_sample(state, P_ONLY_SAMPLE_PERIOD_US, now_us) {
                    Ok(next_sample_us) => StepResult::At(next_sample_us),
                    Err(error) => fail_position_tune(phase, backend, error.into()).await,
                };
            }
            match finish_floor_capture(state, safety, workspace, backend).await {
                Ok(()) => StepResult::ReadyNow,
                Err(fault) => fail_position_tune(phase, backend, fault).await,
            }
        }
        PositionTuneStep::RampedStroke { rung } => {
            if !state.capture_running {
                return match arm_ramped_stroke(state, safety, workspace, backend, rung).await {
                    Ok(()) => StepResult::At(state.next_sample_us),
                    Err(fault) => fail_position_tune(phase, backend, fault).await,
                };
            }
            let now_us = backend.elapsed_us();
            if now_us < state.next_sample_us {
                return StepResult::At(state.next_sample_us);
            }
            let direction = if now_us < state.reversal_us { 1 } else { -1 };
            let step_counts = ramp_step_counts(
                state.commanded_velocity_counts_per_s,
                RAMPED_STROKE_SAMPLE_PERIOD_US,
            );
            state.position_target = state
                .position_target
                .saturating_add(direction * step_counts);
            let position_target = state.position_target as u32;
            if let Err(error) = write_register(backend, PID_POSITION_TARGET, position_target).await
            {
                return fail_position_tune(phase, backend, error.into()).await;
            }
            let position_error =
                match read_sub_register(backend, PID_ERROR_PID_POSITION_ERROR).await {
                    Ok(value) => value,
                    Err(error) => return fail_position_tune(phase, backend, error.into()).await,
                };
            workspace.push(position_error);
            let now_us = backend.elapsed_us();
            if now_us < state.deadline_us {
                return match advance_sample(state, RAMPED_STROKE_SAMPLE_PERIOD_US, now_us) {
                    Ok(next_sample_us) => StepResult::At(next_sample_us),
                    Err(error) => fail_position_tune(phase, backend, error.into()).await,
                };
            }
            safety.end_primitive();
            // The ramped stroke and the five standstill holds share one
            // `TransientBuffer`, so this rung's metrics are pulled out here,
            // the moment its own capture completes, and retained until they
            // are bundled into this rung's `RungEvidence`. Computing them
            // after `resolve_rung` has already decided would read whatever
            // capture happens to be in the buffer by then.
            match ramped_metrics(workspace).and_then(|metrics| {
                let lag_us = position_lag_us(
                    metrics.final_position_error_counts,
                    state.commanded_velocity_counts_per_s,
                )
                .ok_or(PositionTuneOutcome::AccelerationFitRejected)?;
                Ok((metrics, lag_us))
            }) {
                Ok((metrics, lag_us)) => {
                    state.current_ramped_metrics = Some(metrics);
                    state.current_lag_us = lag_us;
                    state.step = PositionTuneStep::StandstillHold {
                        rung,
                        capture_index: 0,
                    };
                    state.capture_running = false;
                    StepResult::ReadyNow
                }
                Err(outcome) => fail_position_tune(phase, backend, outcome.into()).await,
            }
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
            // evidence; every failure path replaces the phase instead.
            None => {
                fail_position_tune(phase, backend, CommissionError::MissingCaptureState.into())
                    .await
            }
        },
    }
}

#[cfg(test)]
mod tests {
    use super::ceiling::{ceiling_trips, compute_k};
    use super::floor::compute_position_floor;
    use super::selection::{RungOutcome, SweepDecision, next_rung, resolve_rung};
    use super::*;
    use crate::autotune::PositionTuneOutcome;
    use crate::commission_dispatch::test_support::{MockBackend, current_limit};
    use foci_tmc4671::pid::PidPGain;

    const PERIOD_US: u32 = 2_000;

    fn metrics_tagged(settling_time_us: u32) -> RampedStrokeMetrics {
        RampedStrokeMetrics {
            settling_time_us,
            overshoot_counts: 0,
            final_position_error_counts: 0,
            oscillation_detected: false,
        }
    }

    fn evidence(position_p_raw: u16, lag_us: u32, settling_time_us: u32) -> RungEvidence {
        RungEvidence {
            outcome: RungOutcome {
                position_p: PidPGain::new(position_p_raw).unwrap(),
                lag_us,
                ceiling_ratios_milli: [1_000; 5],
            },
            metrics: metrics_tagged(settling_time_us),
        }
    }

    #[test]
    fn ceiling_trip_after_the_floor_accepts_the_previous_rungs_own_metrics() {
        // Rung A (previous, quiet) and rung B (current, tripping) carry
        // distinguishable metrics. Accepting B's metrics here would be the
        // retention-timing bug: by the time `resolve_rung` decides, the shared
        // TransientBuffer already holds B's capture, not A's.
        let rung_a = evidence(63, 1_000, 11_111);
        let rung_b = evidence(89, 960, 22_222);

        let resolution =
            resolve_rung_evidence(Some(rung_a), rung_b, /* ceiling_tripped */ true);

        let RungResolution::Accept(accepted) = resolution else {
            panic!("a ceiling trip after the floor rung must accept the previous rung");
        };
        assert_eq!(accepted.outcome.position_p.raw(), 63);
        assert_eq!(accepted.metrics.settling_time_us, 11_111);
    }

    #[test]
    fn lag_plateau_accepts_the_previous_rungs_own_metrics() {
        // 3% improvement versus the previous rung is a plateau.
        let rung_a = evidence(63, 1_000, 11_111);
        let rung_b = evidence(89, 970, 22_222);

        let resolution =
            resolve_rung_evidence(Some(rung_a), rung_b, /* ceiling_tripped */ false);

        let RungResolution::Accept(accepted) = resolution else {
            panic!("a lag plateau must accept the previous rung");
        };
        assert_eq!(accepted.outcome.position_p.raw(), 63);
        assert_eq!(accepted.metrics.settling_time_us, 11_111);
    }

    #[test]
    fn ceiling_trip_at_the_floor_rung_reports_conflict() {
        let floor_rung = evidence(63, 1_000, 11_111);

        let resolution = resolve_rung_evidence(None, floor_rung, /* ceiling_tripped */ true);

        assert_eq!(
            resolution,
            RungResolution::Failed(PositionTuneOutcome::FloorCeilingConflict)
        );
    }

    #[test]
    fn a_quiet_improving_rung_advances_and_retains_its_own_evidence() {
        let rung_a = evidence(63, 1_000, 11_111);
        let rung_b = evidence(89, 500, 22_222);

        let resolution =
            resolve_rung_evidence(Some(rung_a), rung_b, /* ceiling_tripped */ false);

        let RungResolution::Advance { next, retained } = resolution else {
            panic!("a rung that still improves and stays quiet must advance");
        };
        assert!(next.raw() > 89);
        assert_eq!(retained.metrics.settling_time_us, 22_222);
    }

    #[test]
    fn a_quiet_rung_with_no_representable_successor_reports_sweep_exhausted() {
        // `resolve_rung` never produces `Exhausted` itself: the ladder
        // reaching PidPGain::MAX is observed at `next_rung`'s `None`.
        let previous = evidence(PidPGain::MAX.raw(), 1_000, 11_111);
        let current = evidence(PidPGain::MAX.raw(), 100, 22_222);

        let resolution = resolve_rung_evidence(Some(previous), current, false);

        assert_eq!(
            resolution,
            RungResolution::Failed(PositionTuneOutcome::SweepExhausted)
        );
    }

    /// Floor fixture: a 195 RPM per 1 kHz sample onset against a 1000-count
    /// torque step -- the same 195 RPM/s-per-count slope the floor unit test
    /// verifies produces a raw floor of 63, expressed per sample index so the
    /// fit is independent of how many samples the window happens to take.
    const FLOOR_VELOCITY_RPM_PER_SAMPLE: u32 = 195;
    const FLOOR_RUNG_RAW: u16 = 63;
    /// Distinguishable steady following errors, so the accepted rung's
    /// retained metrics identify which rung's capture they came from.
    const FLOOR_RUNG_POSITION_ERROR: i32 = 100;
    const SECOND_RUNG_POSITION_ERROR: i32 = 50;
    /// Peak-to-peak torque-command dither per rung: 5 counts sits near the
    /// quantization-only prediction at rung 63, 40 counts is far past the 2x
    /// margin at rung 89.
    const FLOOR_RUNG_DITHER: i32 = 5;
    const SECOND_RUNG_DITHER: i32 = 40;

    /// Entry gains with a distinct nonzero value in every field this phase
    /// does not tune, so a defaulted terminal field reads as zero rather than
    /// coincidentally matching.
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
            state: PositionTuneState::new(position_tune_entry_snapshot()),
            safety: OuterSafetyLimits::new(0, OuterMotionBudget::firmware_caps()),
        }
    }

    fn position_tune_backend() -> MockBackend {
        let mut backend = MockBackend::new();
        backend.spi_elapsed_step_us = 0;
        backend.elapsed_us_value = 1_000_000;
        backend.current_limit_value = current_limit(5_000);
        backend.mechanical_encoder_quadrature_counts_per_rev_value = 4_000;
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

    /// Drive the phase to a terminal `StepResult`, feeding each sub-step the
    /// synthetic capture its next read will consume.
    ///
    /// Sample values are keyed off the workspace's own length rather than a
    /// tick counter: a sub-step can report `At` without sampling (a not-yet-due
    /// tick), and only an actual push advances the capture.
    fn drive_sweep(
        phase: &mut OuterPhase,
        backend: &mut MockBackend,
        floor_rung_dither: i32,
    ) -> StepResult {
        let mut workspace = TransientBuffer::new();
        let mut clock_us = backend.elapsed_us_value;

        for _ in 0..4_000 {
            let OuterPhase::PositionTune { state, .. } = &*phase else {
                panic!("the phase must stay resident until it reports a terminal");
            };
            let sample_index = u32::from(workspace.len());
            match state.step {
                PositionTuneStep::MeasuringFloor => {
                    let _ = backend.registers.insert(
                        PID_VELOCITY_ACTUAL.addr(),
                        FLOOR_VELOCITY_RPM_PER_SAMPLE * sample_index,
                    );
                }
                PositionTuneStep::RampedStroke { rung } => {
                    let error = if rung.raw() == FLOOR_RUNG_RAW {
                        FLOOR_RUNG_POSITION_ERROR
                    } else {
                        SECOND_RUNG_POSITION_ERROR
                    };
                    let _ = backend
                        .registers
                        .insert(PID_ERROR_PID_POSITION_ERROR.data.addr(), error as u32);
                }
                PositionTuneStep::StandstillHold { rung, .. } => {
                    let amplitude = if rung.raw() == FLOOR_RUNG_RAW {
                        floor_rung_dither
                    } else {
                        SECOND_RUNG_DITHER
                    };
                    let torque = if sample_index % 2 == 0 { 0 } else { amplitude };
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

    #[test]
    fn a_driven_sweep_accepts_the_floor_rung_when_the_next_rung_trips_the_ceiling() {
        let mut backend = position_tune_backend();
        let mut phase = position_tune_phase();

        let terminal = drive_sweep(&mut phase, &mut backend, FLOOR_RUNG_DITHER);

        let StepResult::OuterDone(result) = terminal else {
            panic!("a completed sweep reports OuterDone, got {terminal:?}");
        };
        assert_eq!(result.position_p, FLOOR_RUNG_RAW);
        assert_eq!(result.velocity_p, 350);
    }

    #[test]
    fn a_successful_sweep_carries_every_untuned_entry_gain_into_its_terminal() {
        // The OuterDone restore path stores velocity_limit and the four biquad
        // cutoffs to the channel config with no zero guard, so a terminal built
        // from OuterResult::default() would leave the velocity controller's
        // input clamped to zero after a *successful* tune.
        let entry = position_tune_entry_snapshot();
        let mut backend = position_tune_backend();
        let mut phase = position_tune_phase();

        let terminal = drive_sweep(&mut phase, &mut backend, FLOOR_RUNG_DITHER);

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
        assert_eq!(result.position_p, FLOOR_RUNG_RAW);
        assert_ne!(result.position_p, entry.position_p.raw());
    }

    #[test]
    fn a_driven_sweep_reports_the_accepted_rungs_own_ramped_stroke_metrics() {
        // The regression this phase's capture ordering exists to prevent: rung
        // 89's ramped stroke, and then its five standstill holds, both reuse
        // the buffer rung 63's stroke was captured into. Reporting rung 89's
        // final position error here would mean the metrics were read after the
        // accept decision rather than retained when rung 63's own capture
        // completed.
        let mut backend = position_tune_backend();
        let mut phase = position_tune_phase();

        let _ = drive_sweep(&mut phase, &mut backend, FLOOR_RUNG_DITHER);

        let OuterPhase::PositionTune { state, .. } = &phase else {
            panic!("the phase stays resident after a successful terminal");
        };
        let accepted = state.accepted.expect("a completed sweep retains its rung");
        assert_eq!(accepted.outcome.position_p.raw(), FLOOR_RUNG_RAW);
        assert_eq!(
            accepted.metrics.final_position_error_counts,
            FLOOR_RUNG_POSITION_ERROR
        );
        assert_eq!(state.bw_inner_rad_s, 266);
        assert_ne!(
            accepted.metrics.final_position_error_counts,
            SECOND_RUNG_POSITION_ERROR
        );
    }

    #[test]
    fn a_ceiling_trip_at_the_floor_rung_fails_the_phase() {
        let mut backend = position_tune_backend();
        let mut phase = position_tune_phase();
        // The floor rung itself is loud, so the very first rung tested trips.
        let terminal = drive_sweep(&mut phase, &mut backend, SECOND_RUNG_DITHER);

        assert!(matches!(
            terminal,
            StepResult::OuterFailed(CommissionError::PositionTuneFailed)
        ));
        assert!(matches!(
            phase,
            OuterPhase::Failed(CommissionError::PositionTuneFailed)
        ));
    }

    #[test]
    fn full_pipeline_accepts_the_previous_rung_on_a_ceiling_trip() {
        // Floor: reuse Task 3's exact verified fixture (velocity_p=350,
        // commanded_torque_count=1000 -> floor raw 63).
        let floor_samples: [(u32, i32); 5] = [
            (0, 0),
            (1_000, 195),
            (2_000, 390),
            (3_000, 585),
            (4_000, 780),
        ];
        let velocity_p = PidPGain::new(350).unwrap();
        let floor_result =
            compute_position_floor(&floor_samples, 1_000, velocity_p).expect("valid floor fit");
        let floor = floor_result.floor;
        assert_eq!(floor.raw(), 63);

        // Ceiling: k for PPR=1000, velocity_p=350.
        let k_q16 = compute_k(1_000, velocity_p).expect("valid k");

        // At the floor rung, synthesize 5 standstill captures that all match
        // the pure-quantization prediction closely (no trip) -- the sweep must
        // continue past the floor rung.
        let predicted_at_floor = k_q16.saturating_mul(i64::from(floor.raw())) / 65_536;
        let predicted_at_floor_i32 = i32::try_from(predicted_at_floor).unwrap();
        let quiet_captures: [i32; 5] = [
            predicted_at_floor_i32,
            predicted_at_floor_i32 * 105 / 100,
            predicted_at_floor_i32 * 95 / 100,
            predicted_at_floor_i32 * 102 / 100,
            predicted_at_floor_i32 * 98 / 100,
        ];
        let (floor_ceiling_tripped, floor_ratios) = ceiling_trips(&quiet_captures, k_q16, floor);
        assert!(!floor_ceiling_tripped);

        // This is the "previous rung" outcome the eventual ceiling trip below
        // must be accepted with -- retained by the phase driver across the loop.
        let floor_outcome = RungOutcome {
            position_p: floor,
            lag_us: 1_000,
            ceiling_ratios_milli: floor_ratios,
        };
        let floor_decision = resolve_rung(
            /* is_first_rung */ true,
            floor_ceiling_tripped,
            /* lag_plateaued */ false,
            floor_outcome,
        );
        assert_eq!(floor_decision, SweepDecision::Continue);

        // Next rung: synthesize captures that clearly trip the ceiling (well
        // over the 2x margin on at least 3 of 5).
        let next = next_rung(floor).expect("representable next rung");
        let predicted_at_next = k_q16.saturating_mul(i64::from(next.raw())) / 65_536;
        let predicted_at_next_i32 = i32::try_from(predicted_at_next).unwrap();
        let loud_captures: [i32; 5] = [
            predicted_at_next_i32 * 3,
            predicted_at_next_i32 * 3,
            predicted_at_next_i32,
            predicted_at_next_i32 * 3,
            predicted_at_next_i32,
        ];
        let (next_ceiling_tripped, _next_ratios) = ceiling_trips(&loud_captures, k_q16, next);
        assert!(next_ceiling_tripped);

        // The accepted outcome passed here is `floor_outcome` (the previous
        // rung), NOT a `RungOutcome` for `next` -- `next` is the rung that
        // violated the ceiling, and must never be the accepted result.
        let next_decision = resolve_rung(
            /* is_first_rung */ false,
            next_ceiling_tripped,
            false,
            floor_outcome,
        );
        assert!(
            matches!(next_decision, SweepDecision::Accept(outcome) if outcome.position_p == floor)
        );
    }

    #[test]
    fn stable_following_error_reports_no_overshoot_and_no_oscillation() {
        let samples = [50_i32; 20];
        let metrics = compute_ramped_stroke_metrics(&samples, PERIOD_US).expect("valid capture");

        assert_eq!(metrics.overshoot_counts, 0);
        assert!(!metrics.oscillation_detected);
        assert_eq!(metrics.settling_time_us, 0);
        assert_eq!(metrics.final_position_error_counts, 50);
    }

    #[test]
    fn monotonic_approach_to_nonzero_steady_state_is_not_overshoot() {
        let samples: [i32; 10] = [0, 10, 20, 30, 40, 48, 50, 50, 50, 50];
        let metrics = compute_ramped_stroke_metrics(&samples, PERIOD_US).expect("valid capture");

        assert_eq!(metrics.overshoot_counts, 0);
        assert!(!metrics.oscillation_detected);
        assert_eq!(metrics.final_position_error_counts, 50);
    }

    #[test]
    fn genuine_overshoot_is_measured_past_the_settled_value() {
        let samples: [i32; 12] = [0, 20, 50, 80, 110, 120, 100, 70, 55, 50, 50, 50];
        let metrics = compute_ramped_stroke_metrics(&samples, PERIOD_US).expect("valid capture");

        assert_eq!(metrics.overshoot_counts, 70);
        assert!(metrics.settling_time_us > 5 * PERIOD_US);
        assert_eq!(metrics.final_position_error_counts, 50);
    }

    #[test]
    fn sustained_ringing_after_the_peak_is_detected() {
        let samples: [i32; 12] = [0, 40, 90, 30, 70, 30, 70, 30, 70, 50, 50, 50];
        let metrics = compute_ramped_stroke_metrics(&samples, PERIOD_US).expect("valid capture");

        assert!(metrics.oscillation_detected);
    }

    #[test]
    fn first_sample_equal_to_steady_state_does_not_flip_the_approach_direction() {
        let samples: [i32; 10] = [50, 40, 45, 49, 50, 50, 50, 50, 50, 50];
        let metrics = compute_ramped_stroke_metrics(&samples, PERIOD_US).expect("valid capture");

        assert_eq!(metrics.overshoot_counts, 0);
    }

    #[test]
    fn empty_capture_is_a_reported_failure_not_a_panic() {
        assert_eq!(
            compute_ramped_stroke_metrics(&[], PERIOD_US),
            Err(PositionTuneOutcome::CaptureEmpty)
        );
    }
}
