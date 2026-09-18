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
use crate::commissioning::outer::{
    OuterMotionBudget, OuterPhase, OuterSafetyLimits, fail_outer, fail_outer_position_tune,
};
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
use selection::{RungOutcome, is_new_running_best, next_rung};

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
pub(crate) const STANDSTILL_CAPTURE_COUNT: u8 = 5;

/// Ramped-stroke repeats captured per rung; selection acts on their median
/// `lag_us`.
pub(crate) const RAMPED_STROKE_REPEATS: u8 = 3;

/// Samples the floor's onset window yields on an un-slipped sample grid.
///
/// The fit reconstructs each sample's timestamp from its index, so a capture
/// that collected any other number of samples has a grid slip somewhere in it
/// and its implied slope would be wrong.
const FLOOR_EXPECTED_SAMPLES: usize = (P_ONLY_CAPTURE_US / P_ONLY_SAMPLE_PERIOD_US) as usize;

/// Sample index at which the ramped stroke's commanded direction reverses.
///
/// `arm_ramped_stroke` sets the reversal at exactly half the capture window,
/// and the stroke pushes one sample per `RAMPED_STROKE_SAMPLE_PERIOD_US`, so
/// the first leg is the samples below this index and the second leg is the
/// rest. A capture that ended early simply never reaches it.
///
/// The naive `CAPTURE_US / 2 / SAMPLE_PERIOD_US` is one sample too high: the
/// first sample is taken one period after the capture starts, not at the
/// start itself, so the sample whose timestamp lands exactly on the
/// reversal (index `naive - 1`) is already commanded in the reverse
/// direction and belongs in the second leg, not the first.
const RAMPED_STROKE_REVERSAL_SAMPLE_INDEX: usize =
    (RAMPED_STROKE_CAPTURE_US / 2 / RAMPED_STROKE_SAMPLE_PERIOD_US) as usize - 1;

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
    Advance {
        next: PidPGain,
        running_best: RungEvidence,
    },
    /// A ceiling trip after the floor rung stopped the walk.
    Accept(RungEvidence),
    /// The sweep cannot produce an accepted rung.
    Failed(PositionTuneOutcome),
}

/// Fold one completed rung's evidence into the exhaustive walk.
fn resolve_rung_evidence(
    running_best: Option<RungEvidence>,
    current: RungEvidence,
    ceiling_tripped: bool,
) -> RungResolution {
    let is_first_rung = running_best.is_none();
    let running_best_lag_us = running_best.map(|evidence| evidence.outcome.lag_us);
    defmt::info!(
        "position tune rung: p={} lag_us={} ceiling_tripped={} is_first_rung={} running_best_lag_us={}",
        current.outcome.position_p.raw(),
        current.outcome.lag_us,
        ceiling_tripped,
        is_first_rung,
        running_best_lag_us,
    );
    if ceiling_tripped {
        return if is_first_rung {
            RungResolution::Failed(PositionTuneOutcome::FloorCeilingConflict)
        } else {
            RungResolution::Accept(
                running_best.expect("a non-first rung always has a running best"),
            )
        };
    }
    let updated_best = if is_new_running_best(current.outcome.lag_us, running_best_lag_us) {
        current
    } else {
        running_best.expect("a non-improving comparison always has a running best")
    };
    match next_rung(current.outcome.position_p) {
        Some(next) => RungResolution::Advance {
            next,
            running_best: updated_best,
        },
        None => RungResolution::Failed(PositionTuneOutcome::SweepExhausted),
    }
}

/// Measured response of one rung's ramped reversal stroke, derived from a
/// `TransientBuffer` capture of `PID_ERROR_PID_POSITION_ERROR` samples across
/// the full `RAMPED_STROKE_CAPTURE_US` window.
///
/// A ramped reversal stroke at nonzero commanded velocity has a nonzero
/// steady-state following error by design, so every metric here is measured
/// relative to a steady-state reference, never relative to zero.
///
/// The stroke is bipolar: it ramps one way for half the window and then
/// reverses, so the following error settles near `+lag` on the first leg and
/// near `-lag` on the second. Each leg therefore gets its own steady-state
/// reference, and the two legs' results are combined into the single set of
/// fields the wire format carries. One reference for the whole capture would
/// be the reverse leg's, leaving the forward leg measured against a value it
/// never approaches: it reports the commanded reversal as a failure to settle
/// on every rung, hides any forward-leg overshoot below the reverse leg's
/// steady state, and cannot see forward-leg ringing at all.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) struct RampedStrokeMetrics {
    /// Worst of the two legs' settling times, each measured from its own leg's
    /// start: the elapsed time at which `|position_error - steady_state|`
    /// first drops below 10% of that leg's peak absolute deviation from its
    /// `steady_state` and stays below it for the rest of the leg. Censored to
    /// the full leg when it never does — settling time is a diagnostic, not a
    /// stop condition.
    ///
    /// The worse leg rather than only the terminal leg: both legs are genuine
    /// step responses of the same loop at the same gain, so a forward leg that
    /// rings for 100 ms is exactly as much of a tuning result as a reverse leg
    /// that does, and reporting only the second would discard it silently.
    pub settling_time_us: u32,
    /// Largest excursion **past** a leg's own `steady_state`, in the direction
    /// opposite that leg's initial approach, taken over both legs. A leg whose
    /// approach is monotonic and never crosses past its `steady_state`
    /// contributes `0`, not its approach distance.
    pub overshoot_counts: u32,
    /// The last recorded `position_error` sample, signed, in counts relative
    /// to zero — the actual final tracking error reported to the operator.
    /// Unaffected by the leg split: it is the terminal sample either way.
    pub final_position_error_counts: i32,
    /// True when either leg's `(position_error - steady_state)` crosses zero
    /// more than 2 times after that leg's peak absolute deviation. Counted
    /// per leg against that leg's own reference, so the commanded direction
    /// reversal — a leg boundary, never a within-leg crossing — cannot be
    /// misread as ringing.
    pub oscillation_detected: bool,
}

/// One directional leg's reduction, before the two legs are combined.
struct LegMetrics {
    /// Settling index **within this leg**, in samples from the leg's start.
    settle_index: usize,
    overshoot_counts: u32,
    oscillation_detected: bool,
}

/// Reduce one directional leg against its own tail-window steady state.
///
/// `None` for an empty leg: a capture that ended before the reversal has no
/// second leg, and a zero-length slice has no steady state to measure against.
fn leg_metrics(samples: &[i32]) -> Option<LegMetrics> {
    if samples.is_empty() {
        return None;
    }

    // Steady-state reference: mean of the last 10% of this leg's samples (at
    // least 1 sample). This is what "settled" means for this leg -- whatever
    // following error persists once its transient has decayed, not zero.
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
        // Every sample equals steady_state: zero overshoot, skip the loops
        // below entirely rather than picking an arbitrary direction.
        return Some(LegMetrics {
            settle_index: 0,
            overshoot_counts: 0,
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
    // this leg's peak-deviation sample -- ringing around the value this leg
    // eventually settles at, not around literal zero.
    let mut crossings = 0u8;
    for window in samples[peak_index..].windows(2) {
        let a = i64::from(window[0]) - i64::from(steady_state);
        let b = i64::from(window[1]) - i64::from(steady_state);
        if (a >= 0) != (b >= 0) {
            crossings = crossings.saturating_add(1);
        }
    }
    let oscillation_detected = crossings > 2;

    // Settling index: first index after which every remaining sample in this
    // leg stays within 10% of peak_deviation of steady_state. Censored to the
    // leg's full length if it never settles.
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

    Some(LegMetrics {
        settle_index,
        overshoot_counts,
        oscillation_detected,
    })
}

/// Reduce a full ramped-reversal capture to its wire-format metrics.
///
/// `reversal_index` is the sample index at which the commanded direction
/// reverses; samples below it form the forward leg and the rest the reverse
/// leg. An index at or past `samples.len()` means the capture ended before the
/// reversal, which reduces to a single leg.
fn compute_ramped_stroke_metrics(
    samples: &[i32],
    sample_period_us: u32,
    reversal_index: usize,
) -> Result<RampedStrokeMetrics, PositionTuneOutcome> {
    // A failed, aborted, or never-started capture producing zero samples must
    // be a reported failure, not a panic -- a leg's tail length would
    // otherwise be 0, and dividing by it would panic.
    if samples.is_empty() {
        return Err(PositionTuneOutcome::CaptureEmpty);
    }

    let split = reversal_index.min(samples.len());
    let mut settle_index = 0_usize;
    let mut overshoot_counts = 0_u32;
    let mut oscillation_detected = false;
    for leg in [&samples[..split], &samples[split..]] {
        let Some(metrics) = leg_metrics(leg) else {
            continue;
        };
        settle_index = settle_index.max(metrics.settle_index);
        overshoot_counts = overshoot_counts.max(metrics.overshoot_counts);
        oscillation_detected |= metrics.oscillation_detected;
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
    /// Capturing one of this rung's three ramped-stroke repeats.
    RampedStroke { rung: PidPGain, repeat: u8 },
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
    /// This rung's three ramped-stroke repeat captures.
    pub(crate) repeat_captures:
        [Option<(u32, RampedStrokeMetrics)>; RAMPED_STROKE_REPEATS as usize],
    /// The walk's running-best-seen rung.
    pub(crate) running_best: Option<RungEvidence>,
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
            repeat_captures: [None; RAMPED_STROKE_REPEATS as usize],
            running_best: None,
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
    let _ = backend.write_register(PID_VELOCITY_TARGET, 0).await;
    let _ = backend.write_register(PID_TORQUE_FLUX_TARGET, 0).await;
    let _ = backend.write_register(MODE_RAMP_MODE_MOTION, 0).await;
    match fault {
        PositionTuneFault::Engine(error) => fail_outer(phase, error),
        PositionTuneFault::Outcome(outcome) => {
            defmt::warn!(
                "position tune failed: outcome={} wire_code={}",
                outcome,
                outcome.to_wire_code(),
            );
            fail_outer_position_tune(phase, CommissionError::PositionTuneFailed, outcome)
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

/// Reduce this rung's three completed ramped-stroke repeats to the median
/// `lag_us` and that repeat's own complete `RampedStrokeMetrics`.
///
/// Panics if any of the three slots is still `None`: the caller only calls
/// this once all three repeats have completed.
fn select_median_repeat(
    repeat_captures: &[Option<(u32, RampedStrokeMetrics)>; RAMPED_STROKE_REPEATS as usize],
) -> (u32, RampedStrokeMetrics) {
    let lags = repeat_captures.map(|entry| {
        entry
            .map(|(lag_us, _)| lag_us)
            .expect("every repeat slot is filled before the median is read")
    });
    let index = selection::median_repeat_index(lags);
    repeat_captures[index].expect("every repeat slot is filled before the median is read")
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
    compute_ramped_stroke_metrics(
        &samples[..captured.len()],
        RAMPED_STROKE_SAMPLE_PERIOD_US,
        RAMPED_STROKE_REVERSAL_SAMPLE_INDEX,
    )
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
    for &(elapsed_us, velocity_rpm) in &samples {
        defmt::info!(
            "position tune floor sample: elapsed_us={} velocity_rpm={}",
            elapsed_us,
            velocity_rpm,
        );
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
        repeat: 0,
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
    match resolve_rung_evidence(state.running_best, evidence, ceiling_tripped) {
        RungResolution::Advance { next, running_best } => {
            state.running_best = Some(running_best);
            state.step = PositionTuneStep::RampedStroke {
                rung: next,
                repeat: 0,
            };
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

/// `2*pi` as a Q16.16 fixed-point integer, the same scaled-integer
/// convention `ceiling::compute_k`'s `k_q16` uses. `6.283185307 * 65536 =
/// 411774.86`, so this is exact to within 1.4e-7 relative -- far below the
/// 1 Hz quantization of the field it feeds.
const TAU_Q16: i64 = 411_775;

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
        PositionTuneStep::RampedStroke { rung, repeat } => {
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
            match ramped_metrics(workspace).and_then(|metrics| {
                let lag_us = position_lag_us(
                    metrics.final_position_error_counts,
                    state.commanded_velocity_counts_per_s,
                )
                .ok_or(PositionTuneOutcome::AccelerationFitRejected)?;
                Ok((metrics, lag_us))
            }) {
                Ok((metrics, lag_us)) => {
                    state.repeat_captures[usize::from(repeat)] = Some((lag_us, metrics));
                    if repeat + 1 < RAMPED_STROKE_REPEATS {
                        state.step = PositionTuneStep::RampedStroke {
                            rung,
                            repeat: repeat + 1,
                        };
                        state.capture_running = false;
                        return StepResult::ReadyNow;
                    }
                    let (median_lag_us, median_metrics) =
                        select_median_repeat(&state.repeat_captures);
                    state.current_ramped_metrics = Some(median_metrics);
                    state.current_lag_us = median_lag_us;
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
    use super::selection::{RungOutcome, next_rung};
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

    #[test]
    fn select_median_repeat_returns_the_full_metrics_of_the_median_lag_repeat() {
        let repeats: [Option<(u32, RampedStrokeMetrics)>; 3] = [
            Some((1_500, metrics_tagged(11_111))),
            Some((900, metrics_tagged(22_222))),
            Some((1_200, metrics_tagged(33_333))),
        ];
        let (median_lag_us, metrics) = select_median_repeat(&repeats);
        assert_eq!(median_lag_us, 1_200);
        assert_eq!(metrics.settling_time_us, 33_333);
    }

    #[test]
    fn select_median_repeat_breaks_a_tie_toward_the_earlier_repeat() {
        let repeats: [Option<(u32, RampedStrokeMetrics)>; 3] = [
            Some((1_000, metrics_tagged(11_111))),
            Some((1_000, metrics_tagged(22_222))),
            Some((500, metrics_tagged(33_333))),
        ];
        let (median_lag_us, metrics) = select_median_repeat(&repeats);
        assert_eq!(median_lag_us, 1_000);
        assert_eq!(metrics.settling_time_us, 11_111);
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
    fn ceiling_trip_after_the_floor_accepts_the_running_bests_own_metrics() {
        let rung_a = evidence(63, 1_000, 11_111);
        let rung_b = evidence(89, 960, 22_222);

        let resolution =
            resolve_rung_evidence(Some(rung_a), rung_b, /* ceiling_tripped */ true);

        let RungResolution::Accept(accepted) = resolution else {
            panic!("a ceiling trip after the floor rung must accept the running best");
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
    fn an_improving_rung_advances_and_becomes_the_new_running_best() {
        let rung_a = evidence(63, 1_000, 11_111);
        let rung_b = evidence(89, 500, 22_222); // strictly better lag

        let resolution =
            resolve_rung_evidence(Some(rung_a), rung_b, /* ceiling_tripped */ false);

        let RungResolution::Advance { next, running_best } = resolution else {
            panic!("a rung that improves and stays quiet must advance");
        };
        assert!(next.raw() > 89);
        assert_eq!(running_best.outcome.position_p.raw(), 89);
        assert_eq!(running_best.metrics.settling_time_us, 22_222);
    }

    #[test]
    fn a_non_improving_rung_advances_but_keeps_the_earlier_running_best() {
        let rung_a = evidence(63, 500, 11_111); // the better rung
        let rung_b = evidence(89, 800, 22_222); // worse than rung_a, but not tripped

        let resolution =
            resolve_rung_evidence(Some(rung_a), rung_b, /* ceiling_tripped */ false);

        let RungResolution::Advance { running_best, .. } = resolution else {
            panic!("a quiet rung must advance regardless of its own lag");
        };
        // The running best is still rung_a's evidence, never rung_b's,
        // even though the walk did advance past rung_b.
        assert_eq!(running_best.outcome.position_p.raw(), 63);
        assert_eq!(running_best.metrics.settling_time_us, 11_111);
    }

    #[test]
    fn an_exact_lag_tie_keeps_the_lower_p_running_best() {
        let rung_a = evidence(63, 1_000, 11_111);
        let rung_b = evidence(89, 1_000, 22_222); // exact tie on lag_us

        let resolution =
            resolve_rung_evidence(Some(rung_a), rung_b, /* ceiling_tripped */ false);

        let RungResolution::Advance { running_best, .. } = resolution else {
            panic!("a quiet rung must advance");
        };
        assert_eq!(running_best.outcome.position_p.raw(), 63);
    }

    #[test]
    fn exhaustion_at_pid_p_gain_max_never_falls_back_to_the_running_best() {
        let previous = evidence(PidPGain::MAX.raw(), 1, 11_111);
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
                PositionTuneStep::RampedStroke { rung, .. } => {
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

    /// Like `drive_sweep`, but with a caller-supplied per-rung position-error
    /// table and a single rung (by raw P) whose standstill dither trips the
    /// ceiling. Every other visited rung stays quiet: its measured dither is
    /// pinned to exactly the predicted quantization-only value (computed
    /// live from `state.k_q16`, so the fixture works regardless of the exact
    /// `k` this backend's PPR/velocity-P combination produces), and the
    /// tripping rung's is pinned to 3x that prediction.
    ///
    /// Panics if the walk visits a rung `lag_table` has no entry for --
    /// a missing entry would otherwise silently default to an unintended
    /// lag value instead of failing loudly.
    fn drive_sweep_with(
        phase: &mut OuterPhase,
        backend: &mut MockBackend,
        lag_table: &[(u16, i32)],
        trip_rung_raw: u16,
    ) -> StepResult {
        let mut workspace = TransientBuffer::new();
        let mut clock_us = backend.elapsed_us_value;

        for _ in 0..20_000 {
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
                PositionTuneStep::RampedStroke { rung, .. } => {
                    let (_, error) = lag_table
                        .iter()
                        .find(|(raw, _)| *raw == rung.raw())
                        .unwrap_or_else(|| panic!("no lag_table entry for rung {}", rung.raw()));
                    let _ = backend
                        .registers
                        .insert(PID_ERROR_PID_POSITION_ERROR.data.addr(), *error as u32);
                }
                PositionTuneStep::StandstillHold { rung, .. } => {
                    let predicted = state.k_q16.saturating_mul(i64::from(rung.raw())) / 65_536;
                    let multiplier: i64 = if rung.raw() == trip_rung_raw { 3 } else { 1 };
                    let amplitude = i32::try_from(predicted * multiplier).unwrap_or(i32::MAX);
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
        // The generic error is the same for every sweep failure, so the
        // terminal phase has to retain the specific outcome for the reply
        // builder to have anything but zero to report.
        assert!(matches!(
            phase,
            OuterPhase::PositionTuneFailed {
                error: CommissionError::PositionTuneFailed,
                outcome: PositionTuneOutcome::FloorCeilingConflict,
            }
        ));
    }

    #[test]
    fn stable_following_error_reports_no_overshoot_and_no_oscillation() {
        let samples = [50_i32; 20];
        let metrics = compute_ramped_stroke_metrics(&samples, PERIOD_US, samples.len())
            .expect("valid capture");

        assert_eq!(metrics.overshoot_counts, 0);
        assert!(!metrics.oscillation_detected);
        assert_eq!(metrics.settling_time_us, 0);
        assert_eq!(metrics.final_position_error_counts, 50);
    }

    #[test]
    fn monotonic_approach_to_nonzero_steady_state_is_not_overshoot() {
        let samples: [i32; 10] = [0, 10, 20, 30, 40, 48, 50, 50, 50, 50];
        let metrics = compute_ramped_stroke_metrics(&samples, PERIOD_US, samples.len())
            .expect("valid capture");

        assert_eq!(metrics.overshoot_counts, 0);
        assert!(!metrics.oscillation_detected);
        assert_eq!(metrics.final_position_error_counts, 50);
    }

    #[test]
    fn genuine_overshoot_is_measured_past_the_settled_value() {
        let samples: [i32; 12] = [0, 20, 50, 80, 110, 120, 100, 70, 55, 50, 50, 50];
        let metrics = compute_ramped_stroke_metrics(&samples, PERIOD_US, samples.len())
            .expect("valid capture");

        assert_eq!(metrics.overshoot_counts, 70);
        assert!(metrics.settling_time_us > 5 * PERIOD_US);
        assert_eq!(metrics.final_position_error_counts, 50);
    }

    #[test]
    fn sustained_ringing_after_the_peak_is_detected() {
        let samples: [i32; 12] = [0, 40, 90, 30, 70, 30, 70, 30, 70, 50, 50, 50];
        let metrics = compute_ramped_stroke_metrics(&samples, PERIOD_US, samples.len())
            .expect("valid capture");

        assert!(metrics.oscillation_detected);
    }

    #[test]
    fn first_sample_equal_to_steady_state_does_not_flip_the_approach_direction() {
        let samples: [i32; 10] = [50, 40, 45, 49, 50, 50, 50, 50, 50, 50];
        let metrics = compute_ramped_stroke_metrics(&samples, PERIOD_US, samples.len())
            .expect("valid capture");

        assert_eq!(metrics.overshoot_counts, 0);
    }

    /// A clean bipolar capture: the forward leg settles at `+100` counts of
    /// following error, the reverse leg at `-100`. Nothing in it rings and
    /// nothing overshoots -- the only large excursion is the commanded
    /// direction reversal itself, at the leg boundary.
    const BIPOLAR_REVERSAL_INDEX: usize = 12;
    const CLEAN_BIPOLAR: [i32; 24] = [
        0, 40, 70, 90, 98, 100, 100, 100, 100, 100, 100, 100, // forward leg
        20, -60, -90, -98, -100, -100, -100, -100, -100, -100, -100, -100,
    ];

    #[test]
    fn a_clean_direction_reversal_is_not_read_as_a_failure_to_settle() {
        let metrics =
            compute_ramped_stroke_metrics(&CLEAN_BIPOLAR, PERIOD_US, BIPOLAR_REVERSAL_INDEX)
                .expect("valid capture");

        // Each leg settles within 3 samples of its own start. A single
        // whole-capture reference would instead measure both legs against the
        // reverse leg's -100 and report the reversal point (~14 samples in) as
        // the settling time on every rung, regardless of the actual dynamics.
        assert_eq!(metrics.settling_time_us, 3 * PERIOD_US);
        assert_eq!(metrics.overshoot_counts, 0);
        assert!(!metrics.oscillation_detected);
        assert_eq!(metrics.final_position_error_counts, -100);
    }

    #[test]
    fn reversal_sample_index_lands_one_before_the_scheduled_reversal() {
        // `arm_ramped_stroke` schedules the reversal at exactly half the
        // capture window (`RAMPED_STROKE_CAPTURE_US / 2`), but the first
        // sample is taken one period after capture starts, not at time zero.
        // So the sample whose timestamp equals the reversal instant is
        // already commanded in the reverse direction and must fall in the
        // second leg -- one index below the naive half-window count.
        let naive_half_window_samples =
            (RAMPED_STROKE_CAPTURE_US / 2 / RAMPED_STROKE_SAMPLE_PERIOD_US) as usize;
        assert_eq!(
            RAMPED_STROKE_REVERSAL_SAMPLE_INDEX,
            naive_half_window_samples - 1
        );
        assert_eq!(RAMPED_STROKE_REVERSAL_SAMPLE_INDEX, 127);
    }

    #[test]
    fn overshoot_on_the_forward_leg_survives_into_the_combined_metrics() {
        let mut samples = CLEAN_BIPOLAR;
        // Forward leg now overshoots its own +100 steady state by 40 counts.
        samples[..BIPOLAR_REVERSAL_INDEX]
            .copy_from_slice(&[0, 40, 80, 120, 140, 110, 95, 100, 100, 100, 100, 100]);

        let metrics = compute_ramped_stroke_metrics(&samples, PERIOD_US, BIPOLAR_REVERSAL_INDEX)
            .expect("valid capture");

        // Measured against the forward leg's own +100, not against the reverse
        // leg's -100 -- which the whole excursion sits above, so a single
        // reference reports no overshoot at all.
        assert_eq!(metrics.overshoot_counts, 40);
    }

    #[test]
    fn ringing_confined_to_one_leg_is_detected_against_that_legs_own_reference() {
        let mut samples = CLEAN_BIPOLAR;
        samples[..BIPOLAR_REVERSAL_INDEX]
            .copy_from_slice(&[0, 60, 130, 70, 125, 70, 125, 70, 125, 100, 100, 100]);

        let metrics = compute_ramped_stroke_metrics(&samples, PERIOD_US, BIPOLAR_REVERSAL_INDEX)
            .expect("valid capture");

        // Every one of these swings stays above the reverse leg's -100, so a
        // single whole-capture reference sees a monotone signal and no
        // crossings at all.
        assert!(metrics.oscillation_detected);
    }

    #[test]
    fn nominal_bandwidth_matches_the_floor_rungs_worked_example() {
        // bandwidth_hz = (63/256) * 65536 / (60*2*pi) = 16128 / 376.9911 =
        // 42.78 Hz, which rounds to 43. 63 is the floor rung the rotor-only
        // worked example in `floor.rs` produces.
        assert_eq!(nominal_bandwidth_hz(63), 43);
        // (256/256) * 65536 / (60*2*pi) = 65536 / 376.9911 = 173.84 -> 174.
        assert_eq!(nominal_bandwidth_hz(256), 174);
        assert_eq!(nominal_bandwidth_hz(0), 0);
    }

    #[test]
    fn empty_capture_is_a_reported_failure_not_a_panic() {
        assert_eq!(
            compute_ramped_stroke_metrics(&[], PERIOD_US, 0),
            Err(PositionTuneOutcome::CaptureEmpty)
        );
    }

    #[test]
    fn the_walk_climbs_past_a_local_dip_and_never_admits_the_tripping_rungs_evidence() {
        let lag_table: &[(u16, i32)] = &[
            (63, 2_000),
            (89, 1_500),
            (125, 1_600),
            (176, 900),
            (248, 50),
        ];
        let mut backend = position_tune_backend();
        let mut phase = position_tune_phase();

        let terminal = drive_sweep_with(&mut phase, &mut backend, lag_table, 248);

        let StepResult::OuterDone(result) = terminal else {
            panic!("expected OuterDone, got {terminal:?}");
        };
        assert_eq!(result.position_p, 176);
    }
}
