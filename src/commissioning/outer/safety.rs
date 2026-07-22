//! Outer commissioning safety-envelope checks.

use crate::autotune::CommissionError;
use crate::commissioning::backend::CommissioningBackend;
use crate::commissioning::encoder_domain::{EncoderCount, EncoderDomain, EncoderMathError};
use crate::commissioning::schedule::next_grid_deadline;
use crate::commissioning::types::{
    OUTER_SAFETY_FAULT_DURATION, OUTER_SAFETY_FAULT_INVALID_BUDGET,
    OUTER_SAFETY_FAULT_OBSERVATION_GAP, OUTER_SAFETY_FAULT_POSITION,
    OUTER_SAFETY_FAULT_QUARTER_TURN, OUTER_SAFETY_FAULT_UNUSABLE_BUDGET,
    OUTER_SAFETY_FAULT_VELOCITY, OuterSafetyFault,
};

use super::limits::{MECHANICAL_ID_VELOCITY_CAP_COUNTS_PER_MS, POSITION_WINDOW_COUNTS};
use super::velocity::sweep_plan::{
    ENVELOPE_ABSOLUTE_MARGIN_MREV_S, ENVELOPE_PROPORTIONAL_DEN, ENVELOPE_PROPORTIONAL_NUM,
    MAX_SAFETY_OBSERVATION_GAP_US, RequestedVelocitySource,
};
use super::{
    mechanical_id_velocity_cap_counts_per_ms, position_window_counts, response, state::OuterPhase,
};

const FIRMWARE_MAX_TRAVEL_MREV: u32 = 2_000;
const FIRMWARE_MAX_VELOCITY_MREV_S: u32 = 6_000;
const MIN_USEFUL_DURATION_MS: u16 = 4;
/// Fixed observation period for the independent outer safety envelope.
pub(super) const OUTER_SAFETY_SAMPLE_PERIOD_US: u64 = 1_000;
/// Maximum register operations performed by one due safety observation.
#[allow(dead_code)]
pub(super) const OUTER_SAFETY_SAMPLE_MAX_REGISTER_OPS: usize = 1;

/// Host-provided motor-local motion budget for outer commissioning.
#[derive(Debug, Clone, Copy, defmt::Format)]
pub(crate) struct OuterMotionBudget {
    /// Requested mechanical velocity amplitude.
    pub(crate) requested_velocity_mrev_s: u32,
    /// Kinematics-derived absolute machine ceiling.
    pub(crate) machine_velocity_ceiling_mrev_s: u32,
    /// Explicit-versus-defaulted request provenance.
    pub(crate) requested_velocity_source: RequestedVelocitySource,
    /// Full local displacement window spanning moving and settle.
    pub(crate) max_stroke_travel_mrev: u32,
    /// Host planning reserve inside the full displacement window.
    pub(crate) settle_travel_reserve_mrev: u32,
    /// Negative absolute-position headroom from Stage-2 entry.
    pub(crate) negative_position_headroom_mrev: u32,
    /// Positive absolute-position headroom from Stage-2 entry.
    pub(crate) positive_position_headroom_mrev: u32,
    /// Maximum primitive duration in milliseconds.
    pub(crate) max_duration_ms: u16,
}

impl OuterMotionBudget {
    /// Current firmware hard caps, used by tests that don't exercise host budget tightening.
    #[cfg(test)]
    pub(crate) const fn firmware_caps() -> Self {
        Self {
            requested_velocity_mrev_s: FIRMWARE_MAX_VELOCITY_MREV_S,
            machine_velocity_ceiling_mrev_s: 10_000,
            requested_velocity_source: RequestedVelocitySource::Defaulted,
            max_stroke_travel_mrev: FIRMWARE_MAX_TRAVEL_MREV,
            settle_travel_reserve_mrev: 100,
            negative_position_headroom_mrev: FIRMWARE_MAX_TRAVEL_MREV,
            positive_position_headroom_mrev: FIRMWARE_MAX_TRAVEL_MREV,
            max_duration_ms: 3_000,
        }
    }

    #[cfg(test)]
    pub(crate) const fn test_budget(
        max_stroke_travel_mrev: u32,
        requested_velocity_mrev_s: u32,
        max_duration_ms: u16,
    ) -> Self {
        Self {
            requested_velocity_mrev_s,
            machine_velocity_ceiling_mrev_s: 10_000,
            requested_velocity_source: RequestedVelocitySource::Defaulted,
            max_stroke_travel_mrev,
            settle_travel_reserve_mrev: if max_stroke_travel_mrev > 100 { 100 } else { 0 },
            negative_position_headroom_mrev: 2_000,
            positive_position_headroom_mrev: 2_000,
            max_duration_ms,
        }
    }

    fn travel_counts(self, counts_per_rev: u32) -> u32 {
        mrev_to_counts(self.max_stroke_travel_mrev, counts_per_rev)
    }

    fn velocity_counts_per_ms(self, counts_per_rev: u32) -> u32 {
        let counts_per_rev = u64::from(counts_per_rev);
        let scaled = counts_per_rev.saturating_mul(self.requested_velocity_mrev_s as u64);
        let rounded = (scaled + 500_000) / 1_000_000;
        clamp_positive_u32(rounded)
    }
}

#[derive(Debug, Clone, Copy, defmt::Format)]
struct OuterVelocityObservation {
    absolute_delta_counts: u32,
    elapsed_us: u64,
}

/// Outer safety limits, computed once at the start of outer commissioning.
#[derive(Debug, Clone, Copy, defmt::Format)]
pub(crate) struct OuterSafetyLimits {
    /// Maximum absolute velocity in encoder counts per millisecond.
    pub(crate) velocity_cap: u32,
    /// Maximum distance from start position in encoder counts.
    pub(crate) position_window: u32,
    /// Maximum negative displacement from the Stage-2 entry position.
    pub(crate) negative_position_window: u32,
    /// Maximum positive displacement from the Stage-2 entry position.
    pub(crate) positive_position_window: u32,
    /// Inclusive single-observation quarter-turn fault threshold.
    pub(crate) quarter_turn_fault_counts: u32,
    /// Unwrapped signed position accumulated from wrap-aware deltas.
    pub(crate) unwrapped_position: i32,
    /// Torque cap: the configured run_current limit.
    pub(crate) torque_cap: u16,
    /// Previous normalized encoder count for delta computation.
    pub(crate) prev_encoder_count: Option<EncoderCount>,
    /// Validated encoder modulo domain.
    pub(crate) encoder_domain: Option<EncoderDomain>,
    /// Timestamp (microseconds) of the previous safety sample.
    pub(crate) last_sample_us: Option<u64>,
    /// Timestamp (microseconds) when the active motion primitive began.
    pub(crate) primitive_start_us: Option<u64>,
    /// Unwrapped position when the active motion primitive began.
    pub(crate) primitive_start_position: Option<i32>,
    /// Absolute deadline for the next independent safety observation.
    pub(crate) next_safety_sample_us: Option<u64>,
    /// Previous ordinary velocity observation for the sliding two-interval gate.
    previous_velocity_observation: Option<OuterVelocityObservation>,
    /// Whether the latest accepted observation met the scoring cadence.
    last_observation_scoring_eligible: bool,
    /// Host-provided motion budget used to clamp firmware safety caps.
    pub(crate) budget: OuterMotionBudget,
}

impl OuterSafetyLimits {
    pub(crate) fn new(torque_cap: u16, budget: OuterMotionBudget) -> Self {
        Self {
            velocity_cap: MECHANICAL_ID_VELOCITY_CAP_COUNTS_PER_MS,
            position_window: POSITION_WINDOW_COUNTS,
            negative_position_window: POSITION_WINDOW_COUNTS,
            positive_position_window: POSITION_WINDOW_COUNTS,
            quarter_turn_fault_counts: 0,
            unwrapped_position: 0,
            torque_cap,
            prev_encoder_count: None,
            encoder_domain: None,
            last_sample_us: None,
            primitive_start_us: None,
            primitive_start_position: None,
            next_safety_sample_us: None,
            previous_velocity_observation: None,
            last_observation_scoring_eligible: false,
            budget,
        }
    }

    pub(crate) fn budget_request_is_valid(&self) -> bool {
        self.budget.max_stroke_travel_mrev > 0
            && self.budget.requested_velocity_mrev_s > 0
            && self.budget.machine_velocity_ceiling_mrev_s > 0
            && self.budget.settle_travel_reserve_mrev < self.budget.max_stroke_travel_mrev
            && self.budget.negative_position_headroom_mrev > 0
            && self.budget.positive_position_headroom_mrev > 0
            && self.budget.max_duration_ms > 0
    }

    pub(crate) fn configure_encoder_resolution(
        &mut self,
        counts_per_rev: u32,
    ) -> Result<(), EncoderMathError> {
        let encoder_domain = EncoderDomain::new(counts_per_rev)?;
        self.encoder_domain = Some(encoder_domain);
        self.prev_encoder_count = None;
        self.previous_velocity_observation = None;
        self.quarter_turn_fault_counts = counts_per_rev.div_ceil(4);
        self.velocity_cap = mechanical_id_velocity_cap_counts_per_ms(counts_per_rev)
            .min(self.budget.velocity_counts_per_ms(counts_per_rev));
        self.position_window =
            position_window_counts(counts_per_rev).min(self.budget.travel_counts(counts_per_rev));
        self.negative_position_window = position_window_counts(counts_per_rev).min(mrev_to_counts(
            self.budget.negative_position_headroom_mrev,
            counts_per_rev,
        ));
        self.positive_position_window = position_window_counts(counts_per_rev).min(mrev_to_counts(
            self.budget.positive_position_headroom_mrev,
            counts_per_rev,
        ));
        Ok(())
    }

    /// Configure encoder-domain safety using the complete host-declared sweep windows.
    ///
    /// Mechanical identification retains its legacy two-revolution backstop until the
    /// cutover. The trace-gated velocity sweep instead plans motion from the richer
    /// request contract, so its runtime enforcement must use those same declared windows.
    #[cfg(feature = "trace-velocity-sweep")]
    pub(crate) fn configure_velocity_sweep_encoder_resolution(
        &mut self,
        counts_per_rev: u32,
    ) -> Result<(), EncoderMathError> {
        self.configure_encoder_resolution(counts_per_rev)?;
        self.position_window = self.budget.travel_counts(counts_per_rev);
        self.negative_position_window =
            mrev_to_counts(self.budget.negative_position_headroom_mrev, counts_per_rev);
        self.positive_position_window =
            mrev_to_counts(self.budget.positive_position_headroom_mrev, counts_per_rev);
        Ok(())
    }

    /// Configure the runtime envelope around the frozen sweep amplitude.
    #[allow(dead_code)]
    pub(crate) fn configure_velocity_envelope(
        &mut self,
        planned_velocity_mrev_s: u32,
    ) -> Result<(), CommissionError> {
        if planned_velocity_mrev_s == 0 {
            return Err(CommissionError::InvalidSchedule);
        }
        let proportional = planned_velocity_mrev_s
            .checked_mul(ENVELOPE_PROPORTIONAL_NUM)
            .and_then(|value| value.checked_add(ENVELOPE_PROPORTIONAL_DEN - 1))
            .map(|value| value / ENVELOPE_PROPORTIONAL_DEN)
            .ok_or(CommissionError::InvalidSchedule)?;
        let absolute = planned_velocity_mrev_s
            .checked_add(ENVELOPE_ABSOLUTE_MARGIN_MREV_S)
            .ok_or(CommissionError::InvalidSchedule)?;
        let envelope_mrev_s = proportional.max(absolute);
        self.velocity_cap =
            velocity_mrev_s_to_counts_per_ms(envelope_mrev_s, self.counts_per_rev());
        Ok(())
    }

    pub(crate) fn set_encoder_baseline(&mut self, raw_counts: u32) -> Result<(), EncoderMathError> {
        let domain = self
            .encoder_domain
            .ok_or(EncoderMathError::InvalidCountsPerRev)?;
        self.prev_encoder_count = Some(domain.count(raw_counts));
        self.previous_velocity_observation = None;
        Ok(())
    }

    /// Arm the independent safety-observation grid from a known baseline.
    pub(crate) fn arm_safety_observation(&mut self, now_us: u64) -> Result<(), CommissionError> {
        let next_safety_sample_us = now_us
            .checked_add(OUTER_SAFETY_SAMPLE_PERIOD_US)
            .ok_or(CommissionError::InvalidSchedule)?;
        self.last_sample_us = Some(now_us);
        self.next_safety_sample_us = Some(next_safety_sample_us);
        self.previous_velocity_observation = None;
        self.last_observation_scoring_eligible = false;
        Ok(())
    }

    /// Begin a bounded motion primitive after its nonzero target is applied.
    pub(crate) fn begin_primitive(&mut self, now_us: u64) -> Result<(), CommissionError> {
        if self.primitive_start_us.is_some() {
            return Err(CommissionError::InvalidSchedule);
        }
        self.primitive_start_us = Some(now_us);
        self.primitive_start_position = Some(self.unwrapped_position);
        self.previous_velocity_observation = None;
        Ok(())
    }

    /// End the active motion primitive after its safe target is applied.
    pub(crate) fn end_primitive(&mut self) {
        self.primitive_start_us = None;
        self.primitive_start_position = None;
        self.previous_velocity_observation = None;
    }

    /// Whether the latest accepted observation met the 1 kHz scoring cadence.
    #[allow(dead_code)]
    pub(crate) fn last_observation_scoring_eligible(&self) -> bool {
        self.last_observation_scoring_eligible
    }

    /// Return the next observation deadline while the safety envelope is armed.
    pub(crate) fn next_safety_deadline(&self) -> Option<u64> {
        if self.torque_cap == 0 {
            None
        } else {
            self.next_safety_sample_us
        }
    }

    pub(crate) fn counts_per_rev(&self) -> u32 {
        self.encoder_domain
            .expect("outer encoder domain configured before motion")
            .counts_per_rev()
    }

    pub(crate) fn budget_is_usable(&self) -> bool {
        self.budget_request_is_valid()
            && self.budget.max_duration_ms >= MIN_USEFUL_DURATION_MS
            && self.position_window >= response::MECHANICAL_ID_MIN_TRAVEL_COUNTS
            && self.velocity_cap >= response::MECHANICAL_ID_MIN_PEAK_VELOCITY_COUNTS_PER_MS
    }

    pub(crate) fn motion_budget_mrev(&self) -> u16 {
        self.budget
            .max_stroke_travel_mrev
            .min(FIRMWARE_MAX_TRAVEL_MREV)
            .min(u16::MAX as u32) as u16
    }

    pub(crate) fn budget_clamped_firmware_caps(&self) -> bool {
        self.budget.max_stroke_travel_mrev < FIRMWARE_MAX_TRAVEL_MREV
            || self.budget.requested_velocity_mrev_s < FIRMWARE_MAX_VELOCITY_MREV_S
    }

    pub(crate) fn invalid_budget_fault(&self) -> OuterSafetyFault {
        self.fault(OUTER_SAFETY_FAULT_INVALID_BUDGET, 0, 0, 0, 0)
    }

    pub(crate) fn fault(
        &self,
        reason: u8,
        delta_counts: i32,
        dt_us: u64,
        velocity_counts_per_ms: i32,
        elapsed_us: u64,
    ) -> OuterSafetyFault {
        let duration_cap_us = u64::from(self.budget.max_duration_ms) * 1_000;
        OuterSafetyFault {
            reason,
            max_travel_mrev: clamp_u32_to_u16(self.budget.max_stroke_travel_mrev),
            max_velocity_mrev_s: clamp_u32_to_u16(self.budget.requested_velocity_mrev_s),
            max_duration_ms: self.budget.max_duration_ms,
            direction_mask: 0x03,
            delta_counts,
            dt_us: clamp_u64_to_u32(dt_us),
            velocity_counts_per_ms,
            velocity_cap_counts_per_ms: clamp_u32_to_i32(self.velocity_cap),
            position_counts: self.unwrapped_position,
            position_window_counts: clamp_u32_to_i32(self.position_window),
            elapsed_us: clamp_u64_to_u32(elapsed_us),
            duration_cap_us: clamp_u64_to_u32(duration_cap_us),
        }
    }

    /// Construct the dedicated fail-closed fault for recovery motion away from origin.
    #[cfg(feature = "trace-velocity-sweep")]
    #[allow(dead_code)]
    pub(crate) fn recovery_wrong_way_fault(
        &self,
        start_offset_counts: i32,
        elapsed_us: u64,
    ) -> OuterSafetyFault {
        self.fault(
            crate::commissioning::types::OUTER_SAFETY_FAULT_RECOVERY_WRONG_WAY,
            self.unwrapped_position.saturating_sub(start_offset_counts),
            0,
            0,
            elapsed_us,
        )
    }
}

fn mrev_to_counts(mrev: u32, counts_per_rev: u32) -> u32 {
    let counts_per_rev = u64::from(counts_per_rev);
    let scaled = counts_per_rev.saturating_mul(mrev as u64);
    clamp_positive_u32(scaled / 1_000)
}

#[allow(dead_code)]
fn velocity_mrev_s_to_counts_per_ms(velocity_mrev_s: u32, counts_per_rev: u32) -> u32 {
    let scaled = u64::from(velocity_mrev_s).saturating_mul(u64::from(counts_per_rev));
    clamp_positive_u32(scaled / 1_000_000)
}

fn clamp_positive_u32(value: u64) -> u32 {
    if value == 0 {
        1
    } else if value > u32::MAX as u64 {
        u32::MAX
    } else {
        value as u32
    }
}

fn clamp_u32_to_i32(value: u32) -> i32 {
    value.min(i32::MAX as u32) as i32
}

fn clamp_u32_to_u16(value: u32) -> u16 {
    value.min(u16::MAX as u32) as u16
}

fn clamp_u64_to_u32(value: u64) -> u32 {
    value.min(u32::MAX as u64) as u32
}

fn velocity_counts_per_ms_for_fault(delta_counts: i32, magnitude: u64) -> i32 {
    if delta_counts < 0 {
        let magnitude = magnitude.min(i32::MAX as u64 + 1);
        -(magnitude as i64) as i32
    } else {
        magnitude.min(i32::MAX as u64) as i32
    }
}

/// Map a safety-envelope violation to the dedicated error variant.
///
/// Always returns `SafetyEnvelopeViolation` so the OuterFailed handler
/// can classify it as a safety fault requiring hard disable, distinct
/// from normal phase failures that are eligible for soft restore.
pub(crate) fn safety_error_for_phase(_phase: &OuterPhase) -> CommissionError {
    CommissionError::SafetyEnvelopeViolation
}

/// Per-tick safety envelope check for all outer phases.
///
/// Reads the current encoder position, computes a wrap-aware delta,
/// normalizes velocity by actual elapsed time, and accumulates an
/// unwrapped signed position. Checks:
/// - Time-normalized velocity within cap.
/// - Unwrapped position within window from start.
///
/// Returns `Some(fault)` if a safety limit is violated.
pub(crate) async fn check_outer_safety<B: CommissioningBackend>(
    safety: &mut OuterSafetyLimits,
    backend: &mut B,
) -> Result<Option<OuterSafetyFault>, CommissionError> {
    // Safety limits are initialized by MechanicalId::Setup after it reads
    // the real encoder baseline. Skip the pre-setup placeholder values.
    if safety.torque_cap == 0 {
        return Ok(None);
    }

    let entry_us = backend.elapsed_us();
    let sample_deadline_us = safety.next_safety_sample_us.unwrap_or(entry_us);
    if entry_us < sample_deadline_us {
        return Ok(None);
    }

    if !safety.budget_is_usable() {
        defmt::warn!(
            "outer safety: unusable motion budget travel={} velocity={} duration={} dir={}",
            safety.budget.max_stroke_travel_mrev,
            safety.budget.requested_velocity_mrev_s,
            safety.budget.max_duration_ms,
            0x03_u8,
        );
        return Ok(Some(safety.fault(
            OUTER_SAFETY_FAULT_UNUSABLE_BUDGET,
            0,
            0,
            0,
            0,
        )));
    }

    let encoder_raw = backend
        .try_read_mechanical_encoder_position_counts()
        .await
        .map_err(|_| CommissionError::Spi)?;
    let encoder_count = safety
        .encoder_domain
        .expect("outer encoder domain configured before motion")
        .count(encoder_raw);
    let now_us = backend.elapsed_us();

    let elapsed_us = safety
        .primitive_start_us
        .map(|primitive_start_us| {
            now_us
                .checked_sub(primitive_start_us)
                .ok_or(CommissionError::InvalidSchedule)
        })
        .transpose()?
        .unwrap_or(0);
    let max_duration_us = u64::from(safety.budget.max_duration_ms) * 1_000;
    if safety.primitive_start_us.is_some() && elapsed_us > max_duration_us {
        defmt::warn!(
            "outer safety: duration {}us exceeds cap {}us",
            elapsed_us,
            max_duration_us
        );
        return Ok(Some(safety.fault(
            OUTER_SAFETY_FAULT_DURATION,
            0,
            0,
            0,
            elapsed_us,
        )));
    }

    let dt_us = safety
        .last_sample_us
        .map(|last_sample_us| {
            now_us
                .checked_sub(last_sample_us)
                .ok_or(CommissionError::InvalidSchedule)
        })
        .transpose()?
        .unwrap_or(0);
    if safety.last_sample_us.is_some() && (dt_us == 0 || dt_us > MAX_SAFETY_OBSERVATION_GAP_US) {
        defmt::warn!(
            "outer safety: observation gap {}us exceeds wrap-safe cap {}us",
            dt_us,
            MAX_SAFETY_OBSERVATION_GAP_US
        );
        return Ok(Some(safety.fault(
            OUTER_SAFETY_FAULT_OBSERVATION_GAP,
            0,
            dt_us,
            0,
            elapsed_us,
        )));
    }
    let encoder_domain = safety
        .encoder_domain
        .expect("outer encoder domain configured before motion");
    let previous_count = safety
        .prev_encoder_count
        .expect("outer encoder baseline configured before motion");
    let delta = match encoder_domain.delta(previous_count, encoder_count) {
        Ok(delta) => delta,
        Err(EncoderMathError::AmbiguousHalfTurn) => {
            return Ok(Some(safety.fault(
                OUTER_SAFETY_FAULT_QUARTER_TURN,
                0,
                dt_us,
                0,
                elapsed_us,
            )));
        }
        Err(EncoderMathError::InvalidCountsPerRev) => {
            unreachable!("validated encoder domain cannot become invalid")
        }
    };
    let delta_counts = delta.signed_counts();
    if delta.absolute_counts() >= safety.quarter_turn_fault_counts {
        defmt::warn!(
            "outer safety: delta {} reaches quarter-turn guard {}",
            delta.absolute_counts(),
            safety.quarter_turn_fault_counts
        );
        return Ok(Some(safety.fault(
            OUTER_SAFETY_FAULT_QUARTER_TURN,
            delta_counts,
            dt_us,
            0,
            elapsed_us,
        )));
    }
    let velocity_magnitude_cpms = (u64::from(delta.absolute_counts()) * 1_000)
        .checked_div(dt_us)
        .unwrap_or(0);

    // The accumulator intentionally preserves signed multi-revolution travel.
    // Saturation keeps a corrupt or unexpectedly long sequence from wrapping
    // back inside the allowed safety window.
    safety.prev_encoder_count = Some(encoder_count);
    safety.last_sample_us = Some(now_us);
    safety.next_safety_sample_us = Some(next_grid_deadline(
        sample_deadline_us,
        OUTER_SAFETY_SAMPLE_PERIOD_US,
        now_us,
    )?);
    safety.unwrapped_position = safety.unwrapped_position.saturating_add(delta_counts);
    safety.last_observation_scoring_eligible = dt_us <= OUTER_SAFETY_SAMPLE_PERIOD_US;

    let velocity_observation = OuterVelocityObservation {
        absolute_delta_counts: delta.absolute_counts(),
        elapsed_us: dt_us,
    };
    let previous_velocity_observation = safety
        .previous_velocity_observation
        .replace(velocity_observation);

    // Check velocity using a sliding two-interval window. Cross multiplication
    // preserves actual elapsed time without rounding either interval first.
    let velocity_window_exceeds_cap = previous_velocity_observation.is_some_and(|previous| {
        let absolute_delta_counts = u128::from(previous.absolute_delta_counts)
            + u128::from(velocity_observation.absolute_delta_counts);
        let elapsed_us =
            u128::from(previous.elapsed_us) + u128::from(velocity_observation.elapsed_us);
        absolute_delta_counts * 1_000 > u128::from(safety.velocity_cap) * elapsed_us
    });
    if velocity_window_exceeds_cap {
        defmt::warn!(
            "outer safety: velocity {} exceeds cap {} (delta={} dt_us={})",
            velocity_magnitude_cpms,
            safety.velocity_cap,
            delta_counts,
            dt_us,
        );
        let velocity_cpms = velocity_counts_per_ms_for_fault(delta_counts, velocity_magnitude_cpms);
        return Ok(Some(safety.fault(
            OUTER_SAFETY_FAULT_VELOCITY,
            delta_counts,
            dt_us,
            velocity_cpms,
            elapsed_us,
        )));
    }

    // The primitive-local window begins at the moving target and remains active
    // through the zero-target settle so braking can consume the declared reserve.
    let local_position = safety
        .primitive_start_position
        .map(|start| safety.unwrapped_position.saturating_sub(start));
    let local_position_exceeds_window =
        local_position.is_some_and(|position| position.unsigned_abs() > safety.position_window);
    let absolute_position_exceeds_window = safety.unwrapped_position
        < -(clamp_u32_to_i32(safety.negative_position_window))
        || safety.unwrapped_position > clamp_u32_to_i32(safety.positive_position_window);
    if local_position_exceeds_window || absolute_position_exceeds_window {
        defmt::warn!(
            "outer safety: position {} exceeds local {} or absolute -{}/+{}",
            safety.unwrapped_position,
            safety.position_window,
            safety.negative_position_window,
            safety.positive_position_window,
        );
        let velocity_cpms = velocity_counts_per_ms_for_fault(delta_counts, velocity_magnitude_cpms);
        return Ok(Some(safety.fault(
            OUTER_SAFETY_FAULT_POSITION,
            delta_counts,
            dt_us,
            velocity_cpms,
            elapsed_us,
        )));
    }

    Ok(None)
}

/// Get a mutable reference to the safety limits for the current phase.
///
/// Returns `None` for terminal phases (Done/Failed/Commit) that don't
/// need per-tick safety checks.
pub(super) fn outer_safety_mut(phase: &mut OuterPhase) -> Option<&mut OuterSafetyLimits> {
    match phase {
        OuterPhase::MechanicalId { safety, .. }
        | OuterPhase::VelocityTune { safety, .. }
        | OuterPhase::VelocityValidate { safety, .. }
        | OuterPhase::PositionTune { safety, .. }
        | OuterPhase::FilterTune { safety, .. } => Some(safety),
        #[cfg(feature = "trace-velocity-sweep")]
        OuterPhase::VelocitySweep { safety, .. } | OuterPhase::VelocityIntegral { safety, .. } => {
            Some(safety)
        }
        OuterPhase::Commit { .. }
        | OuterPhase::Done(_)
        | OuterPhase::Failed(_)
        | OuterPhase::FailedWithSafety { .. } => None,
    }
}

#[cfg(test)]
mod tests {
    use super::{
        MAX_SAFETY_OBSERVATION_GAP_US, OUTER_SAFETY_SAMPLE_MAX_REGISTER_OPS, OuterMotionBudget,
        OuterSafetyLimits, check_outer_safety,
    };
    use crate::autotune::{CommissionError, CommissionProfile, OperatingMode, TransientBuffer};
    use crate::commission_dispatch::test_support::{MockBackend, SpiOp};
    use crate::commissioning::outer::{
        MECHANICAL_ID_VELOCITY_CAP_COUNTS_PER_MS, MechanicalIdData, MechanicalIdState, OuterPhase,
        step_outer,
    };
    use crate::commissioning::types::{
        OUTER_SAFETY_FAULT_OBSERVATION_GAP, OUTER_SAFETY_FAULT_POSITION,
        OUTER_SAFETY_FAULT_QUARTER_TURN, OUTER_SAFETY_FAULT_VELOCITY, StepResult,
    };
    use foci_tmc4671::registers::{
        ABN_DECODER_COUNT, MODE_RAMP_MODE_MOTION, PID_TORQUE_FLUX_TARGET,
    };

    fn armed_safety(last_sample_us: Option<u64>, next_sample_us: u64) -> OuterSafetyLimits {
        let mut safety = OuterSafetyLimits::new(100, OuterMotionBudget::firmware_caps());
        safety.configure_encoder_resolution(4_000).unwrap();
        safety.set_encoder_baseline(0).unwrap();
        safety.last_sample_us = last_sample_us;
        safety.next_safety_sample_us = Some(next_sample_us);
        safety
    }

    #[test]
    fn primitive_timer_restarts_for_a_retry_after_the_previous_primitive_ends() {
        let mut safety = OuterSafetyLimits::new(100, OuterMotionBudget::firmware_caps());
        safety.unwrapped_position = 37;

        assert_eq!(safety.begin_primitive(0), Ok(()));
        assert_eq!(safety.primitive_start_us, Some(0));
        assert_eq!(safety.primitive_start_position, Some(37));
        safety.end_primitive();
        assert_eq!(safety.primitive_start_us, None);
        assert_eq!(safety.primitive_start_position, None);
        assert_eq!(safety.begin_primitive(5_000), Ok(()));
        assert_eq!(safety.primitive_start_us, Some(5_000));
        assert_eq!(safety.primitive_start_position, Some(37));
    }

    #[test]
    fn primitive_timer_rejects_an_overlapping_start() {
        let mut safety = OuterSafetyLimits::new(100, OuterMotionBudget::firmware_caps());
        safety.begin_primitive(1_000).unwrap();

        assert!(matches!(
            safety.begin_primitive(2_000),
            Err(CommissionError::InvalidSchedule)
        ));
        assert_eq!(safety.primitive_start_us, Some(1_000));
    }

    #[test]
    fn inactive_settle_does_not_consume_a_primitive_duration_budget() {
        pollster::block_on(async {
            let mut safety = armed_safety(Some(3_000_000), 3_000_001);
            safety.primitive_start_us = None;
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 3_000_001;

            let fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("inactive safety observation should succeed");

            assert!(fault.is_none());
            assert_eq!(safety.primitive_start_us, None);
        });
    }

    #[test]
    fn outer_safety_does_not_read_encoder_before_deadline() {
        pollster::block_on(async {
            let mut safety = armed_safety(Some(1_000), 2_000);
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 1_999;

            let fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("an early check should not access the encoder");

            assert!(fault.is_none());
            assert!(backend.spi_log.is_empty());
            assert_eq!(safety.last_sample_us, Some(1_000));
            assert_eq!(safety.next_safety_sample_us, Some(2_000));
        });
    }

    #[test]
    fn late_outer_safety_sample_skips_to_first_future_grid_point() {
        pollster::block_on(async {
            let mut safety = armed_safety(Some(1_000), 2_000);
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 2_500;
            backend.mechanical_encoder_position_counts_value = 4;

            let fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("a due encoder read should succeed");

            assert!(fault.is_none());
            assert_eq!(safety.last_sample_us, Some(2_500));
            assert_eq!(safety.next_safety_sample_us, Some(3_000));
            assert_eq!(backend.spi_log.len(), OUTER_SAFETY_SAMPLE_MAX_REGISTER_OPS);
        });
    }

    #[test]
    fn outer_safety_accepts_a_legitimate_time_zero_sample() {
        pollster::block_on(async {
            let mut safety = armed_safety(None, 0);
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 0;

            let fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("a due time-zero encoder read should succeed");

            assert!(fault.is_none());
            assert_eq!(safety.last_sample_us, Some(0));
            assert_eq!(safety.primitive_start_us, None);
            assert_eq!(safety.next_safety_sample_us, Some(1_000));
        });
    }

    #[test]
    fn outer_safety_observes_zero_target_settle() {
        pollster::block_on(async {
            let safety = armed_safety(Some(0), 1_000);
            let mut phase = OuterPhase::MechanicalId {
                state: MechanicalIdState::ForwardSettle {
                    deadline_us: 300_000,
                },
                data: MechanicalIdData::new(367),
                safety,
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 1_000;
            backend.mechanical_encoder_position_counts_value = 4;

            let result = step_outer(
                &mut phase,
                &mut buf,
                CommissionProfile::Balanced,
                OperatingMode::Nominal,
                0,
                0,
                false,
                0,
                0,
                &mut backend,
            )
            .await;

            assert!(matches!(result, StepResult::At(2_000)));
            assert!(matches!(
                phase,
                OuterPhase::MechanicalId { safety, .. }
                    if safety.unwrapped_position == 4
                        && safety.next_safety_sample_us == Some(2_000)
            ));
        });
    }

    #[test]
    fn outer_safety_rejects_zero_encoder_resolution() {
        let mut safety = OuterSafetyLimits::new(100, OuterMotionBudget::firmware_caps());

        let result = safety.configure_encoder_resolution(0);

        assert!(result.is_err());
        assert!(safety.encoder_domain.is_none());
    }

    #[test]
    fn outer_safety_retains_a_normalized_typed_baseline() {
        let mut safety = OuterSafetyLimits::new(100, OuterMotionBudget::firmware_caps());

        safety.configure_encoder_resolution(4_000).unwrap();
        safety.set_encoder_baseline(8_003).unwrap();

        assert_eq!(
            safety.prev_encoder_count.map(|count| count.raw_counts()),
            Some(3)
        );
    }

    #[test]
    fn outer_safety_uses_mechanical_id_velocity_cap() {
        let safety = OuterSafetyLimits::new(100, OuterMotionBudget::firmware_caps());
        assert_eq!(
            safety.velocity_cap,
            MECHANICAL_ID_VELOCITY_CAP_COUNTS_PER_MS
        );
    }

    #[test]
    fn outer_safety_limits_scale_with_encoder_resolution() {
        let mut safety = OuterSafetyLimits::new(100, OuterMotionBudget::firmware_caps());

        safety.configure_encoder_resolution(8_000).unwrap();

        assert_eq!(safety.counts_per_rev(), 8_000);
        assert_eq!(safety.velocity_cap, 48);
        assert_eq!(safety.position_window, 16_000);
    }

    #[test]
    fn outer_safety_clamps_position_window_to_host_budget() {
        let mut safety =
            OuterSafetyLimits::new(100, OuterMotionBudget::test_budget(500, 6_000, 3_000));

        safety.configure_encoder_resolution(4_000).unwrap();

        assert_eq!(safety.position_window, 2_000);
        assert_eq!(safety.negative_position_window, 8_000);
        assert_eq!(safety.positive_position_window, 8_000);
        assert_eq!(safety.velocity_cap, 24);
    }

    #[test]
    fn outer_safety_legacy_setup_retains_the_two_revolution_backstop() {
        let mut budget = OuterMotionBudget::test_budget(2_500, 6_000, 3_000);
        budget.negative_position_headroom_mrev = 3_000;
        budget.positive_position_headroom_mrev = 4_000;
        let mut safety = OuterSafetyLimits::new(100, budget);

        safety.configure_encoder_resolution(4_000).unwrap();

        assert_eq!(safety.position_window, 8_000);
        assert_eq!(safety.negative_position_window, 8_000);
        assert_eq!(safety.positive_position_window, 8_000);
    }

    #[test]
    fn outer_safety_uses_the_frozen_amplitude_runtime_envelope() {
        let mut safety = OuterSafetyLimits::new(100, OuterMotionBudget::firmware_caps());
        safety.configure_encoder_resolution(4_000).unwrap();

        safety.configure_velocity_envelope(1_000).unwrap();

        assert_eq!(safety.velocity_cap, 12);
    }

    #[test]
    fn outer_safety_marks_soft_lateness_unscored_without_faulting() {
        pollster::block_on(async {
            let mut safety = armed_safety(Some(0), 1_000);
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 1_500;
            backend.mechanical_encoder_position_counts_value = 1;

            let fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("a soft-late observation should remain safe");

            assert!(fault.is_none());
            assert!(!safety.last_observation_scoring_eligible());
        });
    }

    #[test]
    fn outer_safety_marks_on_time_observations_scoring_eligible() {
        pollster::block_on(async {
            let mut safety = armed_safety(Some(0), 1_000);
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 1_000;
            backend.mechanical_encoder_position_counts_value = 1;

            let fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("an on-time observation should remain safe");

            assert!(fault.is_none());
            assert!(safety.last_observation_scoring_eligible());
        });
    }

    #[test]
    fn outer_safety_faults_on_large_gap_before_interpreting_delta() {
        pollster::block_on(async {
            let mut safety = armed_safety(Some(0), 1_000);
            safety.unwrapped_position = 17;
            let before = safety;
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = MAX_SAFETY_OBSERVATION_GAP_US + 1;
            backend.mechanical_encoder_position_counts_value = 1_999;

            let fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("the observation should produce a safety fault")
                .expect("a read-to-read gap over the configured maximum must fault");

            assert_eq!(fault.reason, OUTER_SAFETY_FAULT_OBSERVATION_GAP);
            assert_eq!(safety.prev_encoder_count, before.prev_encoder_count);
            assert_eq!(safety.last_sample_us, before.last_sample_us);
            assert_eq!(safety.unwrapped_position, before.unwrapped_position);
        });
    }

    #[test]
    fn outer_safety_faults_inclusively_at_one_quarter_turn() {
        pollster::block_on(async {
            let mut safety = armed_safety(Some(0), 1_000);
            safety.unwrapped_position = 17;
            let before = safety;
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 1_000;
            backend.mechanical_encoder_position_counts_value = 1_000;

            let fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("the observation should produce a safety fault")
                .expect("an exact quarter-turn delta must fault");

            assert_eq!(fault.reason, OUTER_SAFETY_FAULT_QUARTER_TURN);
            assert_eq!(safety.prev_encoder_count, before.prev_encoder_count);
            assert_eq!(safety.last_sample_us, before.last_sample_us);
            assert_eq!(safety.unwrapped_position, before.unwrapped_position);
        });
    }

    #[test]
    fn outer_safety_local_window_spans_motion_and_zero_settle() {
        pollster::block_on(async {
            let mut safety =
                OuterSafetyLimits::new(100, OuterMotionBudget::test_budget(500, 6_000, 3_000));
            safety.configure_encoder_resolution(4_000).unwrap();
            safety.set_encoder_baseline(0).unwrap();
            safety.last_sample_us = Some(0);
            safety.next_safety_sample_us = Some(1_000);
            safety.begin_primitive(0).unwrap();
            safety.unwrapped_position = 1_999;
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 1_000;
            backend.mechanical_encoder_position_counts_value = 2;

            let fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("the settle observation should succeed")
                .expect("settle drift beyond the full primitive window must fault");

            assert_eq!(fault.reason, OUTER_SAFETY_FAULT_POSITION);
            assert_eq!(fault.position_counts, 2_001);
        });
    }

    #[test]
    fn outer_safety_allows_healthy_deceleration_to_use_settle_reserve() {
        pollster::block_on(async {
            let mut safety =
                OuterSafetyLimits::new(100, OuterMotionBudget::test_budget(500, 6_000, 3_000));
            safety.configure_encoder_resolution(4_000).unwrap();
            safety.set_encoder_baseline(0).unwrap();
            safety.last_sample_us = Some(0);
            safety.next_safety_sample_us = Some(1_000);
            safety.begin_primitive(0).unwrap();
            safety.unwrapped_position = 1_600;
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 1_000;
            backend.mechanical_encoder_position_counts_value = 200;

            let fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("the settle observation should succeed");

            assert!(fault.is_none());
            assert_eq!(safety.unwrapped_position, 1_800);
        });
    }

    #[test]
    fn outer_safety_asymmetric_window_catches_cumulative_drift() {
        pollster::block_on(async {
            let mut budget = OuterMotionBudget::test_budget(500, 6_000, 3_000);
            budget.negative_position_headroom_mrev = 100;
            budget.positive_position_headroom_mrev = 200;
            let mut safety = OuterSafetyLimits::new(100, budget);
            safety.configure_encoder_resolution(4_000).unwrap();
            safety.set_encoder_baseline(0).unwrap();
            safety.last_sample_us = Some(0);
            safety.next_safety_sample_us = Some(1_000);
            safety.unwrapped_position = 799;
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 1_000;
            backend.mechanical_encoder_position_counts_value = 2;

            let fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("the observation should succeed")
                .expect("positive cumulative drift beyond headroom must fault");

            assert_eq!(fault.reason, OUTER_SAFETY_FAULT_POSITION);
            assert_eq!(fault.position_counts, 801);
        });
    }

    #[test]
    fn outer_safety_refuses_zero_budget() {
        let safety = OuterSafetyLimits::new(100, OuterMotionBudget::test_budget(0, 6_000, 3_000));

        assert!(!safety.budget_is_usable());
    }

    #[test]
    fn outer_safety_refuses_reserve_that_consumes_the_window() {
        let mut budget = OuterMotionBudget::test_budget(500, 6_000, 3_000);
        budget.settle_travel_reserve_mrev = 500;
        let safety = OuterSafetyLimits::new(100, budget);

        assert!(!safety.budget_is_usable());
    }

    #[test]
    fn outer_safety_violates_on_unusable_budget() {
        pollster::block_on(async {
            let mut safety =
                OuterSafetyLimits::new(100, OuterMotionBudget::test_budget(0, 6_000, 3_000));
            let mut backend = MockBackend::new();

            let fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("encoder read should succeed");

            assert!(fault.is_some());
        });
    }

    #[test]
    fn outer_safety_violates_on_duration_budget() {
        pollster::block_on(async {
            let mut safety =
                OuterSafetyLimits::new(100, OuterMotionBudget::test_budget(500, 6_000, 1));
            safety.configure_encoder_resolution(4_000).unwrap();
            safety.primitive_start_us = Some(1_000);
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 2_001;

            let fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("encoder read should succeed");

            assert!(fault.is_some());
        });
    }

    #[test]
    fn outer_safety_normalizes_velocity_by_elapsed_time() {
        pollster::block_on(async {
            let mut safety = OuterSafetyLimits::new(100, OuterMotionBudget::firmware_caps());
            safety.configure_encoder_resolution(4_000).unwrap();
            safety.set_encoder_baseline(2328).unwrap();
            safety.last_sample_us = Some(100_000);
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 101_500;
            backend.mechanical_encoder_position_counts_value = 2339;

            let fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("encoder read should succeed");

            assert!(
                fault.is_none(),
                "time-normalized velocity should stay below the cap"
            );
            assert_eq!(
                safety.prev_encoder_count.map(|count| count.raw_counts()),
                Some(2339)
            );
            assert_eq!(safety.last_sample_us, Some(101_500));
            assert_eq!(safety.unwrapped_position, 11);
        });
    }

    #[test]
    fn outer_safety_velocity_window_tolerates_isolated_one_count_spike() {
        pollster::block_on(async {
            let mut safety = armed_safety(Some(100_000), 101_000);
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 101_000;
            backend.mechanical_encoder_position_counts_value = 25;

            let spike_fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("first velocity observation should succeed");

            assert!(
                spike_fault.is_none(),
                "the first interval only primes the window"
            );

            backend.elapsed_us_value = 102_000;
            backend.mechanical_encoder_position_counts_value = 48;
            let recovery_fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("second velocity observation should succeed");

            assert!(
                recovery_fault.is_none(),
                "25 + 23 counts over 2 ms equals the 24 counts/ms cap"
            );
        });
    }

    #[test]
    fn outer_safety_velocity_window_rejects_sustained_excess_on_second_interval() {
        pollster::block_on(async {
            let mut safety = armed_safety(Some(100_000), 101_000);
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 101_000;
            backend.mechanical_encoder_position_counts_value = 25;

            let first_fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("first velocity observation should succeed");

            assert!(
                first_fault.is_none(),
                "the first interval only primes the window"
            );

            backend.elapsed_us_value = 102_000;
            backend.mechanical_encoder_position_counts_value = 50;
            let second_fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("second velocity observation should succeed")
                .expect("two 25-count intervals must exceed the 24 counts/ms cap");

            assert_eq!(second_fault.reason, OUTER_SAFETY_FAULT_VELOCITY);
        });
    }

    #[test]
    fn outer_safety_velocity_window_resets_with_encoder_baseline() {
        pollster::block_on(async {
            let mut safety = armed_safety(Some(100_000), 101_000);
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 101_000;
            backend.mechanical_encoder_position_counts_value = 25;

            assert!(
                check_outer_safety(&mut safety, &mut backend)
                    .await
                    .expect("first velocity observation should succeed")
                    .is_none()
            );

            safety.set_encoder_baseline(25).unwrap();
            backend.elapsed_us_value = 102_000;
            backend.mechanical_encoder_position_counts_value = 50;

            assert!(
                check_outer_safety(&mut safety, &mut backend)
                    .await
                    .expect("post-baseline observation should succeed")
                    .is_none(),
                "a new encoder baseline must prime a new velocity window"
            );
        });
    }

    #[test]
    fn outer_safety_velocity_window_resets_with_motion_primitive() {
        pollster::block_on(async {
            let mut safety = armed_safety(Some(100_000), 101_000);
            safety.begin_primitive(100_000).unwrap();
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 101_000;
            backend.mechanical_encoder_position_counts_value = 25;

            assert!(
                check_outer_safety(&mut safety, &mut backend)
                    .await
                    .expect("first velocity observation should succeed")
                    .is_none()
            );

            safety.end_primitive();
            safety.begin_primitive(101_000).unwrap();
            backend.elapsed_us_value = 102_000;
            backend.mechanical_encoder_position_counts_value = 50;

            assert!(
                check_outer_safety(&mut safety, &mut backend)
                    .await
                    .expect("new-primitive observation should succeed")
                    .is_none(),
                "a new motion primitive must prime a new velocity window"
            );
        });
    }

    #[test]
    fn outer_safety_tracks_forward_origin_crossing() {
        pollster::block_on(async {
            let mut safety = OuterSafetyLimits::new(100, OuterMotionBudget::firmware_caps());
            safety.configure_encoder_resolution(4_000).unwrap();
            safety.set_encoder_baseline(3_998).unwrap();
            safety.last_sample_us = Some(100_000);
            safety.primitive_start_us = Some(90_000);
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 101_000;
            backend.mechanical_encoder_position_counts_value = 2;

            let fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("encoder read should succeed");

            assert!(fault.is_none());
            assert_eq!(
                safety.prev_encoder_count.map(|count| count.raw_counts()),
                Some(2)
            );
            assert_eq!(safety.unwrapped_position, 4);
        });
    }

    #[test]
    fn outer_safety_tracks_reverse_origin_crossing() {
        pollster::block_on(async {
            let mut safety = OuterSafetyLimits::new(100, OuterMotionBudget::firmware_caps());
            safety.configure_encoder_resolution(4_000).unwrap();
            safety.set_encoder_baseline(2).unwrap();
            safety.last_sample_us = Some(100_000);
            safety.primitive_start_us = Some(90_000);
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 101_000;
            backend.mechanical_encoder_position_counts_value = 3_998;

            let fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("encoder read should succeed");

            assert!(fault.is_none());
            assert_eq!(
                safety.prev_encoder_count.map(|count| count.raw_counts()),
                Some(3_998)
            );
            assert_eq!(safety.unwrapped_position, -4);
        });
    }

    #[test]
    fn outer_safety_half_turn_fault_preserves_accumulation_state() {
        pollster::block_on(async {
            let mut safety = OuterSafetyLimits::new(100, OuterMotionBudget::firmware_caps());
            safety.configure_encoder_resolution(4_000).unwrap();
            safety.set_encoder_baseline(1_000).unwrap();
            safety.last_sample_us = Some(100_000);
            safety.primitive_start_us = Some(90_000);
            safety.unwrapped_position = 17;
            let before = safety;
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 101_000;
            backend.mechanical_encoder_position_counts_value = 3_000;

            let fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("encoder read should succeed")
                .expect("ambiguous half-turn should violate safety");

            assert_eq!(fault.reason, OUTER_SAFETY_FAULT_QUARTER_TURN);
            assert_eq!(safety.prev_encoder_count, before.prev_encoder_count);
            assert_eq!(safety.last_sample_us, before.last_sample_us);
            assert_eq!(safety.primitive_start_us, before.primitive_start_us);
            assert_eq!(safety.unwrapped_position, before.unwrapped_position);
        });
    }

    #[test]
    fn outer_safety_half_turn_stops_dispatch_before_phase_motion() {
        pollster::block_on(async {
            let mut safety = OuterSafetyLimits::new(2448, OuterMotionBudget::firmware_caps());
            safety.configure_encoder_resolution(4_000).unwrap();
            safety.set_encoder_baseline(1_000).unwrap();
            safety.last_sample_us = Some(100_000);
            safety.primitive_start_us = Some(90_000);
            let mut phase = OuterPhase::MechanicalId {
                state: MechanicalIdState::ForwardCapture {
                    next_sample_us: 101_000,
                    capture_start_us: 90_000,
                },
                data: MechanicalIdData::new(367),
                safety,
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 101_000;
            backend.mechanical_encoder_position_counts_value = 3_000;

            let result = step_outer(
                &mut phase,
                &mut buf,
                CommissionProfile::Balanced,
                OperatingMode::Nominal,
                0,
                0,
                false,
                0,
                0,
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::OuterFailed(CommissionError::SafetyEnvelopeViolation)
            ));
            assert!(matches!(
                phase,
                OuterPhase::FailedWithSafety {
                    error: CommissionError::SafetyEnvelopeViolation,
                    safety_fault
                } if safety_fault.reason == OUTER_SAFETY_FAULT_QUARTER_TURN
            ));
            assert!(backend.spi_log.iter().any(|op| matches!(
                op,
                SpiOp::Write(addr, 0) if *addr == PID_TORQUE_FLUX_TARGET.addr()
            )));
            assert!(backend.spi_log.iter().any(|op| matches!(
                op,
                SpiOp::Write(addr, 0) if *addr == MODE_RAMP_MODE_MOTION.addr()
            )));
            let writes_after_failure = backend.spi_log.len();

            let terminal_result = step_outer(
                &mut phase,
                &mut buf,
                CommissionProfile::Balanced,
                OperatingMode::Nominal,
                0,
                0,
                false,
                0,
                0,
                &mut backend,
            )
            .await;

            assert!(matches!(
                terminal_result,
                StepResult::OuterFailed(CommissionError::SafetyEnvelopeViolation)
            ));
            assert_eq!(backend.spi_log.len(), writes_after_failure);
        });
    }

    #[test]
    fn outer_safety_reports_velocity_fault_details() {
        pollster::block_on(async {
            let mut safety =
                OuterSafetyLimits::new(100, OuterMotionBudget::test_budget(750, 6_000, 3_000));
            safety.configure_encoder_resolution(4_000).unwrap();
            safety.set_encoder_baseline(0).unwrap();
            safety.last_sample_us = Some(100_000);
            safety.primitive_start_us = Some(90_000);
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 101_000;
            backend.mechanical_encoder_position_counts_value = 25;

            let first_fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("first velocity observation should succeed");
            assert!(first_fault.is_none());

            backend.elapsed_us_value = 102_000;
            backend.mechanical_encoder_position_counts_value = 50;
            let fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("encoder read should succeed")
                .expect("velocity should violate the safety envelope");

            assert_eq!(fault.reason, OUTER_SAFETY_FAULT_VELOCITY);
            assert_eq!(fault.max_travel_mrev, 750);
            assert_eq!(fault.max_velocity_mrev_s, 6000);
            assert_eq!(fault.max_duration_ms, 3000);
            assert_eq!(fault.direction_mask, 3);
            assert_eq!(fault.delta_counts, 25);
            assert_eq!(fault.dt_us, 1_000);
            assert_eq!(fault.velocity_counts_per_ms, 25);
            assert_eq!(fault.velocity_cap_counts_per_ms, 24);
            assert_eq!(fault.position_counts, 50);
            assert_eq!(fault.position_window_counts, 3000);
            assert_eq!(fault.elapsed_us, 12_000);
            assert_eq!(fault.duration_cap_us, 3_000_000);
        });
    }

    #[test]
    fn outer_safety_reports_large_positive_velocity_fault_evidence() {
        pollster::block_on(async {
            const LARGE_DELTA: u32 = 536_870_910;
            let mut safety = OuterSafetyLimits::new(100, OuterMotionBudget::firmware_caps());
            safety
                .configure_encoder_resolution(i32::MAX as u32)
                .unwrap();
            safety.set_encoder_baseline(0).unwrap();
            safety.last_sample_us = Some(100);
            safety.primitive_start_us = Some(90);
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 101;
            backend.mechanical_encoder_position_counts_value = LARGE_DELTA;

            let first_fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("first velocity observation should succeed");
            assert!(first_fault.is_none());

            backend.elapsed_us_value = 1_101;
            backend.mechanical_encoder_position_counts_value = LARGE_DELTA * 2;
            let fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("encoder read should succeed")
                .expect("velocity should violate the safety envelope");

            assert_eq!(fault.reason, OUTER_SAFETY_FAULT_VELOCITY);
            assert_eq!(fault.delta_counts, LARGE_DELTA as i32);
            assert_eq!(fault.velocity_counts_per_ms, LARGE_DELTA as i32);
        });
    }

    #[test]
    fn outer_safety_reports_large_negative_velocity_fault_evidence() {
        pollster::block_on(async {
            const LARGE_DELTA: u32 = 536_870_910;
            const ENCODER_DOMAIN: u32 = i32::MAX as u32;
            let mut safety = OuterSafetyLimits::new(100, OuterMotionBudget::firmware_caps());
            safety
                .configure_encoder_resolution(i32::MAX as u32)
                .unwrap();
            safety.set_encoder_baseline(0).unwrap();
            safety.last_sample_us = Some(100);
            safety.primitive_start_us = Some(90);
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 101;
            backend.mechanical_encoder_position_counts_value = ENCODER_DOMAIN - LARGE_DELTA;

            let first_fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("first velocity observation should succeed");
            assert!(first_fault.is_none());

            backend.elapsed_us_value = 1_101;
            backend.mechanical_encoder_position_counts_value = ENCODER_DOMAIN - LARGE_DELTA * 2;
            let fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("encoder read should succeed")
                .expect("velocity should violate the safety envelope");

            assert_eq!(fault.reason, OUTER_SAFETY_FAULT_VELOCITY);
            assert_eq!(fault.delta_counts, -(LARGE_DELTA as i32));
            assert_eq!(fault.velocity_counts_per_ms, -(LARGE_DELTA as i32));
        });
    }

    #[test]
    fn outer_safety_read_error_fails_with_spi() {
        pollster::block_on(async {
            let mut safety = OuterSafetyLimits::new(2448, OuterMotionBudget::firmware_caps());
            safety.configure_encoder_resolution(4_000).unwrap();
            safety.last_sample_us = Some(1_000);
            let mut phase = OuterPhase::MechanicalId {
                state: MechanicalIdState::ForwardCapture {
                    next_sample_us: 2_000,
                    capture_start_us: 1_000,
                },
                data: MechanicalIdData::new(367),
                safety,
            };
            let mut buf = TransientBuffer::new();
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 2_000;
            backend.fail_on_read = Some(ABN_DECODER_COUNT.addr());

            let result = step_outer(
                &mut phase,
                &mut buf,
                CommissionProfile::Balanced,
                OperatingMode::Nominal,
                0,
                0,
                false,
                0,
                0,
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::OuterFailed(CommissionError::Spi)
            ));
            assert!(matches!(phase, OuterPhase::Failed(CommissionError::Spi)));
        });
    }
}
