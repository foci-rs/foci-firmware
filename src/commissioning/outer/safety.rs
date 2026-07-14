//! Outer commissioning safety-envelope checks.

use crate::autotune::CommissionError;
use crate::commissioning::backend::CommissioningBackend;
use crate::commissioning::encoder_domain::{EncoderCount, EncoderDomain, EncoderMathError};
use crate::commissioning::schedule::next_grid_deadline;
use crate::commissioning::types::{
    OUTER_SAFETY_FAULT_DURATION, OUTER_SAFETY_FAULT_INVALID_BUDGET, OUTER_SAFETY_FAULT_POSITION,
    OUTER_SAFETY_FAULT_UNUSABLE_BUDGET, OUTER_SAFETY_FAULT_VELOCITY, OuterSafetyFault,
};

use super::limits::{MECHANICAL_ID_VELOCITY_CAP_COUNTS_PER_MS, POSITION_WINDOW_COUNTS};
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
    /// Maximum local motor travel in milli-revolutions.
    pub(crate) max_travel_mrev: u32,
    /// Maximum local motor velocity in milli-revolutions per second.
    pub(crate) max_velocity_mrev_s: u32,
    /// Maximum primitive duration in milliseconds.
    pub(crate) max_duration_ms: u16,
    /// Reserved direction mask. Supports bidirectional value 0x03.
    pub(crate) direction_mask: u8,
}

impl OuterMotionBudget {
    /// Current firmware hard caps, used by tests that don't exercise host budget tightening.
    #[cfg(test)]
    pub(crate) const fn firmware_caps() -> Self {
        Self {
            max_travel_mrev: FIRMWARE_MAX_TRAVEL_MREV,
            max_velocity_mrev_s: FIRMWARE_MAX_VELOCITY_MREV_S,
            max_duration_ms: 3_000,
            direction_mask: 0x03,
        }
    }

    fn travel_counts(self, counts_per_rev: u32) -> u32 {
        mrev_to_counts(self.max_travel_mrev, counts_per_rev)
    }

    fn velocity_counts_per_ms(self, counts_per_rev: u32) -> u32 {
        let counts_per_rev = u64::from(counts_per_rev);
        let scaled = counts_per_rev.saturating_mul(self.max_velocity_mrev_s as u64);
        let rounded = (scaled + 500_000) / 1_000_000;
        clamp_positive_u32(rounded)
    }
}

/// Outer safety limits, computed once at the start of outer commissioning.
#[derive(Debug, Clone, Copy, defmt::Format)]
pub(crate) struct OuterSafetyLimits {
    /// Maximum absolute velocity in encoder counts per millisecond.
    pub(crate) velocity_cap: u32,
    /// Maximum distance from start position in encoder counts.
    pub(crate) position_window: u32,
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
    /// Absolute deadline for the next independent safety observation.
    pub(crate) next_safety_sample_us: Option<u64>,
    /// Host-provided motion budget used to clamp firmware safety caps.
    pub(crate) budget: OuterMotionBudget,
}

impl OuterSafetyLimits {
    pub(crate) fn new(torque_cap: u16, budget: OuterMotionBudget) -> Self {
        Self {
            velocity_cap: MECHANICAL_ID_VELOCITY_CAP_COUNTS_PER_MS,
            position_window: POSITION_WINDOW_COUNTS,
            unwrapped_position: 0,
            torque_cap,
            prev_encoder_count: None,
            encoder_domain: None,
            last_sample_us: None,
            primitive_start_us: None,
            next_safety_sample_us: None,
            budget,
        }
    }

    pub(crate) fn budget_request_is_valid(&self) -> bool {
        self.budget.max_travel_mrev > 0
            && self.budget.max_velocity_mrev_s > 0
            && self.budget.max_duration_ms > 0
            && self.budget.direction_mask == 0x03
    }

    pub(crate) fn configure_encoder_resolution(
        &mut self,
        counts_per_rev: u32,
    ) -> Result<(), EncoderMathError> {
        let encoder_domain = EncoderDomain::new(counts_per_rev)?;
        self.encoder_domain = Some(encoder_domain);
        self.prev_encoder_count = None;
        self.velocity_cap = mechanical_id_velocity_cap_counts_per_ms(counts_per_rev)
            .min(self.budget.velocity_counts_per_ms(counts_per_rev));
        self.position_window =
            position_window_counts(counts_per_rev).min(self.budget.travel_counts(counts_per_rev));
        Ok(())
    }

    pub(crate) fn set_encoder_baseline(&mut self, raw_counts: u32) -> Result<(), EncoderMathError> {
        let domain = self
            .encoder_domain
            .ok_or(EncoderMathError::InvalidCountsPerRev)?;
        self.prev_encoder_count = Some(domain.count(raw_counts));
        Ok(())
    }

    /// Arm the independent safety-observation grid from a known baseline.
    pub(crate) fn arm_safety_observation(&mut self, now_us: u64) -> Result<(), CommissionError> {
        let next_safety_sample_us = now_us
            .checked_add(OUTER_SAFETY_SAMPLE_PERIOD_US)
            .ok_or(CommissionError::InvalidSchedule)?;
        self.last_sample_us = Some(now_us);
        self.next_safety_sample_us = Some(next_safety_sample_us);
        Ok(())
    }

    /// Begin a bounded motion primitive after its nonzero target is applied.
    pub(crate) fn begin_primitive(&mut self, now_us: u64) -> Result<(), CommissionError> {
        if self.primitive_start_us.is_some() {
            return Err(CommissionError::InvalidSchedule);
        }
        self.primitive_start_us = Some(now_us);
        Ok(())
    }

    /// End the active motion primitive after its safe target is applied.
    pub(crate) fn end_primitive(&mut self) {
        self.primitive_start_us = None;
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
            .max_travel_mrev
            .min(FIRMWARE_MAX_TRAVEL_MREV)
            .min(u16::MAX as u32) as u16
    }

    pub(crate) fn budget_clamped_firmware_caps(&self) -> bool {
        self.budget.max_travel_mrev < FIRMWARE_MAX_TRAVEL_MREV
            || self.budget.max_velocity_mrev_s < FIRMWARE_MAX_VELOCITY_MREV_S
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
            max_travel_mrev: clamp_u32_to_u16(self.budget.max_travel_mrev),
            max_velocity_mrev_s: clamp_u32_to_u16(self.budget.max_velocity_mrev_s),
            max_duration_ms: self.budget.max_duration_ms,
            direction_mask: self.budget.direction_mask,
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
}

fn mrev_to_counts(mrev: u32, counts_per_rev: u32) -> u32 {
    let counts_per_rev = u64::from(counts_per_rev);
    let scaled = counts_per_rev.saturating_mul(mrev as u64);
    clamp_positive_u32(scaled / 1_000)
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
            safety.budget.max_travel_mrev,
            safety.budget.max_velocity_mrev_s,
            safety.budget.max_duration_ms,
            safety.budget.direction_mask,
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
                OUTER_SAFETY_FAULT_VELOCITY,
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

    // Check velocity cap using the typed displacement magnitude.
    if velocity_magnitude_cpms > u64::from(safety.velocity_cap) {
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

    // Check position window against unwrapped accumulator.
    let abs_position = safety.unwrapped_position.unsigned_abs();
    if abs_position > safety.position_window {
        defmt::warn!(
            "outer safety: position {} exceeds window {}",
            abs_position,
            safety.position_window
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
        OuterPhase::Commit { .. }
        | OuterPhase::Done(_)
        | OuterPhase::Failed(_)
        | OuterPhase::FailedWithSafety { .. } => None,
    }
}

#[cfg(test)]
mod tests {
    use super::{
        OUTER_SAFETY_SAMPLE_MAX_REGISTER_OPS, OuterMotionBudget, OuterSafetyLimits,
        check_outer_safety,
    };
    use crate::autotune::{CommissionError, CommissionProfile, OperatingMode, TransientBuffer};
    use crate::commission_dispatch::test_support::{MockBackend, SpiOp};
    use crate::commissioning::outer::{
        MECHANICAL_ID_VELOCITY_CAP_COUNTS_PER_MS, MechanicalIdData, MechanicalIdState, OuterPhase,
        step_outer,
    };
    use crate::commissioning::types::{OUTER_SAFETY_FAULT_VELOCITY, StepResult};
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

        assert_eq!(safety.begin_primitive(0), Ok(()));
        assert_eq!(safety.primitive_start_us, Some(0));
        safety.end_primitive();
        assert_eq!(safety.primitive_start_us, None);
        assert_eq!(safety.begin_primitive(5_000), Ok(()));
        assert_eq!(safety.primitive_start_us, Some(5_000));
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
            let mut safety = armed_safety(Some(0), 3_000_001);
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
            backend.elapsed_us_value = 4_500;
            backend.mechanical_encoder_position_counts_value = 4;

            let fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("a due encoder read should succeed");

            assert!(fault.is_none());
            assert_eq!(safety.last_sample_us, Some(4_500));
            assert_eq!(safety.next_safety_sample_us, Some(5_000));
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
        let mut safety = OuterSafetyLimits::new(
            100,
            OuterMotionBudget {
                max_travel_mrev: 500,
                max_velocity_mrev_s: 6000,
                max_duration_ms: 3000,
                direction_mask: 3,
            },
        );

        safety.configure_encoder_resolution(4_000).unwrap();

        assert_eq!(safety.position_window, 2_000);
        assert_eq!(safety.velocity_cap, 24);
    }

    #[test]
    fn outer_safety_refuses_zero_budget() {
        let safety = OuterSafetyLimits::new(
            100,
            OuterMotionBudget {
                max_travel_mrev: 0,
                max_velocity_mrev_s: 6000,
                max_duration_ms: 3000,
                direction_mask: 3,
            },
        );

        assert!(!safety.budget_is_usable());
    }

    #[test]
    fn outer_safety_refuses_non_bidirectional_direction_mask() {
        let safety = OuterSafetyLimits::new(
            100,
            OuterMotionBudget {
                max_travel_mrev: 500,
                max_velocity_mrev_s: 6000,
                max_duration_ms: 3000,
                direction_mask: 1,
            },
        );

        assert!(!safety.budget_is_usable());
    }

    #[test]
    fn outer_safety_violates_on_unusable_budget() {
        pollster::block_on(async {
            let mut safety = OuterSafetyLimits::new(
                100,
                OuterMotionBudget {
                    max_travel_mrev: 0,
                    max_velocity_mrev_s: 6000,
                    max_duration_ms: 3000,
                    direction_mask: 3,
                },
            );
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
            let mut safety = OuterSafetyLimits::new(
                100,
                OuterMotionBudget {
                    max_travel_mrev: 500,
                    max_velocity_mrev_s: 6000,
                    max_duration_ms: 1,
                    direction_mask: 3,
                },
            );
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
            backend.elapsed_us_value = 124_909;
            backend.mechanical_encoder_position_counts_value = 2525; // delta=197 over 24.909ms => ~7 counts/ms

            let fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("encoder read should succeed");

            assert!(
                fault.is_none(),
                "time-normalized velocity should stay below the cap"
            );
            assert_eq!(
                safety.prev_encoder_count.map(|count| count.raw_counts()),
                Some(2525)
            );
            assert_eq!(safety.last_sample_us, Some(124_909));
            assert_eq!(safety.unwrapped_position, 197);
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

            assert_eq!(fault.reason, OUTER_SAFETY_FAULT_VELOCITY);
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
                } if safety_fault.reason == OUTER_SAFETY_FAULT_VELOCITY
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
            let mut safety = OuterSafetyLimits::new(
                100,
                OuterMotionBudget {
                    max_travel_mrev: 750,
                    max_velocity_mrev_s: 6000,
                    max_duration_ms: 3000,
                    direction_mask: 3,
                },
            );
            safety.configure_encoder_resolution(4_000).unwrap();
            safety.set_encoder_baseline(0).unwrap();
            safety.last_sample_us = Some(100_000);
            safety.primitive_start_us = Some(90_000);
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 104_000;
            backend.mechanical_encoder_position_counts_value = 125;

            let fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("encoder read should succeed")
                .expect("velocity should violate the safety envelope");

            assert_eq!(fault.reason, OUTER_SAFETY_FAULT_VELOCITY);
            assert_eq!(fault.max_travel_mrev, 750);
            assert_eq!(fault.max_velocity_mrev_s, 6000);
            assert_eq!(fault.max_duration_ms, 3000);
            assert_eq!(fault.direction_mask, 3);
            assert_eq!(fault.delta_counts, 125);
            assert_eq!(fault.dt_us, 4_000);
            assert_eq!(fault.velocity_counts_per_ms, 31);
            assert_eq!(fault.velocity_cap_counts_per_ms, 24);
            assert_eq!(fault.position_counts, 125);
            assert_eq!(fault.position_window_counts, 3000);
            assert_eq!(fault.elapsed_us, 14_000);
            assert_eq!(fault.duration_cap_us, 3_000_000);
        });
    }

    #[test]
    fn outer_safety_saturates_positive_velocity_fault_evidence() {
        pollster::block_on(async {
            let mut safety = OuterSafetyLimits::new(100, OuterMotionBudget::firmware_caps());
            safety
                .configure_encoder_resolution(i32::MAX as u32)
                .unwrap();
            safety.set_encoder_baseline(0).unwrap();
            safety.last_sample_us = Some(100);
            safety.primitive_start_us = Some(90);
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 101;
            backend.mechanical_encoder_position_counts_value = 1_073_741_823;

            let fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("encoder read should succeed")
                .expect("velocity should violate the safety envelope");

            assert_eq!(fault.reason, OUTER_SAFETY_FAULT_VELOCITY);
            assert_eq!(fault.delta_counts, 1_073_741_823);
            assert_eq!(fault.velocity_counts_per_ms, i32::MAX);
        });
    }

    #[test]
    fn outer_safety_saturates_negative_velocity_fault_evidence() {
        pollster::block_on(async {
            let mut safety = OuterSafetyLimits::new(100, OuterMotionBudget::firmware_caps());
            safety
                .configure_encoder_resolution(i32::MAX as u32)
                .unwrap();
            safety.set_encoder_baseline(0).unwrap();
            safety.last_sample_us = Some(100);
            safety.primitive_start_us = Some(90);
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 101;
            backend.mechanical_encoder_position_counts_value = 1_073_741_824;

            let fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("encoder read should succeed")
                .expect("velocity should violate the safety envelope");

            assert_eq!(fault.reason, OUTER_SAFETY_FAULT_VELOCITY);
            assert_eq!(fault.delta_counts, -1_073_741_823);
            assert_eq!(fault.velocity_counts_per_ms, i32::MIN);
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
