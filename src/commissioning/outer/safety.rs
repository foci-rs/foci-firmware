//! Outer commissioning safety-envelope checks.

use crate::autotune::CommissionError;
use crate::commissioning::backend::CommissioningBackend;
use crate::commissioning::encoder_domain::{EncoderCount, EncoderDomain, EncoderMathError};
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
    /// Zero means the next sample initializes the baseline.
    pub(crate) last_sample_us: u64,
    /// Timestamp (microseconds) of the first armed safety sample.
    pub(crate) start_sample_us: u64,
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
            last_sample_us: 0,
            start_sample_us: 0,
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
) -> Result<Option<OuterSafetyFault>, B::Error> {
    // Safety limits are initialized by MechanicalId::Setup after it reads
    // the real encoder baseline. Skip the pre-setup placeholder values.
    if safety.torque_cap == 0 {
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
        .await?;
    let encoder_count = safety
        .encoder_domain
        .expect("outer encoder domain configured before motion")
        .count(encoder_raw);
    let now_us = backend.elapsed_us();

    if safety.start_sample_us == 0 {
        safety.start_sample_us = now_us;
    }
    let elapsed_us = now_us.wrapping_sub(safety.start_sample_us);
    let max_duration_us = u64::from(safety.budget.max_duration_ms) * 1_000;
    if elapsed_us > max_duration_us {
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

    // Wrap-aware delta in encoder counts since the last safety sample.
    let delta_counts = response::encoder_delta(
        encoder_count.raw_counts(),
        safety
            .prev_encoder_count
            .expect("outer encoder baseline configured before motion")
            .raw_counts(),
        safety.counts_per_rev(),
    );
    let dt_us = if safety.last_sample_us == 0 {
        0
    } else {
        now_us.wrapping_sub(safety.last_sample_us)
    };
    let velocity_cpms = response::counts_per_ms(delta_counts, dt_us);
    safety.prev_encoder_count = Some(encoder_count);
    safety.last_sample_us = now_us;
    safety.unwrapped_position += delta_counts;

    // Check velocity cap (absolute value).
    let abs_velocity = velocity_cpms.unsigned_abs();
    if abs_velocity > safety.velocity_cap {
        defmt::warn!(
            "outer safety: velocity {} exceeds cap {} (delta={} dt_us={})",
            abs_velocity,
            safety.velocity_cap,
            delta_counts,
            dt_us,
        );
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
    use super::{OuterMotionBudget, OuterSafetyLimits, check_outer_safety};
    use crate::autotune::{CommissionError, CommissionProfile, OperatingMode, TransientBuffer};
    use crate::commission_dispatch::test_support::MockBackend;
    use crate::commissioning::outer::{
        MECHANICAL_ID_VELOCITY_CAP_COUNTS_PER_MS, MechanicalIdData, MechanicalIdState, OuterPhase,
        step_outer,
    };
    use crate::commissioning::types::{OUTER_SAFETY_FAULT_VELOCITY, StepResult};
    use foci_tmc4671::registers::ABN_DECODER_COUNT;

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
            safety.start_sample_us = 1_000;
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
            safety.last_sample_us = 100_000;
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
            assert_eq!(safety.last_sample_us, 124_909);
            assert_eq!(safety.unwrapped_position, 197);
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
            safety.last_sample_us = 100_000;
            safety.start_sample_us = 90_000;
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
    fn outer_safety_read_error_fails_with_spi() {
        pollster::block_on(async {
            let mut safety = OuterSafetyLimits::new(2448, OuterMotionBudget::firmware_caps());
            safety.configure_encoder_resolution(4_000).unwrap();
            safety.last_sample_us = 1_000;
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
