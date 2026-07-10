//! Outer commissioning safety-envelope checks.

use crate::autotune::CommissionError;
use crate::commissioning::backend::CommissioningBackend;

use super::limits::{
    MECHANICAL_ID_VELOCITY_CAP_COUNTS_PER_MS, POSITION_WINDOW_COUNTS, normalized_counts_per_rev,
};
use super::{
    mechanical_id_velocity_cap_counts_per_ms, position_window_counts, response, state::OuterPhase,
};

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
            max_travel_mrev: 2_000,
            max_velocity_mrev_s: 6_000,
            max_duration_ms: 3_000,
            direction_mask: 0x03,
        }
    }

    fn travel_counts(self, counts_per_rev: i32) -> i32 {
        mrev_to_counts(self.max_travel_mrev, counts_per_rev)
    }

    fn velocity_counts_per_ms(self, counts_per_rev: i32) -> i32 {
        let counts_per_rev = normalized_counts_per_rev(counts_per_rev) as u64;
        let scaled = counts_per_rev.saturating_mul(self.max_velocity_mrev_s as u64);
        let rounded = (scaled + 500_000) / 1_000_000;
        clamp_positive_i32(rounded)
    }
}

/// Outer safety limits, computed once at the start of outer commissioning.
#[derive(Debug, Clone, Copy, defmt::Format)]
pub(crate) struct OuterSafetyLimits {
    /// Maximum absolute velocity in encoder counts per millisecond.
    pub(crate) velocity_cap: i32,
    /// Maximum distance from start position in encoder counts.
    pub(crate) position_window: i32,
    /// Unwrapped signed position accumulated from wrap-aware deltas.
    pub(crate) unwrapped_position: i32,
    /// Torque cap: the configured run_current limit.
    pub(crate) torque_cap: u16,
    /// Previous raw encoder count for delta computation.
    pub(crate) prev_encoder_count: i32,
    /// Encoder counts per revolution (PPR * 4). Used to normalize
    /// wrap-around deltas when the ABN decoder counter rolls over.
    pub(crate) counts_per_rev: i32,
    /// Timestamp (microseconds) of the previous safety sample.
    /// Zero means the next sample initializes the baseline.
    pub(crate) last_sample_us: u64,
    /// Timestamp (microseconds) of the first armed safety sample.
    pub(crate) start_sample_us: u64,
    /// Host-provided motion budget used to clamp firmware safety caps.
    pub(crate) budget: OuterMotionBudget,
}

impl OuterSafetyLimits {
    pub(crate) fn new(start_position: i32, torque_cap: u16, budget: OuterMotionBudget) -> Self {
        Self {
            velocity_cap: MECHANICAL_ID_VELOCITY_CAP_COUNTS_PER_MS,
            position_window: POSITION_WINDOW_COUNTS,
            unwrapped_position: 0,
            torque_cap,
            prev_encoder_count: start_position,
            counts_per_rev: 0, // Set by MechanicalId::Setup after reading ABN_DECODER_PPR
            last_sample_us: 0,
            start_sample_us: 0,
            budget,
        }
    }

    pub(crate) fn configure_encoder_resolution(&mut self, counts_per_rev: i32) {
        let counts_per_rev = normalized_counts_per_rev(counts_per_rev);
        self.counts_per_rev = counts_per_rev;
        self.velocity_cap = mechanical_id_velocity_cap_counts_per_ms(counts_per_rev)
            .min(self.budget.velocity_counts_per_ms(counts_per_rev));
        self.position_window =
            position_window_counts(counts_per_rev).min(self.budget.travel_counts(counts_per_rev));
    }

    pub(crate) fn budget_is_usable(&self) -> bool {
        self.budget.max_travel_mrev > 0
            && self.budget.max_velocity_mrev_s > 0
            && self.budget.max_duration_ms > 0
            && self.budget.direction_mask == 0x03
    }
}

fn mrev_to_counts(mrev: u32, counts_per_rev: i32) -> i32 {
    let counts_per_rev = normalized_counts_per_rev(counts_per_rev) as u64;
    let scaled = counts_per_rev.saturating_mul(mrev as u64);
    clamp_positive_i32(scaled / 1_000)
}

fn clamp_positive_i32(value: u64) -> i32 {
    if value == 0 {
        1
    } else if value > i32::MAX as u64 {
        i32::MAX
    } else {
        value as i32
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
/// Returns `true` if a safety limit is violated.
pub(crate) async fn check_outer_safety<B: CommissioningBackend>(
    safety: &mut OuterSafetyLimits,
    backend: &mut B,
) -> Result<bool, B::Error> {
    // Safety limits are initialized by MechanicalId::Setup after it reads
    // the real encoder baseline. Skip the pre-setup placeholder values.
    if safety.torque_cap == 0 {
        return Ok(false);
    }

    if !safety.budget_is_usable() {
        defmt::warn!(
            "outer safety: unusable motion budget travel={} velocity={} duration={} dir={}",
            safety.budget.max_travel_mrev,
            safety.budget.max_velocity_mrev_s,
            safety.budget.max_duration_ms,
            safety.budget.direction_mask,
        );
        return Ok(true);
    }

    let encoder_raw = backend.try_read_encoder_count().await?;
    let encoder_count = encoder_raw as i32;
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
        return Ok(true);
    }

    // Wrap-aware delta in encoder counts since the last safety sample.
    let delta_counts = response::encoder_delta(
        encoder_count,
        safety.prev_encoder_count,
        safety.counts_per_rev,
    );
    let dt_us = if safety.last_sample_us == 0 {
        0
    } else {
        now_us.wrapping_sub(safety.last_sample_us)
    };
    let velocity_cpms = response::counts_per_ms(delta_counts, dt_us);
    safety.prev_encoder_count = encoder_count;
    safety.last_sample_us = now_us;
    safety.unwrapped_position += delta_counts;

    // Check velocity cap (absolute value).
    let abs_velocity = velocity_cpms.unsigned_abs() as i32;
    if abs_velocity > safety.velocity_cap {
        defmt::warn!(
            "outer safety: velocity {} exceeds cap {} (delta={} dt_us={})",
            abs_velocity,
            safety.velocity_cap,
            delta_counts,
            dt_us,
        );
        return Ok(true);
    }

    // Check position window against unwrapped accumulator.
    let abs_position = safety.unwrapped_position.unsigned_abs() as i32;
    if abs_position > safety.position_window {
        defmt::warn!(
            "outer safety: position {} exceeds window {}",
            abs_position,
            safety.position_window
        );
        return Ok(true);
    }

    Ok(false)
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
        OuterPhase::Commit { .. } | OuterPhase::Done(_) | OuterPhase::Failed(_) => None,
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
    use crate::commissioning::types::StepResult;
    use foci_tmc4671::registers::ABN_DECODER_COUNT;

    #[test]
    fn outer_safety_uses_mechanical_id_velocity_cap() {
        let safety = OuterSafetyLimits::new(0, 100, OuterMotionBudget::firmware_caps());
        assert_eq!(
            safety.velocity_cap,
            MECHANICAL_ID_VELOCITY_CAP_COUNTS_PER_MS
        );
    }

    #[test]
    fn outer_safety_limits_scale_with_encoder_resolution() {
        let mut safety = OuterSafetyLimits::new(0, 100, OuterMotionBudget::firmware_caps());

        safety.configure_encoder_resolution(8_000);

        assert_eq!(safety.counts_per_rev, 8_000);
        assert_eq!(safety.velocity_cap, 48);
        assert_eq!(safety.position_window, 16_000);
    }

    #[test]
    fn outer_safety_clamps_position_window_to_host_budget() {
        let mut safety = OuterSafetyLimits::new(
            0,
            100,
            OuterMotionBudget {
                max_travel_mrev: 500,
                max_velocity_mrev_s: 6000,
                max_duration_ms: 3000,
                direction_mask: 3,
            },
        );

        safety.configure_encoder_resolution(4_000);

        assert_eq!(safety.position_window, 2_000);
        assert_eq!(safety.velocity_cap, 24);
    }

    #[test]
    fn outer_safety_refuses_zero_budget() {
        let safety = OuterSafetyLimits::new(
            0,
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
            0,
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
                0,
                100,
                OuterMotionBudget {
                    max_travel_mrev: 0,
                    max_velocity_mrev_s: 6000,
                    max_duration_ms: 3000,
                    direction_mask: 3,
                },
            );
            let mut backend = MockBackend::new();

            let violated = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("encoder read should succeed");

            assert!(violated);
        });
    }

    #[test]
    fn outer_safety_violates_on_duration_budget() {
        pollster::block_on(async {
            let mut safety = OuterSafetyLimits::new(
                0,
                100,
                OuterMotionBudget {
                    max_travel_mrev: 500,
                    max_velocity_mrev_s: 6000,
                    max_duration_ms: 1,
                    direction_mask: 3,
                },
            );
            safety.configure_encoder_resolution(4_000);
            safety.start_sample_us = 1_000;
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 2_001;

            let violated = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("encoder read should succeed");

            assert!(violated);
        });
    }

    #[test]
    fn outer_safety_normalizes_velocity_by_elapsed_time() {
        pollster::block_on(async {
            let mut safety = OuterSafetyLimits::new(0, 100, OuterMotionBudget::firmware_caps());
            safety.counts_per_rev = 4000;
            safety.prev_encoder_count = 2328;
            safety.last_sample_us = 100_000;
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 124_909;
            backend.encoder_count_value = 2525; // delta=197 over 24.909ms => ~7 counts/ms

            let violated = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("encoder read should succeed");

            assert!(
                !violated,
                "time-normalized velocity should stay below the cap"
            );
            assert_eq!(safety.prev_encoder_count, 2525);
            assert_eq!(safety.last_sample_us, 124_909);
            assert_eq!(safety.unwrapped_position, 197);
        });
    }

    #[test]
    fn outer_safety_read_error_fails_with_spi() {
        pollster::block_on(async {
            let mut safety = OuterSafetyLimits::new(0, 2448, OuterMotionBudget::firmware_caps());
            safety.counts_per_rev = 4000;
            safety.last_sample_us = 1_000;
            let mut phase = OuterPhase::MechanicalId {
                state: MechanicalIdState::ForwardCapture {
                    next_sample_us: 2_000,
                    capture_start_us: 1_000,
                },
                data: MechanicalIdData::new(0, 367),
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
