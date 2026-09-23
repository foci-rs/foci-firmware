//! Immutable fixed-I fixed-gain amplitude planning.

use super::integral_plan::{
    AcceptanceSource, INTEGRAL_OBSERVATIONS_PER_RUNG, INTEGRAL_SLOT_TRANSITION_ALLOWANCE_US,
    IntegralSlotOrder, VelocityIntegralAcquisitionKind, VelocityIntegralExecutionPlan,
    VelocityIntegralPlan,
};
use super::integral_reproduction::{AcceptancePoint, IntegralPlanIdentity};
use super::recovery::commanded_side_lower_rate_q;
use super::reproduction::BUILD_FEATURES;
use super::sweep_plan::VelocitySweepRequest;
use super::velocity_clamp::VelocityLimitEvidenceCapability;
use crate::autotune::velocity_measurement::VelocityDirection;

/// Current independent fixed-gain amplitude schema.
pub const FIXED_GAIN_AMPLITUDE_SCHEMA_REVISION: u16 = 6;
/// Number of frozen target amplitudes in each run.
pub const FIXED_GAIN_AMPLITUDE_TARGETS: usize = 5;
/// Capture-wrapper allowance for one amplitude command.
pub const FIXED_GAIN_AMPLITUDE_CAPTURE_ALLOWANCE_S: u32 = 110;

const TARGET_DIVISORS: [i32; FIXED_GAIN_AMPLITUDE_TARGETS] = [16, 8, 4, 2, 1];
const MICROS_PER_MILLISECOND: u64 = 1_000;
const OBSERVATIONS_PER_DIRECTION_PER_AMPLITUDE: u16 = 4;

/// Immutable amplitude order selected by the operator-facing acquisition action.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
#[repr(u8)]
pub enum FixedGainAmplitudeOrder {
    /// Lowest to highest frozen target.
    Ascending = 1,
    /// Highest to lowest frozen target.
    Descending = 2,
}

/// Pack the amplitude order and the slot order into one wire byte.
///
/// The amplitude order occupies the low nibble and the slot order the high one.
/// `ForwardFirst` encodes as zero, so every non-mirrored run keeps the byte it
/// had before slot order was recorded.
pub const fn schedule_order_code(
    order: FixedGainAmplitudeOrder,
    slot_order: IntegralSlotOrder,
) -> u8 {
    (order as u8) | (slot_order.report_code() << 4)
}

/// Recover the amplitude and slot orders from one packed wire byte.
pub const fn decode_schedule_order_code(
    code: u8,
) -> Option<(FixedGainAmplitudeOrder, IntegralSlotOrder)> {
    let order = match code & 0x0f {
        1 => FixedGainAmplitudeOrder::Ascending,
        2 => FixedGainAmplitudeOrder::Descending,
        _ => return None,
    };
    let slot_order = match code >> 4 {
        0 => IntegralSlotOrder::ForwardFirst,
        // 1 and 2 were `ReverseFirst`/`PairedOutAndBack`. Their only producers
        // (the live combined P-then-I workflow and the mirrored-amplitude
        // actions) were removed, and with them the variants themselves,
        // so a wire byte carrying either nibble can no
        // longer name a live order. Reject it as decode failure rather than
        // reintroducing removed variants.
        _ => return None,
    };
    Some((order, slot_order))
}

/// Failure to construct one exact fixed-I fixed-gain amplitude plan.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum FixedGainAmplitudePlanError {
    /// The retained point and reconstructed combined plan do not bind exactly.
    IdentityMismatch,
    /// Rounding produced a zero or duplicate target.
    UnsupportedTargets,
    /// Checked arithmetic exceeded the retained representation.
    Arithmetic,
}

/// Complete immutable fixed-gain-amplitude plan emitted before motion.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct FixedGainAmplitudePlan {
    /// Independent amplitude evidence schema.
    pub schema_revision: u16,
    /// Combined Integral identity that established the operating point.
    pub acceptance_identity: IntegralPlanIdentity,
    /// Immutable acquisition order.
    pub order: FixedGainAmplitudeOrder,
    /// Immutable execution order of the forward and reverse observation slots.
    pub slot_order: IntegralSlotOrder,
    /// Exact order-dependent mechanical-RPM targets.
    pub target_rpm: [i16; FIXED_GAIN_AMPLITUDE_TARGETS],
    /// Firmware-selected proportional gain.
    pub selected_p: u16,
    /// Firmware-selected raw Q4.12 integral gain.
    pub selected_i: u16,
    /// Packed inherited validity and evidence-quality qualifiers.
    pub qualifier_bits: u16,
    /// Frozen PWM frequency.
    pub pwm_hz: u32,
    /// Frozen mechanical encoder quadrature counts per revolution.
    pub encoder_counts_per_rev: u32,
    /// Number of amplitude rungs.
    pub amplitude_count: u8,
    /// Total forward and reverse moving observations.
    pub expected_observations: u16,
    /// Per-direction simultaneous confidence family.
    pub family_size: u16,
    /// Fixed moving-capture horizon.
    pub moving_stroke_us: u32,
    /// Fixed I=0 rest horizon.
    pub zero_settle_us: u32,
    /// Cooperative analysis allowance.
    pub analysis_budget_us: u32,
    /// Exact nominal fixed-schedule duration.
    pub nominal_workflow_ms: u32,
    /// Exact maximum fixed-schedule duration.
    pub maximum_workflow_ms: u32,
    /// Inclusive immutable-origin recovery band.
    pub recovery_origin_band_counts: u32,
    /// Conservative top-point settled-rate bounds used only for recovery deadlines.
    pub recovery_lower_rate_q: [u64; 2],
    /// Nominal allowance for one recovery slot.
    pub recovery_nominal_slot_us: u32,
    /// Maximum allowance for one recovery slot.
    pub recovery_maximum_slot_us: u32,
    /// One recovery slot per amplitude.
    pub recovery_slot_count: u8,
    /// Configured hard torque-target limit.
    pub hard_torque_limit: u16,
    /// Inclusive usable torque-target boundary.
    pub usable_torque_limit: u16,
    /// Current-contact observability capability code.
    pub limit_evidence_capability: u8,
    /// One opposite-sign meter quantum can contact the clamp during recovery.
    pub recovery_quantization_exposed: bool,
    /// The commissioning probe clamp narrowed the accepted operating component.
    pub probe_constrained_test_point: bool,
    /// Complete request retained for safety and identity.
    pub motion_request: VelocitySweepRequest,
}

impl FixedGainAmplitudePlan {
    /// Hash every exact acceptance, geometry, duration, and safety field.
    pub fn digest(self) -> u64 {
        let mut hash = 0xcbf2_9ce4_8422_2325_u64;
        macro_rules! hash_value {
            ($value:expr) => {
                for byte in $value.to_le_bytes() {
                    hash = fnv1a64_byte(hash, byte);
                }
            };
        }
        hash_value!(self.schema_revision);
        hash_value!(self.acceptance_identity.channel);
        hash_value!(self.acceptance_identity.build_features);
        hash_value!(self.acceptance_identity.schema_revision);
        hash_value!(self.acceptance_identity.plan_digest);
        hash_value!(schedule_order_code(self.order, self.slot_order));
        for target in self.target_rpm {
            hash_value!(target);
        }
        hash_value!(self.selected_p);
        hash_value!(self.selected_i);
        hash_value!(self.qualifier_bits);
        hash_value!(self.pwm_hz);
        hash_value!(self.encoder_counts_per_rev);
        hash_value!(self.amplitude_count);
        hash_value!(self.expected_observations);
        hash_value!(self.family_size);
        hash_value!(self.moving_stroke_us);
        hash_value!(self.zero_settle_us);
        hash_value!(self.analysis_budget_us);
        hash_value!(INTEGRAL_SLOT_TRANSITION_ALLOWANCE_US);
        if matches!(
            self.schema_revision,
            3 | FIXED_GAIN_AMPLITUDE_SCHEMA_REVISION
        ) {
            hash_value!(super::recovery::STABILIZED_RECOVERY_SCHEDULE_CODE);
        }
        hash_value!(self.nominal_workflow_ms);
        hash_value!(self.maximum_workflow_ms);
        hash_value!(self.recovery_origin_band_counts);
        for lower_rate_q in self.recovery_lower_rate_q {
            hash_value!(lower_rate_q);
        }
        hash_value!(self.recovery_nominal_slot_us);
        hash_value!(self.recovery_maximum_slot_us);
        hash_value!(self.recovery_slot_count);
        hash_value!(self.hard_torque_limit);
        hash_value!(self.usable_torque_limit);
        hash_value!(self.limit_evidence_capability);
        hash_value!(u8::from(self.recovery_quantization_exposed));
        hash_value!(u8::from(self.probe_constrained_test_point));
        hash_value!(self.motion_request.requested_velocity_mrev_s);
        hash_value!(self.motion_request.machine_velocity_ceiling_mrev_s);
        hash_value!(self.motion_request.requested_velocity_source as u8);
        hash_value!(self.motion_request.max_stroke_travel_mrev);
        hash_value!(self.motion_request.settle_travel_reserve_mrev);
        hash_value!(self.motion_request.negative_position_headroom_mrev);
        hash_value!(self.motion_request.positive_position_headroom_mrev);
        hash_value!(self.motion_request.max_duration_ms);
        hash
    }

    /// Compact the emitted amplitude plan into existing velocity-integral execution state.
    pub fn execution_plan(self) -> VelocityIntegralExecutionPlan {
        let top_target = self.target_rpm.iter().copied().max().unwrap_or(0);
        VelocityIntegralExecutionPlan {
            slot_order: self.slot_order,
            plan_digest: self.digest(),
            acceptance_plan_digest: self.acceptance_identity.plan_digest,
            qualifier_bits: self.qualifier_bits,
            fixed_p: self.selected_p,
            planned_velocity_mrev_s: self.motion_request.requested_velocity_mrev_s,
            target_velocity_rpm: i32::from(top_target),
            pwm_hz: self.pwm_hz,
            encoder_counts_per_rev: self.encoder_counts_per_rev,
            i_start: self.selected_i,
            positive_rung_count: self.amplitude_count,
            // Not applicable to a fixed-I fixed-gain-amplitude acquisition;
            // only a response-ladder plan regenerates a half-octave rung
            // from this field.
            i_top_cap: 0,
            family_size: self.family_size,
            moving_stroke_us: self.moving_stroke_us,
            zero_settle_us: self.zero_settle_us,
            analysis_budget_us: self.analysis_budget_us,
            maximum_workflow_ms: self.maximum_workflow_ms,
            recovery_origin_band_counts: self.recovery_origin_band_counts,
            recovery_nominal_slot_us: self.recovery_nominal_slot_us,
            recovery_maximum_slot_us: self.recovery_maximum_slot_us,
            recovery_slot_count: self.recovery_slot_count,
            hard_torque_limit: self.hard_torque_limit,
            usable_torque_limit: self.usable_torque_limit,
            limit_evidence_capability: match self.limit_evidence_capability {
                value if value == VelocityLimitEvidenceCapability::BestEffortPolling as u8 => {
                    VelocityLimitEvidenceCapability::BestEffortPolling
                }
                _ => VelocityLimitEvidenceCapability::LatchedFlag,
            },
            acquisition_kind: match self.order {
                FixedGainAmplitudeOrder::Ascending => {
                    VelocityIntegralAcquisitionKind::FixedGainAmplitudeAscending
                }
                FixedGainAmplitudeOrder::Descending => {
                    VelocityIntegralAcquisitionKind::FixedGainAmplitudeDescending
                }
            },
            acceptance_source: AcceptanceSource::NonBreakaway,
            amplitude_targets_rpm: self.target_rpm,
            // Amplitude acquisitions carry no real Proportional response intervals;
            // the retained identity and qualifiers now have their own
            // fields below.
            selected_response_intervals_q: [[0; 2]; 2],
            recovery_lower_rate_q: self.recovery_lower_rate_q,
            recovery_quantization_exposed: self.recovery_quantization_exposed,
            probe_constrained_test_point: self.probe_constrained_test_point,
        }
    }
}

/// Derive five positive, unique targets with round-to-nearest halves upward.
pub fn derive_fixed_gain_amplitude_targets(
    top_target_rpm: i32,
) -> Result<[i16; FIXED_GAIN_AMPLITUDE_TARGETS], FixedGainAmplitudePlanError> {
    if top_target_rpm <= 0 || top_target_rpm > i32::from(i16::MAX) {
        return Err(FixedGainAmplitudePlanError::Arithmetic);
    }
    let mut targets = [0_i16; FIXED_GAIN_AMPLITUDE_TARGETS];
    for (index, divisor) in TARGET_DIVISORS.into_iter().enumerate() {
        let rounded = top_target_rpm
            .checked_add(divisor / 2)
            .ok_or(FixedGainAmplitudePlanError::Arithmetic)?
            / divisor;
        targets[index] =
            i16::try_from(rounded).map_err(|_| FixedGainAmplitudePlanError::Arithmetic)?;
        if targets[index] <= 0 || (index != 0 && targets[index] <= targets[index - 1]) {
            return Err(FixedGainAmplitudePlanError::UnsupportedTargets);
        }
    }
    Ok(targets)
}

/// Build one immutable amplitude plan entirely from retained firmware authority.
pub fn plan_fixed_gain_amplitude(
    point: AcceptancePoint,
    request: VelocitySweepRequest,
    order: FixedGainAmplitudeOrder,
    slot_order: IntegralSlotOrder,
    current_config: VelocityIntegralPlan,
) -> Result<FixedGainAmplitudePlan, FixedGainAmplitudePlanError> {
    // Every retained authority family (combined and breakaway alike) must carry
    // amplitude acceptance authority: well-ordered, correctly signed response
    // intervals per direction, not merely a nonempty pair. This is the same
    // `commanded_side_lower_rate_q` call the recovery geometry below performs,
    // so the gate and the computation can never disagree about what "finite"
    // means here.
    let recovery_authority_consistent = commanded_side_lower_rate_q(
        VelocityDirection::Forward,
        i64::from(point.selected_response_intervals_q[0][0]),
        i64::from(point.selected_response_intervals_q[0][1]),
    )
    .is_some()
        && commanded_side_lower_rate_q(
            VelocityDirection::Reverse,
            i64::from(point.selected_response_intervals_q[1][0]),
            i64::from(point.selected_response_intervals_q[1][1]),
        )
        .is_some();
    if point.identity.build_features != BUILD_FEATURES
        || point.identity.schema_revision != current_config.schema_revision
        || point.identity.plan_digest != current_config.digest()
        || point.selected_p != current_config.fixed_p
        || point.target_velocity_rpm != current_config.target_velocity_rpm
        || request != current_config.motion_request
        || !recovery_authority_consistent
        || !current_config.positive_rungs().contains(&point.selected_i)
    {
        return Err(FixedGainAmplitudePlanError::IdentityMismatch);
    }

    let mut target_rpm = derive_fixed_gain_amplitude_targets(point.target_velocity_rpm)?;
    if order == FixedGainAmplitudeOrder::Descending {
        target_rpm.reverse();
    }
    let amplitude_count = u8::try_from(FIXED_GAIN_AMPLITUDE_TARGETS)
        .map_err(|_| FixedGainAmplitudePlanError::Arithmetic)?;
    let expected_observations = u16::from(amplitude_count)
        .checked_mul(
            u16::try_from(INTEGRAL_OBSERVATIONS_PER_RUNG)
                .map_err(|_| FixedGainAmplitudePlanError::Arithmetic)?,
        )
        .ok_or(FixedGainAmplitudePlanError::Arithmetic)?;
    let family_size = u16::from(amplitude_count)
        .checked_mul(OBSERVATIONS_PER_DIRECTION_PER_AMPLITUDE)
        .ok_or(FixedGainAmplitudePlanError::Arithmetic)?;
    let recovery_lower_rate_q = [
        commanded_side_lower_rate_q(
            VelocityDirection::Forward,
            i64::from(point.selected_response_intervals_q[0][0]),
            i64::from(point.selected_response_intervals_q[0][1]),
        )
        .ok_or(FixedGainAmplitudePlanError::Arithmetic)?,
        commanded_side_lower_rate_q(
            VelocityDirection::Reverse,
            i64::from(point.selected_response_intervals_q[1][0]),
            i64::from(point.selected_response_intervals_q[1][1]),
        )
        .ok_or(FixedGainAmplitudePlanError::Arithmetic)?,
    ];
    let primitive_us = u64::from(current_config.moving_stroke_us)
        .checked_add(u64::from(current_config.zero_settle_us))
        .and_then(|value| value.checked_add(u64::from(current_config.analysis_budget_us)))
        .ok_or(FixedGainAmplitudePlanError::Arithmetic)?;
    let observation_us = u64::from(expected_observations)
        .checked_mul(primitive_us)
        .ok_or(FixedGainAmplitudePlanError::Arithmetic)?;
    let transition_us = u64::from(expected_observations)
        .checked_mul(u64::from(INTEGRAL_SLOT_TRANSITION_ALLOWANCE_US))
        .ok_or(FixedGainAmplitudePlanError::Arithmetic)?;
    let cleanup_us = u64::from(current_config.zero_settle_us)
        .checked_add(u64::from(current_config.analysis_budget_us))
        .ok_or(FixedGainAmplitudePlanError::Arithmetic)?;
    let duration_ms = |recovery_slot_us: u32| {
        observation_us
            .checked_add(
                u64::from(amplitude_count)
                    .checked_mul(u64::from(recovery_slot_us))
                    .ok_or(FixedGainAmplitudePlanError::Arithmetic)?,
            )
            .and_then(|value| value.checked_add(transition_us))
            .and_then(|value| value.checked_add(cleanup_us))
            .and_then(|value| value.checked_add(MICROS_PER_MILLISECOND - 1))
            .and_then(|value| u32::try_from(value / MICROS_PER_MILLISECOND).ok())
            .ok_or(FixedGainAmplitudePlanError::Arithmetic)
    };

    Ok(FixedGainAmplitudePlan {
        schema_revision: FIXED_GAIN_AMPLITUDE_SCHEMA_REVISION,
        acceptance_identity: point.identity,
        order,
        slot_order,
        target_rpm,
        selected_p: point.selected_p,
        selected_i: point.selected_i,
        qualifier_bits: point.qualifier_bits,
        pwm_hz: current_config.pwm_hz,
        encoder_counts_per_rev: current_config.encoder_counts_per_rev,
        amplitude_count,
        expected_observations,
        family_size,
        moving_stroke_us: current_config.moving_stroke_us,
        zero_settle_us: current_config.zero_settle_us,
        analysis_budget_us: current_config.analysis_budget_us,
        nominal_workflow_ms: duration_ms(current_config.recovery_nominal_slot_us)?,
        maximum_workflow_ms: duration_ms(current_config.recovery_maximum_slot_us)?,
        recovery_origin_band_counts: current_config.recovery_origin_band_counts,
        recovery_lower_rate_q,
        recovery_nominal_slot_us: current_config.recovery_nominal_slot_us,
        recovery_maximum_slot_us: current_config.recovery_maximum_slot_us,
        recovery_slot_count: amplitude_count,
        hard_torque_limit: current_config.hard_torque_limit,
        usable_torque_limit: current_config.usable_torque_limit,
        limit_evidence_capability: current_config.limit_evidence_capability as u8,
        recovery_quantization_exposed: current_config.recovery_quantization_exposed,
        probe_constrained_test_point: current_config.probe_constrained_test_point,
        motion_request: request,
    })
}

const fn fnv1a64_byte(hash: u64, byte: u8) -> u64 {
    (hash ^ byte as u64).wrapping_mul(0x0000_0100_0000_01b3)
}
