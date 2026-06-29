use super::types::CommissionProfile;

/// First-order-plus-dead-time (FOPDT) current plant model.
///
/// Estimated during electrical identification (Phase 5). Contains
/// both internal-unit values (used for gain computation) and physical
/// values (milliohms, microhenries — for host display and persistence).
///
/// Plant model: `G(s) = K / (tau_e * s + 1) * e^(-theta_e * s)`
/// where `K = 1 / R_int` (DC gain in internal units).
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct ElectricalModel {
    /// Winding resistance in scaled internal (ADC/DAC) units.
    /// `R_int = V_excitation / I_steady_state * ELECTRICAL_MODEL_RESISTANCE_SCALE`.
    pub r_int: u32,
    /// Winding inductance in internal micro-units.
    /// `L_int = r_int * tau_e_us / ELECTRICAL_MODEL_RESISTANCE_SCALE`.
    pub l_int: u32,
    /// Electrical time constant L/R in microseconds.
    pub tau_e_us: u32,
    /// Effective transport delay in microseconds (from transient onset).
    pub theta_e_us: u32,
    /// Physical resistance in milliohms (for display/persistence).
    pub r_mohm: u32,
    /// Physical inductance in microhenries (for display/persistence).
    pub l_uh: u32,
    /// Tau from two-point transient fit in microseconds.
    /// Compared against L/R tau for plant model validation.
    pub tau_e_crosscheck_us: u32,
}

/// Fixed-point scale used for [`ElectricalModel::r_int`].
pub const ELECTRICAL_MODEL_RESISTANCE_SCALE: u32 = 1000;

/// Computed PI gains for the TMC4671 current (flux/torque) loops.
///
/// Produced by [`simc_current_gains`] from an [`ElectricalModel`] and
/// a [`CommissionProfile`]. Includes the effective bandwidth and lambda
/// for diagnostics and downstream use (velocity loop bandwidth constraint).
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct CurrentGains {
    /// Flux P gain — TMC4671 Q8.8 register value.
    pub flux_p: u16,
    /// Flux I gain — raw Q8.8; effective advanced-PI zero is I/65536.
    pub flux_i: u16,
    /// Torque P gain — TMC4671 Q8.8 register value.
    pub torque_p: u16,
    /// Torque I gain — raw Q8.8; effective advanced-PI zero is I/65536.
    pub torque_i: u16,
    /// Effective closed-loop bandwidth in Hz.
    pub effective_bandwidth_hz: u32,
    /// Closed-loop time constant lambda in microseconds.
    pub lambda_us: u32,
}

/// Input to production current-loop gain synthesis.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct CurrentGainSynthesisInput {
    /// Identified symmetric resistance/inductance model.
    pub model: ElectricalModel,
    /// Tuning profile used for SIMC lambda selection.
    pub profile: CommissionProfile,
    /// Measured axis split ratio, permille where 1000 = 1.000x.
    pub measured_axis_split_permille: u16,
    /// Applied axis split ratio after profile clamping, permille.
    pub applied_axis_split_permille: u16,
    /// `AXIS_SPLIT_SOURCE_*`.
    pub axis_split_source: u8,
    /// `CURRENT_AXIS_*` axis with the higher inductance.
    pub high_inductance_axis: u8,
}

/// Output of production current-loop gain synthesis.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct CurrentGainSynthesis {
    /// Synthesized per-axis current-loop gains.
    pub gains: CurrentGains,
    /// Firmware-owned evidence for the selected synthesis tier and split policy.
    pub evidence: CurrentLoopEvidence,
    /// Flux-axis electrical time constant used for synthesis.
    pub flux_tau_us: u32,
    /// Torque-axis electrical time constant used for synthesis.
    pub torque_tau_us: u32,
}

// Stable persisted/report numeric codes. Do not renumber existing values;
// append new codes instead.

/// Gain synthesis failed.
pub const CURRENT_GAINS_SOURCE_FAILED: u8 = 0;
/// Gains were synthesized from measured commissioning data.
pub const CURRENT_GAINS_SOURCE_MEASURED: u8 = 1;
/// Default conservative gains were applied after validation fallback.
pub const CURRENT_GAINS_SOURCE_DEFAULT: u8 = 2;

/// No independent axis split source was used.
pub const AXIS_SPLIT_SOURCE_NONE: u8 = 0;
/// Axis split came from measured impedance asymmetry.
pub const AXIS_SPLIT_SOURCE_IMPEDANCE: u8 = 1;

/// No current-loop gain tier was produced.
pub const CURRENT_GAINS_TIER_NONE: u8 = 0;
/// Measured symmetric current-loop gains were produced.
pub const CURRENT_GAINS_TIER_MEASURED_SYMMETRIC: u8 = 1;
/// Measured split current-loop gains were produced.
pub const CURRENT_GAINS_TIER_MEASURED_SPLIT: u8 = 2;
/// Default conservative current-loop gains were applied.
pub const CURRENT_GAINS_TIER_DEFAULT: u8 = 3;

/// Flux-axis validation passed.
pub const CURRENT_VALIDATION_AXIS_FLUX: u8 = 1 << 0;
/// Torque-axis validation passed.
pub const CURRENT_VALIDATION_AXIS_TORQUE: u8 = 1 << 1;

/// Flux current-loop axis identifier.
pub const CURRENT_AXIS_FLUX: u8 = 0;
/// Torque current-loop axis identifier.
pub const CURRENT_AXIS_TORQUE: u8 = 1;

/// Validation sample count for the flux current-loop axis.
pub const CURRENT_FLUX_VALIDATION_SAMPLE_COUNT: usize = 4;
/// Validation sample count for the torque current-loop axis.
pub const CURRENT_TORQUE_VALIDATION_SAMPLE_COUNT: usize = 2;
/// Maximum current-loop validation sample count for any axis.
pub const CURRENT_VALIDATION_MAX_SAMPLES_PER_AXIS: usize = 4;

/// Current-loop synthesis and validation succeeded.
pub const CURRENT_LOOP_FAILURE_NONE: u8 = 0;
/// Resistance measurement data was invalid.
pub const CURRENT_LOOP_FAILURE_RESISTANCE_INVALID: u8 = 1;
/// Impedance measurement data was invalid.
pub const CURRENT_LOOP_FAILURE_IMPEDANCE_INVALID: u8 = 2;
/// Current-loop gain synthesis failed.
pub const CURRENT_LOOP_FAILURE_GAIN_SYNTHESIS: u8 = 3;
/// Flux-axis current-loop validation failed.
pub const CURRENT_LOOP_FAILURE_FLUX_VALIDATION: u8 = 4;
/// Torque-axis current-loop validation failed.
pub const CURRENT_LOOP_FAILURE_TORQUE_VALIDATION: u8 = 5;
/// Current-loop validation detected saturation.
pub const CURRENT_LOOP_FAILURE_SATURATION: u8 = 6;
/// Current-loop validation detected motion.
pub const CURRENT_LOOP_FAILURE_MOTION: u8 = 7;
/// Current-loop commissioning failed due to status flags.
pub const CURRENT_LOOP_FAILURE_STATUS_FLAGS: u8 = 8;
/// Current-loop validation retry budget was exhausted.
pub const CURRENT_LOOP_FAILURE_RETRY_EXHAUSTED: u8 = 9;
/// Current-loop commissioning failed due to SPI communication.
pub const CURRENT_LOOP_FAILURE_SPI: u8 = 10;

/// Firmware-owned evidence for current-loop gain source, split policy,
/// validation coverage, and failure reason.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct CurrentLoopEvidence {
    /// `CURRENT_GAINS_SOURCE_*`.
    pub gains_source: u8,
    /// Candidate `CURRENT_GAINS_SOURCE_*` before validation verdict.
    pub candidate_gains_source: u8,
    /// `AXIS_SPLIT_SOURCE_*`.
    pub axis_split_source: u8,
    /// Candidate `AXIS_SPLIT_SOURCE_*` before validation verdict.
    pub candidate_axis_split_source: u8,
    /// `CURRENT_GAINS_TIER_*`.
    pub gains_tier: u8,
    /// Candidate `CURRENT_GAINS_TIER_*` before validation verdict.
    pub candidate_gains_tier: u8,
    /// Measured axis split ratio, permille where 1000 = 1.000x.
    pub measured_axis_split_permille: u16,
    /// Candidate measured axis split ratio before validation verdict.
    pub candidate_measured_axis_split_permille: u16,
    /// Applied axis split ratio after profile clamp, permille.
    pub applied_axis_split_permille: u16,
    /// Candidate applied axis split ratio before validation verdict.
    pub candidate_applied_axis_split_permille: u16,
    /// Non-zero when `applied_axis_split_permille` is clamped below measured.
    pub axis_split_clamped: u8,
    /// Candidate axis split clamp flag before validation verdict.
    pub candidate_axis_split_clamped: u8,
    /// Bitmask of `CURRENT_VALIDATION_AXIS_*` axes that passed validation.
    pub current_validation_axes: u8,
    /// Non-zero when the section-level retry budget was consumed before all
    /// retryable axes passed.
    pub retry_budget_exhausted: u8,
    /// `CURRENT_LOOP_FAILURE_*`.
    pub failure_reason: u8,
}

impl CurrentLoopEvidence {
    /// Build a failed evidence record with symmetric ratio defaults.
    pub const fn failed(failure_reason: u8) -> Self {
        Self {
            gains_source: CURRENT_GAINS_SOURCE_FAILED,
            candidate_gains_source: CURRENT_GAINS_SOURCE_FAILED,
            axis_split_source: AXIS_SPLIT_SOURCE_NONE,
            candidate_axis_split_source: AXIS_SPLIT_SOURCE_NONE,
            gains_tier: CURRENT_GAINS_TIER_NONE,
            candidate_gains_tier: CURRENT_GAINS_TIER_NONE,
            measured_axis_split_permille: 1000,
            candidate_measured_axis_split_permille: 1000,
            applied_axis_split_permille: 1000,
            candidate_applied_axis_split_permille: 1000,
            axis_split_clamped: 0,
            candidate_axis_split_clamped: 0,
            current_validation_axes: 0,
            retry_budget_exhausted: 0,
            failure_reason,
        }
    }

    /// Build a measured symmetric evidence record.
    pub const fn measured_symmetric() -> Self {
        Self {
            gains_source: CURRENT_GAINS_SOURCE_MEASURED,
            candidate_gains_source: CURRENT_GAINS_SOURCE_MEASURED,
            axis_split_source: AXIS_SPLIT_SOURCE_NONE,
            candidate_axis_split_source: AXIS_SPLIT_SOURCE_NONE,
            gains_tier: CURRENT_GAINS_TIER_MEASURED_SYMMETRIC,
            candidate_gains_tier: CURRENT_GAINS_TIER_MEASURED_SYMMETRIC,
            measured_axis_split_permille: 1000,
            candidate_measured_axis_split_permille: 1000,
            applied_axis_split_permille: 1000,
            candidate_applied_axis_split_permille: 1000,
            axis_split_clamped: 0,
            candidate_axis_split_clamped: 0,
            current_validation_axes: 0,
            retry_budget_exhausted: 0,
            failure_reason: CURRENT_LOOP_FAILURE_NONE,
        }
    }

    /// Build a measured split evidence record.
    pub const fn measured_split(
        measured_axis_split_permille: u16,
        applied_axis_split_permille: u16,
        clamped: bool,
    ) -> Self {
        Self {
            gains_source: CURRENT_GAINS_SOURCE_MEASURED,
            candidate_gains_source: CURRENT_GAINS_SOURCE_MEASURED,
            axis_split_source: AXIS_SPLIT_SOURCE_IMPEDANCE,
            candidate_axis_split_source: AXIS_SPLIT_SOURCE_IMPEDANCE,
            gains_tier: CURRENT_GAINS_TIER_MEASURED_SPLIT,
            candidate_gains_tier: CURRENT_GAINS_TIER_MEASURED_SPLIT,
            measured_axis_split_permille,
            candidate_measured_axis_split_permille: measured_axis_split_permille,
            applied_axis_split_permille,
            candidate_applied_axis_split_permille: applied_axis_split_permille,
            axis_split_clamped: clamped as u8,
            candidate_axis_split_clamped: clamped as u8,
            current_validation_axes: 0,
            retry_budget_exhausted: 0,
            failure_reason: CURRENT_LOOP_FAILURE_NONE,
        }
    }

    /// Build evidence for default conservative gains applied after validation fallback.
    pub const fn default_gains(
        failure_reason: u8,
        retry_budget_exhausted: bool,
        current_validation_axes: u8,
    ) -> Self {
        Self {
            gains_source: CURRENT_GAINS_SOURCE_DEFAULT,
            candidate_gains_source: CURRENT_GAINS_SOURCE_DEFAULT,
            axis_split_source: AXIS_SPLIT_SOURCE_NONE,
            candidate_axis_split_source: AXIS_SPLIT_SOURCE_NONE,
            gains_tier: CURRENT_GAINS_TIER_DEFAULT,
            candidate_gains_tier: CURRENT_GAINS_TIER_DEFAULT,
            measured_axis_split_permille: 1000,
            candidate_measured_axis_split_permille: 1000,
            applied_axis_split_permille: 1000,
            candidate_applied_axis_split_permille: 1000,
            axis_split_clamped: 0,
            candidate_axis_split_clamped: 0,
            current_validation_axes,
            retry_budget_exhausted: retry_budget_exhausted as u8,
            failure_reason,
        }
    }
}

/// First-order mechanical plant model estimated during Phase 8.
///
/// Represents `G_mech(s) = 1 / (J_eff * s + B_eff)` where J_eff includes
/// carriage mass, belt compliance, and reflected rotor inertia. Used by
/// Phase 9 (velocity-loop SIMC synthesis) to compute velocity gains.
///
/// All values are in internal units derived from encoder counts and
/// torque DAC values. Physical conversion happens on the host side.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct MechanicalModel {
    /// Effective inertia in internal units.
    /// J_eff = torque / initial_acceleration (from first few samples).
    pub j_eff: u32,
    /// Effective damping in internal units.
    /// B_eff = torque / steady_state_velocity, or from deceleration curve.
    pub b_eff: u32,
    /// Mechanical time constant J_eff / B_eff in milliseconds.
    pub tau_mech_ms: u32,
    /// Effective mechanical delay in microseconds.
    /// Time from torque step to first measurable velocity change.
    pub theta_mech_us: u32,
    /// Static friction estimate in torque units. 0 if not measured.
    pub t_f: u16,
}

/// Computed PI gains for the TMC4671 velocity loop.
///
/// Produced by [`simc_velocity_gains`] from a [`MechanicalModel`],
/// inner-loop bandwidth, and tuning profile. The velocity limit is
/// derived from the peak velocity observed during mechanical ID and is
/// represented in encoder counts/ms during commissioning validation. The
/// validation phase converts it to TMC velocity-register units at the SPI
/// write boundary.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct VelocityGains {
    /// Velocity P gain — TMC4671 Q8.8 register value.
    pub velocity_p: u16,
    /// Velocity I gain — TMC4671 Q8.8 register value.
    pub velocity_i: u16,
    /// Commissioning velocity limit in encoder counts/ms.
    pub velocity_limit: u32,
    /// Effective closed-loop velocity bandwidth in Hz.
    pub effective_bandwidth_hz: u32,
    /// Closed-loop time constant lambda_vel in milliseconds.
    pub lambda_vel_ms: u32,
}

/// Maximum model-synthesized velocity P accepted by outer commissioning.
///
/// Keep this centralized so SIMC synthesis and the Phase 10/commit envelope
/// use the same bound.
pub(crate) const VELOCITY_SIMC_MAX_P_REG: u16 = 1152 * 4;

/// Computed position-loop gains from Phase 11 P ramp-up.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct PositionGains {
    /// Position P gain — TMC4671 Q8.8 register value.
    pub position_p: u16,
    /// Position I gain — TMC4671 Q8.8 register value (usually 0).
    pub position_i: u16,
}

/// Filter cutoff recommendations from Phase 12.
///
/// Each field is `None` when no filtering is needed, or `Some(cutoff_hz)`
/// when the corresponding validation stage detected noise/chatter.
/// The firmware uses these with [`crate::biquad::butterworth_lowpass`]
/// to compute register coefficients.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct FilterConfig {
    /// Velocity biquad low-pass cutoff Hz. Enabled if zero-speed chatter detected.
    pub velocity_filter_hz: Option<u32>,
    /// Position target low-pass cutoff Hz. Enabled if position ramp-up hit oscillation early.
    pub position_filter_hz: Option<u32>,
    /// Flux biquad low-pass cutoff Hz. Enabled if current ringing detected.
    pub flux_filter_hz: Option<u32>,
    /// Torque biquad low-pass cutoff Hz. Enabled if current ringing detected.
    pub torque_filter_hz: Option<u32>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn electrical_model_tau_from_l_over_r() {
        let model = ElectricalModel {
            r_int: 512,
            l_int: 256,
            tau_e_us: 50,
            theta_e_us: 10,
            r_mohm: 1125,
            l_uh: 450,
            tau_e_crosscheck_us: 48,
        };
        assert_eq!(model.r_int, 512);
        assert_eq!(model.tau_e_us, 50);
        assert_eq!(model.theta_e_us, 10);
    }
}
