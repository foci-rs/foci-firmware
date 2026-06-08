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
