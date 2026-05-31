//! Public data types for commissioning.
//!
//! These types form the cross-crate API surface of the commissioning
//! engine. They are referenced by board binaries for engine construction
//! and result dispatch.

use crate::autotune::{CommissionError, CommissionStatus, PhaseId};

/// Compatibility re-export for existing `commissioning::types` backend users.
pub use crate::commissioning::backend::CommissioningBackend;

/// Discriminant for the four ways a `CommissioningEngine` can be constructed.
/// Used by the dispatch to choose the right reply payload at terminal arms.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum EngineKind {
    /// Engine constructed via `new_commission` — Stage 1 commissioning.
    Commission,
    /// Engine constructed via `new_calibrate` — ADC calibration + alignment.
    Calibrate,
    /// Engine constructed via `new_selftest` — diagnostic selftest run.
    Selftest,
    /// Engine constructed via `new_outer` — Stage 2 mechanical/velocity tune.
    OuterTune,
}

/// Result of a `step()` call.
#[derive(Debug, defmt::Format)]
pub enum StepResult {
    /// Engine is still working. Call `step()` again next tick.
    Continue,
    /// A phase transition occurred. Report to host.
    PhaseChanged(PhaseId),
    /// Inner commissioning failed.
    InnerFailed(CommissionError),
    /// Outer commissioning completed successfully.
    OuterDone(OuterResult),
    /// Outer commissioning failed.
    OuterFailed(CommissionError),
    /// ADC calibration completed. Hardware remains active for encoder
    /// alignment.
    CalibrateDone(AdcCalResult),
    /// Selftest phases completed. Hardware restored to safe state.
    SelftestDone,
    /// Stage 1 commissioning complete. Motor is in closed-loop position mode.
    /// Carries inner result + fallback outer gains for the Python host to
    /// cache and persist.
    CommissionDone(CommissionResult),
}

/// Result of inner (electrical) commissioning.
#[derive(Debug, Clone, defmt::Format)]
pub struct InnerResult {
    /// Overall status.
    pub status: CommissionStatus,
    /// Resistance in scaled internal ADC/DAC units.
    pub r_int: u32,
    /// Inductance in internal micro-units.
    pub l_int: u32,
    /// Resistance in milliohms (for host display/persistence).
    pub r_mohm: u32,
    /// Inductance in microhenries (for host display/persistence).
    pub l_uh: u32,
    /// Computed flux P gain (Q8.8).
    pub flux_p: u16,
    /// Computed flux I gain (raw Q8.8; effective advanced-PI zero is I/65536).
    pub flux_i: u16,
    /// Computed torque P gain (Q8.8).
    pub torque_p: u16,
    /// Computed torque I gain (raw Q8.8; effective advanced-PI zero is I/65536).
    pub torque_i: u16,
    /// Inner-loop closed-loop lambda in microseconds.
    pub lambda_us: u32,
    /// Electrical transport delay in microseconds.
    pub theta_e_us: u32,
    /// Max ringing count from forward/reverse current validation.
    pub ringing_count: u8,
    /// Effective current-loop bandwidth in Hz.
    pub bandwidth_hz: u32,
    /// Electrical time constant L/R in microseconds (Phase 5 measurement).
    pub tau_e_us: u32,
    /// Tau crosscheck from the transient onset (microseconds).
    pub tau_e_crosscheck_us: u32,
    /// L/R-vs-transient tau residual in permille.
    pub tau_residual_permille: u16,
    /// Warning bitfield (bit 0..3 model gates, bit 4 retry accepted,
    /// bit 5 fallback gains, bit 6 host-default confidence, bit 7 reserved).
    pub inner_warning_flags: u8,
}

/// Result of outer (mechanical) commissioning.
#[derive(Debug, Clone, defmt::Format)]
pub struct OuterResult {
    /// Overall status.
    pub status: CommissionStatus,
    /// Velocity P gain (Q8.8).
    pub velocity_p: u16,
    /// Velocity I gain (Q8.8).
    pub velocity_i: u16,
    /// Velocity limit.
    pub velocity_limit: u32,
    /// Position P gain (Q8.8).
    pub position_p: u16,
    /// Position I gain (Q8.8).
    pub position_i: u16,
    /// Velocity filter cutoff Hz (0 = disabled).
    pub velocity_filter_hz: u16,
    /// Position filter cutoff Hz (0 = disabled).
    pub position_filter_hz: u16,
    /// Flux filter cutoff Hz (0 = disabled).
    pub flux_filter_hz: u16,
    /// Torque filter cutoff Hz (0 = disabled).
    pub torque_filter_hz: u16,
    /// Identified effective inertia from mechanical ID (internal units).
    pub j_eff: u32,
    /// Identified effective damping from mechanical ID (internal units).
    pub b_eff: u32,
    /// Warning reason for accepted-with-warning results (0 = none).
    pub warning_code: u8,
    /// Torque step used by the final mechanical-ID attempt.
    pub mech_torque_step: u16,
    /// Forward initial acceleration, absolute encoder counts/ms^2.
    pub mech_fwd_accel: u32,
    /// Reverse initial acceleration, absolute encoder counts/ms^2.
    pub mech_rev_accel: u32,
    /// Forward peak velocity, absolute encoder counts/ms.
    pub mech_fwd_peak_velocity: u32,
    /// Reverse peak velocity, absolute encoder counts/ms.
    pub mech_rev_peak_velocity: u32,
    /// Forward total travel during mechanical ID, encoder counts.
    pub mech_fwd_travel: u32,
    /// Reverse total travel during mechanical ID, encoder counts.
    pub mech_rev_travel: u32,
    /// Velocity P before safety fallback, if synthesis reached that far.
    pub synth_velocity_p: u16,
    /// Velocity I before safety fallback, if synthesis reached that far.
    pub synth_velocity_i: u16,
    /// Position P before safety fallback, if position tuning reached that far.
    pub synth_position_p: u16,
    /// Position I before safety fallback, if position tuning reached that far.
    pub synth_position_i: u16,
}

/// Result of the calibration subset requested by `new_calibrate`.
#[derive(Debug, Clone, defmt::Format)]
pub struct AdcCalResult {
    /// ADC I0 offset.
    pub adc_i0: u16,
    /// ADC I1 offset.
    pub adc_i1: u16,
    /// Encoder count captured before the alignment phase zeroed it.
    pub encoder_count: u32,
}

/// Result of Stage 1 commissioning.
///
/// Combines the inner result (current-loop gains, electrical model) with
/// the fallback outer gains applied during closed-loop entry. Matches the
/// wire result `foci_commission_result`.
#[derive(Debug, Clone, defmt::Format)]
pub struct CommissionResult {
    /// Inner commissioning result (current-loop gains, electrical model).
    pub inner: InnerResult,
    /// Fallback velocity P gain (Q8.8) written during closed-loop entry.
    pub fallback_velocity_p: u16,
    /// Fallback velocity I gain (Q8.8) written during closed-loop entry.
    pub fallback_velocity_i: u16,
    /// Fallback position P gain (Q8.8) written during closed-loop entry.
    pub fallback_position_p: u16,
    /// Fallback position I gain (Q8.8) written during closed-loop entry.
    pub fallback_position_i: u16,
    /// Fallback PID velocity limit register value.
    pub fallback_velocity_limit: u32,
}
