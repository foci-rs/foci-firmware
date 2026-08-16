//! Cause codes for the reversal-standstill robustness gate.
//!
//! This module currently holds only the `ROBUSTNESS_CAUSE_*` namespace,
//! consumed by `robustness_evidence::summary_cause`. A later change extends it with
//! the preflight/terminal-application dispatch logic.

/// No direction tripped a threshold gate and no direction was inconclusive.
pub const ROBUSTNESS_CAUSE_PASS: u8 = 0;
/// At least one direction's `reconvergence_time_us` exceeded its threshold.
pub const ROBUSTNESS_CAUSE_RECONVERGENCE_TIME_EXCEEDED: u8 = 1;
/// At least one direction's `settled_residual_q` exceeded its threshold.
pub const ROBUSTNESS_CAUSE_RESIDUAL_EXCEEDED: u8 = 2;
/// At least one direction's `recovery_iae_q` exceeded its threshold.
pub const ROBUSTNESS_CAUSE_IAE_EXCEEDED: u8 = 3;
/// At least one direction never reached a sustained band-entry dwell within
/// its measurement budget.
pub const ROBUSTNESS_CAUSE_RECONVERGENCE_INCONCLUSIVE: u8 = 4;
/// No candidate survived acceptance to be probed for robustness.
pub const ROBUSTNESS_CAUSE_NO_ACCEPTED_CANDIDATE: u8 = 5;
/// Envelope/velocity-limit safety fault passthrough only.
pub const ROBUSTNESS_CAUSE_SAFETY_FAULT: u8 = 6;
/// Accepted-point identity mismatch, set at terminal application (Task 6).
/// Never returned by `summary_cause`.
pub const ROBUSTNESS_CAUSE_EVIDENCE_INTEGRITY: u8 = 7;
