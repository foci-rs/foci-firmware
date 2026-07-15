//! Shared resistance-identification engine used by commissioning and diagnostics.

pub mod fit;
pub mod profile;
pub mod runner;
mod state;
pub mod types;

#[cfg(test)]
mod tests;

pub use fit::{evaluate_resistance_result, fit_resistance_axis};
pub use profile::{ResistanceProfile, STANDARD_UD_POINTS};
pub use runner::{
    ResistanceBackend, ResistanceBackendError, ResistancePoll, ResistanceRunMode, ResistanceRunner,
    ResistanceTimingEvidence, run_resistance_identification,
};
pub use types::{
    ElectricalAxis, ResistanceAxisResult, ResistanceCurrentEvidence, ResistanceError,
    ResistancePoint, ResistanceRunResult, ResistanceWarningFlags, SignedAnchorEvidence,
};
