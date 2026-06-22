//! Shared resistance-identification engine used by commissioning and diagnostics.

pub mod fit;
pub mod profile;
pub mod types;

pub use fit::{evaluate_resistance_result, fit_resistance_axis};
pub use profile::{ResistanceProfile, STANDARD_UD_POINTS};
pub use types::{
    ElectricalAxis, ResistanceAxisResult, ResistanceError, ResistancePoint, ResistanceRunResult,
    ResistanceWarningFlags, SignedAnchorEvidence,
};
