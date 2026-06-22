//! Shared resistance-identification engine used by commissioning and diagnostics.

pub mod profile;
pub mod types;

pub use profile::{ResistanceProfile, STANDARD_UD_POINTS};
pub use types::{ElectricalAxis, ResistanceError, ResistancePoint, ResistanceWarningFlags};
