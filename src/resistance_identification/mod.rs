//! Shared resistance-identification engine used by commissioning and diagnostics.

pub mod bus_voltage;
pub mod fit;
pub mod profile;
pub mod runner;
mod state;
pub mod types;

#[cfg(test)]
mod tests;

pub use bus_voltage::{
    BUS_MILLIVOLTS_MAX, BUS_MILLIVOLTS_MIN, BusVoltageSense, bus_millivolts,
    bus_millivolts_in_range, counts_per_volt_milli, scale_profile_to_bus,
};
pub use fit::{evaluate_resistance_result, fit_resistance_axis};
pub use profile::{
    REFERENCE_BUS_MILLIVOLTS, RESISTANCE_DIAGNOSTIC_CURRENT_FRACTION, ResistanceProfile,
    STANDARD_UD_POINTS,
};
pub use runner::{
    ResistanceBackend, ResistanceBackendError, ResistancePoll, ResistanceRunMode,
    ResistanceRunTerminal, ResistanceRunner, ResistanceTimingEvidence,
    run_resistance_identification,
};
pub use types::{
    ElectricalAxis, ResistanceAxisResult, ResistanceCurrentEvidence, ResistanceError,
    ResistancePoint, ResistanceRunResult, ResistanceWarningFlags, SignedAnchorEvidence,
};
