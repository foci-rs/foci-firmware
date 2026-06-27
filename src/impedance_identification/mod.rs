//! Shared impedance-identification profile and public result types.

pub mod fit;
pub mod profile;
pub mod runner;
pub mod types;

#[cfg(test)]
mod tests;

pub use fit::fit_impedance;
pub use profile::{IMPEDANCE_STATUS_FAIL_MASK, IMPEDANCE_STATUS_WARN_MASK, ImpedanceProfile};
pub use runner::{
    ImpedanceBackend, ImpedanceBackendError, ImpedanceRunResult, run_impedance_identification,
};
pub use types::{
    IMPEDANCE_POST_SWEEP_ZERO_INDEX, IMPEDANCE_PROFILE_POINTS, IMPEDANCE_RETURN_ZERO_SETTLE_INDEX,
    IMPEDANCE_SAMPLE_COUNT, IMPEDANCE_STABILITY_ZERO_INDEX, ImpedanceAngleObservation,
    ImpedanceBaselineObservation, ImpedanceError, ImpedanceFilterState, ImpedanceFitSummary,
    ImpedancePoint, ImpedancePowerStageReadback, ImpedanceReadbackMismatch, ImpedanceSample,
    ImpedanceWarningFlags,
};
