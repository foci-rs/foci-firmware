//! Shared impedance-identification profile and public result types.

pub mod fit;
pub mod profile;
pub mod runner;
pub mod types;

#[cfg(test)]
mod tests;

pub use fit::{
    ImpedanceFitPoint, ImpedanceFitSummary, evaluate_impedance_result, fit_impedance_point,
};
pub use profile::{
    ImpedanceProfile, OPENFFBOARD_IMPEDANCE_STATUS_FAIL_MASK,
    OPENFFBOARD_IMPEDANCE_STATUS_WARN_MASK,
};
pub use runner::{
    ImpedanceBackend, ImpedanceBackendError, ImpedanceRunResult, run_impedance_identification,
};
pub use types::{
    IMPEDANCE_PROFILE_POINTS, IMPEDANCE_SAMPLE_COUNT, ImpedanceError, ImpedanceInjectionPoint,
    ImpedanceSample, ImpedanceWarningFlags,
};
