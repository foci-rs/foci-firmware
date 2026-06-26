//! Shared impedance-identification profile and public result types.

pub mod fit;
pub mod profile;
mod pulsating_fit;
mod pulsating_profile;
pub mod runner;
pub mod types;

#[cfg(test)]
mod tests;

pub use fit::{
    ImpedanceFitPoint, ImpedanceFitSummary, evaluate_impedance_result, fit_impedance_point,
    fit_impedance_point_with_transformed_current,
};
pub use profile::{
    ImpedanceProfile, OPENFFBOARD_IMPEDANCE_STATUS_FAIL_MASK,
    OPENFFBOARD_IMPEDANCE_STATUS_WARN_MASK,
};
pub use pulsating_fit::fit_pulsating_impedance;
pub use pulsating_profile::PulsatingImpedanceProfile;
pub use runner::{
    ImpedanceBackend, ImpedanceBackendError, ImpedanceRunResult, run_impedance_identification,
};
pub use types::{
    IMPEDANCE_PROFILE_POINTS, IMPEDANCE_SAMPLE_COUNT, ImpedanceBaselineObservation, ImpedanceError,
    ImpedanceInjectionPoint, ImpedancePointObservation, ImpedancePowerStageReadback,
    ImpedanceReadbackMismatch, ImpedanceSample, ImpedanceWarningFlags,
    PULSATING_IMPEDANCE_PROFILE_POINTS, PULSATING_IMPEDANCE_SAMPLE_COUNT,
    PulsatingAngleObservation, PulsatingBaselineObservation, PulsatingFilterState,
    PulsatingFitSummary, PulsatingImpedancePoint,
};
