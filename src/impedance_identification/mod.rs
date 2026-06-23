//! Shared impedance-identification profile and public result types.

pub mod profile;
pub mod types;

#[cfg(test)]
mod tests;

pub use profile::{
    ImpedanceProfile, OPENFFBOARD_IMPEDANCE_STATUS_FAIL_MASK,
    OPENFFBOARD_IMPEDANCE_STATUS_WARN_MASK,
};
pub use types::{
    ImpedanceError, ImpedanceInjectionPoint, ImpedanceSample, ImpedanceWarningFlags,
    IMPEDANCE_PROFILE_POINTS, IMPEDANCE_SAMPLE_COUNT,
};
