//! Velocity-loop gain synthesis and validation helpers.
//!
//! The model-relative SIMC synthesis path and its step-response validation
//! metrics have been removed: the live breakaway/Stage-C/robustness
//! commissioning path selects velocity gains directly from friction-aware
//! evidence (see `crate::commissioning::outer::velocity`) rather than from a
//! mechanical plant model. [`super::VelocityGains`] remains the shared gain
//! output type.
