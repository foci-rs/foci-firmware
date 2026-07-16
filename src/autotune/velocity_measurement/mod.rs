//! Fixed-memory encoder velocity measurement and confidence primitives.

mod blocker;
mod decision;
mod observation;

pub use blocker::{BlockLevel, DyadicBlocker, PlateauSelection, VELOCITY_MIN_SELECTED_BLOCKS};
pub use decision::{
    AccuracyDecision, ConfidenceEstimate, TwoIntervalRateWindow, classify_absolute_rate,
    estimate_confidence,
};
pub use observation::{
    BlockMoments, RateBlock, RawVelocityObservation, VELOCITY_RATE_Q_SCALE,
    VelocityMeasurementError,
};
