//! Fixed-memory encoder velocity measurement and confidence primitives.

mod blocker;
mod decision;
mod observation;

pub use blocker::{
    BlockLevel, DyadicBlocker, PlateauSelection, VELOCITY_MIN_SELECTED_BLOCKS, VelocityDirection,
    VelocityDirectionEvidence,
};
pub use decision::{
    AccuracyDecision, ConfidenceEstimate, TwoIntervalRateWindow, VelocityMeasurementOutcome,
    classify_absolute_rate, classify_plateau, estimate_confidence,
};
pub use observation::{
    BlockMoments, RateBlock, RawVelocityObservation, VELOCITY_RATE_Q_SCALE,
    VelocityMeasurementError,
};
