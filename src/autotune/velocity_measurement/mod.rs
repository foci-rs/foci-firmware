//! Fixed-memory encoder velocity measurement and confidence primitives.

mod blocker;
mod decision;
mod observation;
mod stationarity;

pub use blocker::{
    BlockLevel, DyadicBlocker, PlateauSelection, VELOCITY_MIN_SELECTED_BLOCKS, VelocityDirection,
    VelocityDirectionEvidence,
};
pub use decision::{
    AccuracyDecision, ConfidenceEstimate, MAX_CONFIDENCE_FAMILY_SIZE, TwoIntervalRateWindow,
    VelocityMeasurementOutcome, classify_absolute_rate, classify_plateau,
    confidence_multiplier_95_df15, estimate_confidence,
};
pub use observation::{
    BlockMoments, RateBlock, RawVelocityObservation, VELOCITY_RATE_Q_SCALE,
    VelocityMeasurementError,
};
pub use stationarity::{
    StationarityDiagnostics, StationarityExclusion, StationarityResult, StationarityWork,
    StationarityWorkPoll, StationarySuffix, SuffixDiagnostic, VELOCITY_SUFFIX_LENGTHS,
    VelocityObservationSource, analyze_stationarity_diagnostics, select_stationary_suffix,
    select_stationary_suffix_from_source,
};
