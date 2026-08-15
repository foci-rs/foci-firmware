//! Fixed-memory encoder velocity measurement and confidence primitives.

mod blocker;
mod decision;
mod observation;
mod rest;
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
pub use rest::{
    RestSelectionDecision, RestSelectionDisposition, RestStructuralCode, VELOCITY_SUFFIX_LENGTHS,
};
// `sweep_reporting` re-exports these through the
// module root; `stationarity` reaches them via `super::rest` directly.
pub(crate) use rest::{RestSuffixEvaluation, select_causal_rest_suffixes};
pub use stationarity::{
    RestSelectionCompletion, StationarityDiagnostics, StationarityExclusion, StationarityResult,
    StationarityWork, StationarityWorkPoll, StationarySuffix, StructuredStationarityEvidence,
    SuffixDiagnostic, VelocityObservationSource, analyze_stationarity_diagnostics,
    lag_one_passes_family, select_stationary_suffix, select_stationary_suffix_from_source,
};
