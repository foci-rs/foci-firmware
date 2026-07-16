//! Fixed-memory encoder velocity measurement and confidence primitives.

mod observation;

pub use observation::{
    BlockMoments, RateBlock, RawVelocityObservation, VELOCITY_RATE_Q_SCALE,
    VelocityMeasurementError,
};
