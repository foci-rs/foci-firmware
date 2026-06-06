//! Shared USB trace protocol, events, pressure state, and transport helpers.

pub mod events;

#[cfg(feature = "trace")]
pub mod pressure;

#[cfg(feature = "trace")]
pub mod stream;
