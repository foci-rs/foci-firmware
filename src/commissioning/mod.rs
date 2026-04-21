//! Commissioning state machine — public types and (forthcoming) submodules.
//!
//! This module is currently under active extraction from the sibling
//! `commission` module. During the split, `commission` holds the step
//! functions and state machine while `commissioning::types` owns the
//! data structures and trait surface re-exported to consumers.

pub mod helpers;
pub mod inner;
pub mod types;
