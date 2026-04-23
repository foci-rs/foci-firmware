//! `#[klipper_output]` struct declarations for openffboard-fw.
//!
//! Each struct in this module is a Klipper output shape sent to the host
//! via `::ankyra::klipper_output!(Path, field: ty = expr, ...)`. Outputs
//! differ from replies only in the wire-direction marker trait
//! (`OutputPayload` vs `ReplyPayload`); the encoding and dictionary
//! entries are built identically.
//!
//! # Wire-name preservation
//!
//! Ankyra derives the protocol name from the struct identifier and
//! automatically converts PascalCase idents to `snake_case` on the wire
//! (e.g. `Stats` → `stats`). In the absence of an explicit
//! `#[klipper_output(format = "...")]` override, ankyra synthesizes the
//! Klipper-style message format as `"<wire_name> <field1>=%<spec1> ..."`.
//! Struct names in this module therefore use idiomatic Rust PascalCase
//! while preserving the on-the-wire protocol names expected by the
//! Klipper host.

use ankyra::prelude::klipper_output;

/// `stats` output — the once-per-second watchdog/accumulator frame
/// Klipper's logs surface as `Stats <clock> mcu=<name> ...`.
///
/// Emitted by the `stats_tick` task each time the watchdog is petted.
/// The field names and order reproduce the pre-migration anchor wire
/// format `"stats count=%u sum=%u sumsq=%u"` so the Klipper host's
/// dictionary match and per-field parser continue to work unchanged.
/// Ankyra's PascalCase-to-snake_case conversion emits the wire name
/// `stats` from the `Stats` ident.
#[klipper_output]
pub struct Stats {
    /// Number of stats ticks accumulated since the last emission.
    pub count: u32,
    /// Sum of the per-tick elapsed clock deltas.
    pub sum: u32,
    /// Sum of the squared per-tick elapsed deltas (scaled by
    /// `STATS_SUMSQ_BASE`).
    pub sumsq: u32,
}
