//! Ankyra Klipper-protocol surface shared across all FOCI boards.
//!
//! - `handlers::*` — `#[klipper_command]` handlers taking capability
//!   trait objects. Each subsystem has its own module.
//! - `replies.rs` — `#[klipper_reply]` struct declarations.
//! - `constants.rs` — firmware-wide `#[klipper_constant]` items.
//! - `provider.rs` — `FOCI_FIRMWARE_PROVIDER`.
//!
//! Board crates compose this provider with their own `BOARD_PROVIDER`
//! in `ankyra_config!`.

pub mod constants;
pub mod handlers;
pub mod provider;
pub mod replies;
pub mod velocity_sweep_encoding;
